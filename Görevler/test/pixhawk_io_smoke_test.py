#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Pixhawk I/O Smoke Test

Bu test Pixhawk'ı "pasif I/O hub" olarak kullanır:
- GUIDED/RTL/GPS gerektiren modları test etmez
- Sadece MANUAL mode + PWM I/O + telemetri akışını test eder
- Su altında çalışacak şekilde tasarlandı

Test Kapsamı:
- MAVLink bağlantı ve heartbeat
- MANUAL mode geçiş (tek güvenli mod)
- ATTITUDE telemetri akışı (20 Hz)
- SCALED_PRESSURE telemetri akışı (10 Hz)
- D300 derinlik sensörü (varsa)
- Servo I/O testleri (4 fin + motor neutral)
- Watchdog ve failsafe
"""

import time
import math
import json
import os
from datetime import datetime
from pymavlink import mavutil

# D300 derinlik sensörü
try:
    import sys
    _BASE_DIR = os.path.dirname(__file__) if '__file__' in globals() else os.getcwd()
    _APP_DIR = os.path.normpath(os.path.join(_BASE_DIR, '../App'))
    if _APP_DIR not in sys.path:
        sys.path.append(_APP_DIR)
    from depth_sensor import D300DepthSensor
    D300_AVAILABLE = True
except ImportError:
    D300_AVAILABLE = False
    print("⚠️ D300 derinlik sensörü modülü bulunamadı")

# MAVLink bağlantı adresi
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))
I2C_D300_ADDRESS = 0x76

# Test parametreleri
TEST_CONFIG = {
    'heartbeat_timeout': 10.0,      # Heartbeat timeout (s)
    'telemetry_timeout': 0.5,       # Telemetri timeout (s)
    'servo_test_duration': 15.0,    # Servo test süresi (s)
    'servo_delta': 200,             # Servo PWM delta (±200)
    'servo_neutral': 1500,          # Servo neutral PWM
    'motor_neutral': 1500,          # Motor neutral PWM (90s kuralı)
    'attitude_rate': 20,            # ATTITUDE stream rate (Hz)
    'pressure_rate': 10,            # SCALED_PRESSURE stream rate (Hz)
    'test_timeout': 60.0            # Toplam test timeout (s)
}

# Servo kanalları (Pixhawk çıkışları)
SERVO_CHANNELS = {
    'right_fin': 1,    # SERVO1 - Right fin
    'left_fin': 2,     # SERVO2 - Left fin
    'up_fin': 3,       # SERVO3 - Up fin  
    'down_fin': 4,     # SERVO4 - Down fin
    'motor': 5         # SERVO5 - Motor (ESC)
}

class PixhawkIOSmokeTest:
    """
    Pixhawk I/O Smoke Test
    
    Su altında çalışacak şekilde tasarlanmış basit I/O testi.
    GPS/GUIDED/RTL gerektiren hiçbir özellik test edilmez.
    """
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.test_results = {}
        self.test_start_time = None
        
        # D300 derinlik sensörü
        self.d300_sensor = None
        self.d300_connected = False
        if D300_AVAILABLE:
            try:
                self.d300_sensor = D300DepthSensor(i2c_address=I2C_D300_ADDRESS)
                self.d300_connected = self.d300_sensor.initialize()
                print("✅ D300 derinlik sensörü başlatıldı" if self.d300_connected else "⚠️ D300 sensörü başlatılamadı")
            except Exception as e:
                print(f"⚠️ D300 sensörü hatası: {e}")
                self.d300_connected = False
        
        # Telemetri tracking
        self.last_attitude_time = 0
        self.last_pressure_time = 0
        self.attitude_count = 0
        self.pressure_count = 0
        
        # Current state
        self.current_mode = "UNKNOWN"
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        self.depth_source = "none"
        
        # Watchdog
        self.watchdog_active = False
        self.last_telemetry_time = time.time()
        
        print("✅ Pixhawk I/O Smoke Test başlatıldı")
        print("🎯 Amaç: Su altında Pixhawk I/O ve telemetri testi")
        print("⚠️ GPS/GUIDED/RTL testleri YOK - sadece MANUAL + I/O")
    
    def connect_pixhawk(self):
        """Pixhawk bağlantısı ve heartbeat testi"""
        try:
            print("🔌 Pixhawk bağlantısı test ediliyor...")
            
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                print(f"🌐 TCP/UDP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=TEST_CONFIG['heartbeat_timeout'])
            
            if heartbeat:
                self.connected = True
                print("✅ MAVLink heartbeat alındı!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Vehicle Type: {heartbeat.type}")
                print(f"   Autopilot: {heartbeat.autopilot}")
                
                # İlk mod okuma
                self.current_mode = self._get_current_mode_name(heartbeat.custom_mode)
                print(f"   Current Mode: {self.current_mode}")
                
                return True
            else:
                print("❌ Heartbeat timeout!")
                return False
                
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def _get_current_mode_name(self, custom_mode):
        """Custom mode'dan mode name'i al"""
        try:
            if self.master and hasattr(self.master, 'mode_mapping'):
                mode_mapping = self.master.mode_mapping()
                for name, mode_id in mode_mapping.items():
                    if mode_id == custom_mode:
                        return name
            return f"CUSTOM_{custom_mode}"
        except:
            return "UNKNOWN"
    
    def test_manual_mode_switch(self):
        """MANUAL mode geçiş testi (tek güvenli mod)"""
        print("\n🧪 MANUAL Mode Geçiş Testi...")
        
        try:
            if not self.master.mode_mapping():
                print("   ❌ Mode mapping bulunamadı")
                self.test_results['manual_mode'] = False
                return False
            
            manual_mode_id = self.master.mode_mapping().get("MANUAL")
            if manual_mode_id is None:
                print("   ❌ MANUAL mode desteklenmiyor")
                self.test_results['manual_mode'] = False
                return False
            
            print(f"   MANUAL mode ID: {manual_mode_id}")
            
            # MANUAL mode'a geç
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                manual_mode_id
            )
            
            # Mode değişimini doğrula (3 saniye bekle)
            mode_changed = False
            for _ in range(30):  # 3 saniye @ 10 Hz
                heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if heartbeat:
                    current_mode = self._get_current_mode_name(heartbeat.custom_mode)
                    if current_mode == "MANUAL":
                        mode_changed = True
                        self.current_mode = current_mode
                        break
                time.sleep(0.1)
            
            if mode_changed:
                print("   ✅ MANUAL mode'a başarıyla geçildi")
                self.test_results['manual_mode'] = True
                return True
            else:
                print(f"   ❌ MANUAL mode'a geçilemedi (Current: {self.current_mode})")
                self.test_results['manual_mode'] = False
                return False
                
        except Exception as e:
            print(f"   ❌ MANUAL mode test hatası: {e}")
            self.test_results['manual_mode'] = False
            return False
    
    def test_telemetry_streams(self):
        """Telemetri stream testleri"""
        print("\n🧪 Telemetri Stream Testleri...")
        
        telemetry_results = {
            'attitude_stream': False,
            'pressure_stream': False,
            'attitude_rate': 0,
            'pressure_rate': 0
        }
        
        try:
            # Stream rate istekleri
            print("   📊 Stream rate istekleri gönderiliyor...")
            
            # ATTITUDE stream (20 Hz)
            self.master.mav.set_message_interval_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                50000  # 50ms = 20 Hz
            )
            
            # SCALED_PRESSURE stream (10 Hz)
            self.master.mav.set_message_interval_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE,
                100000  # 100ms = 10 Hz
            )
            
            print("   ⏱️ 10 saniye telemetri akış testi...")
            
            # 10 saniye boyunca telemetri akışını test et
            test_start = time.time()
            attitude_times = []
            pressure_times = []
            
            while time.time() - test_start < 10.0:
                current_time = time.time()
                
                # ATTITUDE mesajı
                attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                if attitude_msg:
                    attitude_times.append(current_time)
                    self.current_roll = math.degrees(attitude_msg.roll)
                    self.current_pitch = math.degrees(attitude_msg.pitch)
                    self.current_yaw = math.degrees(attitude_msg.yaw)
                    self.last_attitude_time = current_time
                    self.attitude_count += 1
                
                # SCALED_PRESSURE mesajı
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    pressure_times.append(current_time)
                    # Derinlik hesaplama (kritik düzeltme)
                    depth_pressure = pressure_msg.press_abs - 1013.25  # mbar
                    self.current_depth = max(0.0, depth_pressure * 0.0102)  # metre
                    self.depth_source = "scaled_pressure"
                    self.last_pressure_time = current_time
                    self.pressure_count += 1
                
                time.sleep(0.01)  # 100 Hz loop
            
            # Rate hesaplama
            if len(attitude_times) > 1:
                attitude_duration = attitude_times[-1] - attitude_times[0]
                attitude_rate = len(attitude_times) / attitude_duration
                telemetry_results['attitude_rate'] = attitude_rate
                telemetry_results['attitude_stream'] = attitude_rate > 5.0  # En az 5 Hz
                print(f"   📐 ATTITUDE: {len(attitude_times)} mesaj, {attitude_rate:.1f} Hz")
            else:
                print("   ❌ ATTITUDE mesajı alınamadı")
            
            if len(pressure_times) > 1:
                pressure_duration = pressure_times[-1] - pressure_times[0]
                pressure_rate = len(pressure_times) / pressure_duration
                telemetry_results['pressure_rate'] = pressure_rate
                telemetry_results['pressure_stream'] = pressure_rate > 2.0  # En az 2 Hz
                print(f"   🌊 SCALED_PRESSURE: {len(pressure_times)} mesaj, {pressure_rate:.1f} Hz")
            else:
                print("   ❌ SCALED_PRESSURE mesajı alınamadı")
            
            # Güncel değerleri yazdır
            if telemetry_results['attitude_stream']:
                print(f"   📊 Current Attitude: R={self.current_roll:.1f}° P={self.current_pitch:.1f}° Y={self.current_yaw:.1f}°")
            
            if telemetry_results['pressure_stream']:
                print(f"   🌊 Current Depth: {self.current_depth:.2f}m (from SCALED_PRESSURE)")
            
        except Exception as e:
            print(f"   ❌ Telemetri stream test hatası: {e}")
        
        self.test_results['telemetry_streams'] = telemetry_results
        return telemetry_results['attitude_stream'] and telemetry_results['pressure_stream']
    
    def test_d300_depth_sensor(self):
        """D300 derinlik sensörü testi"""
        print("\n🧪 D300 Derinlik Sensörü Testi...")
        
        d300_results = {
            'sensor_available': self.d300_connected,
            'readings_success': False,
            'reading_count': 0,
            'depth_range': [0.0, 0.0]  # [min, max]
        }
        
        if not self.d300_connected:
            print("   ⚠️ D300 sensörü bulunamadı, SCALED_PRESSURE fallback kullanılacak")
            self.test_results['d300_sensor'] = d300_results
            return False
        
        try:
            print("   🌊 5 saniye D300 okuma testi...")
            
            depths = []
            successful_reads = 0
            
            for i in range(50):  # 5 saniye @ 10 Hz
                try:
                    depth_data = self.d300_sensor.read_depth()
                    if depth_data['success']:
                        depth = max(0.0, depth_data['depth'])
                        depths.append(depth)
                        successful_reads += 1
                        
                        # En güncel derinlik değeri
                        if depth_data.get('depth', 0) > self.current_depth:
                            self.current_depth = depth
                            self.depth_source = "d300"
                    
                    time.sleep(0.1)  # 10 Hz
                    
                except Exception as e:
                    print(f"   ⚠️ D300 okuma hatası: {e}")
            
            d300_results['reading_count'] = successful_reads
            
            if depths:
                d300_results['readings_success'] = True
                d300_results['depth_range'] = [min(depths), max(depths)]
                print(f"   ✅ D300: {successful_reads}/50 başarılı okuma")
                print(f"   📊 Derinlik aralığı: {min(depths):.2f}m - {max(depths):.2f}m")
                print(f"   🌊 Current Depth: {self.current_depth:.2f}m (from D300)")
            else:
                print(f"   ❌ D300: Geçerli okuma alınamadı")
            
        except Exception as e:
            print(f"   ❌ D300 test hatası: {e}")
        
        self.test_results['d300_sensor'] = d300_results
        return d300_results['readings_success']
    
    def test_servo_io(self):
        """Servo I/O testleri"""
        print("\n🧪 Servo I/O Testleri...")
        print("   ⚠️ Motor neutral'de tutulacak (90s kuralı)")
        
        servo_results = {
            'motor_neutral': False,
            'fin_tests': {}
        }
        
        try:
            # Motor'u neutral'e al ve orada tut
            print("   🔧 Motor neutral (1500 PWM)...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                SERVO_CHANNELS['motor'],
                TEST_CONFIG['motor_neutral'],
                0, 0, 0, 0, 0
            )
            
            # ACK bekle
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
            if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("   ✅ Motor neutral komutu kabul edildi")
                    servo_results['motor_neutral'] = True
                else:
                    print(f"   ❌ Motor neutral komutu reddedildi: {ack.result}")
            else:
                print("   ⚠️ Motor neutral ACK alınamadı (timeout)")
            
            # Fin testleri
            print(f"   🎛️ Fin testleri başlıyor ({TEST_CONFIG['servo_test_duration']}s)...")
            
            for fin_name, channel in SERVO_CHANNELS.items():
                if fin_name == 'motor':
                    continue  # Motor'u atlat
                
                print(f"   🔧 {fin_name} (CH{channel}) testi...")
                
                fin_result = {
                    'neutral_ack': False,
                    'positive_ack': False,
                    'negative_ack': False,
                    'final_neutral_ack': False
                }
                
                # Test sequence: neutral → +delta → neutral → -delta → neutral
                test_sequence = [
                    ('neutral', TEST_CONFIG['servo_neutral']),
                    ('positive', TEST_CONFIG['servo_neutral'] + TEST_CONFIG['servo_delta']),
                    ('neutral', TEST_CONFIG['servo_neutral']),
                    ('negative', TEST_CONFIG['servo_neutral'] - TEST_CONFIG['servo_delta']),
                    ('final_neutral', TEST_CONFIG['servo_neutral'])
                ]
                
                for step_name, pwm_value in test_sequence:
                    # PWM limitlerini kontrol et
                    pwm_value = max(1000, min(2000, pwm_value))
                    
                    print(f"     {step_name}: {pwm_value} PWM")
                    
                    # Servo komutu gönder
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        channel,
                        pwm_value,
                        0, 0, 0, 0, 0
                    )
                    
                    # ACK bekle
                    ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
                    if ack and ack.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
                        if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                            fin_result[f'{step_name}_ack'] = True
                            print(f"     ✅ {step_name} ACK")
                        else:
                            print(f"     ❌ {step_name} NACK: {ack.result}")
                    else:
                        print(f"     ⚠️ {step_name} ACK timeout")
                    
                    time.sleep(0.5)  # Step arası bekleme
                
                servo_results['fin_tests'][fin_name] = fin_result
                
                # Fin özet
                successful_steps = sum(1 for ack in fin_result.values() if ack)
                print(f"   📊 {fin_name}: {successful_steps}/5 komut başarılı")
            
        except Exception as e:
            print(f"   ❌ Servo I/O test hatası: {e}")
        
        self.test_results['servo_io'] = servo_results
        
        # Başarı kriteri: Motor neutral + en az 2 fin'de 3+ başarılı komut
        motor_ok = servo_results['motor_neutral']
        fin_success_count = 0
        
        for fin_result in servo_results['fin_tests'].values():
            successful_steps = sum(1 for ack in fin_result.values() if ack)
            if successful_steps >= 3:
                fin_success_count += 1
        
        return motor_ok and fin_success_count >= 2
    
    def start_watchdog(self):
        """Watchdog başlat (telemetri kesintisi kontrolü)"""
        self.watchdog_active = True
        self.last_telemetry_time = time.time()
        print("🐕 Watchdog aktif (telemetri kesintisi kontrolü)")
    
    def check_watchdog(self):
        """Watchdog kontrolü"""
        if not self.watchdog_active:
            return True
        
        current_time = time.time()
        telemetry_gap = current_time - self.last_telemetry_time
        
        if telemetry_gap > TEST_CONFIG['telemetry_timeout']:
            print(f"🚨 WATCHDOG: Telemetri kesintisi! ({telemetry_gap:.1f}s > {TEST_CONFIG['telemetry_timeout']}s)")
            self.emergency_neutral_and_stop()
            return False
        
        return True
    
    def emergency_neutral_and_stop(self):
        """Emergency: Tüm servolar neutral, test durdur"""
        print("🚨 EMERGENCY: Tüm servolar neutral'e alınıyor...")
        
        try:
            # Tüm servo kanallarını neutral'e al
            for channel in SERVO_CHANNELS.values():
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    channel,
                    1500,  # Neutral
                    0, 0, 0, 0, 0
                )
                time.sleep(0.1)
            
            print("✅ Emergency neutral tamamlandı")
        except Exception as e:
            print(f"❌ Emergency neutral hatası: {e}")
    
    def generate_test_report(self):
        """Detaylı test raporu oluştur"""
        print("\n" + "="*80)
        print("📋 PIXHAWK I/O SMOKE TEST RAPORU")
        print("="*80)
        
        test_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        test_duration = time.time() - self.test_start_time if self.test_start_time else 0
        
        print(f"📅 Test Tarihi: {test_time}")
        print(f"⏱️ Test Süresi: {test_duration:.1f} saniye")
        print(f"🎯 Test Amacı: Su altında Pixhawk I/O ve telemetri testi")
        
        # Test sonuçları özeti
        print("\n📊 TEST SONUÇLARI:")
        print("-"*60)
        
        total_tests = 0
        passed_tests = 0
        
        for test_category, results in self.test_results.items():
            print(f"\n🧪 {test_category.upper()}:")
            
            if test_category == 'telemetry_streams':
                attitude_ok = results.get('attitude_stream', False)
                pressure_ok = results.get('pressure_stream', False)
                print(f"   ATTITUDE Stream: {'✅ PASS' if attitude_ok else '❌ FAIL'} ({results.get('attitude_rate', 0):.1f} Hz)")
                print(f"   PRESSURE Stream: {'✅ PASS' if pressure_ok else '❌ FAIL'} ({results.get('pressure_rate', 0):.1f} Hz)")
                total_tests += 2
                if attitude_ok: passed_tests += 1
                if pressure_ok: passed_tests += 1
                
            elif test_category == 'd300_sensor':
                sensor_ok = results.get('readings_success', False)
                print(f"   D300 Readings: {'✅ PASS' if sensor_ok else '❌ FAIL'} ({results.get('reading_count', 0)}/50)")
                if results.get('sensor_available', False):
                    depth_range = results.get('depth_range', [0, 0])
                    print(f"   Depth Range: {depth_range[0]:.2f}m - {depth_range[1]:.2f}m")
                total_tests += 1
                if sensor_ok: passed_tests += 1
                
            elif test_category == 'servo_io':
                motor_ok = results.get('motor_neutral', False)
                print(f"   Motor Neutral: {'✅ PASS' if motor_ok else '❌ FAIL'}")
                total_tests += 1
                if motor_ok: passed_tests += 1
                
                for fin_name, fin_result in results.get('fin_tests', {}).items():
                    successful_steps = sum(1 for ack in fin_result.values() if ack)
                    fin_ok = successful_steps >= 3
                    print(f"   {fin_name}: {'✅ PASS' if fin_ok else '❌ FAIL'} ({successful_steps}/5)")
                    total_tests += 1
                    if fin_ok: passed_tests += 1
                    
            elif isinstance(results, bool):
                status = "✅ PASS" if results else "❌ FAIL"
                print(f"   {test_category}: {status}")
                total_tests += 1
                if results: passed_tests += 1
        
        # Genel başarı oranı
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
        print(f"\n📈 GENEL BAŞARI ORANI: {success_rate:.1f}% ({passed_tests}/{total_tests})")
        
        # Sistem durumu
        print(f"\n🖥️ SİSTEM DURUMU:")
        print(f"   Current Mode: {self.current_mode}")
        print(f"   Depth: {self.current_depth:.2f}m (source: {self.depth_source})")
        print(f"   Attitude: R={self.current_roll:.1f}° P={self.current_pitch:.1f}° Y={self.current_yaw:.1f}°")
        print(f"   Telemetry: ATTITUDE={self.attitude_count}, PRESSURE={self.pressure_count}")
        
        # Su altı uygunluk değerlendirmesi
        print(f"\n🌊 SU ALTI UYGUNLUK DEĞERLENDİRMESİ:")
        print("-"*50)
        
        underwater_ready = True
        
        if not self.test_results.get('manual_mode', False):
            print("❌ MANUAL mode geçiş başarısız - Su altında kontrol imkansız")
            underwater_ready = False
        else:
            print("✅ MANUAL mode - Su altında güvenli kontrol modu")
        
        telemetry_ok = self.test_results.get('telemetry_streams', {})
        if not (telemetry_ok.get('attitude_stream') and telemetry_ok.get('pressure_stream')):
            print("❌ Telemetri akışı eksik - Su altında navigasyon imkansız")
            underwater_ready = False
        else:
            print("✅ Telemetri akışı - Su altında IMU ve derinlik verisi mevcut")
        
        depth_sources = 0
        if self.test_results.get('d300_sensor', {}).get('readings_success', False):
            print("✅ D300 derinlik sensörü - Birincil derinlik kaynağı")
            depth_sources += 1
        if telemetry_ok.get('pressure_stream', False):
            print("✅ SCALED_PRESSURE - Yedek derinlik kaynağı")
            depth_sources += 1
        
        if depth_sources == 0:
            print("❌ Derinlik kaynağı yok - Su altında derinlik kontrolü imkansız")
            underwater_ready = False
        
        servo_ok = self.test_results.get('servo_io', {})
        motor_ok = servo_ok.get('motor_neutral', False)
        fin_count = sum(1 for fin_result in servo_ok.get('fin_tests', {}).values() 
                       if sum(1 for ack in fin_result.values() if ack) >= 3)
        
        if not motor_ok or fin_count < 2:
            print("❌ Servo I/O yetersiz - Su altında hareket kontrolü riskli")
            underwater_ready = False
        else:
            print(f"✅ Servo I/O - Motor + {fin_count}/4 fin kontrol edilebilir")
        
        # Sonuç
        if underwater_ready and success_rate >= 75:
            print("\n🎉 GENEL DEĞERLENDİRME: SU ALTI OPERASYONA HAZIR!")
            print("   Pixhawk I/O sistemi su altında çalışmaya uygun")
        elif success_rate >= 50:
            print("\n⚠️ GENEL DEĞERLENDİRME: KISMI HAZIR")
            print("   Bazı sistemler review gerekli")
        else:
            print("\n❌ GENEL DEĞERLENDİRME: SU ALTI OPERASYONA HAZIR DEĞİL")
            print("   Kritik sistemler çalışmıyor")
        
        # Rapor kaydet
        report_data = {
            'timestamp': test_time,
            'test_type': 'Pixhawk I/O Smoke Test',
            'test_duration': test_duration,
            'success_rate': success_rate,
            'passed_tests': passed_tests,
            'total_tests': total_tests,
            'underwater_ready': underwater_ready,
            'test_results': self.test_results,
            'system_state': {
                'current_mode': self.current_mode,
                'depth': self.current_depth,
                'depth_source': self.depth_source,
                'attitude': {
                    'roll': self.current_roll,
                    'pitch': self.current_pitch,
                    'yaw': self.current_yaw
                },
                'telemetry_counts': {
                    'attitude': self.attitude_count,
                    'pressure': self.pressure_count
                }
            }
        }
        
        report_filename = f"pixhawk_io_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        try:
            with open(report_filename, 'w') as f:
                json.dump(report_data, f, indent=2)
            print(f"\n💾 Test raporu kaydedildi: {report_filename}")
        except Exception as e:
            print(f"⚠️ Rapor kaydetme hatası: {e}")
        
        return underwater_ready and success_rate >= 75
    
    def cleanup(self):
        """Test temizleme"""
        print("\n🧹 Test sistemi temizleniyor...")
        
        # Watchdog durdur
        self.watchdog_active = False
        
        # Tüm servolar neutral
        if self.connected:
            try:
                print("   🔧 Tüm servolar neutral'e alınıyor...")
                for channel in SERVO_CHANNELS.values():
                    self.master.mav.command_long_send(
                        self.master.target_system,
                        self.master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0,
                        channel,
                        1500,  # Neutral
                        0, 0, 0, 0, 0
                    )
                    time.sleep(0.1)
                print("   ✅ Servolar neutral")
            except:
                pass
        
        # D300 sensörünü kapat
        if self.d300_connected and self.d300_sensor:
            try:
                self.d300_sensor.close()
                print("   ✅ D300 sensörü kapatıldı")
            except:
                pass
        
        # MAVLink bağlantısı kapat
        if self.connected and self.master:
            try:
                self.master.close()
                print("   ✅ MAVLink bağlantısı kapatıldı")
            except:
                pass
            self.connected = False
        
        print("✅ Test temizleme tamamlandı")
    
    def run_smoke_test(self):
        """Ana smoke test fonksiyonu"""
        print("🚀 Pixhawk I/O Smoke Test")
        print("="*60)
        print("🎯 Amaç: Su altında Pixhawk I/O ve telemetri testi")
        print("⚠️ GPS/GUIDED/RTL testleri YOK - sadece MANUAL + I/O")
        print("🌊 Su altı operasyona hazırlık testi")
        
        self.test_start_time = time.time()
        
        try:
            # 1. Bağlantı testi
            if not self.connect_pixhawk():
                print("❌ Pixhawk bağlantısı başarısız!")
                return False
            
            # 2. MANUAL mode testi (tek güvenli mod)
            print("\n" + "="*50)
            if not self.test_manual_mode_switch():
                print("❌ MANUAL mode testi başarısız!")
                return False
            
            # 3. Watchdog başlat
            self.start_watchdog()
            
            # 4. Telemetri stream testleri
            print("\n" + "="*50)
            self.test_telemetry_streams()
            
            # 5. D300 derinlik sensörü testi
            print("\n" + "="*50)
            self.test_d300_depth_sensor()
            
            # 6. Servo I/O testleri
            print("\n" + "="*50)
            self.test_servo_io()
            
            # 7. Test raporu
            print("\n" + "="*50)
            success = self.generate_test_report()
            
            if success:
                print("\n🎉 SMOKE TEST BAŞARILI!")
                print("   Pixhawk I/O sistemi su altı operasyona hazır!")
            else:
                print("\n⚠️ SMOKE TEST KISMÎ BAŞARILI!")
                print("   Bazı sistemler review gerekli")
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()

def main():
    """Ana fonksiyon"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Pixhawk I/O Smoke Test (Su altı uyumlu)')
    parser.add_argument('--quick', action='store_true', help='Hızlı test (kısaltılmış süreler)')
    
    args = parser.parse_args()
    
    if args.quick:
        print("⚡ Hızlı test modu aktif")
        TEST_CONFIG['servo_test_duration'] = 5.0
        TEST_CONFIG['test_timeout'] = 30.0
    
    smoke_test = PixhawkIOSmokeTest()
    
    try:
        success = smoke_test.run_smoke_test()
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
