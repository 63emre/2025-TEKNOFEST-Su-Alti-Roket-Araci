#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
Plus Wing (+) Konfigürasyonu Kapsamlı Servo Test Sistemi
Gerçek Zamanlı Pixhawk Sensor Movement + PID Kontrol + Hassas Ölçümler

Hardware:
- AUX 3: Ön Servo (DS3230MG 30kg)
- AUX 4: Sol Servo (DS3230MG 30kg)  
- AUX 5: Sağ Servo (DS3230MG 30kg)
- AUX 6: Arka Servo (DS3230MG 30kg)
- Pixhawk IMU (Roll, Pitch, Yaw)

Test Özellikleri:
- Gerçek zamanlı IMU feedback
- PID kontrol döngüsü
- Plus Wing vs X Wing karşılaştırması
- Türkçe arayüz ve raporlama
"""

import os
import sys
import time
import threading
import math
import json
from datetime import datetime
from collections import deque
from pymavlink import mavutil

# Plus Wing konfigürasyonu import et
try:
    from hardware_config import (
        PLUS_WING_SERVO_CHANNELS, 
        PLUS_WING_CONFIG,
        calculate_plus_wing_pwm,
        get_plus_wing_config
    )
except ImportError:
    print("❌ hardware_config.py bulunamadı!")
    sys.exit(1)

# Connection configuration - dynamic IP support
try:
    import sys
    sys.path.append('../Test')
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Fallback connection: {MAV_ADDRESS}")

# D300 Depth Sensor - Removed (not used)

class PIDController:
    """PID Controller - Plus Wing için optimize edilmiş"""
    
    def __init__(self, kp, ki, kd, max_output=400, integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral_limit = integral_limit
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Performans metrikleri
        self.error_history = deque(maxlen=100)
        self.output_history = deque(maxlen=100)
        
    def update(self, target, current):
        """PID çıkışını güncelle"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01  # Minimum delta time
            
        # Error hesaplama
        error = target - current
        self.error_history.append(error)
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (windup protection)
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_error = (error - self.last_error) / dt
        d_term = self.kd * d_error
        
        # PID output
        output = p_term + i_term + d_term
        output = max(-self.max_output, min(self.max_output, output))
        
        self.output_history.append(output)
        
        # Update için değerleri sakla
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def get_performance_stats(self):
        """PID performans istatistikleri"""
        if not self.error_history:
            return {}
            
        errors = list(self.error_history)
        outputs = list(self.output_history)
        
        return {
            'rms_error': math.sqrt(sum(e*e for e in errors) / len(errors)),
            'max_error': max(abs(e) for e in errors),
            'avg_output': sum(outputs) / len(outputs) if outputs else 0,
            'output_range': max(outputs) - min(outputs) if outputs else 0
        }
    
    def reset(self):
        """PID controller'ı sıfırla"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.error_history.clear()
        self.output_history.clear()

# D300DepthSensor class removed - not used in this configuration

class PlusWingServoTester:
    """Plus Wing Servo Test Sistemi - Gerçek Zamanlı PID Kontrol"""
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.armed = False
        self.testing = False
        
        # Sensor systems - D300 removed
        
        # PID Controllers - Plus Wing optimize edilmiş parametreler
        config = get_plus_wing_config()
        pid_params = config['PID_PARAMS']
        
        self.pid_roll = PIDController(**pid_params['roll'])
        self.pid_pitch = PIDController(**pid_params['pitch'])
        self.pid_yaw = PIDController(**pid_params['yaw'])
        # Depth PID removed - no motor control
        
        # Sensor data storage
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'timestamp': time.time()
        }
        
        # Depth data removed - no D300 sensor
        
        # Test metrics
        self.test_results = {
            'start_time': None,
            'test_duration': 0,
            'servo_commands_sent': 0,
            'pid_updates': 0,
            'sensor_readings': 0,
            'performance_metrics': {}
        }
        
        # Threading control
        self.sensor_thread = None
        self.control_thread = None
        self.stop_threads = False
        
        print("🚀 TEKNOFEST Plus Wing (+) Servo Test Sistemi")
        print("=" * 60)
        print(f"📡 MAVLink: {MAV_ADDRESS}")
        print(f"🔧 Servo Kanalları: {PLUS_WING_SERVO_CHANNELS}")
        print("📊 D300 Sensör: Kaldırıldı (kullanılmıyor)")
    
    def connect(self):
        """MAVLink bağlantısı kur"""
        try:
            print(f"📡 Pixhawk bağlantısı kuruluyor...")
            
            # Parse connection string
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=15)
            
            if heartbeat:
                self.connected = True
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print("✅ MAVLink bağlantısı kuruldu!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Armed Status: {'ARMED' if self.armed else 'DISARMED'}")
                
                # Start sensor threads
                self.start_sensor_threads()
                return True
            else:
                print("❌ Heartbeat alınamadı!")
                return False
                
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def start_sensor_threads(self):
        """Sensor okuma thread'lerini başlat"""
        self.stop_threads = False
        
        # IMU data thread
        self.sensor_thread = threading.Thread(target=self._sensor_reader_thread, daemon=True)
        self.sensor_thread.start()
        
        print("🔄 Sensor okuma thread'leri başlatıldı")
    
    def _sensor_reader_thread(self):
        """Sensor data okuma thread'i"""
        while not self.stop_threads and self.connected:
            try:
                # IMU data oku
                msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                if msg:
                    self.imu_data = {
                        'roll': math.degrees(msg.roll),
                        'pitch': math.degrees(msg.pitch),
                        'yaw': math.degrees(msg.yaw),
                        'timestamp': time.time()
                    }
                    self.test_results['sensor_readings'] += 1
                
                # Depth sensor removed
                
                time.sleep(0.02)  # 50Hz sensor okuma
                
            except Exception as e:
                print(f"❌ Sensor okuma hatası: {e}")
                time.sleep(0.1)
    
    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        if not self.connected:
            return False
        
        pwm_value = max(1000, min(2000, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            self.test_results['servo_commands_sent'] += 1
            return True
            
        except Exception as e:
            print(f"❌ Servo PWM komutu hatası: {e}")
            return False
    
    def send_plus_wing_commands(self, roll_cmd, pitch_cmd, yaw_cmd):
        """Plus Wing servo komutlarını gönder"""
        # Plus Wing PWM hesaplama
        pwm_values = calculate_plus_wing_pwm(roll_cmd, pitch_cmd, yaw_cmd)
        
        # Servo komutlarını gönder
        results = []
        for servo_name, pwm_value in pwm_values.items():
            channel = PLUS_WING_SERVO_CHANNELS[servo_name]
            success = self.set_servo_pwm(channel, pwm_value)
            results.append(success)
        
        return all(results)
    
    def test_individual_servos(self):
        """Bireysel servo testi - Plus Wing konfigürasyonu"""
        print("\n🎮 Plus Wing Bireysel Servo Testi")
        print("-" * 40)
        
        test_sequence = [
            ('Neutral', 1500),
            ('Min Position', 1000),
            ('Max Position', 2000),
            ('Neutral', 1500)
        ]
        
        for servo_name, channel in PLUS_WING_SERVO_CHANNELS.items():
            # Test all servos - no motor in this configuration
                
            print(f"\n🔧 Test: {servo_name.title()} (Channel {channel})")
            
            for position_name, pwm_value in test_sequence:
                print(f"   → {position_name}: {pwm_value}µs")
                
                if self.set_servo_pwm(channel, pwm_value):
                    print(f"   ✅ Komut başarılı")
                else:
                    print(f"   ❌ Komut başarısız")
                
                time.sleep(2.0)
        
        print("✅ Bireysel servo testi tamamlandı")
    
    def test_plus_wing_movements(self):
        """Plus Wing hareket testi"""
        print("\n🎮 Plus Wing Hareket Matrisi Testi")
        print("-" * 40)
        
        movements = [
            (0, 0, 0, "Neutral Pozisyon"),
            (50, 0, 0, "Roll Sağ (+50)"),
            (-50, 0, 0, "Roll Sol (-50)"),
            (0, 50, 0, "Pitch Yukarı (+50)"),
            (0, -50, 0, "Pitch Aşağı (-50)"),
            (0, 0, 50, "Yaw Sağ (+50)"),
            (0, 0, -50, "Yaw Sol (-50)"),
            (25, 25, 0, "Roll+Pitch Kombine"),
            (0, 0, 0, "Neutral Pozisyon")
        ]
        
        for roll, pitch, yaw, description in movements:
            print(f"\n🎯 Hareket: {description}")
            print(f"   Komutlar: Roll={roll}, Pitch={pitch}, Yaw={yaw}")
            
            # PWM değerlerini hesapla ve göster
            pwm_values = calculate_plus_wing_pwm(roll, pitch, yaw)
            for servo, pwm in pwm_values.items():
                print(f"   {servo}: {pwm}µs")
            
            # Komutları gönder
            if self.send_plus_wing_commands(roll, pitch, yaw):
                print("   ✅ Tüm servo komutları başarılı")
            else:
                print("   ❌ Servo komut hatası")
            
            time.sleep(3.0)
        
        print("✅ Plus Wing hareket testi tamamlandı")
    
    def test_pid_control_loop(self, duration=30):
        """PID kontrol döngüsü testi - Gerçek zamanlı feedback (Servo only)"""
        print(f"\n🎮 PID Kontrol Döngüsü Testi ({duration}s)")
        print("-" * 40)
        
        if not self.armed:
            print("⚠️ Sistem DISARMED. PID testi için ARM gerekli.")
            return
        
        # Target değerleri (servo control only)
        target_roll = 0.0
        target_pitch = 0.0
        target_yaw = 0.0
        
        print(f"🎯 Hedef Değerler:")
        print(f"   Roll: {target_roll}°")
        print(f"   Pitch: {target_pitch}°")
        print(f"   Yaw: {target_yaw}°")
        
        start_time = time.time()
        self.testing = True
        
        # PID kontrolörleri sıfırla
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
        
        try:
            while (time.time() - start_time) < duration and self.testing:
                current_time = time.time()
                
                # Mevcut sensor değerleri
                current_roll = self.imu_data['roll']
                current_pitch = self.imu_data['pitch']
                current_yaw = self.imu_data['yaw']
                
                # PID hesaplamaları
                roll_output = self.pid_roll.update(target_roll, current_roll)
                pitch_output = self.pid_pitch.update(target_pitch, current_pitch)
                yaw_output = self.pid_yaw.update(target_yaw, current_yaw)
                
                # Servo komutlarına çevir (scale to -100, +100)
                roll_cmd = max(-100, min(100, roll_output / 4))
                pitch_cmd = max(-100, min(100, pitch_output / 4))
                yaw_cmd = max(-100, min(100, yaw_output / 3))
                
                # Plus Wing komutlarını gönder
                self.send_plus_wing_commands(roll_cmd, pitch_cmd, yaw_cmd)
                
                self.test_results['pid_updates'] += 1
                
                # Progress display (her 2 saniyede bir)
                elapsed = current_time - start_time
                if int(elapsed) % 2 == 0 and int(elapsed * 10) % 10 == 0:
                    print(f"\n⏱️ {elapsed:.1f}s - Mevcut Durum:")
                    print(f"   Roll: {current_roll:.2f}° → {roll_cmd:.1f}")
                    print(f"   Pitch: {current_pitch:.2f}° → {pitch_cmd:.1f}")
                    print(f"   Yaw: {current_yaw:.2f}° → {yaw_cmd:.1f}")
                
                time.sleep(0.05)  # 20Hz kontrol döngüsü
                
        except KeyboardInterrupt:
            print("\n⚠️ PID testi kullanıcı tarafından durduruldu")
        
        self.testing = False
        
        # Servo'ları neutral'a getir
        self.send_plus_wing_commands(0, 0, 0)
        
        # PID performans istatistikleri
        print(f"\n📊 PID Performans İstatistikleri:")
        for name, pid in [('Roll', self.pid_roll), ('Pitch', self.pid_pitch), ('Yaw', self.pid_yaw)]:
            stats = pid.get_performance_stats()
            if stats:
                print(f"   {name}:")
                print(f"     RMS Error: {stats['rms_error']:.3f}")
                print(f"     Max Error: {stats['max_error']:.3f}")
                print(f"     Avg Output: {stats['avg_output']:.3f}")
        
        print("✅ PID kontrol döngüsü testi tamamlandı")
    
    def test_precision_measurements(self):
        """Hassas ölçüm testi - Plus Wing precision"""
        print("\n🎮 Hassas Ölçüm ve Precision Testi")
        print("-" * 40)
        
        # Hassasiyet test senaryoları
        precision_tests = [
            (5, 0, 0, "Küçük Roll (+5°)"),
            (-5, 0, 0, "Küçük Roll (-5°)"),
            (0, 5, 0, "Küçük Pitch (+5°)"),
            (0, -5, 0, "Küçük Pitch (-5°)"),
            (0, 0, 5, "Küçük Yaw (+5°)"),
            (0, 0, -5, "Küçük Yaw (-5°)"),
            (2, 2, 2, "Mikro Kombine Hareket"),
            (0, 0, 0, "Neutral Return")
        ]
        
        precision_results = []
        
        for roll_target, pitch_target, yaw_target, description in precision_tests:
            print(f"\n🎯 Precision Test: {description}")
            
            # Başlangıç pozisyonu kaydet
            start_roll = self.imu_data['roll']
            start_pitch = self.imu_data['pitch'] 
            start_yaw = self.imu_data['yaw']
            
            # Komutu gönder
            self.send_plus_wing_commands(roll_target * 10, pitch_target * 10, yaw_target * 10)
            
            # Stabilizasyon bekle
            time.sleep(2.0)
            
            # Son pozisyonu ölç
            end_roll = self.imu_data['roll']
            end_pitch = self.imu_data['pitch']
            end_yaw = self.imu_data['yaw']
            
            # Precision hesapla
            roll_error = abs((end_roll - start_roll) - roll_target)
            pitch_error = abs((end_pitch - start_pitch) - pitch_target)
            yaw_error = abs((end_yaw - start_yaw) - yaw_target)
            
            precision_results.append({
                'test': description,
                'roll_error': roll_error,
                'pitch_error': pitch_error,
                'yaw_error': yaw_error,
                'total_error': roll_error + pitch_error + yaw_error
            })
            
            print(f"   Roll Error: {roll_error:.3f}°")
            print(f"   Pitch Error: {pitch_error:.3f}°")
            print(f"   Yaw Error: {yaw_error:.3f}°")
            print(f"   Total Error: {roll_error + pitch_error + yaw_error:.3f}°")
            
            time.sleep(1.0)
        
        # Precision özeti
        avg_roll_error = sum(r['roll_error'] for r in precision_results) / len(precision_results)
        avg_pitch_error = sum(r['pitch_error'] for r in precision_results) / len(precision_results)
        avg_yaw_error = sum(r['yaw_error'] for r in precision_results) / len(precision_results)
        
        print(f"\n📊 Plus Wing Precision Özeti:")
        print(f"   Ortalama Roll Error: {avg_roll_error:.3f}°")
        print(f"   Ortalama Pitch Error: {avg_pitch_error:.3f}°")
        print(f"   Ortalama Yaw Error: {avg_yaw_error:.3f}°")
        print(f"   Genel Precision: {avg_roll_error + avg_pitch_error + avg_yaw_error:.3f}°")
        
        self.test_results['performance_metrics']['precision'] = {
            'avg_roll_error': avg_roll_error,
            'avg_pitch_error': avg_pitch_error,
            'avg_yaw_error': avg_yaw_error,
            'total_precision_error': avg_roll_error + avg_pitch_error + avg_yaw_error
        }
        
        print("✅ Hassas ölçüm testi tamamlandı")
    
    def run_comprehensive_test(self):
        """Kapsamlı Plus Wing test paketi"""
        print("🚀 TEKNOFEST Plus Wing (+) Kapsamlı Test Sistemi")
        print("=" * 60)
        
        # Test başlangıç zamanı
        self.test_results['start_time'] = datetime.now()
        start_time = time.time()
        
        # Bağlantı testi
        if not self.connect():
            print("❌ Bağlantı testi başarısız!")
            return False
        
        # ARM durumu kontrolü
        if not self.armed:
            print("⚠️ Sistem DISARMED. Bazı testler sınırlı olabilir.")
        
        # Test suite
        tests = [
            ("Bireysel Servo Testi", self.test_individual_servos),
            ("Plus Wing Hareket Testi", self.test_plus_wing_movements),
            ("Hassas Ölçüm Testi", self.test_precision_measurements),
        ]
        
        # ARM durumunda ek testler
        if self.armed:
            tests.append(("PID Kontrol Döngüsü Testi", lambda: self.test_pid_control_loop(20)))
        
        # Testleri çalıştır
        results = []
        for test_name, test_func in tests:
            try:
                print(f"\n{'='*20} {test_name} {'='*20}")
                test_func()
                results.append((test_name, True))
                print(f"✅ {test_name}: TAMAMLANDI")
            except Exception as e:
                print(f"❌ {test_name}: HATA - {e}")
                results.append((test_name, False))
        
        # Test süresi hesapla
        self.test_results['test_duration'] = time.time() - start_time
        
        # Test özeti
        self.print_test_summary(results)
        
        # Test sonuçlarını kaydet
        self.save_test_results()
        
        return all(result for _, result in results)
    
    def print_test_summary(self, results):
        """Test özeti yazdır"""
        print(f"\n📋 PLUS WING TEST ÖZETİ")
        print("=" * 50)
        
        completed = sum(1 for _, result in results if result)
        total = len(results)
        
        # Test sonuçları
        for test_name, result in results:
            status = "✅ BAŞARILI" if result else "❌ BAŞARISIZ"
            print(f"   {test_name}: {status}")
        
        # Genel istatistikler
        print(f"\n📊 Test İstatistikleri:")
        print(f"   Toplam Test: {total}")
        print(f"   Başarılı: {completed}")
        print(f"   Başarı Oranı: {completed/total*100:.1f}%")
        print(f"   Test Süresi: {self.test_results['test_duration']:.1f}s")
        print(f"   Servo Komutları: {self.test_results['servo_commands_sent']}")
        print(f"   PID Updates: {self.test_results['pid_updates']}")
        print(f"   Sensor Okumaları: {self.test_results['sensor_readings']}")
        
        # Performans metrikleri
        if 'precision' in self.test_results['performance_metrics']:
            precision = self.test_results['performance_metrics']['precision']
            print(f"\n🎯 Plus Wing Precision Performansı:")
            print(f"   Total Precision Error: {precision['total_precision_error']:.3f}°")
            print(f"   Roll Precision: {precision['avg_roll_error']:.3f}°")
            print(f"   Pitch Precision: {precision['avg_pitch_error']:.3f}°")
            print(f"   Yaw Precision: {precision['avg_yaw_error']:.3f}°")
        
        # Genel değerlendirme
        if completed == total:
            print("\n🎉 TÜM TESTLER BAŞARILI! Plus Wing sistemi hazır!")
        elif completed > total // 2:
            print("\n⚠️ KISMEN BAŞARILI. Bazı testler başarısız ancak temel sistem çalışıyor.")
        else:
            print("\n❌ MAJOR SORUNLAR. Plus Wing sistemi troubleshooting gerektirir.")
    
    def save_test_results(self):
        """Test sonuçlarını JSON dosyasına kaydet"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"plus_wing_test_results_{timestamp}.json"
        
        # Test sonuçlarını serileştirilebilir hale getir
        results_to_save = {
            'test_info': {
                'system': 'Plus Wing (+) Configuration',
                'timestamp': self.test_results['start_time'].isoformat() if self.test_results['start_time'] else None,
                'duration': self.test_results['test_duration'],
                'mav_address': MAV_ADDRESS
            },
            'statistics': {
                'servo_commands_sent': self.test_results['servo_commands_sent'],
                'pid_updates': self.test_results['pid_updates'],
                'sensor_readings': self.test_results['sensor_readings']
            },
            'performance_metrics': self.test_results['performance_metrics'],
            'system_config': {
                'servo_channels': PLUS_WING_SERVO_CHANNELS,
                'armed_status': self.armed,
                'depth_sensor_connected': False  # D300 removed
            }
        }
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(results_to_save, f, indent=2, ensure_ascii=False)
            print(f"\n💾 Test sonuçları kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Test sonuçları kaydetme hatası: {e}")
    
    def disconnect(self):
        """Bağlantıyı kapat ve cleanup"""
        # Thread'leri durdur
        self.stop_threads = True
        if self.sensor_thread:
            self.sensor_thread.join(timeout=2.0)
        
        # Servo'ları neutral'a getir
        if self.connected:
            print("🔄 Tüm servo'ları neutral pozisyona getiriliyor...")
            self.send_plus_wing_commands(0, 0, 0)
            time.sleep(1.0)
        
        # MAVLink bağlantısını kapat
        if self.master:
            try:
                self.master.close()
                print("🔌 MAVLink bağlantısı kapatıldı")
            except:
                pass
        
        self.connected = False

def main():
    """Ana test fonksiyonu"""
    tester = PlusWingServoTester()
    
    try:
        # Kapsamlı test suite çalıştır
        success = tester.run_comprehensive_test()
        
        # Cleanup
        tester.disconnect()
        
        # Exit code
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
        tester.disconnect()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Test suite hatası: {e}")
        tester.disconnect()
        sys.exit(1)

if __name__ == "__main__":
    main()
