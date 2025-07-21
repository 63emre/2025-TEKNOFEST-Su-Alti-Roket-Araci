#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Sızdırmazlık Testi Demo
Video çekimi için sızdırmazlık kanıtlama scripti
"""

import time
import threading
from datetime import datetime
from pymavlink import mavutil
import json

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

# Test parametreleri
MINIMUM_TEST_DEPTH = 1.0    # Minimum 1m derinlik (şartname)
STATIC_TEST_DURATION = 90   # Statik test süresi (saniye)
DYNAMIC_TEST_DURATION = 120 # Dinamik test süresi (saniye)
SURFACE_PRESSURE = 1013.25  # Deniz seviyesi basıncı (hPa)

# Motor ve servo kanalları
MOTOR_CHANNEL = 8
SERVO_CHANNELS = [1, 2, 3, 4]  # Fin servolar
PAYLOAD_SERVO = 9  # Kapak açma/kapama servo

# PWM değerleri
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

class WaterproofDemo:
    def __init__(self):
        self.master = None
        self.connected = False
        self.test_active = False
        
        # Sensör verileri
        self.current_depth = 0.0
        self.current_pressure = SURFACE_PRESSURE
        self.surface_pressure = SURFACE_PRESSURE
        
        # Test durumu
        self.test_stage = "PREPARATION"
        self.test_start_time = None
        self.leak_detected = False
        self.max_depth_achieved = 0.0
        
        # Video çekimi için veri
        self.test_log = []
        self.telemetry_data = []
        
        # Threading
        self.monitoring_thread = None
        self.running = False
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı kur"""
        try:
            print("🔌 Pixhawk bağlantısı kuruluyor...")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def read_sensors(self):
        """Sensör verilerini oku"""
        if not self.connected:
            return False
            
        try:
            # Basınç sensörü oku
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if pressure_msg:
                self.current_pressure = pressure_msg.press_abs
                pressure_diff = self.current_pressure - self.surface_pressure
                self.current_depth = max(0, pressure_diff * 0.10197)  # hPa to meter
                
                # Maximum derinlik kaydet
                self.max_depth_achieved = max(self.max_depth_achieved, self.current_depth)
                
            # Telemetri verisi ekle
            timestamp = time.time()
            telemetry_entry = {
                'timestamp': timestamp,
                'depth': self.current_depth,
                'pressure': self.current_pressure,
                'stage': self.test_stage,
                'leak_detected': self.leak_detected
            }
            self.telemetry_data.append(telemetry_entry)
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def calibrate_surface_pressure(self):
        """Yüzey basıncını kalibre et"""
        print("🔧 Yüzey basınç kalibrasyonu...")
        
        pressure_samples = []
        for i in range(30):  # 30 saniye kalibrasyon
            if self.read_sensors():
                pressure_samples.append(self.current_pressure)
            
            print(f"  📊 Kalibrasyon: {i+1}/30 - Basınç: {self.current_pressure:.2f} hPa")
            time.sleep(1)
        
        if pressure_samples:
            self.surface_pressure = sum(pressure_samples) / len(pressure_samples)
            print(f"✅ Yüzey basıncı: {self.surface_pressure:.2f} hPa")
            return True
        else:
            print("❌ Kalibrasyon başarısız!")
            return False
    
    def check_for_leaks(self):
        """Sızıntı kontrol algoritması"""
        # Bu gerçek uygulamada sensör verileriyle yapılır
        # Demo için basit kontrol
        
        # Ani basınç değişimleri sızıntı belirtisi olabilir
        if len(self.telemetry_data) >= 10:
            recent_pressures = [data['pressure'] for data in self.telemetry_data[-10:]]
            pressure_variation = max(recent_pressures) - min(recent_pressures)
            
            # Anormal basınç değişimi kontrolü
            if pressure_variation > 5.0:  # 5 hPa threshold
                return True
                
        # Su içeride sıcaklık sensörü ile de kontrol edilebilir
        # Elektronic bay içi nem sensörü kontrolü
        
        return False
    
    def display_test_status(self):
        """Test durumunu ekrana yazdır"""
        print("\n" + "="*60)
        print(f"🎬 TEKNOFEST - SIZDIMAZLIK TESTİ - {self.test_stage}")
        print("="*60)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        test_duration = (time.time() - self.test_start_time) if self.test_start_time else 0
        
        print(f"⏰ Zaman: {timestamp} | Test Süresi: {test_duration:.0f}s")
        print(f"🌊 Mevcut Derinlik: {self.current_depth:.2f}m")
        print(f"📊 Maksimum Derinlik: {self.max_depth_achieved:.2f}m") 
        print(f"🔧 Basınç: {self.current_pressure:.2f} hPa")
        
        leak_status = "🔴 SIZINTI TESPIT EDİLDİ" if self.leak_detected else "✅ SIZDIMAZLIK TAMAM"
        print(f"💧 Sızdırmazlık: {leak_status}")
        
        # Şartname gereksinimlerini kontrol
        depth_status = "✅ YETER" if self.current_depth >= MINIMUM_TEST_DEPTH else f"❌ EKSİK ({MINIMUM_TEST_DEPTH:.1f}m gerekli)"
        print(f"📏 Derinlik Gereksinimi: {depth_status}")
        
        print("="*60)
    
    def set_motor_throttle(self, throttle_pwm):
        """Motor kontrolü"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL, throttle_pwm, 0, 0, 0, 0, 0
            )
            return True
        except:
            return False
    
    def set_servo_position(self, channel, pwm_value):
        """Servo kontrolü"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm_value, 0, 0, 0, 0, 0
            )
            return True
        except:
            return False
    
    def monitoring_loop(self):
        """Sürekli izleme döngüsü"""
        while self.running and self.test_active:
            self.read_sensors()
            
            # Sızıntı kontrolü
            if self.check_for_leaks():
                self.leak_detected = True
                print("🚨 SİZINTI TESPİT EDİLDİ!")
            
            # Her 5 saniyede durum göster
            if len(self.telemetry_data) % 5 == 0:
                self.display_test_status()
            
            time.sleep(1)
    
    def run_static_waterproof_test(self):
        """Statik sızdırmazlık testi"""
        print("\n🔧 1. STATİK SIZDIMAZLIK TESTİ BAŞLIYOR")
        print("="*50)
        
        self.test_stage = "STATIC_TEST"
        
        # Hedef derinliğe in
        target_depth = 1.2  # 1.2m hedef
        print(f"🎯 Hedef derinlik: {target_depth}m")
        
        # Derinlik kontrol döngüsü
        descend_start = time.time()
        while time.time() - descend_start < 60:  # Max 60s inme süresi
            # Motor ile yavaşça aşağı in
            self.set_motor_throttle(PWM_NEUTRAL - 100)  # Reverse thrust
            
            self.read_sensors()
            
            print(f"  📊 İniliyor: {self.current_depth:.2f}m / {target_depth}m")
            
            if self.current_depth >= target_depth:
                break
                
            time.sleep(2)
        
        # Motor durdur
        self.set_motor_throttle(PWM_NEUTRAL)
        
        if self.current_depth < MINIMUM_TEST_DEPTH:
            print(f"❌ Yeterli derinliğe inilemedi! ({self.current_depth:.2f}m < {MINIMUM_TEST_DEPTH}m)")
            return False
        
        print(f"✅ Hedef derinliğe ulaşıldı: {self.current_depth:.2f}m")
        
        # Statik test başlat
        print(f"\n⏱️ STATİK TEST BAŞLADI - {STATIC_TEST_DURATION} saniye beklenecek...")
        
        static_start = time.time()
        while time.time() - static_start < STATIC_TEST_DURATION:
            remaining_time = STATIC_TEST_DURATION - (time.time() - static_start)
            
            self.read_sensors()
            
            # Pozisyon tut (depth hold)
            depth_error = self.current_depth - target_depth
            if abs(depth_error) > 0.1:  # 10cm tolerance
                if depth_error > 0:
                    # Too deep, go up
                    self.set_motor_throttle(PWM_NEUTRAL + 50)
                else:
                    # Too shallow, go down  
                    self.set_motor_throttle(PWM_NEUTRAL - 50)
            else:
                self.set_motor_throttle(PWM_NEUTRAL)
            
            print(f"  ⏰ STATİK TEST - Kalan: {remaining_time:.0f}s | Derinlik: {self.current_depth:.2f}m")
            
            # Sızıntı kontrolü
            if self.leak_detected:
                print("🚨 STATİK TEST İPTAL - SIZINTI!")
                return False
                
            time.sleep(5)
        
        print("✅ STATİK SIZDIMAZLIK TESTİ TAMAMLANDI!")
        
        # Test log kaydet
        self.test_log.append({
            'stage': 'STATIC_TEST',
            'duration': STATIC_TEST_DURATION,
            'max_depth': self.max_depth_achieved,
            'leak_detected': self.leak_detected,
            'success': not self.leak_detected and self.current_depth >= MINIMUM_TEST_DEPTH
        })
        
        return True
    
    def run_dynamic_waterproof_test(self):
        """Dinamik sızdırmazlık testi"""
        print("\n🚀 2. DİNAMİK SIZDIMAZLIK TESTİ BAŞLIYOR")
        print("="*50)
        
        self.test_stage = "DYNAMIC_TEST"
        
        # Dinamik manevralar sırası
        maneuvers = [
            ("➡️ İleri seyir", PWM_NEUTRAL + 100, 20),
            ("↩️ Sol dönüş", PWM_NEUTRAL + 80, 15),
            ("↪️ Sağ dönüş", PWM_NEUTRAL + 80, 15),
            ("🔼 Yukarı yunuslama", PWM_NEUTRAL + 120, 15),
            ("🔽 Aşağı yunuslama", PWM_NEUTRAL + 120, 15),
            ("🌊 Zigzag hareketi", PWM_NEUTRAL + 90, 30),
            ("⏸️ Durma testi", PWM_NEUTRAL, 10)
        ]
        
        print(f"🎬 {len(maneuvers)} farklı manevrabilite testi yapılacak...")
        
        for i, (description, motor_pwm, duration) in enumerate(maneuvers):
            print(f"\n📍 Manevra {i+1}/{len(maneuvers)}: {description} ({duration}s)")
            
            maneuver_start = time.time()
            
            while time.time() - maneuver_start < duration:
                remaining = duration - (time.time() - maneuver_start)
                
                # Motor kontrolü
                self.set_motor_throttle(motor_pwm)
                
                # Zigzag için özel servo kontrolü
                if "Zigzag" in description:
                    elapsed = time.time() - maneuver_start
                    zigzag_angle = int(200 * math.sin(elapsed))  # Sinüs dalgası
                    for channel in SERVO_CHANNELS[:2]:  # Sol/sağ finler
                        self.set_servo_position(channel, PWM_NEUTRAL + zigzag_angle)
                else:
                    # Normal fin pozisyonu
                    for channel in SERVO_CHANNELS:
                        self.set_servo_position(channel, PWM_NEUTRAL)
                
                self.read_sensors()
                
                print(f"    ⏰ {remaining:.0f}s | Derinlik: {self.current_depth:.2f}m")
                
                # Sızıntı kontrolü
                if self.leak_detected:
                    print("🚨 DİNAMİK TEST İPTAL - SIZINTI!")
                    return False
                
                time.sleep(2)
            
            # Manevra arası stabilizasyon
            print("    ⏸️ Stabilizasyon...")
            self.set_motor_throttle(PWM_NEUTRAL)
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL)
            time.sleep(3)
        
        print("✅ DİNAMİK SIZDIMAZLIK TESTİ TAMAMLANDI!")
        
        # Test log kaydet
        self.test_log.append({
            'stage': 'DYNAMIC_TEST',
            'duration': sum([duration for _, _, duration in maneuvers]),
            'maneuvers': len(maneuvers),
            'max_depth': self.max_depth_achieved,
            'leak_detected': self.leak_detected,
            'success': not self.leak_detected
        })
        
        return True
    
    def run_payload_bay_test(self):
        """Kapak mekanizması sızdırmazlık testi"""
        print("\n📦 3. KAPAK MEKANİZMASI TESTİ BAŞLIYOR")
        print("="*50)
        
        self.test_stage = "PAYLOAD_TEST"
        
        # Kapak kapalı pozisyonda su altına in
        print("🔒 Kapak kapalı pozisyonda test...")
        
        # Kapağı tamamen kapat
        self.set_servo_position(PAYLOAD_SERVO, PWM_MIN)  # Kapalı pozisyon
        time.sleep(5)
        
        # 1.5m derinliğe in
        target_depth = 1.5
        descend_start = time.time()
        
        while time.time() - descend_start < 45:  # 45s max
            self.set_motor_throttle(PWM_NEUTRAL - 80)
            self.read_sensors()
            
            print(f"    📊 İniliyor: {self.current_depth:.2f}m / {target_depth}m")
            
            if self.current_depth >= target_depth:
                break
            time.sleep(2)
        
        self.set_motor_throttle(PWM_NEUTRAL)
        
        # 30 saniye kapak kapalı bekle
        print("⏱️ Kapak kapalı 30 saniye bekle...")
        for i in range(30):
            self.read_sensors()
            
            if self.leak_detected:
                print("🚨 KAPAK SIZINTI TESPİT EDİLDİ!")
                return False
                
            print(f"    ⏰ {30-i}s | Kapak: KAPALI | Derinlik: {self.current_depth:.2f}m")
            time.sleep(1)
        
        # Kapağı aç (kontrollü test)
        print("🔓 Kapak açma testi...")
        
        # Yavaşça aç (sızdırmazlık kontrolü için)
        open_positions = [PWM_MIN + 100, PWM_MIN + 200, PWM_MIN + 300, PWM_NEUTRAL]
        
        for pos in open_positions:
            self.set_servo_position(PAYLOAD_SERVO, pos)
            time.sleep(2)
            
            self.read_sensors()
            
            # Her pozisyonda 5 saniye bekle
            for j in range(5):
                if self.leak_detected:
                    print(f"🚨 KAPAK AÇILIRKEN SIZINTI! Pozisyon: {pos}")
                    return False
                time.sleep(1)
            
            print(f"    📊 Kapak pozisyon: {pos} | Derinlik: {self.current_depth:.2f}m")
        
        # Kapağı tekrar kapat
        print("🔒 Kapak kapatma testi...")
        self.set_servo_position(PAYLOAD_SERVO, PWM_MIN)
        time.sleep(5)
        
        # Final kontrol
        for i in range(15):
            self.read_sensors()
            if self.leak_detected:
                print("🚨 KAPAK KAPATMA SONRASI SIZINTI!")
                return False
            time.sleep(1)
        
        print("✅ KAPAK MEKANİZMASI SIZDIMAZLIK TESTİ TAMAMLANDI!")
        
        # Test log kaydet
        self.test_log.append({
            'stage': 'PAYLOAD_TEST',
            'success': not self.leak_detected,
            'max_depth': self.max_depth_achieved,
            'leak_detected': self.leak_detected
        })
        
        return True
    
    def generate_test_report(self):
        """Test raporu oluştur"""
        print("\n" + "="*70)
        print("📋 SIZDIMAZLIK TESTİ RAPORU")
        print("="*70)
        
        test_duration = time.time() - self.test_start_time if self.test_start_time else 0
        
        print(f"📅 Test Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Toplam Test Süresi: {test_duration/60:.1f} dakika")
        print(f"🌊 Maksimum Derinlik: {self.max_depth_achieved:.2f}m")
        print(f"📊 Toplam Veri Noktası: {len(self.telemetry_data)}")
        
        print(f"\n📋 TEST SONUÇLARI:")
        print("-"*50)
        
        overall_success = True
        for log_entry in self.test_log:
            stage = log_entry['stage']
            success = log_entry['success']
            status_icon = "✅" if success else "❌"
            
            stage_names = {
                'STATIC_TEST': 'Statik Sızdırmazlık',
                'DYNAMIC_TEST': 'Dinamik Sızdırmazlık', 
                'PAYLOAD_TEST': 'Kapak Mekanizması'
            }
            
            stage_name = stage_names.get(stage, stage)
            print(f"  {status_icon} {stage_name}: {'BAŞARILI' if success else 'BAŞARISIZ'}")
            
            overall_success = overall_success and success
        
        # Şartname değerlendirmesi
        print(f"\n🎯 ŞARTNAME GEREKSİNİMLERİ:")
        print("-"*40)
        
        depth_ok = self.max_depth_achieved >= MINIMUM_TEST_DEPTH
        depth_icon = "✅" if depth_ok else "❌"
        print(f"  {depth_icon} Minimum Derinlik (≥{MINIMUM_TEST_DEPTH}m): {self.max_depth_achieved:.2f}m")
        
        leak_ok = not self.leak_detected
        leak_icon = "✅" if leak_ok else "❌"
        print(f"  {leak_icon} Sızdırmazlık: {'BAŞARILI' if leak_ok else 'BAŞARISIZ'}")
        
        static_ok = any(log['stage'] == 'STATIC_TEST' and log['success'] for log in self.test_log)
        static_icon = "✅" if static_ok else "❌"
        print(f"  {static_icon} Statik Test: {'BAŞARILI' if static_ok else 'BAŞARISIZ'}")
        
        dynamic_ok = any(log['stage'] == 'DYNAMIC_TEST' and log['success'] for log in self.test_log)
        dynamic_icon = "✅" if dynamic_ok else "❌"
        print(f"  {dynamic_icon} Dinamik Test: {'BAŞARILI' if dynamic_ok else 'BAŞARISIZ'}")
        
        # Final değerlendirme
        final_success = overall_success and depth_ok and leak_ok and static_ok and dynamic_ok
        
        print(f"\n🏆 GENEL SONUÇ:")
        print("="*30)
        
        if final_success:
            print("🎉 SIZDIMAZLIK TESTİ TAM BAŞARI!")
            print("📹 Video çekimi için hazır!")
        else:
            print("❌ SIZDIMAZLIK TESTİ BAŞARISIZ!")
            print("🔧 Sistem kontrolü gerekli!")
        
        # Veri dosyasına kaydet
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'test_duration': test_duration,
            'max_depth': self.max_depth_achieved,
            'leak_detected': self.leak_detected,
            'test_results': self.test_log,
            'telemetry_data': self.telemetry_data[-100:],  # Son 100 veri noktası
            'final_success': final_success
        }
        
        with open(f'waterproof_test_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
            json.dump(report_data, f, indent=2)
        
        print(f"\n💾 Test raporu kaydedildi: waterproof_test_report_*.json")
        
        return final_success
    
    def run_full_waterproof_demo(self):
        """Tam sızdırmazlık demo testi"""
        print("🎬 TEKNOFEST Su Altı Roket Aracı - SIZDIMAZLIK TESTİ")
        print("="*70)
        print("📹 Video çekimi için kabiliyet gösterimi")
        print("⏱️ Tahmini süre: 8-10 dakika")
        print("🎯 Şartname: ≥1m derinlik, statik+dinamik test")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("\n⚠️ GÜVENLİK UYARISI:")
        print("- Su altı acil müdahale ekibi hazır mı?")
        print("- Kameralar çalışıyor mu?")
        print("- Acil yüzeye çıkış planı hazır mı?")
        
        ready = input("\n✅ Hazır mısınız? Video çekimi başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Test iptal edildi")
            return False
        
        self.test_start_time = time.time()
        self.test_active = True
        self.running = True
        
        # Monitoring thread başlat
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        try:
            # 0. Kalibrasyon
            print("\n🔧 SİSTEM KALİBRASYONU...")
            if not self.calibrate_surface_pressure():
                return False
            
            input("\n⏸️ Video çekimi hazır mı? Devam için ENTER...")
            
            # 1. Statik test
            if not self.run_static_waterproof_test():
                return False
            
            input("\n⏸️ Statik test tamam! Dinamik teste geçilsin mi? ENTER...")
            
            # 2. Dinamik test
            if not self.run_dynamic_waterproof_test():
                return False
            
            input("\n⏸️ Dinamik test tamam! Kapak testine geçilsin mi? ENTER...")
            
            # 3. Kapak mekanizması test
            if not self.run_payload_bay_test():
                return False
            
            # 4. Test raporu
            success = self.generate_test_report()
            
            if success:
                print("\n🎉 SIZDIRAZLIK TESTİ VIDEO ÇEKIMI BAŞARILI!")
                print("📹 Video montaja hazır!")
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.test_active = False
        self.running = False
        
        print("\n🧹 Sistem temizleniyor...")
        
        # Motorları durdur
        if self.connected:
            self.set_motor_throttle(PWM_NEUTRAL)
            
            # Servoları orta pozisyon
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL)
            
            # Kapağı kapat
            self.set_servo_position(PAYLOAD_SERVO, PWM_MIN)
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı")

def main():
    """Ana fonksiyon"""
    demo = WaterproofDemo()
    
    try:
        success = demo.run_full_waterproof_demo()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    import math
    sys.exit(main()) 