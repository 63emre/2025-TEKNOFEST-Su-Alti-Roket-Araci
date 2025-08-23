#!/usr/bin/env python3
"""
TEKNOFEST 2025 - X-Wing Stabilizasyon Test Scripti
Bu script X-Wing konfigürasyonunda stabilizasyon sistemini test eder.

Test Senaryoları:
1. Roll Stabilizasyon Testi
2. Pitch Stabilizasyon Testi  
3. Yaw Stabilizasyon Testi
4. Kombine Stabilizasyon Testi
5. PWM→Hız→Mesafe Odometri Testi

Hardware: AUX 1→Motor, AUX 3,4,5,6→Kanatlar
"""

import sys
import os
import time
import math
import threading
from datetime import datetime

# Mission kodlarından import
sys.path.append('../xwing')
try:
    from mission_1_nav import Mission1Navigator, SERVO_CHANNELS, MOTOR_CHANNEL
    from mission_1_nav import PWM_NEUTRAL, PWM_SAFE_MIN, PWM_SAFE_MAX
    # X-Wing matrix tanımı
    X_WING_MATRIX = {
        'roll':  [1, -1, -1, 1],   # Çapraz kontrol
        'pitch': [1, 1, -1, -1],   # Ön/arka kontrol
        'yaw':   [1, -1, 1, -1]    # Rotasyon kontrol
    }
    SERVO_MAX_DELTA = 300
    X_WING_AVAILABLE = True
    print("✅ X-Wing modülü yüklendi")
except ImportError as e:
    print(f"❌ X-Wing modülü yüklenemedi: {e}")
    X_WING_AVAILABLE = False

class XWingStabilizationTester:
    """X-Wing stabilizasyon test sınıfı"""
    
    def __init__(self):
        print("🚀 TEKNOFEST X-Wing Stabilizasyon Test Sistemi")
        print("="*60)
        
        # X-Wing modülü kontrolü
        if not X_WING_AVAILABLE:
            print("❌ X-Wing modülü mevcut değil!")
            return
            
        # Mission navigator'ı test modunda başlat
        self.navigator = Mission1Navigator()
        self.test_active = False
        self.test_results = {}
        
        # Test parametreleri
        self.test_duration = 30.0  # Her test 30 saniye
        self.test_amplitude = 100  # PWM test genliği
        self.test_frequency = 0.5  # Test frekansı (Hz)
        
    def connect_and_initialize(self):
        """Pixhawk bağlantısı ve başlatma"""
        print("🔌 Pixhawk'a bağlanılıyor...")
        if not self.navigator.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("✅ Pixhawk bağlantısı başarılı!")
        print(f"   Motor Kanal: {MOTOR_CHANNEL}")
        print(f"   Servo Kanalları: {SERVO_CHANNELS}")
        
        # Arming interlock'u atla (test modu)
        self.navigator._arming_done = True
        print("🔓 Test modu: Arming interlock atlandı")
        
        return True
    
    def test_roll_stabilization(self):
        """Roll ekseni stabilizasyon testi"""
        print("\n🔄 ROLL STABİLİZASYON TESTİ BAŞLIYOR")
        print("-" * 40)
        
        start_time = time.time()
        test_data = []
        
        print("📊 Test parametreleri:")
        print(f"   Süre: {self.test_duration}s")
        print(f"   Genlik: ±{self.test_amplitude} PWM")
        print(f"   Frekans: {self.test_frequency} Hz")
        
        print("\n🎯 Pixhawk'ı roll ekseninde hareket ettirin...")
        print("   Sistem kanatları stabilize etmeye çalışacak")
        
        while time.time() - start_time < self.test_duration:
            # Sensör verilerini oku
            self.navigator.read_sensors()
            
            # Sinüsoidal test sinyali (opsiyonel - manuel test için)
            elapsed = time.time() - start_time
            test_signal = self.test_amplitude * math.sin(2 * math.pi * self.test_frequency * elapsed)
            
            # Roll stabilizasyon komutları
            roll_error = self.navigator.current_roll  # Sıfır roll hedefi
            roll_cmd = -roll_error * 5.0  # Basit P kontrolü
            
            # Kanat komutları gönder
            self.navigator.set_control_surfaces(roll_cmd=roll_cmd)
            
            # Test verisi kaydet
            test_data.append({
                'time': elapsed,
                'roll': self.navigator.current_roll,
                'roll_cmd': roll_cmd,
                'test_signal': test_signal
            })
            
            # Progress göster
            if int(elapsed) % 5 == 0 and elapsed > 0:
                remaining = self.test_duration - elapsed
                print(f"⏱️ Roll testi: {elapsed:.0f}s / {self.test_duration:.0f}s (kalan: {remaining:.0f}s)")
                print(f"   Mevcut Roll: {self.navigator.current_roll:+.1f}°, Komut: {roll_cmd:+.1f}")
            
            time.sleep(0.1)  # 10Hz
        
        # Test sonuçlarını analiz et
        avg_roll_error = sum(abs(d['roll']) for d in test_data) / len(test_data)
        max_roll_error = max(abs(d['roll']) for d in test_data)
        
        print(f"\n📊 ROLL TESTİ SONUÇLARI:")
        print(f"   Ortalama Roll Hatası: {avg_roll_error:.2f}°")
        print(f"   Maksimum Roll Hatası: {max_roll_error:.2f}°")
        print(f"   Test Süresi: {self.test_duration:.0f}s")
        print(f"   Veri Noktası: {len(test_data)}")
        
        # Başarı kriteri
        success = avg_roll_error < 5.0 and max_roll_error < 15.0
        print(f"   Sonuç: {'✅ BAŞARILI' if success else '❌ BAŞARISIZ'}")
        
        self.test_results['roll'] = {
            'success': success,
            'avg_error': avg_roll_error,
            'max_error': max_roll_error,
            'data': test_data
        }
        
        return success
    
    def test_pitch_stabilization(self):
        """Pitch ekseni stabilizasyon testi"""
        print("\n⬆️ PITCH STABİLİZASYON TESTİ BAŞLIYOR")
        print("-" * 40)
        
        start_time = time.time()
        test_data = []
        
        print("🎯 Pixhawk'ı pitch ekseninde hareket ettirin...")
        print("   Sistem kanatları stabilize etmeye çalışacak")
        
        while time.time() - start_time < self.test_duration:
            # Sensör verilerini oku
            self.navigator.read_sensors()
            
            elapsed = time.time() - start_time
            
            # Pitch stabilizasyon komutları
            pitch_error = self.navigator.current_pitch  # Sıfır pitch hedefi
            pitch_cmd = -pitch_error * 5.0  # Basit P kontrolü
            
            # Kanat komutları gönder
            self.navigator.set_control_surfaces(pitch_cmd=pitch_cmd)
            
            # Test verisi kaydet
            test_data.append({
                'time': elapsed,
                'pitch': self.navigator.current_pitch,
                'pitch_cmd': pitch_cmd
            })
            
            # Progress göster
            if int(elapsed) % 5 == 0 and elapsed > 0:
                remaining = self.test_duration - elapsed
                print(f"⏱️ Pitch testi: {elapsed:.0f}s / {self.test_duration:.0f}s (kalan: {remaining:.0f}s)")
                print(f"   Mevcut Pitch: {self.navigator.current_pitch:+.1f}°, Komut: {pitch_cmd:+.1f}")
            
            time.sleep(0.1)  # 10Hz
        
        # Test sonuçlarını analiz et
        avg_pitch_error = sum(abs(d['pitch']) for d in test_data) / len(test_data)
        max_pitch_error = max(abs(d['pitch']) for d in test_data)
        
        print(f"\n📊 PITCH TESTİ SONUÇLARI:")
        print(f"   Ortalama Pitch Hatası: {avg_pitch_error:.2f}°")
        print(f"   Maksimum Pitch Hatası: {max_pitch_error:.2f}°")
        
        success = avg_pitch_error < 5.0 and max_pitch_error < 15.0
        print(f"   Sonuç: {'✅ BAŞARILI' if success else '❌ BAŞARISIZ'}")
        
        self.test_results['pitch'] = {
            'success': success,
            'avg_error': avg_pitch_error,
            'max_error': max_pitch_error,
            'data': test_data
        }
        
        return success
    
    def test_yaw_stabilization(self):
        """Yaw ekseni stabilizasyon testi"""
        print("\n🧭 YAW STABİLİZASYON TESTİ BAŞLIYOR")
        print("-" * 40)
        
        # İlk heading'i referans al
        self.navigator.read_sensors()
        target_heading = self.navigator.current_heading
        
        start_time = time.time()
        test_data = []
        
        print(f"🎯 Hedef Heading: {target_heading:.1f}°")
        print("   Pixhawk'ı yaw ekseninde hareket ettirin...")
        
        while time.time() - start_time < self.test_duration:
            # Sensör verilerini oku
            self.navigator.read_sensors()
            
            elapsed = time.time() - start_time
            
            # Yaw stabilizasyon komutları
            heading_error = target_heading - self.navigator.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            yaw_cmd = heading_error * 3.0  # P kontrolü
            
            # Kanat komutları gönder
            self.navigator.set_control_surfaces(yaw_cmd=yaw_cmd)
            
            # Test verisi kaydet
            test_data.append({
                'time': elapsed,
                'heading': self.navigator.current_heading,
                'heading_error': heading_error,
                'yaw_cmd': yaw_cmd
            })
            
            # Progress göster
            if int(elapsed) % 5 == 0 and elapsed > 0:
                remaining = self.test_duration - elapsed
                print(f"⏱️ Yaw testi: {elapsed:.0f}s / {self.test_duration:.0f}s (kalan: {remaining:.0f}s)")
                print(f"   Heading: {self.navigator.current_heading:.1f}°, Hata: {heading_error:+.1f}°, Komut: {yaw_cmd:+.1f}")
            
            time.sleep(0.1)  # 10Hz
        
        # Test sonuçlarını analiz et
        avg_heading_error = sum(abs(d['heading_error']) for d in test_data) / len(test_data)
        max_heading_error = max(abs(d['heading_error']) for d in test_data)
        
        print(f"\n📊 YAW TESTİ SONUÇLARI:")
        print(f"   Ortalama Heading Hatası: {avg_heading_error:.2f}°")
        print(f"   Maksimum Heading Hatası: {max_heading_error:.2f}°")
        
        success = avg_heading_error < 10.0 and max_heading_error < 30.0
        print(f"   Sonuç: {'✅ BAŞARILI' if success else '❌ BAŞARISIZ'}")
        
        self.test_results['yaw'] = {
            'success': success,
            'avg_error': avg_heading_error,
            'max_error': max_heading_error,
            'data': test_data
        }
        
        return success
    
    def test_distance_odometry(self):
        """PWM→Hız→Mesafe odometri testi"""
        print("\n📏 MESAFE ODOMETRİ TESTİ BAŞLIYOR")
        print("-" * 40)
        
        # Test parametreleri
        test_pwm_values = [1550, 1600, 1650, 1700]  # Farklı PWM değerleri
        test_duration_per_pwm = 10.0  # Her PWM için 10 saniye
        
        start_time = time.time()
        test_data = []
        
        print("🚁 Motor simülasyonu: Farklı PWM değerlerinde mesafe hesaplama")
        print("   (Motor çalışmayacak, sadece odometri test edilecek)")
        
        for pwm_value in test_pwm_values:
            print(f"\n🔧 PWM Testi: {pwm_value} ({pwm_value-PWM_NEUTRAL:+d} delta)")
            
            # PWM'i set et (arming interlock atlandığı için çalışacak)
            self.navigator.current_pwm = pwm_value
            
            pwm_start_time = time.time()
            initial_distance = self.navigator.traveled_distance
            
            while time.time() - pwm_start_time < test_duration_per_pwm:
                # Odometri güncellemesi
                self.navigator.update_pwm_based_odometry()
                
                elapsed = time.time() - pwm_start_time
                distance_increment = self.navigator.traveled_distance - initial_distance
                
                # Test verisi kaydet
                test_data.append({
                    'pwm': pwm_value,
                    'time': elapsed,
                    'estimated_speed': self.navigator.estimated_speed,
                    'filtered_speed': self.navigator.filtered_speed,
                    'distance_increment': distance_increment
                })
                
                time.sleep(0.1)  # 10Hz
            
            final_distance = self.navigator.traveled_distance - initial_distance
            expected_distance = self.navigator.filtered_speed * test_duration_per_pwm
            
            print(f"   📊 PWM {pwm_value} sonuçları:")
            print(f"      Kestirilen hız: {self.navigator.filtered_speed:.3f} m/s")
            print(f"      Hesaplanan mesafe: {final_distance:.2f}m")
            print(f"      Beklenen mesafe: {expected_distance:.2f}m")
        
        print(f"\n📊 ODOMETRİ TESTİ SONUÇLARI:")
        print(f"   Test edilen PWM değerleri: {test_pwm_values}")
        print(f"   Toplam test süresi: {len(test_pwm_values) * test_duration_per_pwm:.0f}s")
        print(f"   Toplam mesafe: {self.navigator.traveled_distance:.2f}m")
        print(f"   Veri noktası: {len(test_data)}")
        
        success = len(test_data) > 0 and self.navigator.traveled_distance > 0
        print(f"   Sonuç: {'✅ BAŞARILI' if success else '❌ BAŞARISIZ'}")
        
        self.test_results['odometry'] = {
            'success': success,
            'total_distance': self.navigator.traveled_distance,
            'data': test_data
        }
        
        return success
    
    def run_all_tests(self):
        """Tüm testleri sırayla çalıştır"""
        print("🚀 TÜM TESTLER BAŞLIYOR...")
        print("="*60)
        
        if not self.connect_and_initialize():
            return False
        
        self.test_active = True
        
        try:
            # Test sırası
            tests = [
                ("Roll Stabilizasyon", self.test_roll_stabilization),
                ("Pitch Stabilizasyon", self.test_pitch_stabilization),
                ("Yaw Stabilizasyon", self.test_yaw_stabilization),
                ("Mesafe Odometri", self.test_distance_odometry)
            ]
            
            results = []
            
            for test_name, test_func in tests:
                print(f"\n{'='*60}")
                print(f"🎯 {test_name.upper()} TESTİ")
                print(f"{'='*60}")
                
                # Test öncesi kanatları nötr yap
                self.navigator.set_control_surfaces(0, 0, 0)
                time.sleep(2.0)
                
                # Testi çalıştır
                success = test_func()
                results.append((test_name, success))
                
                # Test sonrası kanatları nötr yap
                self.navigator.set_control_surfaces(0, 0, 0)
                time.sleep(1.0)
                
                if not success:
                    print(f"⚠️ {test_name} testi başarısız!")
            
            # Genel sonuçlar
            self.print_final_results(results)
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
        finally:
            self.cleanup()
    
    def print_final_results(self, results):
        """Final sonuçları yazdır"""
        print("\n" + "="*60)
        print("📊 X-WING STABİLİZASYON TEST SONUÇLARI")
        print("="*60)
        
        passed = 0
        total = len(results)
        
        for test_name, success in results:
            status = "✅ BAŞARILI" if success else "❌ BAŞARISIZ"
            print(f"   {test_name:<25}: {status}")
            if success:
                passed += 1
        
        print(f"\n📈 GENEL SONUÇ: {passed}/{total} test başarılı")
        
        if passed == total:
            print("🎉 TÜM TESTLER BAŞARILI! X-Wing stabilizasyon sistemi çalışıyor.")
        elif passed >= total // 2:
            print("⚠️ KISMEN BAŞARILI. Bazı sistemler düzeltme gerektirebilir.")
        else:
            print("❌ TESTLER BAŞARISIZ. Sistem kontrolü gerekli.")
        
        # Test raporunu kaydet
        self.save_test_report(results)
    
    def save_test_report(self, results):
        """Test raporunu dosyaya kaydet"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"x_wing_test_report_{timestamp}.json"
        
        import json
        report = {
            'timestamp': datetime.now().isoformat(),
            'test_type': 'X-Wing Stabilization Test',
            'results': results,
            'detailed_results': self.test_results
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"\n💾 Test raporu kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Rapor kayıt hatası: {e}")
    
    def cleanup(self):
        """Temizlik işlemleri"""
        print("\n🧹 Test sistemi temizleniyor...")
        
        self.test_active = False
        
        # Kanatları nötr yap
        if self.navigator.connected:
            self.navigator.set_control_surfaces(0, 0, 0)
            self.navigator.set_motor_throttle_pwm(PWM_NEUTRAL)
        
        # Navigator'ı temizle
        self.navigator.cleanup()
        
        print("✅ Test sistemi temizlendi")

def main():
    """Ana test fonksiyonu"""
    print("🚀 TEKNOFEST X-Wing Stabilizasyon Test Sistemi")
    print("Bu test Pixhawk'ı manuel hareket ettirerek stabilizasyon tepkilerini ölçer")
    
    tester = XWingStabilizationTester()
    
    try:
        ready = input("\n✅ X-Wing stabilizasyon testine hazır mısınız? (y/n): ").lower()
        if ready != 'y':
            print("❌ Test iptal edildi")
            return 1
        
        tester.run_all_tests()
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
        tester.cleanup()
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
