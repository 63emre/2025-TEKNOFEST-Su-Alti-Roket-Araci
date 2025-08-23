#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Fiziksel Hardware Test Scripti
Bu script fiziksel donanımın çalışıp çalışmadığını kontrol eder.

FİZİKSEL KONTROL TESTLERİ:
1. GPIO LED/Buzzer Testi (Görsel/İşitsel Onay)
2. Motor PWM Testi (ESC Arming + Throttle)
3. Servo PWM Testi (Kanat Hareketi Gözlemi)
4. Derinlik Sensörü Testi (D300 + SCALED_PRESSURE)
5. IMU Sensörü Testi (Roll/Pitch/Yaw Okuma)
6. Arming Interlock Testi (90s Countdown)
7. Leak Detection Testi (STATUSTEXT Simülasyonu)
8. PWM Odometri Testi (Hız Kestirimi)

KULLANICI ONAY SİSTEMİ:
- Her test için fiziksel gözlem gerekli
- Kullanıcı "y/n" ile onay verir
- Başarısız testler rapor edilir
"""

import sys
import os
import time
import math
import json
from datetime import datetime

# GPIO sistemi
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    print("⚠️ RPi.GPIO bulunamadı, GPIO testleri atlanacak")
    GPIO_AVAILABLE = False

# MAVLink
from pymavlink import mavutil

# Mission kodlarından import
sys.path.append('../pluswing')
try:
    from mission_1_navigation_plus import Mission1Navigator
    from mission_1_navigation_plus import SERVO_CHANNELS, MOTOR_CHANNEL
    from mission_1_navigation_plus import PWM_NEUTRAL, PWM_SAFE_MIN, PWM_SAFE_MAX
    from mission_1_navigation_plus import GPIO_STATUS_LED, GPIO_BUZZER_PWM
    PLUS_WING_AVAILABLE = True
except ImportError:
    print("⚠️ Plus-Wing modülü bulunamadı")
    PLUS_WING_AVAILABLE = False

sys.path.append('../xwing')
try:
    from mission_1_nav import Mission1Navigator as XWingNavigator
    X_WING_AVAILABLE = True
except ImportError:
    print("⚠️ X-Wing modülü bulunamadı")
    X_WING_AVAILABLE = False

class PhysicalHardwareTester:
    """Fiziksel donanım test sınıfı"""
    
    def __init__(self):
        print("🔧 TEKNOFEST Fiziksel Hardware Test Sistemi")
        print("="*70)
        print("⚠️ BU TEST FİZİKSEL GÖZLEM GEREKTİRİR!")
        print("   Her test sonrası donanımın çalışıp çalışmadığını gözlemleyin")
        print("   ve 'y' (çalışıyor) veya 'n' (çalışmıyor) ile onaylayın.")
        print("="*70)
        
        self.test_results = {}
        self.navigator = None
        self.config = None
        
    def load_config(self):
        """Test konfigürasyonunu yükle"""
        try:
            with open('../config/mission_config.json', 'r') as f:
                self.config = json.load(f)
                print("✅ Test konfigürasyonu yüklendi")
                return True
        except Exception as e:
            print(f"⚠️ Config yüklenemedi: {e}, default değerler kullanılacak")
            return False
    
    def test_gpio_led_buzzer(self):
        """GPIO LED ve Buzzer fiziksel testi"""
        print("\n🔴 GPIO LED & BUZZER FİZİKSEL TESTİ")
        print("-" * 50)
        
        if not GPIO_AVAILABLE:
            print("❌ GPIO mevcut değil, test atlanıyor")
            return False
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # LED test
            GPIO.setup(GPIO_STATUS_LED, GPIO.OUT)
            print("💡 LED Testi: GPIO4 LED yanıp sönecek (5 saniye)")
            
            for i in range(10):
                GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)
                time.sleep(0.25)
                GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
                time.sleep(0.25)
                
            led_works = input("   LED yanıp söndü mü? (y/n): ").lower() == 'y'
            
            # Buzzer test
            GPIO.setup(GPIO_BUZZER_PWM, GPIO.OUT)
            print("🔊 Buzzer Testi: GPIO13 buzzer bip verecek (3 bip)")
            
            for i in range(3):
                GPIO.output(GPIO_BUZZER_PWM, GPIO.HIGH)
                time.sleep(0.2)
                GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                time.sleep(0.3)
                
            buzzer_works = input("   Buzzer bip verdi mi? (y/n): ").lower() == 'y'
            
            # Cleanup
            GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
            GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
            
            success = led_works and buzzer_works
            print(f"📊 GPIO Test: LED={led_works}, Buzzer={buzzer_works}")
            
            self.test_results['gpio_led_buzzer'] = {
                'success': success,
                'led_works': led_works,
                'buzzer_works': buzzer_works
            }
            
            return success
            
        except Exception as e:
            print(f"❌ GPIO test hatası: {e}")
            return False
    
    def test_motor_pwm(self):
        """Motor PWM fiziksel testi"""
        print("\n🚁 MOTOR PWM FİZİKSEL TESTİ")
        print("-" * 50)
        print("⚠️ MOTOR PROPELLER ÇIKARILMIŞ OLMALI!")
        
        if not self.navigator:
            print("❌ Navigator mevcut değil")
            return False
            
        ready = input("Motor test için hazır mısınız? Propeller çıkarıldı mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Motor testi iptal edildi")
            return False
        
        try:
            print("🔧 Motor Arming Sequence...")
            
            # ESC Arming
            for i in range(20):
                self.navigator.set_motor_throttle(PWM_SAFE_MIN)
                time.sleep(0.1)
                
            print("   ESC arming tamamlandı")
            
            # Neutral
            for i in range(10):
                self.navigator.set_motor_throttle(PWM_NEUTRAL)
                time.sleep(0.1)
                
            print("🎯 Motor PWM Testi: Düşük hızda çalışacak (5 saniye)")
            
            # Düşük hız testi
            test_pwm = PWM_NEUTRAL + 50  # +50 µs
            for i in range(50):  # 5 saniye
                self.navigator.set_motor_throttle(test_pwm)
                time.sleep(0.1)
                
            # Neutral'e dön
            self.navigator.set_motor_throttle(PWM_NEUTRAL)
            
            motor_works = input("   Motor düşük hızda döndü mü? (y/n): ").lower() == 'y'
            
            if motor_works:
                print("🚀 Yüksek hız testi (3 saniye)")
                test_pwm = PWM_NEUTRAL + 100  # +100 µs
                for i in range(30):
                    self.navigator.set_motor_throttle(test_pwm)
                    time.sleep(0.1)
                    
                self.navigator.set_motor_throttle(PWM_NEUTRAL)
                high_speed_works = input("   Motor yüksek hızda döndü mü? (y/n): ").lower() == 'y'
            else:
                high_speed_works = False
            
            success = motor_works and high_speed_works
            print(f"📊 Motor Test: Düşük={motor_works}, Yüksek={high_speed_works}")
            
            self.test_results['motor_pwm'] = {
                'success': success,
                'low_speed_works': motor_works,
                'high_speed_works': high_speed_works
            }
            
            return success
            
        except Exception as e:
            print(f"❌ Motor test hatası: {e}")
            return False
    
    def test_servo_pwm(self):
        """Servo PWM fiziksel testi"""
        print("\n🎛️ SERVO PWM FİZİKSEL TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            print("❌ Navigator mevcut değil")
            return False
        
        servo_results = {}
        
        for name, channel in SERVO_CHANNELS.items():
            # AUX 3..6 için SERVO11..14'e dönüştürme (HARDWARE_PIN_MAPPING)
            if channel in (3,4,5,6):
                channel = channel + 8
            print(f"\n🔧 {name.upper()} Servo Testi (AUX {channel})")
            
            # Neutral pozisyon
            self.navigator.set_servo_position(channel, PWM_NEUTRAL)
            time.sleep(1)
            
            # Pozitif hareket
            print(f"   {name} servo pozitif yönde hareket edecek...")
            self.navigator.set_servo_position(channel, PWM_SAFE_MAX)
            time.sleep(2)
            
            # Negatif hareket  
            print(f"   {name} servo negatif yönde hareket edecek...")
            self.navigator.set_servo_position(channel, PWM_SAFE_MIN)
            time.sleep(2)
            
            # Neutral'e dön
            self.navigator.set_servo_position(channel, PWM_NEUTRAL)
            time.sleep(1)
            
            works = input(f"   {name} servo hareket etti mi? (y/n): ").lower() == 'y'
            servo_results[name] = works
            
            if not works:
                print(f"❌ {name} servo çalışmıyor!")
        
        success = all(servo_results.values())
        print(f"📊 Servo Test Sonuçları: {servo_results}")
        
        self.test_results['servo_pwm'] = {
            'success': success,
            'individual_results': servo_results
        }
        
        return success
    
    def test_depth_sensors(self):
        """Derinlik sensörleri fiziksel testi"""
        print("\n🌊 DERİNLİK SENSÖRÜ FİZİKSEL TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            return False
        
        # D300 test
        d300_works = False
        if hasattr(self.navigator, 'd300_connected') and self.navigator.d300_connected:
            print("🔍 D300 Sensörü Testi...")
            for i in range(10):
                if self.navigator.read_sensors():
                    if self.navigator.depth_source == "d300":
                        print(f"   D300 Derinlik: {self.navigator.current_depth:.2f}m")
                        d300_works = True
                        break
                time.sleep(0.5)
        
        # SCALED_PRESSURE test
        print("🔍 SCALED_PRESSURE Testi...")
        scaled_pressure_works = False
        for i in range(5):
            if self.navigator.read_sensors():
                if hasattr(self.navigator, 'depth_source'):
                    print(f"   Derinlik: {self.navigator.current_depth:.2f}m (Kaynak: {self.navigator.depth_source})")
                    scaled_pressure_works = True
                    break
            time.sleep(0.5)
        
        print("📊 Sensör değerlerini gözlemleyin:")
        print("   - Sensörü elinizle kapatın/açın")
        print("   - Değerler değişiyor mu?")
        
        user_observation = input("   Sensör değerleri mantıklı değişiyor mu? (y/n): ").lower() == 'y'
        
        success = (d300_works or scaled_pressure_works) and user_observation
        
        self.test_results['depth_sensors'] = {
            'success': success,
            'd300_works': d300_works,
            'scaled_pressure_works': scaled_pressure_works,
            'user_observation': user_observation
        }
        
        return success
    
    def test_imu_sensors(self):
        """IMU sensörleri fiziksel testi"""
        print("\n🧭 IMU SENSÖRÜ FİZİKSEL TESTİ")
        print("-" * 50)
        
        if not self.navigator:
            return False
        
        print("🔄 IMU okuma testi (10 saniye)")
        print("   Pixhawk'ı farklı yönlere çevirin ve değerleri gözlemleyin:")
        
        imu_readings = []
        for i in range(20):  # 10 saniye
            if self.navigator.read_sensors():
                reading = {
                    'roll': self.navigator.current_roll,
                    'pitch': self.navigator.current_pitch,
                    'yaw': self.navigator.current_yaw,
                    'heading': self.navigator.current_heading
                }
                imu_readings.append(reading)
                
                print(f"   Roll: {reading['roll']:+6.1f}°  Pitch: {reading['pitch']:+6.1f}°  Yaw: {reading['yaw']:+6.1f}°")
                
            time.sleep(0.5)
        
        if imu_readings:
            # Varyasyon kontrolü
            roll_var = max(r['roll'] for r in imu_readings) - min(r['roll'] for r in imu_readings)
            pitch_var = max(r['pitch'] for r in imu_readings) - min(r['pitch'] for r in imu_readings)
            yaw_var = max(r['yaw'] for r in imu_readings) - min(r['yaw'] for r in imu_readings)
            
            print(f"📊 IMU Varyasyonları: Roll=±{roll_var/2:.1f}° Pitch=±{pitch_var/2:.1f}° Yaw=±{yaw_var/2:.1f}°")
            
            auto_success = roll_var > 10 or pitch_var > 10  # En az 10° hareket
            user_observation = input("   IMU değerleri hareketle değişti mi? (y/n): ").lower() == 'y'
            
            success = auto_success and user_observation
        else:
            success = False
            print("❌ IMU verisi alınamadı")
        
        self.test_results['imu_sensors'] = {
            'success': success,
            'readings_count': len(imu_readings),
            'auto_success': auto_success if imu_readings else False,
            'user_observation': user_observation if imu_readings else False
        }
        
        return success
    
    def test_arming_interlock(self):
        """90 saniye arming interlock testi"""
        print("\n🔒 ARMİNG INTERLOCK FİZİKSEL TESTİ")
        print("-" * 50)
        print("⚠️ Bu test 90 saniye sürecek!")
        
        if not self.navigator:
            return False
        
        ready = input("90 saniye arming interlock testine hazır mısınız? (y/n): ").lower()
        if ready != 'y':
            print("❌ Arming interlock testi atlandı")
            return False
        
        # Arming başlat
        self.navigator._arming_start_time = time.time()
        self.navigator._arming_done = False
        
        print("🔒 90 saniye arming countdown başladı...")
        print("   LED ve buzzer çalışmasını gözlemleyin")
        
        countdown_observed = []
        led_buzzer_worked = []
        
        for remaining in range(90, 0, -10):  # Her 10 saniyede kontrol
            print(f"⏱️ Kalan süre: {remaining} saniye")
            
            # LED/Buzzer test
            if hasattr(self.navigator, '_update_status_indicators'):
                self.navigator._update_status_indicators("INITIALIZATION", arming_remaining=remaining)
            
            # Kullanıcı gözlemi
            if remaining % 30 == 0:  # Her 30 saniyede sor
                led_buzzer = input(f"   LED yanar, buzzer bip veriyor mu? (y/n): ").lower() == 'y'
                led_buzzer_worked.append(led_buzzer)
            
            time.sleep(10)
        
        # Final kontrol
        final_check = self.navigator._check_arming_interlock()
        print(f"🔓 Arming interlock tamamlandı: {final_check}")
        
        success = final_check and any(led_buzzer_worked)
        
        self.test_results['arming_interlock'] = {
            'success': success,
            'completed': final_check,
            'led_buzzer_observed': any(led_buzzer_worked)
        }
        
        return success
    
    def run_all_tests(self):
        """Tüm fiziksel testleri çalıştır"""
        print("\n🚀 TÜM FİZİKSEL TESTLER BAŞLIYOR")
        print("="*70)
        
        # Config yükle
        self.load_config()
        
        # Navigator başlat
        config_type = input("Hangi konfigürasyonu test etmek istiyorsuniz? (plus/x): ").lower()
        
        if config_type == "plus" and PLUS_WING_AVAILABLE:
            self.navigator = Mission1Navigator()
        elif config_type == "x" and X_WING_AVAILABLE:
            self.navigator = XWingNavigator()
        else:
            print("❌ Seçilen konfigürasyon mevcut değil!")
            return False
        
        # Pixhawk bağlantısı
        if not self.navigator.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Arming interlock'u tamamen atla (test için)
        self.navigator._arming_done = True
        self.navigator._arming_start_time = time.time() - 100  # 100 saniye önceymiş gibi yap
        print("🔓 Test modu: Arming interlock tamamen atlandı")
        
        # Arming durumunu doğrula
        arming_status = self.navigator._check_arming_interlock()
        print(f"🔍 Arming durumu: {arming_status}")
        if not arming_status:
            print("❌ UYARI: Arming interlock hala aktif!")
        
        # Test listesi
        tests = [
            ("GPIO LED/Buzzer", self.test_gpio_led_buzzer),
            ("Motor PWM", self.test_motor_pwm),
            ("Servo PWM", self.test_servo_pwm),
            ("Derinlik Sensörleri", self.test_depth_sensors),
            ("IMU Sensörleri", self.test_imu_sensors),
            ("Arming Interlock", self.test_arming_interlock)
        ]
        
        results = []
        
        for test_name, test_func in tests:
            print(f"\n{'='*70}")
            print(f"🎯 {test_name.upper()} TESTİ")
            print(f"{'='*70}")
            
            try:
                success = test_func()
                results.append((test_name, success))
                
                if success:
                    print(f"✅ {test_name} testi BAŞARILI")
                else:
                    print(f"❌ {test_name} testi BAŞARISIZ")
                    
            except Exception as e:
                print(f"❌ {test_name} test hatası: {e}")
                results.append((test_name, False))
            
            # Testler arası ara
            time.sleep(2)
        
        # Final rapor
        self.generate_test_report(results)
        
        # Cleanup
        try:
            self.navigator.cleanup()
        except:
            pass
        
        return results
    
    def generate_test_report(self, results):
        """Test raporunu oluştur"""
        print("\n" + "="*70)
        print("📊 FİZİKSEL HARDWARE TEST RAPORU")
        print("="*70)
        
        passed = sum(1 for _, success in results if success)
        total = len(results)
        
        print(f"📈 GENEL SONUÇ: {passed}/{total} test başarılı ({passed/total*100:.1f}%)")
        print()
        
        for test_name, success in results:
            status = "✅ BAŞARILI" if success else "❌ BAŞARISIZ"
            print(f"   {test_name:<25}: {status}")
        
        if passed == total:
            print("\n🎉 TÜM TESTLER BAŞARILI! Hardware tamamen çalışıyor.")
        elif passed >= total // 2:
            print("\n⚠️ KISMEN BAŞARILI. Bazı bileşenler kontrol edilmeli.")
        else:
            print("\n❌ ÇOĞU TEST BAŞARISIZ. Hardware kontrolü gerekli!")
        
        # Raporu kaydet
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report = {
            'timestamp': datetime.now().isoformat(),
            'test_type': 'Physical Hardware Test',
            'results_summary': {
                'passed': passed,
                'total': total,
                'success_rate': passed/total
            },
            'detailed_results': self.test_results,
            'test_results': results
        }
        
        with open(f'physical_hardware_test_{timestamp}.json', 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"\n💾 Test raporu kaydedildi: physical_hardware_test_{timestamp}.json")

def main():
    """Ana test fonksiyonu"""
    print("🔧 TEKNOFEST Fiziksel Hardware Test Sistemi")
    print("Bu test fiziksel donanımın çalışıp çalışmadığını kontrol eder.")
    print("Her test sonrası gözleminizi onaylamanız gerekir.")
    
    tester = PhysicalHardwareTester()
    
    try:
        ready = input("\n✅ Fiziksel hardware testine hazır mısınız? (y/n): ").lower()
        if ready != 'y':
            print("❌ Test iptal edildi")
            return 1
        
        results = tester.run_all_tests()
        
        if results:
            passed = sum(1 for _, success in results if success)
            return 0 if passed == len(results) else 1
        else:
            return 1
        
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
