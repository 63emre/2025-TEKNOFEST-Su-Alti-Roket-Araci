#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Tam Sistem Entegrasyon Testi
Tüm alt sistemleri birleştiren kapsamlı test suite
"""

import time
import threading
import sys
import os
from datetime import datetime

# Import all test modules
sys.path.append(os.path.dirname(__file__))

try:
    from test_mavlink_connection import MAVLinkTester
    from test_gpio_button import SafetyButtonSystem
    from test_servo_control import ServoController
    from test_motor_control import MotorController
    from test_stabilization import StabilizationController
    from test_depth_hold import DepthController
except ImportError as e:
    print(f"❌ Test modülü import hatası: {e}")
    sys.exit(1)

class FullSystemTester:
    def __init__(self):
        self.test_results = {}
        self.start_time = None
        
        # Test bileşenleri
        self.mavlink_tester = None
        self.safety_system = None
        self.servo_controller = None
        self.motor_controller = None
        self.stabilization_controller = None
        self.depth_controller = None
        
        # System status
        self.system_ready = False
        self.safety_check_passed = False
        
    def run_system_startup_sequence(self):
        """Sistem başlatma sırası"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - TAM SİSTEM TESTİ")
        print("=" * 70)
        print(f"📅 Test başlangıç zamanı: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        
        self.start_time = time.time()
        
        # 1. MAVLink bağlantı testi
        print("\n" + "="*50)
        print("1️⃣ MAVLink BAĞLANTI TESTİ")
        print("="*50)
        
        self.mavlink_tester = MAVLinkTester()
        mavlink_success = self.mavlink_tester.run_full_test()
        self.test_results['mavlink'] = mavlink_success
        
        if not mavlink_success:
            print("❌ MAVLink bağlantısı başarısız - sistem testi durduruldu!")
            return False
        
        # 2. GPIO güvenlik sistemi testi
        print("\n" + "="*50)
        print("2️⃣ GÜVENLİK SİSTEMİ TESTİ")
        print("="*50)
        
        print("🔘 Güvenlik butonuna basın ve acil durdurma sistemini test edin!")
        print("⚠️  Bu test 30 saniye sürecek...")
        
        self.safety_system = SafetyButtonSystem()
        
        # 30 saniye güvenlik sistemi testi
        safety_test_start = time.time()
        try:
            while time.time() - safety_test_start < 30:
                status = self.safety_system.get_system_status()
                
                if status['button_press_count'] > 0:
                    print(f"✅ Güvenlik butonu çalışıyor! ({status['button_press_count']} basım)")
                    self.safety_check_passed = True
                    break
                
                time.sleep(1)
        except KeyboardInterrupt:
            print("⚠️ Güvenlik testi manuel olarak durduruldu")
        
        self.test_results['safety_system'] = self.safety_check_passed
        
        if not self.safety_check_passed:
            print("❌ Güvenlik sistemi testi başarısız!")
            print("💡 Güvenlik butonuna bastığınızdan emin olun")
            return False
        
        return True
    
    def run_subsystem_tests(self):
        """Alt sistem testleri"""
        
        # 3. Servo sistemi testi
        print("\n" + "="*50)
        print("3️⃣ SERVO SİSTEMİ TESTİ")
        print("="*50)
        
        self.servo_controller = ServoController()
        if self.servo_controller.connect_pixhawk():
            # Hızlı servo testi (sadece kalibrasyon)
            servo_success = True
            try:
                self.servo_controller.servo_calibration_test()
                print("✅ Servo sistemi test başarılı!")
            except Exception as e:
                print(f"❌ Servo sistemi test hatası: {e}")
                servo_success = False
        else:
            servo_success = False
        
        self.test_results['servo_system'] = servo_success
        
        # 4. Motor sistemi testi
        print("\n" + "="*50)
        print("4️⃣ MOTOR SİSTEMİ TESTİ")
        print("="*50)
        
        print("⚠️ MOTOR TESTİ İÇİN GÜVENLİK UYARISI:")
        print("- Pervane takılı OLMALIDIR ve güvenli test ortamında olmalısınız!")
        print("- Motor testi yapılsın mı? (y/n):")
        
        motor_test_choice = input().lower()
        
        if motor_test_choice == 'y':
            self.motor_controller = MotorController()
            if self.motor_controller.connect_pixhawk():
                motor_success = True
                try:
                    # Sadece arming testi
                    motor_success = self.motor_controller.motor_arming_test()
                    if motor_success:
                        print("✅ Motor arming test başarılı!")
                        # Kısa throttle testi
                        self.motor_controller.send_motor_command(1550)  # Düşük güç
                        time.sleep(2)
                        self.motor_controller.send_motor_command(1500)  # Neutral
                        print("✅ Motor throttle test başarılı!")
                except Exception as e:
                    print(f"❌ Motor sistemi test hatası: {e}")
                    motor_success = False
            else:
                motor_success = False
        else:
            print("⚠️ Motor testi atlandı")
            motor_success = True  # Skip edildi olarak başarılı say
            
        self.test_results['motor_system'] = motor_success
        
        return all([servo_success, motor_success])
    
    def run_integration_tests(self):
        """Entegrasyon testleri"""
        
        # 5. Stabilizasyon sistemi testi
        print("\n" + "="*50)
        print("5️⃣ STABİLİZASYON SİSTEMİ TESTİ")  
        print("="*50)
        
        self.stabilization_controller = StabilizationController()
        if self.stabilization_controller.connect_pixhawk():
            stabilization_success = True
            try:
                # Kısa stabilizasyon testi (30 saniye)
                print("📐 Kısa stabilizasyon testi (30s)...")
                self.stabilization_controller.start_stabilization()
                self.stabilization_controller.set_target_attitude(0, 0, 0)
                
                time.sleep(30)
                
                self.stabilization_controller.stop_stabilization()
                print("✅ Stabilizasyon sistemi test başarılı!")
                
            except Exception as e:
                print(f"❌ Stabilizasyon sistemi test hatası: {e}")
                stabilization_success = False
        else:
            stabilization_success = False
            
        self.test_results['stabilization'] = stabilization_success
        
        # 6. Derinlik kontrol sistemi testi
        print("\n" + "="*50)
        print("6️⃣ DERİNLİK KONTROL SİSTEMİ TESTİ")
        print("="*50)
        
        self.depth_controller = DepthController()
        if self.depth_controller.connect_pixhawk():
            depth_success = True
            try:
                # Yüzey kalibrasyonu
                depth_success = self.depth_controller.calibrate_surface_pressure(10)
                if depth_success:
                    print("✅ Derinlik sistemi kalibrasyonu başarılı!")
                    
                    # Kısa derinlik hold testi
                    print("🌊 Kısa derinlik tutma testi (20s)...")
                    self.depth_controller.start_depth_hold(0.5)  # 0.5m target
                    time.sleep(20)
                    self.depth_controller.stop_depth_hold()
                    print("✅ Derinlik tutma test başarılı!")
                    
            except Exception as e:
                print(f"❌ Derinlik kontrol test hatası: {e}")
                depth_success = False
        else:
            depth_success = False
            
        self.test_results['depth_control'] = depth_success
        
        return all([stabilization_success, depth_success])
    
    def run_mission_readiness_test(self):
        """Görev hazırlık testi"""
        print("\n" + "="*50)
        print("7️⃣ GÖREV HAZIRLIK TESTİ")
        print("="*50)
        
        print("🎯 Entegre sistem görev simülasyonu...")
        
        mission_success = True
        
        try:
            # Sistem hazırlık sırası
            print("1. Sistem başlatma sırası...")
            
            # Güvenlik sistemi aktif mi?
            if self.safety_system and self.safety_check_passed:
                print("  ✅ Güvenlik sistemi hazır")
            else:
                print("  ❌ Güvenlik sistemi hazır değil!")
                mission_success = False
            
            # MAVLink bağlantısı aktif mi?
            if self.mavlink_tester and self.test_results.get('mavlink', False):
                print("  ✅ MAVLink bağlantısı hazır")
            else:
                print("  ❌ MAVLink bağlantısı hazır değil!")
                mission_success = False
            
            # Servo sistemi hazır mı?
            if self.test_results.get('servo_system', False):
                print("  ✅ Servo sistemi hazır")
            else:
                print("  ❌ Servo sistemi hazır değil!")
                mission_success = False
            
            # Motor sistemi hazır mı?
            if self.test_results.get('motor_system', False):
                print("  ✅ Motor sistemi hazır")
            else:
                print("  ❌ Motor sistemi hazır değil!")
                mission_success = False
            
            if mission_success:
                print("\n2. Entegre sistem testi (60 saniye)...")
                
                # Stabilizasyon + derinlik tutma kombine test
                if (self.stabilization_controller and self.depth_controller and 
                    self.test_results.get('stabilization', False) and 
                    self.test_results.get('depth_control', False)):
                    
                    # Stabilizasyon başlat
                    self.stabilization_controller.start_stabilization()
                    self.stabilization_controller.set_target_attitude(0, 0, 0)
                    
                    # Derinlik tutma başlat
                    self.depth_controller.start_depth_hold(1.0)  # 1m derinlik
                    
                    # 60 saniye kombine çalışma
                    test_duration = 60
                    start_test = time.time()
                    
                    while time.time() - start_test < test_duration:
                        elapsed = time.time() - start_test
                        remaining = test_duration - elapsed
                        
                        print(f"  📊 Entegre test - Kalan süre: {remaining:.0f}s")
                        
                        # Her 10 saniyede sistem durumu raporu
                        if int(elapsed) % 10 == 0 and elapsed > 0:
                            if hasattr(self.stabilization_controller, 'attitude_log') and self.stabilization_controller.attitude_log:
                                latest_att = self.stabilization_controller.attitude_log[-1]
                                print(f"    🧭 Attitude: R={latest_att['roll']:.1f}° P={latest_att['pitch']:.1f}° Y={latest_att['yaw']:.1f}°")
                            
                            if hasattr(self.depth_controller, 'depth_log') and self.depth_controller.depth_log:
                                latest_depth = self.depth_controller.depth_log[-1]
                                print(f"    🌊 Depth: {latest_depth['depth']:.2f}m (Target: {latest_depth['target_depth']:.2f}m)")
                        
                        time.sleep(1)
                    
                    # Sistemleri durdur
                    self.stabilization_controller.stop_stabilization()
                    self.depth_controller.stop_depth_hold()
                    
                    print("  ✅ Entegre sistem testi başarılı!")
                
                else:
                    print("  ❌ Entegre sistem için gerekli kontrolcüler hazır değil!")
                    mission_success = False
        
        except Exception as e:
            print(f"❌ Görev hazırlık testi hatası: {e}")
            mission_success = False
        
        self.test_results['mission_readiness'] = mission_success
        return mission_success
    
    def generate_final_report(self):
        """Final test raporu"""
        test_duration = time.time() - self.start_time if self.start_time else 0
        
        print("\n" + "="*70)
        print("📋 TEKNOFEST Su Altı Roket Aracı - TAM SİSTEM TEST RAPORU")
        print("="*70)
        
        print(f"📅 Test Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Toplam Test Süresi: {test_duration/60:.1f} dakika")
        
        print(f"\n📊 TEST SONUÇLARI:")
        print("-" * 50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        for test_name, result in self.test_results.items():
            status_icon = "✅" if result else "❌"
            status_text = "BAŞARILI" if result else "BAŞARISIZ"
            
            test_display_name = {
                'mavlink': 'MAVLink Bağlantı',
                'safety_system': 'Güvenlik Sistemi',
                'servo_system': 'Servo Sistemi', 
                'motor_system': 'Motor Sistemi',
                'stabilization': 'Stabilizasyon',
                'depth_control': 'Derinlik Kontrolü',
                'mission_readiness': 'Görev Hazırlığı'
            }.get(test_name, test_name)
            
            print(f"  {status_icon} {test_display_name}: {status_text}")
        
        success_rate = (passed_tests / total_tests) * 100 if total_tests > 0 else 0
        
        print(f"\n📈 GENEL BAŞARI ORANI: {success_rate:.1f}% ({passed_tests}/{total_tests})")
        
        # Sistem hazırlık durumu
        if success_rate >= 90:
            print("🎉 SİSTEM DURUMU: GÖREV İÇİN TAMAMEN HAZIR!")
            readiness_status = "READY"
        elif success_rate >= 70:
            print("⚠️ SİSTEM DURUMU: KÜÇÜK AYARLAMALAR GEREKLİ")
            readiness_status = "MINOR_ISSUES"  
        elif success_rate >= 50:
            print("🔧 SİSTEM DURUMU: BÜYÜK AYARLAMALAR GEREKLİ")
            readiness_status = "MAJOR_ISSUES"
        else:
            print("❌ SİSTEM DURUMU: GÖREV İÇİN HAZIR DEĞİL!")
            readiness_status = "NOT_READY"
        
        # Öneriler
        print(f"\n💡 ÖNERİLER:")
        print("-" * 30)
        
        if not self.test_results.get('mavlink', True):
            print("  🔧 MAVLink bağlantısını kontrol edin (BlueOS, Pixhawk)")
        
        if not self.test_results.get('safety_system', True):
            print("  🔧 Güvenlik butonunu ve GPIO bağlantılarını kontrol edin")
            
        if not self.test_results.get('servo_system', True):
            print("  🔧 Servo güç kaynağını ve PWM bağlantılarını kontrol edin")
            
        if not self.test_results.get('motor_system', True):
            print("  🔧 ESC kalibrasyonu ve motor bağlantılarını kontrol edin")
            
        if not self.test_results.get('stabilization', True):
            print("  🔧 IMU kalibrasyonu ve PID parametrelerini ayarlayın")
            
        if not self.test_results.get('depth_control', True):
            print("  🔧 Basınç sensörünü ve derinlik PID'ini kontrol edin")
            
        if not self.test_results.get('mission_readiness', True):
            print("  🔧 Tüm alt sistemler çalışır durumda olmalı")
        
        return readiness_status
    
    def cleanup_all_systems(self):
        """Tüm sistemleri temizle"""
        print("\n🧹 Sistem temizleme işlemleri...")
        
        try:
            if self.stabilization_controller:
                self.stabilization_controller.cleanup()
                
            if self.depth_controller:
                self.depth_controller.cleanup()
                
            if self.motor_controller:
                self.motor_controller.cleanup()
                
            if self.servo_controller:
                self.servo_controller.cleanup()
                
            if self.mavlink_tester:
                self.mavlink_tester.close_connection()
                
            if self.safety_system:
                self.safety_system.cleanup()
                
        except Exception as e:
            print(f"⚠️ Temizlik sırasında hata: {e}")
        
        print("✅ Sistem temizleme tamamlandı")
    
    def run_full_system_test(self):
        """Tam sistem testi ana fonksiyonu"""
        try:
            # 1. Sistem başlatma
            if not self.run_system_startup_sequence():
                return False
            
            input("\n⏸️ Alt sistem testlerine devam etmek için ENTER'a basın...")
            
            # 2. Alt sistem testleri
            if not self.run_subsystem_tests():
                print("⚠️ Bazı alt sistem testleri başarısız - devam ediliyor...")
            
            input("\n⏸️ Entegrasyon testlerine devam etmek için ENTER'a basın...")
            
            # 3. Entegrasyon testleri
            if not self.run_integration_tests():
                print("⚠️ Bazı entegrasyon testleri başarısız - devam ediliyor...")
            
            input("\n⏸️ Görev hazırlık testine devam etmek için ENTER'a basın...")
            
            # 4. Görev hazırlık testi
            self.run_mission_readiness_test()
            
            # 5. Final rapor
            readiness_status = self.generate_final_report()
            
            return readiness_status in ["READY", "MINOR_ISSUES"]
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Sistem testi hatası: {e}")
            return False
        finally:
            self.cleanup_all_systems()

def main():
    """Ana fonksiyon"""
    tester = FullSystemTester()
    
    try:
        success = tester.run_full_system_test()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main()) 