"""
TEKNOFEST 2025 Su Altı Roket Aracı
X Wing Servo Kontrol Test Scripti

Bu script X Wing konfigürasyonu için servo kontrollerini test eder.
Fin isimleri: Üst Sağ, Üst Sol, Alt Sol, Alt Sağ
"""

import os
import sys
import time
import signal

# Proje dizinini path'e ekle
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from x_wing.hardware_pinmap import FinControlConfig, PixhawkConfig

class XWingServoTester:
    """X Wing servo test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.mav = None
        self.servo_controller = None
        
        # Signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Bağlantı ayarları
        self.connection_string = PixhawkConfig.MAVLINK_PORT
        self.baud_rate = PixhawkConfig.MAVLINK_BAUD
    
    def signal_handler(self, sig, frame):
        """Ctrl+C ile güvenli çıkış"""
        print("\n\nTest durduruluyor...")
        self.running = False
        if self.servo_controller:
            self.servo_controller.emergency_stop()
        if self.mav:
            self.mav.disconnect()
        sys.exit(0)
    
    def setup_connections(self):
        """MAVLink ve servo kontrolcüsünü ayarla"""
        print("="*60)
        print("X WING SERVO KONTROL TESİ")
        print("="*60)
        
        # MAVLink bağlantısı
        self.mav = MAVLinkController(self.connection_string, self.baud_rate)
        if not self.mav.connect():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Servo kontrolcüsü
        self.servo_controller = ServoController(self.mav, FinControlConfig.FINS)
        print("✅ Servo kontrolcüsü hazırlandı")
        
        print("\nX Wing Fin Konfigürasyonu:")
        for fin_name, config in FinControlConfig.FINS.items():
            print(f"  {config['name']} (AUX {config['aux_port']})")
        
        print("-"*60)
        return True
    
    def test_individual_servos(self):
        """Her servoyu ayrı ayrı test et"""
        print("\nTEKİL SERVO TESTLERİ")
        print("Her servo ayrı ayrı test ediliyor...")
        print("-"*60)
        
        for fin_name, config in FinControlConfig.FINS.items():
            if not self.running:
                break
                
            print(f"\n🔧 {config['name']} Test Ediliyor...")
            
            # Test dizisi: Nötr -> Min -> Nötr -> Max -> Nötr
            test_positions = [
                (PixhawkConfig.SERVO_NEUTRAL, "Nötr Pozisyon"),
                (PixhawkConfig.SERVO_MIN, "Minimum Pozisyon"),
                (PixhawkConfig.SERVO_NEUTRAL, "Nötr Pozisyon"),
                (PixhawkConfig.SERVO_MAX, "Maksimum Pozisyon"),
                (PixhawkConfig.SERVO_NEUTRAL, "Nötr Pozisyon")
            ]
            
            for pwm_value, position_desc in test_positions:
                print(f"  → {position_desc} (PWM: {pwm_value})")
                self.servo_controller.set_servo_position(fin_name, pwm_value)
                time.sleep(1.5)  # Her pozisyon için bekleme
            
            print(f"✅ {config['name']} testi tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Tekil servo testleri tamamlandı")
    
    def test_movement_commands(self):
        """X Wing hareket komutlarını test et"""
        print("\nX WING HAREKET KOMUTLARI TESİ")
        print("Çapraz fin hareketleri test ediliyor...")
        print("-"*60)
        
        # Test edilecek hareket komutları
        movements_to_test = [
            ("nötr", "Tüm finler nötr"),
            ("yukarı", "Yukarı hareket (üst finler max, alt finler min)"),
            ("nötr", "Nötr pozisyon"),
            ("aşağı", "Aşağı hareket (üst finler min, alt finler max)"), 
            ("nötr", "Nötr pozisyon"),
            ("sola", "Sola hareket (çapraz kontrol)"),
            ("nötr", "Nötr pozisyon"),
            ("sağa", "Sağa hareket (çapraz kontrol)"),
            ("nötr", "Nötr pozisyon"),
            ("roll_sağ", "Sağa roll hareketi"),
            ("nötr", "Nötr pozisyon"),
            ("roll_sol", "Sola roll hareketi"),
            ("nötr", "Nötr pozisyon")
        ]
        
        for movement, description in movements_to_test:
            if not self.running:
                break
                
            print(f"\n🎯 {description}")
            
            # Hareket komutunu al ve uygula
            movement_cmd = FinControlConfig.MOVEMENT_COMMANDS.get(movement)
            if movement_cmd:
                print("   Fin PWM değerleri:")
                for fin_name, pwm_value in movement_cmd.items():
                    fin_desc = FinControlConfig.FINS[fin_name]['name']
                    print(f"   • {fin_desc}: {pwm_value}")
                
                self.servo_controller.execute_movement_command(movement_cmd)
            else:
                print(f"   ⚠️ Hareket komutu bulunamadı: {movement}")
            
            time.sleep(2.5)  # Her hareket için bekleme
        
        print("\n✅ Hareket komutları testi tamamlandı")
    
    def test_smooth_transitions(self):
        """Yumuşak geçiş testleri"""
        print("\nYUMUŞAK GEÇİŞ TESTLERİ")
        print("Servolar yumuşak hareket ile test ediliyor...")
        print("-"*60)
        
        # Her servoyu yumuşak hareket ile test et
        for fin_name, config in FinControlConfig.FINS.items():
            if not self.running:
                break
                
            print(f"\n🔄 {config['name']} Yumuşak Hareket Testi")
            
            # Nötrden max'a yumuşak hareket
            print("   Nötr → Maksimum (3 saniye)")
            self.servo_controller.move_servo_smooth(
                fin_name, PixhawkConfig.SERVO_MAX, 3.0, 20
            )
            
            time.sleep(0.5)
            
            # Max'tan min'a yumuşak hareket
            print("   Maksimum → Minimum (3 saniye)")
            self.servo_controller.move_servo_smooth(
                fin_name, PixhawkConfig.SERVO_MIN, 3.0, 20
            )
            
            time.sleep(0.5)
            
            # Min'dan nötre yumuşak hareket
            print("   Minimum → Nötr (2 saniye)")
            self.servo_controller.move_servo_smooth(
                fin_name, PixhawkConfig.SERVO_NEUTRAL, 2.0, 15
            )
            
            print(f"   ✅ {config['name']} yumuşak hareket tamamlandı")
            time.sleep(1)
        
        print("\n✅ Yumuşak geçiş testleri tamamlandı")
    
    def test_coordinated_movements(self):
        """Koordineli hareket testleri"""
        print("\nKOORDİNELİ HAREKET TESTLERİ")
        print("Kombinasyonlu hareketler test ediliyor...")
        print("-"*60)
        
        # Koordineli hareket senaryoları
        coordinated_tests = [
            {
                "name": "Yukarı + Sağa Roll",
                "description": "Yukarı hareket ile sağa roll kombinasyonu",
                "commands": [
                    ("yukarı", 1.0),
                    ("roll_sağ", 1.5),
                    ("nötr", 1.0)
                ]
            },
            {
                "name": "Aşağı + Sola Roll", 
                "description": "Aşağı hareket ile sola roll kombinasyonu",
                "commands": [
                    ("aşağı", 1.0),
                    ("roll_sol", 1.5),
                    ("nötr", 1.0)
                ]
            },
            {
                "name": "Sola + Yukarı",
                "description": "Sola ve yukarı hareket kombinasyonu",
                "commands": [
                    ("sola", 1.0),
                    ("yukarı", 1.0),
                    ("nötr", 1.0)
                ]
            }
        ]
        
        for test_scenario in coordinated_tests:
            if not self.running:
                break
                
            print(f"\n🎪 {test_scenario['name']}")
            print(f"   {test_scenario['description']}")
            
            for command, duration in test_scenario['commands']:
                movement_cmd = FinControlConfig.MOVEMENT_COMMANDS.get(command)
                if movement_cmd:
                    print(f"   → {command} ({duration}s)")
                    self.servo_controller.execute_movement_command(movement_cmd)
                    time.sleep(duration)
                
            print(f"   ✅ {test_scenario['name']} tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Koordineli hareket testleri tamamlandı")
    
    def test_response_speed(self):
        """Servo yanıt hızı testi"""
        print("\nSERVO YANIT HIZI TESİ")
        print("Servo yanıt süreleri ölçülüyor...")
        print("-"*60)
        
        # Hızlı değişim testi
        print("Hızlı pozisyon değişiklikleri...")
        
        for fin_name, config in FinControlConfig.FINS.items():
            if not self.running:
                break
                
            print(f"\n⚡ {config['name']} Hız Testi")
            
            # Hızlı pozisyon değişiklikleri (10 kez)
            for i in range(10):
                # Min ve max arasında hızlı değişim
                target_pwm = PixhawkConfig.SERVO_MAX if i % 2 == 0 else PixhawkConfig.SERVO_MIN
                position_name = "MAX" if i % 2 == 0 else "MIN"
                
                start_time = time.time()
                self.servo_controller.set_servo_position(fin_name, target_pwm)
                
                print(f"   {i+1:2d}. {position_name} → {time.time() - start_time:.3f}s")
                time.sleep(0.2)  # Kısa bekleme
            
            # Nötre getir
            self.servo_controller.set_servo_position(fin_name, PixhawkConfig.SERVO_NEUTRAL)
            print(f"   ✅ {config['name']} hız testi tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Servo yanıt hızı testleri tamamlandı")
    
    def test_stress_test(self):
        """Stress testi"""
        print("\nSTRESS TESİ")
        print("Servolar 2 dakika boyunca sürekli hareket edecek...")
        
        confirm = input("Stress testi başlatılsın mı? (e/h): ").lower()
        if confirm != 'e':
            print("Stress testi iptal edildi")
            return
        
        print("-"*60)
        
        # 2 dakika stress testi
        start_time = time.time()
        test_duration = 120  # 2 dakika
        
        movement_cycle = ["yukarı", "sağa", "aşağı", "sola", "roll_sağ", "roll_sol", "nötr"]
        cycle_index = 0
        
        while self.running and (time.time() - start_time) < test_duration:
            remaining = test_duration - (time.time() - start_time)
            
            # Hareket komutunu uygula
            movement = movement_cycle[cycle_index % len(movement_cycle)]
            movement_cmd = FinControlConfig.MOVEMENT_COMMANDS.get(movement)
            
            if movement_cmd:
                self.servo_controller.execute_movement_command(movement_cmd)
                print(f"⏱️ {remaining:5.1f}s kaldı | Hareket: {movement}", end='\r')
            
            cycle_index += 1
            time.sleep(0.5)  # Her 0.5 saniyede hareket değiştir
        
        # Test sonunda nötre getir
        self.servo_controller.all_servos_neutral()
        print(f"\n✅ Stress testi tamamlandı - {test_duration} saniye")
    
    def run_full_test(self):
        """Tam test senaryosu"""
        try:
            # Bağlantıları ayarla
            if not self.setup_connections():
                return
            
            # Tekil servo testleri
            self.test_individual_servos()
            
            if not self.running:
                return
            
            # Hareket komutları testi
            self.test_movement_commands()
            
            if not self.running:
                return
            
            # Yumuşak geçiş testleri
            self.test_smooth_transitions()
            
            if not self.running:
                return
            
            # Koordineli hareket testleri
            self.test_coordinated_movements()
            
            if not self.running:
                return
            
            # Yanıt hızı testi
            self.test_response_speed()
            
            if not self.running:
                return
            
            # İsteğe bağlı stress testi
            stress_test = input("\nStress testi yapılsın mı? (e/h): ").lower()
            if stress_test == 'e' and self.running:
                self.test_stress_test()
            
            print("\n" + "="*60)
            print("🎉 X WING SERVO TESTLERİ BAŞARIYLA TAMAMLANDI!")
            print("="*60)
            
        except Exception as e:
            print(f"\n❌ TEST HATASI: {e}")
            
        finally:
            # Güvenlik: Tüm servolar nötr
            if self.servo_controller:
                self.servo_controller.emergency_stop()
            if self.mav:
                self.mav.disconnect()

def main():
    """Ana test fonksiyonu"""
    tester = XWingServoTester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
