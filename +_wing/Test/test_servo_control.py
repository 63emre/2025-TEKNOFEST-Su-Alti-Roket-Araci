"""
TEKNOFEST 2025 Su Altı Roket Aracı
+ Wing Servo Kontrol Test Scripti

Bu script + Wing konfigürasyonu için servo kontrollerini test eder.
Fin isimleri: Üst, Alt, Sol, Sağ
"""

import os
import sys
import time
import signal

# Proje dizinini path'e ekle
project_root = os.path.join(os.path.dirname(__file__), '../..')
sys.path.append(project_root)

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController

# + Wing konfigürasyonunu import et
import importlib.util
plus_wing_path = os.path.join(project_root, '+_wing', 'hardware_pinmap.py')
spec = importlib.util.spec_from_file_location("plus_wing_pinmap", plus_wing_path)
plus_wing_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plus_wing_config)

class PlusWingServoTester:
    """+ Wing servo test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.mav = None
        self.servo_controller = None
        
        # Signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Bağlantı ayarları
        self.connection_string = plus_wing_config.PixhawkConfig.MAVLINK_PORT
        self.baud_rate = plus_wing_config.PixhawkConfig.MAVLINK_BAUD
    
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
        print("+ WING SERVO KONTROL TESİ")
        print("="*60)
        
        # MAVLink bağlantısı
        self.mav = MAVLinkController(self.connection_string, self.baud_rate)
        if not self.mav.connect():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Servo kontrolcüsü
        self.servo_controller = ServoController(self.mav, plus_wing_config.FinControlConfig.FINS)
        print("✅ Servo kontrolcüsü hazırlandı")
        
        print("\n+ Wing Fin Konfigürasyonu:")
        for fin_name, config in plus_wing_config.FinControlConfig.FINS.items():
            print(f"  {config['name']} (AUX {config['aux_port']})")
        
        print("-"*60)
        return True
    
    def test_individual_servos(self):
        """Her servoyu ayrı ayrı test et"""
        print("\nTEKİL SERVO TESTLERİ")
        print("Her servo ayrı ayrı test ediliyor...")
        print("-"*60)
        
        for fin_name, config in plus_wing_config.FinControlConfig.FINS.items():
            if not self.running:
                break
                
            print(f"\n🔧 {config['name']} Test Ediliyor...")
            
            # Test dizisi: Nötr -> Min -> Nötr -> Max -> Nötr
            test_positions = [
                (plus_wing_config.PixhawkConfig.SERVO_NEUTRAL, "Nötr Pozisyon"),
                (plus_wing_config.PixhawkConfig.SERVO_MIN, "Minimum Pozisyon"),
                (plus_wing_config.PixhawkConfig.SERVO_NEUTRAL, "Nötr Pozisyon"),
                (plus_wing_config.PixhawkConfig.SERVO_MAX, "Maksimum Pozisyon"),
                (plus_wing_config.PixhawkConfig.SERVO_NEUTRAL, "Nötr Pozisyon")
            ]
            
            for pwm_value, position_desc in test_positions:
                print(f"  → {position_desc} (PWM: {pwm_value})")
                self.servo_controller.set_servo_position(fin_name, pwm_value)
                time.sleep(1.5)  # Her pozisyon için bekleme
            
            print(f"✅ {config['name']} testi tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Tekil servo testleri tamamlandı")
    
    def test_movement_commands(self):
        """+ Wing hareket komutlarını test et"""
        print("\n+ WING HAREKET KOMUTLARI TESİ")
        print("Ortogonal fin hareketleri test ediliyor...")
        print("-"*60)
        
        # Test edilecek hareket komutları
        movements_to_test = [
            ("nötr", "Tüm finler nötr"),
            ("yukarı", "Yukarı hareket (üst fin max, alt fin min)"),
            ("nötr", "Nötr pozisyon"),
            ("aşağı", "Aşağı hareket (üst fin min, alt fin max)"), 
            ("nötr", "Nötr pozisyon"),
            ("sola", "Sola hareket (sol fin max, sağ fin min)"),
            ("nötr", "Nötr pozisyon"),
            ("sağa", "Sağa hareket (sol fin min, sağ fin max)"),
            ("nötr", "Son nötr pozisyon")
        ]
        
        for movement, description in movements_to_test:
            if not self.running:
                break
                
            print(f"\n🎯 {description}")
            
            # Hareket komutunu al ve uygula
            movement_cmd = plus_wing_config.FinControlConfig.MOVEMENT_COMMANDS.get(movement)
            if movement_cmd:
                print("   Fin PWM değerleri:")
                for fin_name, pwm_value in movement_cmd.items():
                    fin_desc = plus_wing_config.FinControlConfig.FINS[fin_name]['name']
                    print(f"   • {fin_desc}: {pwm_value}")
                
                self.servo_controller.execute_movement_command(movement_cmd)
            else:
                print(f"   ⚠️ Hareket komutu bulunamadı: {movement}")
            
            time.sleep(2.5)  # Her hareket için bekleme
        
        print("\n✅ Hareket komutları testi tamamlandı")
    
    def test_axis_isolation(self):
        """Eksen izolasyon testleri (+ Wing'in avantajı)"""
        print("\nEKSEN İZOLASYON TESTLERİ")
        print("+ Wing konfigürasyonunun eksen ayrımı test ediliyor...")
        print("-"*60)
        
        isolation_tests = [
            {
                "name": "Yalnız Pitch Kontrolü",
                "description": "Sadece üst-alt finler hareket ediyor",
                "test_sequence": [
                    ("Pitch Up", {"upper": plus_wing_config.PixhawkConfig.SERVO_MAX, 
                                 "lower": plus_wing_config.PixhawkConfig.SERVO_MIN,
                                 "left": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                 "right": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL}),
                    ("Pitch Down", {"upper": plus_wing_config.PixhawkConfig.SERVO_MIN,
                                   "lower": plus_wing_config.PixhawkConfig.SERVO_MAX,
                                   "left": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                   "right": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL}),
                    ("Pitch Neutral", {"upper": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                      "lower": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                      "left": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                      "right": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL})
                ]
            },
            {
                "name": "Yalnız Roll Kontrolü",
                "description": "Sadece sol-sağ finler hareket ediyor",
                "test_sequence": [
                    ("Roll Left", {"upper": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                  "lower": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                  "left": plus_wing_config.PixhawkConfig.SERVO_MAX,
                                  "right": plus_wing_config.PixhawkConfig.SERVO_MIN}),
                    ("Roll Right", {"upper": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                   "lower": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                   "left": plus_wing_config.PixhawkConfig.SERVO_MIN,
                                   "right": plus_wing_config.PixhawkConfig.SERVO_MAX}),
                    ("Roll Neutral", {"upper": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                     "lower": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                     "left": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                                     "right": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL})
                ]
            }
        ]
        
        for test_group in isolation_tests:
            if not self.running:
                break
                
            print(f"\n🎪 {test_group['name']}")
            print(f"   {test_group['description']}")
            
            for movement_name, servo_positions in test_group['test_sequence']:
                print(f"   → {movement_name}")
                self.servo_controller.set_multiple_servos(servo_positions)
                time.sleep(2.0)
            
            print(f"   ✅ {test_group['name']} tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Eksen izolasyon testleri tamamlandı")
    
    def test_precision_control(self):
        """Hassas kontrol testleri"""
        print("\nHASSAS KONTROL TESTLERİ")
        print("+ Wing'in hassas kontrol kabiliyeti test ediliyor...")
        print("-"*60)
        
        # Hassas PWM değerleri ile test
        precision_steps = [1500, 1510, 1520, 1530, 1540, 1550, 1540, 1530, 1520, 1510, 1500,
                          1490, 1480, 1470, 1460, 1450, 1460, 1470, 1480, 1490, 1500]
        
        for fin_name, config in plus_wing_config.FinControlConfig.FINS.items():
            if not self.running:
                break
                
            print(f"\n🎯 {config['name']} Hassas Kontrol")
            print(f"   PWM değerleri: {min(precision_steps)} - {max(precision_steps)}")
            
            for i, pwm_value in enumerate(precision_steps):
                self.servo_controller.set_servo_position(fin_name, pwm_value)
                print(f"   Step {i+1:2d}: PWM {pwm_value}", end='\r')
                time.sleep(0.3)
            
            print(f"\n   ✅ {config['name']} hassas kontrol tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Hassas kontrol testleri tamamlandı")
    
    def test_plus_wing_advantages(self):
        """+ Wing avantajları demonstrasyonu"""
        print("\n+ WING AVANTAJLARI DEMONSTRASYONu")
        print("+ Wing konfigürasyonunun avantajları gösteriliyor...")
        print("-"*60)
        
        advantages_demo = [
            {
                "name": "Bağımsız Eksen Kontrolü",
                "description": "Pitch ve roll hareketleri birbirini etkilemiyor",
                "demo": [
                    ("Sadece Pitch", {"upper": 1600, "lower": 1400, "left": 1500, "right": 1500}),
                    ("Pitch + Roll", {"upper": 1600, "lower": 1400, "left": 1600, "right": 1400}),
                    ("Sadece Roll", {"upper": 1500, "lower": 1500, "left": 1600, "right": 1400}),
                    ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500})
                ]
            },
            {
                "name": "Stabilite Avantajı",
                "description": "Dört nokta kontrolü ile yüksek stabilite",
                "demo": [
                    ("Küçük Düzeltme", {"upper": 1520, "lower": 1480, "left": 1480, "right": 1520}),
                    ("Orta Düzeltme", {"upper": 1550, "lower": 1450, "left": 1450, "right": 1550}),
                    ("Büyük Düzeltme", {"upper": 1600, "lower": 1400, "left": 1400, "right": 1600}),
                    ("Geri Düzeltme", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500})
                ]
            }
        ]
        
        for demo_group in advantages_demo:
            if not self.running:
                break
                
            print(f"\n🌟 {demo_group['name']}")
            print(f"   {demo_group['description']}")
            
            for demo_name, servo_positions in demo_group['demo']:
                print(f"   → {demo_name}")
                self.servo_controller.set_multiple_servos(servo_positions)
                time.sleep(2.0)
            
            print(f"   ✅ {demo_group['name']} tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ + Wing avantajları demonstrasyonu tamamlandı")
    
    def test_response_characteristics(self):
        """+ Wing yanıt karakteristikleri"""
        print("\n+ WING YANIT KARAKTERİSTİKLERİ")
        print("Servo yanıt süreleri ve karakteristikleri ölçülüyor...")
        print("-"*60)
        
        # Yanıt süresi testi
        for fin_name, config in plus_wing_config.FinControlConfig.FINS.items():
            if not self.running:
                break
                
            print(f"\n⚡ {config['name']} Yanıt Karakteristiği")
            
            # Step yanıt testi
            step_responses = []
            for i in range(5):
                start_pwm = plus_wing_config.PixhawkConfig.SERVO_MIN if i % 2 == 0 else plus_wing_config.PixhawkConfig.SERVO_MAX
                end_pwm = plus_wing_config.PixhawkConfig.SERVO_MAX if i % 2 == 0 else plus_wing_config.PixhawkConfig.SERVO_MIN
                
                start_time = time.time()
                self.servo_controller.set_servo_position(fin_name, end_pwm)
                response_time = time.time() - start_time
                
                step_responses.append(response_time)
                print(f"   Test {i+1}: {start_pwm} → {end_pwm}, {response_time:.3f}s")
                time.sleep(0.5)
            
            avg_response = sum(step_responses) / len(step_responses)
            print(f"   Ortalama yanıt süresi: {avg_response:.3f}s")
            
            # Nötre getir
            self.servo_controller.set_servo_position(fin_name, plus_wing_config.PixhawkConfig.SERVO_NEUTRAL)
            print(f"   ✅ {config['name']} yanıt testi tamamlandı")
            time.sleep(0.5)
        
        print("\n✅ Yanıt karakteristikleri testleri tamamlandı")
    
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
            
            # Eksen izolasyon testleri (+ Wing'e özel)
            self.test_axis_isolation()
            
            if not self.running:
                return
            
            # Hassas kontrol testleri
            self.test_precision_control()
            
            if not self.running:
                return
            
            # + Wing avantajları demonstrasyonu
            self.test_plus_wing_advantages()
            
            if not self.running:
                return
            
            # Yanıt karakteristikleri
            self.test_response_characteristics()
            
            print("\n" + "="*60)
            print("🎉 + WING SERVO TESTLERİ BAŞARIYLA TAMAMLANDI!")
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
    tester = PlusWingServoTester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
