"""
TEKNOFEST 2025 Su Altı Roket Aracı
+ Wing - Manuel Görev 2: Servo Kalibrasyonu ve Fine-Tuning

Bu manuel görev + Wing konfigürasyon kalibrasyonu ve PID fine-tuning
için kullanılır. + Wing'in bağımsız eksen avantajları kullanılır.
"""

import os
import sys
import time
import json
import threading

# Proje modüllerini import et
project_root = os.path.join(os.path.dirname(__file__), '../..')
sys.path.append(project_root)

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.pid_controller import SubmarineStabilizer

# + Wing konfigürasyonu
import importlib.util
plus_wing_path = os.path.join(project_root, '+_wing', 'hardware_pinmap.py')
spec = importlib.util.spec_from_file_location("plus_wing_pinmap", plus_wing_path)
plus_wing_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plus_wing_config)

class PlusWingManualMission2:
    """+ Wing Manuel Görev 2 - Kalibrasyon ve Fine-Tuning"""
    
    def __init__(self):
        self.running = False
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        self.stabilizer = None
        
        # Kalibrasyon verileri
        self.calibration_data = {
            "servo_limits": {},
            "pid_values": {},
            "mixing_matrix": {},
            "plus_wing_specific": {},
            "calibration_date": None
        }
        
        # Test durumu
        self.test_running = False
        self.test_thread = None
        
        print("+ Wing Manuel Görev 2 - Servo Kalibrasyonu ve Fine-Tuning")
        print("="*65)
        print("+ Wing Avantajı: Bağımsız eksen kalibrasyonu!")
    
    def setup_systems(self):
        """+ Wing sistemleri ayarla"""
        print("+ Wing kalibrasyon sistemleri ayarlanıyor...")
        
        try:
            # MAVLink bağlantısı
            self.mav = MAVLinkController(
                plus_wing_config.PixhawkConfig.MAVLINK_PORT,
                plus_wing_config.PixhawkConfig.MAVLINK_BAUD
            )
            if not self.mav.connect():
                raise Exception("MAVLink bağlantısı başarısız")
            print("✅ MAVLink bağlantısı kuruldu")
            
            # Servo kontrolcüsü
            self.servo_controller = ServoController(self.mav, plus_wing_config.FinControlConfig.FINS)
            print("✅ + Wing servo kontrolcüsü hazır")
            
            # Stabilizasyon sistemi
            pid_configs = {
                "roll": plus_wing_config.PIDConfig.ROLL_PID,
                "pitch": plus_wing_config.PIDConfig.PITCH_PID,
                "yaw": plus_wing_config.PIDConfig.YAW_PID
            }
            
            self.stabilizer = SubmarineStabilizer(
                pid_configs,
                plus_wing_config.FinMixingConfig.MIXING_MATRIX,
                plus_wing_config.FinMixingConfig.FIN_EFFECTIVENESS
            )
            
            self.stabilizer.set_callbacks(self._servo_callback, None)
            print("✅ + Wing stabilizasyon sistemi hazır")
            
            # Kalibrasyon verilerini yükle
            self.load_calibration_data()
            
            print("✅ + Wing kalibrasyon sistemleri hazır!")
            return True
            
        except Exception as e:
            print(f"❌ Sistem ayarlama hatası: {e}")
            return False
    
    def _servo_callback(self, servo_outputs):
        """Servo çıkış callback'i"""
        for fin_name, pwm_value in servo_outputs.items():
            if fin_name in plus_wing_config.FinControlConfig.FINS:
                aux_port = plus_wing_config.FinControlConfig.FINS[fin_name]["aux_port"]
                self.mav.set_servo_pwm(aux_port, pwm_value)
    
    def load_calibration_data(self):
        """Kalibrasyon verilerini yükle"""
        calibration_file = os.path.join(os.path.dirname(__file__), "plus_wing_calibration_data.json")
        
        if os.path.exists(calibration_file):
            try:
                with open(calibration_file, 'r') as f:
                    self.calibration_data = json.load(f)
                print(f"✅ + Wing kalibrasyon verileri yüklendi: {self.calibration_data.get('calibration_date', 'Bilinmiyor')}")
            except Exception as e:
                print(f"⚠️ Kalibrasyon verileri yüklenemedi: {e}")
        else:
            print("ℹ️ Önceki + Wing kalibrasyon verisi bulunamadı")
    
    def save_calibration_data(self):
        """+ Wing kalibrasyon verilerini kaydet"""
        calibration_file = os.path.join(os.path.dirname(__file__), "plus_wing_calibration_data.json")
        
        self.calibration_data["calibration_date"] = time.strftime("%Y-%m-%d %H:%M:%S")
        
        try:
            with open(calibration_file, 'w') as f:
                json.dump(self.calibration_data, f, indent=4)
            print(f"✅ + Wing kalibrasyon verileri kaydedildi: {calibration_file}")
        except Exception as e:
            print(f"❌ Kalibrasyon verileri kaydedilemedi: {e}")
    
    def show_help(self):
        """+ Wing kalibrasyon yardım menüsü"""
        print("\n" + "="*70)
        print("+ WING KALİBRASYON - KOMUT LİSTESİ")
        print("="*70)
        print("SİSTEM:")
        print("  help, h      - Bu yardım menüsü")
        print("  quit, q      - Çıkış")
        print("  status       - Sistem durumu")
        print("  save         - Kalibrasyon verilerini kaydet")
        print("  load         - Kalibrasyon verilerini yükle")
        print()
        print("+ WING SERVO KALİBRASYON:")
        print("  servo_cal <fin>     - Belirli fin'i kalibre et")
        print("  servo_test <fin>    - Belirli fin'i test et")
        print("  servo_limits <fin>  - Fin limitlerini ayarla")
        print("  servo_neutral <fin> - Fin nötr pozisyonunu ayarla")
        print("  servo_all_test      - Tüm servolar test")
        print()
        print("+ WING FİN İSİMLERİ (Ortogonal):")
        print("  upper  - Üst fin (pitch kontrolü)")
        print("  lower  - Alt fin (pitch kontrolü)")
        print("  left   - Sol fin (roll kontrolü)")
        print("  right  - Sağ fin (roll kontrolü)")
        print()
        print("+ WING ÖZEL KALİBRASYON:")
        print("  pitch_cal       - Pitch ekseni kalibrasyonu (üst/alt)")
        print("  roll_cal        - Roll ekseni kalibrasyonu (sol/sağ)")
        print("  isolation_test  - Eksen bağımsızlığı testi")
        print("  precision_cal   - Hassasiyet kalibrasyonu")
        print()
        print("PID KALİBRASYON:")
        print("  pid_tune <axis>     - PID manuel ayarlama")
        print("  pid_auto <axis>     - PID otomatik ayarlama")
        print("  pid_test <axis>     - PID yanıt testi")
        print("  pid_show <axis>     - PID değerlerini göster")
        print("    + Wing Eksenleri: roll, pitch, yaw")
        print()
        print("MİXİNG MATRİSİ:")
        print("  mixing_show         - + Wing mixing matrisini göster")
        print("  mixing_test         - + Wing mixing testi")
        print("  mixing_calibrate    - + Wing mixing kalibrasyonu")
        print()
        print("TEST MODLARI:")
        print("  stability_test      - + Wing stabilite testi")
        print("  response_test       - + Wing yanıt süresi testi")
        print("  plus_wing_demo      - + Wing avantajları demo")
        print("="*70)
    
    def calibrate_pitch_axis(self):
        """Pitch ekseni kalibrasyonu (üst/alt finler)"""
        print("\n+ WING PITCH EKSENİ KALİBRASYONU")
        print("Üst ve alt finler kalibre ediliyor...")
        print("="*50)
        
        # Önce üst fin
        print("1. ÜST FİN KALİBRASYONU:")
        self.calibrate_servo("upper")
        
        time.sleep(1)
        
        # Sonra alt fin
        print("\n2. ALT FİN KALİBRASYONU:")
        self.calibrate_servo("lower")
        
        # Pitch ekseni testi
        print("\n3. PITCH EKSENİ TESTİ:")
        self._test_pitch_calibration()
        
        print("✅ + Wing Pitch ekseni kalibrasyonu tamamlandı!")
    
    def calibrate_roll_axis(self):
        """Roll ekseni kalibrasyonu (sol/sağ finler)"""
        print("\n+ WING ROLL EKSENİ KALİBRASYONU")
        print("Sol ve sağ finler kalibre ediliyor...")
        print("="*50)
        
        # Önce sol fin
        print("1. SOL FİN KALİBRASYONU:")
        self.calibrate_servo("left")
        
        time.sleep(1)
        
        # Sonra sağ fin
        print("\n2. SAĞ FİN KALİBRASYONU:")
        self.calibrate_servo("right")
        
        # Roll ekseni testi
        print("\n3. ROLL EKSENİ TESTİ:")
        self._test_roll_calibration()
        
        print("✅ + Wing Roll ekseni kalibrasyonu tamamlandı!")
    
    def _test_pitch_calibration(self):
        """Pitch kalibrasyonu testi"""
        print("Pitch ekseni kalibrasyon testi...")
        
        pitch_tests = [
            ("Nötr", {"upper": 1500, "lower": 1500}),
            ("Pitch Yukarı", {"upper": 1700, "lower": 1300}),
            ("Pitch Aşağı", {"upper": 1300, "lower": 1700}),
            ("Nötr", {"upper": 1500, "lower": 1500})
        ]
        
        for test_name, pitch_commands in pitch_tests:
            print(f"  → {test_name}: Üst={pitch_commands['upper']}, Alt={pitch_commands['lower']}")
            
            # Sadece pitch finlerini hareket ettir, roll finleri sabit tut
            all_commands = {
                "upper": pitch_commands["upper"],
                "lower": pitch_commands["lower"],
                "left": 1500,  # Sol fin sabit
                "right": 1500  # Sağ fin sabit
            }
            
            self.servo_controller.set_multiple_servos(all_commands)
            time.sleep(2)
    
    def _test_roll_calibration(self):
        """Roll kalibrasyonu testi"""
        print("Roll ekseni kalibrasyon testi...")
        
        roll_tests = [
            ("Nötr", {"left": 1500, "right": 1500}),
            ("Roll Sola", {"left": 1700, "right": 1300}),
            ("Roll Sağa", {"left": 1300, "right": 1700}),
            ("Nötr", {"left": 1500, "right": 1500})
        ]
        
        for test_name, roll_commands in roll_tests:
            print(f"  → {test_name}: Sol={roll_commands['left']}, Sağ={roll_commands['right']}")
            
            # Sadece roll finlerini hareket ettir, pitch finleri sabit tut
            all_commands = {
                "upper": 1500,  # Üst fin sabit
                "lower": 1500,  # Alt fin sabit
                "left": roll_commands["left"],
                "right": roll_commands["right"]
            }
            
            self.servo_controller.set_multiple_servos(all_commands)
            time.sleep(2)
    
    def test_axis_isolation(self):
        """Eksen bağımsızlığı testi"""
        print("\n+ WING EKSEN BAĞIMSIZLIĞI TESİ")
        print("Roll ve pitch eksenlerinin birbirini etkilemediği test ediliyor...")
        print("="*60)
        
        isolation_tests = [
            {
                "name": "Başlangıç - Tümü Nötr",
                "commands": {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500},
                "check": "Tüm finler nötr pozisyonda"
            },
            {
                "name": "Sadece Pitch Hareket (Roll sabit)",
                "commands": {"upper": 1600, "lower": 1400, "left": 1500, "right": 1500},
                "check": "Sol/sağ finler etkilenmiyor mu?"
            },
            {
                "name": "Sadece Roll Hareket (Pitch sabit)",
                "commands": {"upper": 1500, "lower": 1500, "left": 1600, "right": 1400},
                "check": "Üst/alt finler etkilenmiyor mu?"
            },
            {
                "name": "Kombineli Hareket (Bağımsız)",
                "commands": {"upper": 1600, "lower": 1400, "left": 1400, "right": 1600},
                "check": "Pitch yukarı + roll sağa bağımsız"
            },
            {
                "name": "Karşıt Yönler",
                "commands": {"upper": 1400, "lower": 1600, "left": 1600, "right": 1400},
                "check": "Pitch aşağı + roll sola bağımsız"
            },
            {
                "name": "Son - Nötr",
                "commands": {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500},
                "check": "Test tamamlandı"
            }
        ]
        
        for test in isolation_tests:
            print(f"\n🎯 {test['name']}")
            print(f"   PWM: Üst={test['commands']['upper']}, Alt={test['commands']['lower']}")
            print(f"        Sol={test['commands']['left']}, Sağ={test['commands']['right']}")
            print(f"   Kontrol: {test['check']}")
            
            self.servo_controller.set_multiple_servos(test['commands'])
            
            input("   Devam etmek için Enter basın...")
        
        print("✅ + Wing eksen bağımsızlığı testi tamamlandı!")
    
    def calibrate_servo(self, fin_name: str):
        """+ Wing servo kalibrasyonu"""
        if fin_name not in plus_wing_config.FinControlConfig.FINS:
            print(f"❌ Geçersiz fin adı: {fin_name}")
            return
        
        fin_config = plus_wing_config.FinControlConfig.FINS[fin_name]
        print(f"\n{fin_config['name']} Servo Kalibrasyonu")
        print("="*50)
        
        # + Wing spesifik bilgi
        if fin_name in ["upper", "lower"]:
            print(f"📍 Bu fin PITCH kontrolü için kullanılır")
        else:
            print(f"📍 Bu fin ROLL kontrolü için kullanılır")
        
        # Mevcut kalibrasyon verisi
        if fin_name in self.calibration_data.get("servo_limits", {}):
            current_cal = self.calibration_data["servo_limits"][fin_name]
            print(f"Mevcut kalibrasyon: Min={current_cal.get('min', 1000)}, "
                  f"Neutral={current_cal.get('neutral', 1500)}, "
                  f"Max={current_cal.get('max', 2000)}")
        
        # Kalibrasyon verilerini başlat
        if "servo_limits" not in self.calibration_data:
            self.calibration_data["servo_limits"] = {}
        
        servo_cal = {}
        
        # 1. Minimum pozisyon
        print(f"\n1. {fin_config['name']} Minimum Pozisyon:")
        min_pwm = self._calibrate_servo_position(fin_name, "minimum", 1000)
        if min_pwm:
            servo_cal["min"] = min_pwm
        
        # 2. Maksimum pozisyon
        print(f"\n2. {fin_config['name']} Maksimum Pozisyon:")
        max_pwm = self._calibrate_servo_position(fin_name, "maksimum", 2000)
        if max_pwm:
            servo_cal["max"] = max_pwm
        
        # 3. Nötr pozisyon
        print(f"\n3. {fin_config['name']} Nötr Pozisyon:")
        neutral_pwm = self._calibrate_servo_position(fin_name, "nötr", 1500)
        if neutral_pwm:
            servo_cal["neutral"] = neutral_pwm
        
        # Kalibrasyon verilerini kaydet
        self.calibration_data["servo_limits"][fin_name] = servo_cal
        
        # 4. + Wing spesifik doğrulama
        print(f"\n4. {fin_config['name']} + Wing Doğrulama:")
        self._verify_plus_wing_servo(fin_name, servo_cal)
        
        print(f"✅ {fin_config['name']} kalibrasyonu tamamlandı!")
    
    def _calibrate_servo_position(self, fin_name: str, position_name: str, default_pwm: int):
        """Servo pozisyon kalibrasyonu"""
        current_pwm = default_pwm
        
        print(f"\n{position_name.capitalize()} pozisyon ayarı:")
        print("Komutlar: +10, -10, +1, -1, test, ok, cancel")
        
        while True:
            # Mevcut pozisyonu ayarla
            self.servo_controller.set_servo_position(fin_name, current_pwm)
            print(f"Mevcut PWM: {current_pwm}")
            
            cmd = input(f"{position_name} pozisyon komutu: ").strip().lower()
            
            if cmd == "+10":
                current_pwm = min(2000, current_pwm + 10)
            elif cmd == "-10":
                current_pwm = max(1000, current_pwm - 10)
            elif cmd == "+1":
                current_pwm = min(2000, current_pwm + 1)
            elif cmd == "-1":
                current_pwm = max(1000, current_pwm - 1)
            elif cmd == "test":
                print("+ Wing test süresi: 3 saniye")
                for _ in range(3):
                    self.servo_controller.set_servo_position(fin_name, current_pwm)
                    time.sleep(0.5)
                    self.servo_controller.set_servo_position(fin_name, 1500)
                    time.sleep(0.5)
            elif cmd == "ok":
                print(f"✅ {position_name} pozisyonu PWM {current_pwm} olarak kaydedildi")
                return current_pwm
            elif cmd == "cancel":
                print(f"❌ {position_name} pozisyon ayarı iptal edildi")
                return None
            else:
                print("Geçersiz komut! (+10, -10, +1, -1, test, ok, cancel)")
    
    def _verify_plus_wing_servo(self, fin_name: str, servo_cal: dict):
        """+ Wing spesifik servo doğrulama"""
        print("+ Wing kalibrasyon doğrulaması yapılıyor...")
        
        # Diğer finleri nötr tut
        other_fins = {name: 1500 for name in plus_wing_config.FinControlConfig.FINS.keys() if name != fin_name}
        
        test_sequence = [
            ("neutral", servo_cal.get("neutral", 1500)),
            ("min", servo_cal.get("min", 1000)),
            ("neutral", servo_cal.get("neutral", 1500)),
            ("max", servo_cal.get("max", 2000)),
            ("neutral", servo_cal.get("neutral", 1500))
        ]
        
        for pos_name, pwm_value in test_sequence:
            if pwm_value:
                print(f"  → {pos_name.capitalize()}: PWM {pwm_value}")
                
                # Kalibre edilen fin + diğer finler nötr
                all_commands = other_fins.copy()
                all_commands[fin_name] = pwm_value
                
                self.servo_controller.set_multiple_servos(all_commands)
                time.sleep(1.5)
        
        verify = input("+ Wing kalibrasyonu doğru görünüyor mu? (e/h): ").lower()
        if verify == 'e':
            print("✅ + Wing kalibrasyonu doğrulandı")
        else:
            print("⚠️ + Wing kalibrasyonu tekrar gözden geçirilmeli")
    
    def test_plus_wing_mixing(self):
        """+ Wing mixing matrisi testi"""
        print("\n+ WING MIXİNG MATRİSİ TESİ")
        print("Ortogonal kontrol mixing testi...")
        print("="*50)
        
        # + Wing mixing test senaryoları
        mixing_tests = [
            {
                "name": "Roll Sağ",
                "description": "Sadece sol/sağ finler (pitch etkilenmiyor)",
                "control_input": {"roll": 1.0, "pitch": 0, "yaw": 0}
            },
            {
                "name": "Roll Sol",
                "description": "Sadece sol/sağ finler (pitch etkilenmiyor)",
                "control_input": {"roll": -1.0, "pitch": 0, "yaw": 0}
            },
            {
                "name": "Pitch Yukarı",
                "description": "Sadece üst/alt finler (roll etkilenmiyor)",
                "control_input": {"roll": 0, "pitch": 1.0, "yaw": 0}
            },
            {
                "name": "Pitch Aşağı", 
                "description": "Sadece üst/alt finler (roll etkilenmiyor)",
                "control_input": {"roll": 0, "pitch": -1.0, "yaw": 0}
            },
            {
                "name": "Yaw Sağ",
                "description": "Tüm finler kısmi katkı",
                "control_input": {"roll": 0, "pitch": 0, "yaw": 1.0}
            },
            {
                "name": "Kombineli (Pitch+Roll)",
                "description": "Bağımsız kombinasyon",
                "control_input": {"roll": 0.7, "pitch": 0.7, "yaw": 0}
            }
        ]
        
        for test in mixing_tests:
            print(f"\n🎯 {test['name']}")
            print(f"   {test['description']}")
            
            # + Wing mixing matrix uygula
            mixed_outputs = self._apply_plus_wing_mixing(test['control_input'])
            
            # Sonuçları göster
            print("   + Wing Fin PWM Çıkışları:")
            for fin_name, pwm_output in mixed_outputs.items():
                fin_desc = plus_wing_config.FinControlConfig.FINS[fin_name]["name"]
                print(f"     {fin_desc}: {pwm_output}")
            
            # Servolar uygula
            self.servo_controller.set_multiple_servos(mixed_outputs)
            
            input("   Devam etmek için Enter basın...")
        
        # Nötr pozisyona getir
        self.servo_controller.all_servos_neutral()
        print("✅ + Wing mixing matrisi testi tamamlandı")
    
    def _apply_plus_wing_mixing(self, control_input):
        """+ Wing mixing matrix uygula"""
        mixed_outputs = {}
        
        for fin_name in plus_wing_config.FinControlConfig.FINS.keys():
            output_value = 1500  # Nötr başlangıç
            
            # Her kontrol ekseni için mixing uygula
            for axis, axis_value in control_input.items():
                if axis in plus_wing_config.FinMixingConfig.MIXING_MATRIX:
                    if fin_name in plus_wing_config.FinMixingConfig.MIXING_MATRIX[axis]:
                        contribution = (axis_value * 
                                      plus_wing_config.FinMixingConfig.MIXING_MATRIX[axis][fin_name] * 
                                      200)  # 200 PWM maksimum katkı
                        output_value += contribution
            
            # PWM limitlerini uygula
            mixed_outputs[fin_name] = max(1000, min(2000, int(output_value)))
        
        return mixed_outputs
    
    def process_command(self, command: str):
        """+ Wing kalibrasyon komut işleme"""
        try:
            parts = command.strip().lower().split()
            if not parts:
                return True
            
            cmd = parts[0]
            
            # Sistem komutları
            if cmd in ['h', 'help']:
                self.show_help()
                
            elif cmd in ['q', 'quit']:
                return False
                
            elif cmd == 'status':
                self.show_system_status()
                
            elif cmd == 'save':
                self.save_calibration_data()
                
            elif cmd == 'load':
                self.load_calibration_data()
            
            # + Wing özel kalibrasyon komutları
            elif cmd == 'pitch_cal':
                self.calibrate_pitch_axis()
                
            elif cmd == 'roll_cal':
                self.calibrate_roll_axis()
                
            elif cmd == 'isolation_test':
                self.test_axis_isolation()
            
            # Servo kalibrasyon komutları
            elif cmd == 'servo_cal':
                if len(parts) == 2:
                    self.calibrate_servo(parts[1])
                else:
                    print("❌ Kullanım: servo_cal <fin>")
                    
            elif cmd == 'servo_test':
                if len(parts) == 2:
                    fin_name = parts[1]
                    if fin_name in plus_wing_config.FinControlConfig.FINS:
                        self.servo_controller.test_servo_range(fin_name, 2.0)
                    else:
                        print(f"❌ Geçersiz + Wing fin: {fin_name}")
                else:
                    print("❌ Kullanım: servo_test <fin>")
                    
            elif cmd == 'servo_all_test':
                self.servo_controller.test_all_servos(2.0)
            
            # Mixing matrisi komutları
            elif cmd == 'mixing_show':
                print("\n+ WING MIXİNG MATRİSİ:")
                for axis, fins in plus_wing_config.FinMixingConfig.MIXING_MATRIX.items():
                    print(f"{axis.upper()}:")
                    for fin_name, contribution in fins.items():
                        fin_desc = plus_wing_config.FinControlConfig.FINS[fin_name]["name"]
                        print(f"  {fin_desc}: {contribution:+.1f}")
                    print()
                    
            elif cmd == 'mixing_test':
                self.test_plus_wing_mixing()
            
            # + Wing demo
            elif cmd == 'plus_wing_demo':
                self.demo_plus_wing_advantages()
            
            else:
                print(f"❌ Bilinmeyen komut: {cmd}")
                print("Yardım için 'help' yazın")
                
        except Exception as e:
            print(f"❌ Komut işleme hatası: {e}")
        
        return True
    
    def demo_plus_wing_advantages(self):
        """+ Wing avantajları demonstrasyonu"""
        print("\n🌟 + WING AVANTAJLARI DEMONSTRASYONu")
        print("="*60)
        
        advantages = [
            {
                "title": "1. Bağımsız Pitch Kontrolü",
                "description": "Sadece üst/alt finler pitch kontrolü yapar",
                "demo": [
                    {"commands": {"upper": 1600, "lower": 1400, "left": 1500, "right": 1500}, 
                     "explanation": "Pitch yukarı - Sol/sağ finler sabit"}
                ]
            },
            {
                "title": "2. Bağımsız Roll Kontrolü", 
                "description": "Sadece sol/sağ finler roll kontrolü yapar",
                "demo": [
                    {"commands": {"upper": 1500, "lower": 1500, "left": 1600, "right": 1400},
                     "explanation": "Roll sola - Üst/alt finler sabit"}
                ]
            },
            {
                "title": "3. Bağımsız Kombinasyon",
                "description": "Pitch ve roll birbirini etkilemiyor",
                "demo": [
                    {"commands": {"upper": 1600, "lower": 1400, "left": 1600, "right": 1400},
                     "explanation": "Pitch yukarı + roll sola bağımsız"}
                ]
            },
            {
                "title": "4. Yüksek Stabilite",
                "description": "4 nokta kontrolü ile maksimum stabilite",
                "demo": [
                    {"commands": {"upper": 1520, "lower": 1480, "left": 1480, "right": 1520},
                     "explanation": "Hassas stabilizasyon düzeltmesi"}
                ]
            }
        ]
        
        for advantage in advantages:
            print(f"\n{advantage['title']}")
            print(f"📖 {advantage['description']}")
            print("-" * 40)
            
            for demo_item in advantage['demo']:
                print(f"🎬 {demo_item['explanation']}")
                self.servo_controller.set_multiple_servos(demo_item['commands'])
                time.sleep(3)
            
            input("Devam etmek için Enter basın...")
        
        # Nötr pozisyon
        self.servo_controller.all_servos_neutral()
        print("\n✅ + Wing avantajları demonstrasyonu tamamlandı!")
    
    def show_system_status(self):
        """+ Wing kalibrasyon sistem durumu"""
        print("\n" + "="*60)
        print("+ WING KALİBRASYON SİSTEMİ DURUMU")
        print("="*60)
        
        # Bağlantı durumları
        print(f"MAVLink: {'🟢 Bağlı' if self.mav and self.mav.is_connected() else '🔴 Kopuk'}")
        print(f"Konfigürasyon: 🔵 + Wing (Ortogonal Kontrol)")
        
        # Kalibrasyon durumu
        print(f"\n+ Wing Kalibrasyon Verileri:")
        print(f"  Servo Limitler: {len(self.calibration_data.get('servo_limits', {}))} fin kalibre edildi")
        print(f"  PID Değerleri: {len(self.calibration_data.get('pid_values', {}))} eksen ayarlandı")
        print(f"  Son Kalibrasyon: {self.calibration_data.get('calibration_date', 'Henüz yok')}")
        
        # + Wing servo durumları
        if hasattr(self.servo_controller, 'current_positions'):
            print(f"\n+ Wing Servo Pozisyonları:")
            for fin_name, position in self.servo_controller.get_current_positions().items():
                fin_desc = plus_wing_config.FinControlConfig.FINS[fin_name]["name"]
                axis = "Pitch" if fin_name in ["upper", "lower"] else "Roll"
                print(f"  {fin_desc} ({axis}): PWM {position}")
        
        print("="*60)
    
    def run_mission(self):
        """Ana + Wing kalibrasyon döngüsü"""
        if not self.setup_systems():
            return False
        
        self.running = True
        
        print("\n+ Wing Kalibrasyon Sistemi Başlatıldı!")
        print("+ Wing Avantajı: Bağımsız eksen kalibrasyonu!")
        print("Yardım için 'help', çıkmak için 'quit' yazın")
        print("-"*60)
        
        try:
            while self.running:
                try:
                    command = input("\n+ Wing Kalibrasyon komutu: ").strip()
                    if not self.process_command(command):
                        break
                except KeyboardInterrupt:
                    print("\n\nÇıkılıyor...")
                    break
                except EOFError:
                    break
                    
        finally:
            self._cleanup_systems()
        
        return True
    
    def _cleanup_systems(self):
        """+ Wing sistem temizliği"""
        print("+ Wing kalibrasyon sistem temizliği yapılıyor...")
        
        self.running = False
        
        # Son kalibrasyon verilerini kaydet
        self.save_calibration_data()
        
        if self.stabilizer:
            self.stabilizer.disable_stabilization()
        
        if self.servo_controller:
            self.servo_controller.emergency_stop()
        
        if self.mav:
            self.mav.set_motor_speed(0)
            self.mav.disconnect()
        
        print("✅ + Wing kalibrasyon sistemi temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    mission = PlusWingManualMission2()
    
    try:
        mission.run_mission()
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")
    
    print("+ Wing kalibrasyon görevi sonlandı.")

if __name__ == "__main__":
    main()
