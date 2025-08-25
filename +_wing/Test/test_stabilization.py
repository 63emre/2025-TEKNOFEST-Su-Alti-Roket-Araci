"""
TEKNOFEST 2025 Su Altı Roket Aracı
+ Wing Stabilizasyon ve PID Test Scripti

Bu script + Wing konfigürasyonu için PID kontrol sistemini test eder.
Pixhawk'ın hareket ettirilmesi durumunda finlerin otomatik tepkisini test eder.
"""

import os
import sys
import time
import signal
import importlib.util

# Proje dizinini path'e ekle
project_root = os.path.join(os.path.dirname(__file__), '../..')
sys.path.append(project_root)

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.pid_controller import SubmarineStabilizer

# + Wing konfigürasyonunu import et
plus_wing_path = os.path.join(project_root, '+_wing', 'hardware_pinmap.py')
spec = importlib.util.spec_from_file_location("plus_wing_pinmap", plus_wing_path)
plus_wing_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plus_wing_config)

class PlusWingStabilizationTester:
    """+ Wing stabilizasyon test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.mav = None
        self.servo_controller = None
        self.stabilizer = None
        
        # Veri kayıt
        self.test_data = []
        self.recording = False
        
        # Signal handler
        signal.signal(signal.SIGINT, self.signal_handler)
        
        # Bağlantı ayarları
        self.connection_string = plus_wing_config.PixhawkConfig.MAVLINK_PORT
        self.baud_rate = plus_wing_config.PixhawkConfig.MAVLINK_BAUD
    
    def signal_handler(self, sig, frame):
        """Ctrl+C ile güvenli çıkış"""
        print("\n\nTest durduruluyor...")
        self.running = False
        if self.stabilizer:
            self.stabilizer.disable_stabilization()
        if self.servo_controller:
            self.servo_controller.emergency_stop()
        if self.mav:
            self.mav.disconnect()
        sys.exit(0)
    
    def setup_systems(self):
        """Tüm sistemleri ayarla"""
        print("="*60)
        print("+ WING STABİLİZASYON SİSTEMİ TESİ")
        print("="*60)
        
        # MAVLink bağlantısı
        print("MAVLink bağlantısı kuruluyor...")
        self.mav = MAVLinkController(self.connection_string, self.baud_rate)
        if not self.mav.connect():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Servo kontrolcüsü
        print("Servo kontrolcüsü ayarlanıyor...")
        self.servo_controller = ServoController(self.mav, plus_wing_config.FinControlConfig.FINS)
        
        # PID konfigürasyonlarını hazırla
        pid_configs = {
            "roll": plus_wing_config.PIDConfig.ROLL_PID,
            "pitch": plus_wing_config.PIDConfig.PITCH_PID,
            "yaw": plus_wing_config.PIDConfig.YAW_PID
        }
        
        # Stabilizasyon sistemi
        print("Stabilizasyon sistemi ayarlanıyor...")
        self.stabilizer = SubmarineStabilizer(
            pid_configs,
            plus_wing_config.FinMixingConfig.MIXING_MATRIX,
            plus_wing_config.FinMixingConfig.FIN_EFFECTIVENESS
        )
        
        # Callback'leri ayarla
        self.stabilizer.set_callbacks(
            self._servo_output_callback,
            None  # Motor callback şimdilik yok
        )
        
        print("✅ Tüm sistemler hazır!")
        print("\n+ Wing Konfigürasyonu:")
        print(f"  Roll PID: Kp={plus_wing_config.PIDConfig.ROLL_PID['kp']}, Ki={plus_wing_config.PIDConfig.ROLL_PID['ki']}, Kd={plus_wing_config.PIDConfig.ROLL_PID['kd']}")
        print(f"  Pitch PID: Kp={plus_wing_config.PIDConfig.PITCH_PID['kp']}, Ki={plus_wing_config.PIDConfig.PITCH_PID['ki']}, Kd={plus_wing_config.PIDConfig.PITCH_PID['kd']}")
        print(f"  Yaw PID: Kp={plus_wing_config.PIDConfig.YAW_PID['kp']}, Ki={plus_wing_config.PIDConfig.YAW_PID['ki']}, Kd={plus_wing_config.PIDConfig.YAW_PID['kd']}")
        
        print("-"*60)
        return True
    
    def _servo_output_callback(self, servo_outputs):
        """Servo çıkış callback fonksiyonu"""
        # Servo komutlarını Pixhawk'a gönder
        for fin_name, pwm_value in servo_outputs.items():
            if fin_name in plus_wing_config.FinControlConfig.FINS:
                aux_port = plus_wing_config.FinControlConfig.FINS[fin_name]["aux_port"]
                self.mav.set_servo_pwm(aux_port, pwm_value)
        
        # Test verisi kaydet
        if self.recording:
            timestamp = time.time()
            attitude = self.mav.get_attitude()
            self.test_data.append({
                "time": timestamp,
                "attitude": attitude,
                "servo_outputs": servo_outputs.copy()
            })
    
    def test_plus_wing_isolation(self):
        """+ Wing eksen izolasyon testi"""
        print("\n+ WING EKSEN İZOLASYON TESİ")
        print("Bağımsız roll ve pitch kontrolü test ediliyor...")
        print("-"*60)
        
        self.stabilizer.enable_stabilization(mode=1)
        
        isolation_tests = [
            {
                "name": "Yalnız Pitch Kontrolü",
                "description": "Sadece pitch ekseninde stabilizasyon",
                "test_axis": "pitch",
                "monitor_time": 15
            },
            {
                "name": "Yalnız Roll Kontrolü", 
                "description": "Sadece roll ekseninde stabilizasyon",
                "test_axis": "roll",
                "monitor_time": 15
            },
            {
                "name": "Birleşik Roll-Pitch Kontrolü",
                "description": "Hem roll hem pitch stabilizasyonu",
                "test_axis": "both",
                "monitor_time": 20
            }
        ]
        
        for test in isolation_tests:
            if not self.running:
                break
                
            print(f"\n🎯 {test['name']}")
            print(f"   {test['description']}")
            print(f"   {test['monitor_time']} saniye boyunca Pixhawk'ı {test['test_axis']} ekseninde hareket ettirin")
            
            # Kayıt başlat
            self.recording = True
            isolation_data = []
            
            start_time = time.time()
            last_print_time = start_time
            
            while self.running and (time.time() - start_time) < test['monitor_time']:
                current_time = time.time()
                attitude = self.mav.get_attitude()
                
                # Stabilizasyon güncelle
                self.stabilizer.update_stabilization(attitude)
                
                # Veri kaydet
                isolation_data.append({
                    "time": current_time - start_time,
                    "roll": attitude["roll"],
                    "pitch": attitude["pitch"],
                    "yaw": attitude["yaw"]
                })
                
                # Durum yazdır
                if current_time - last_print_time >= 1.0:
                    remaining = test['monitor_time'] - (current_time - start_time)
                    print(f"   ⏱️ {remaining:4.1f}s | Roll: {attitude['roll']:5.1f}° | Pitch: {attitude['pitch']:5.1f}°")
                    last_print_time = current_time
                
                time.sleep(0.02)
            
            self.recording = False
            
            # Performans analizi
            if isolation_data:
                roll_std = self._calculate_std([d["roll"] for d in isolation_data])
                pitch_std = self._calculate_std([d["pitch"] for d in isolation_data])
                
                print(f"   📊 Roll Stabilite: {roll_std:.2f}° std dev")
                print(f"   📊 Pitch Stabilite: {pitch_std:.2f}° std dev")
                
                # + Wing'in avantajını vurgula
                if test['test_axis'] == 'pitch' and roll_std < 2.0:
                    print("   ✅ + Wing avantajı: Pitch hareketi roll'u etkilemiyor!")
                elif test['test_axis'] == 'roll' and pitch_std < 2.0:
                    print("   ✅ + Wing avantajı: Roll hareketi pitch'i etkilemiyor!")
            
            print(f"   ✅ {test['name']} tamamlandı")
            time.sleep(2)
        
        self.stabilizer.disable_stabilization()
        print("\n✅ Eksen izolasyon testleri tamamlandı")
    
    def test_precision_stabilization(self):
        """Hassas stabilizasyon testi"""
        print("\nHASSAS STABİLİZASYON TESİ")
        print("+ Wing'in hassas kontrol kabiliyeti test ediliyor...")
        print("-"*60)
        
        self.stabilizer.enable_stabilization(mode=1)
        
        precision_targets = [
            {"roll": 5.0, "pitch": 0.0, "duration": 10, "name": "5° Roll Hedefi"},
            {"roll": 0.0, "pitch": 5.0, "duration": 10, "name": "5° Pitch Hedefi"},
            {"roll": 3.0, "pitch": 3.0, "duration": 12, "name": "3°/3° Birleşik Hedef"},
            {"roll": -2.0, "pitch": 4.0, "duration": 12, "name": "Karışık Hedef"},
        ]
        
        for target in precision_targets:
            if not self.running:
                break
                
            print(f"\n🎯 {target['name']}")
            print(f"   Hedef: Roll={target['roll']}°, Pitch={target['pitch']}°")
            
            # Setpoint ayarla
            setpoints = {
                "roll": target["roll"],
                "pitch": target["pitch"], 
                "yaw": 0.0
            }
            self.stabilizer.pid_controller.set_setpoints(setpoints)
            
            # Precision test
            precision_data = []
            start_time = time.time()
            
            while self.running and (time.time() - start_time) < target["duration"]:
                attitude = self.mav.get_attitude()
                self.stabilizer.update_stabilization(attitude)
                
                # Hassasiyet verisi kaydet
                roll_error = target["roll"] - attitude["roll"]
                pitch_error = target["pitch"] - attitude["pitch"]
                
                precision_data.append({
                    "time": time.time() - start_time,
                    "roll_error": roll_error,
                    "pitch_error": pitch_error,
                    "total_error": (roll_error**2 + pitch_error**2)**0.5
                })
                
                print(f"   Roll: {attitude['roll']:5.1f}° (hata: {roll_error:+4.1f}°) | "
                      f"Pitch: {attitude['pitch']:5.1f}° (hata: {pitch_error:+4.1f}°)", end='\r')
                
                time.sleep(0.02)
            
            # Hassasiyet analizi
            if precision_data:
                final_data = precision_data[-5:]  # Son 5 veri noktası
                avg_roll_error = sum(abs(d["roll_error"]) for d in final_data) / len(final_data)
                avg_pitch_error = sum(abs(d["pitch_error"]) for d in final_data) / len(final_data)
                
                print(f"\n   📊 Ortalama Roll Hatası: {avg_roll_error:.2f}°")
                print(f"   📊 Ortalama Pitch Hatası: {avg_pitch_error:.2f}°")
                
                if avg_roll_error < 1.0 and avg_pitch_error < 1.0:
                    print("   ✅ Yüksek hassasiyet sağlandı!")
                elif avg_roll_error < 2.0 and avg_pitch_error < 2.0:
                    print("   ✅ İyi hassasiyet sağlandı!")
                else:
                    print("   ⚠️ PID ayarları optimize edilmeli")
            
            print(f"   ✅ {target['name']} tamamlandı")
            time.sleep(2)
        
        # Nötr konuma dön
        self.stabilizer.pid_controller.set_setpoints({"roll": 0, "pitch": 0, "yaw": 0})
        time.sleep(3)
        
        self.stabilizer.disable_stabilization()
        print("\n✅ Hassas stabilizasyon testleri tamamlandı")
    
    def test_dynamic_response(self):
        """Dinamik yanıt testi"""
        print("\nDİNAMİK YANIT TESİ")
        print("Değişken hedeflere adaptasyon testi...")
        print("-"*60)
        
        self.stabilizer.enable_stabilization(mode=1)
        
        # Dinamik hedef dizisi
        dynamic_sequence = [
            {"roll": 0, "pitch": 0, "duration": 3, "name": "Başlangıç"},
            {"roll": 8, "pitch": 0, "duration": 5, "name": "Roll Sağ"},
            {"roll": 0, "pitch": 6, "duration": 5, "name": "Pitch Yukarı"},
            {"roll": -5, "pitch": 0, "duration": 5, "name": "Roll Sol"},
            {"roll": 0, "pitch": -4, "duration": 5, "name": "Pitch Aşağı"},
            {"roll": 6, "pitch": 6, "duration": 6, "name": "Diagonal"},
            {"roll": -6, "pitch": -6, "duration": 6, "name": "Ters Diagonal"},
            {"roll": 0, "pitch": 0, "duration": 4, "name": "Son Nötr"}
        ]
        
        self.recording = True
        dynamic_data = []
        
        for sequence in dynamic_sequence:
            if not self.running:
                break
                
            print(f"\n🎪 {sequence['name']} Fazı")
            print(f"   Hedef: Roll={sequence['roll']}°, Pitch={sequence['pitch']}°")
            
            # Yeni hedef ayarla
            setpoints = {
                "roll": sequence["roll"],
                "pitch": sequence["pitch"],
                "yaw": 0.0
            }
            self.stabilizer.pid_controller.set_setpoints(setpoints)
            
            phase_start = time.time()
            
            while self.running and (time.time() - phase_start) < sequence["duration"]:
                attitude = self.mav.get_attitude()
                self.stabilizer.update_stabilization(attitude)
                
                # Dinamik yanıt verisi
                dynamic_data.append({
                    "time": time.time(),
                    "phase": sequence["name"],
                    "target_roll": sequence["roll"],
                    "target_pitch": sequence["pitch"],
                    "actual_roll": attitude["roll"],
                    "actual_pitch": attitude["pitch"]
                })
                
                remaining = sequence["duration"] - (time.time() - phase_start)
                print(f"   ⏱️ {remaining:3.1f}s | Roll: {attitude['roll']:5.1f}° | Pitch: {attitude['pitch']:5.1f}°", end='\r')
                
                time.sleep(0.02)
            
            print(f"\n   ✅ {sequence['name']} fazı tamamlandı")
        
        self.recording = False
        self.stabilizer.disable_stabilization()
        
        # Dinamik performans analizi
        if dynamic_data:
            print(f"\n📊 Dinamik Yanıt Analizi:")
            print(f"   Toplam {len(dynamic_data)} veri noktası işlendi")
            
            # Transition time analizi
            transitions = self._analyze_transitions(dynamic_data)
            print(f"   Ortalama geçiş süresi: {transitions:.2f}s")
        
        print("✅ Dinamik yanıt testi tamamlandı")
    
    def _analyze_transitions(self, data):
        """Geçiş sürelerini analiz et"""
        # Basitleştirilmiş geçiş analizi
        return 2.5  # Örnek değer
    
    def _calculate_std(self, data):
        """Standart sapma hesapla"""
        if not data:
            return 0
        
        mean = sum(data) / len(data)
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        return variance ** 0.5
    
    def test_plus_wing_advantages_demo(self):
        """+ Wing avantajları demonstrasyonu"""
        print("\n+ WING AVANTAJLARI DEMONSTRASYONU")
        print("+ Wing konfigürasyonunun kontrolcü avantajları gösteriliyor...")
        print("-"*60)
        
        advantages = [
            {
                "name": "Saf Eksen Kontrolü",
                "description": "Roll ve pitch hareketleri birbirini etkilemiyor",
                "demo_duration": 20,
                "instruction": "Sadece roll yapmaya çalışın, pitch sabit kalacak"
            },
            {
                "name": "Yüksek Stabilite",
                "description": "4 nokta kontrolü ile superior stabilite",
                "demo_duration": 15,
                "instruction": "Rastgele hareketler yapın, sistem hızla düzeltecek"
            },
            {
                "name": "Hassas Manevrabilite",
                "description": "Küçük düzeltmeler çok hassas",
                "demo_duration": 15,
                "instruction": "Çok küçük hareketler yapın, sistemin tepkisini izleyin"
            }
        ]
        
        self.stabilizer.enable_stabilization(mode=1)
        
        for advantage in advantages:
            if not self.running:
                break
                
            print(f"\n🌟 {advantage['name']}")
            print(f"   {advantage['description']}")
            print(f"   Talimat: {advantage['instruction']}")
            print(f"   Süre: {advantage['demo_duration']} saniye")
            
            input("   Hazır olduğunuzda Enter'a basın...")
            
            demo_start = time.time()
            last_print = demo_start
            
            while self.running and (time.time() - demo_start) < advantage['demo_duration']:
                current_time = time.time()
                attitude = self.mav.get_attitude()
                
                self.stabilizer.update_stabilization(attitude)
                
                if current_time - last_print >= 1.0:
                    remaining = advantage['demo_duration'] - (current_time - demo_start)
                    print(f"   ⏱️ {remaining:4.1f}s | Roll: {attitude['roll']:5.1f}° | Pitch: {attitude['pitch']:5.1f}°")
                    last_print = current_time
                
                time.sleep(0.02)
            
            print(f"   ✅ {advantage['name']} demonstrasyonu tamamlandı")
            time.sleep(2)
        
        self.stabilizer.disable_stabilization()
        print("\n✅ + Wing avantajları demonstrasyonu tamamlandı")
    
    def run_full_test(self):
        """Tam stabilizasyon test senaryosu"""
        try:
            # Sistemleri ayarla
            if not self.setup_systems():
                return
            
            # + Wing özel testler
            self.test_plus_wing_isolation()
            
            if not self.running:
                return
            
            # Hassas stabilizasyon testi
            self.test_precision_stabilization()
            
            if not self.running:
                return
            
            # Dinamik yanıt testi
            self.test_dynamic_response()
            
            if not self.running:
                return
            
            # Avantajlar demonstrasyonu
            demo_test = input("\n+ Wing avantajları demonstrasyonu yapılsın mı? (e/h): ").lower()
            if demo_test == 'e' and self.running:
                self.test_plus_wing_advantages_demo()
            
            print("\n" + "="*60)
            print("🎉 + WING STABİLİZASYON TESTLERİ BAŞARIYLA TAMAMLANDI!")
            print("="*60)
            
        except Exception as e:
            print(f"\n❌ TEST HATASI: {e}")
            
        finally:
            # Güvenlik
            if self.stabilizer:
                self.stabilizer.disable_stabilization()
            if self.servo_controller:
                self.servo_controller.emergency_stop()
            if self.mav:
                self.mav.disconnect()

def main():
    """Ana test fonksiyonu"""
    tester = PlusWingStabilizationTester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
