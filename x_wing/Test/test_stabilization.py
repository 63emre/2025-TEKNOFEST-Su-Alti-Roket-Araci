"""
TEKNOFEST 2025 Su Altı Roket Aracı
X Wing Stabilizasyon ve PID Test Scripti

Bu script X Wing konfigürasyonu için PID kontrol sistemini test eder.
Pixhawk'ın hareket ettirilmesi durumunda finlerin otomatik tepkisini test eder.
"""

import os
import sys
import time
import signal
import threading

# Proje dizinini path'e ekle
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.pid_controller import SubmarineStabilizer
from x_wing.hardware_pinmap import (PixhawkConfig, FinControlConfig, 
                                   PIDConfig, FinMixingConfig)

class XWingStabilizationTester:
    """X Wing stabilizasyon test sınıfı"""
    
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
        self.connection_string = PixhawkConfig.MAVLINK_PORT
        self.baud_rate = PixhawkConfig.MAVLINK_BAUD
    
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
        print("X WING STABİLİZASYON SİSTEMİ TESİ")
        print("="*60)
        
        # MAVLink bağlantısı
        print("MAVLink bağlantısı kuruluyor...")
        self.mav = MAVLinkController(self.connection_string, self.baud_rate)
        if not self.mav.connect():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Servo kontrolcüsü
        print("Servo kontrolcüsü ayarlanıyor...")
        self.servo_controller = ServoController(self.mav, FinControlConfig.FINS)
        
        # PID konfigürasyonlarını hazırla
        pid_configs = {
            "roll": PIDConfig.ROLL_PID,
            "pitch": PIDConfig.PITCH_PID,
            "yaw": PIDConfig.YAW_PID
        }
        
        # Stabilizasyon sistemi
        print("Stabilizasyon sistemi ayarlanıyor...")
        self.stabilizer = SubmarineStabilizer(
            pid_configs,
            FinMixingConfig.MIXING_MATRIX,
            FinMixingConfig.FIN_EFFECTIVENESS
        )
        
        # Callback'leri ayarla
        self.stabilizer.set_callbacks(
            self._servo_output_callback,
            None  # Motor callback şimdilik yok
        )
        
        print("✅ Tüm sistemler hazır!")
        print("\nX Wing Konfigürasyonu:")
        print(f"  Roll PID: Kp={PIDConfig.ROLL_PID['kp']}, Ki={PIDConfig.ROLL_PID['ki']}, Kd={PIDConfig.ROLL_PID['kd']}")
        print(f"  Pitch PID: Kp={PIDConfig.PITCH_PID['kp']}, Ki={PIDConfig.PITCH_PID['ki']}, Kd={PIDConfig.PITCH_PID['kd']}")
        print(f"  Yaw PID: Kp={PIDConfig.YAW_PID['kp']}, Ki={PIDConfig.YAW_PID['ki']}, Kd={PIDConfig.YAW_PID['kd']}")
        
        print("-"*60)
        return True
    
    def _servo_output_callback(self, servo_outputs):
        """Servo çıkış callback fonksiyonu"""
        # Servo komutlarını Pixhawk'a gönder
        for fin_name, pwm_value in servo_outputs.items():
            if fin_name in FinControlConfig.FINS:
                aux_port = FinControlConfig.FINS[fin_name]["aux_port"]
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
    
    def test_attitude_reading(self):
        """Attitude verisi okuma testi"""
        print("\nATTITUDE VERİ OKUMA TESİ")
        print("Pixhawk'tan roll, pitch, yaw verileri okunuyor...")
        print("Pixhawk'ı manuel olarak hareket ettirin!")
        print("-"*60)
        
        print("10 saniye boyunca attitude verisi okunacak...")
        start_time = time.time()
        
        while self.running and (time.time() - start_time) < 10:
            attitude = self.mav.get_attitude()
            
            print(f"Roll: {attitude['roll']:6.1f}° | "
                  f"Pitch: {attitude['pitch']:6.1f}° | "
                  f"Yaw: {attitude['yaw']:6.1f}°", end='\r')
            
            time.sleep(0.1)  # 10Hz güncelleme
        
        print("\n✅ Attitude veri okuma testi tamamlandı")
    
    def test_manual_stabilization(self):
        """Manuel stabilizasyon testi"""
        print("\nMANUEL STABİLİZASYON TESİ")
        print("Stabilizasyon sistemi etkinleştiriliyor...")
        print("Pixhawk'ı manuel olarak hareket ettirin - finler otomatik cevap verecek!")
        print("-"*60)
        
        # Stabilizasyonu etkinleştir
        self.stabilizer.enable_stabilization(mode=1)  # STABILIZE modu
        
        # Kayıt başlat
        self.recording = True
        self.test_data.clear()
        
        # Test süresi
        test_duration = 30
        print(f"{test_duration} saniye stabilizasyon testi...")
        print("Pixhawk'ı farklı yönlere çevirin!")
        
        start_time = time.time()
        last_print_time = start_time
        
        while self.running and (time.time() - start_time) < test_duration:
            current_time = time.time()
            remaining = test_duration - (current_time - start_time)
            
            # Sensör verilerini al
            attitude = self.mav.get_attitude()
            
            # Stabilizasyon güncellemesi
            self.stabilizer.update_stabilization(attitude)
            
            # Her saniye durum yazdır
            if current_time - last_print_time >= 1.0:
                print(f"⏱️ {remaining:4.1f}s | "
                      f"Roll: {attitude['roll']:5.1f}° | "
                      f"Pitch: {attitude['pitch']:5.1f}° | "
                      f"Yaw: {attitude['yaw']:5.1f}°")
                last_print_time = current_time
            
            time.sleep(0.02)  # 50Hz döngü
        
        # Kayıt durdur
        self.recording = False
        self.stabilizer.disable_stabilization()
        
        print(f"\n✅ Stabilizasyon testi tamamlandı - {len(self.test_data)} veri noktası")
    
    def test_step_response(self):
        """Step yanıt testi"""
        print("\nSTEP YANIT TESİ")
        print("PID sisteminin step yanıtları ölçülüyor...")
        print("-"*60)
        
        # Test senaryoları
        step_tests = [
            {"axis": "roll", "setpoint": 10.0, "duration": 15, "description": "10° Roll Step"},
            {"axis": "roll", "setpoint": -10.0, "duration": 15, "description": "-10° Roll Step"},
            {"axis": "pitch", "setpoint": 8.0, "duration": 15, "description": "8° Pitch Step"},
            {"axis": "pitch", "setpoint": -8.0, "duration": 15, "description": "-8° Pitch Step"},
        ]
        
        self.stabilizer.enable_stabilization(mode=1)
        
        for test_scenario in step_tests:
            if not self.running:
                break
                
            print(f"\n🎯 {test_scenario['description']}")
            
            # Setpoint ayarla
            setpoints = {"roll": 0, "pitch": 0, "yaw": 0}
            setpoints[test_scenario["axis"]] = test_scenario["setpoint"]
            self.stabilizer.pid_controller.set_setpoints(setpoints)
            
            # Kayıt başlat
            self.recording = True
            step_data = []
            
            start_time = time.time()
            
            while self.running and (time.time() - start_time) < test_scenario["duration"]:
                attitude = self.mav.get_attitude()
                self.stabilizer.update_stabilization(attitude)
                
                # Step yanıt verisi kaydet
                step_data.append({
                    "time": time.time() - start_time,
                    "setpoint": test_scenario["setpoint"],
                    "actual": attitude[test_scenario["axis"]],
                    "error": test_scenario["setpoint"] - attitude[test_scenario["axis"]]
                })
                
                # Durum yazdır
                error = test_scenario["setpoint"] - attitude[test_scenario["axis"]]
                print(f"   Hedef: {test_scenario['setpoint']:5.1f}° | "
                      f"Gerçek: {attitude[test_scenario['axis']]:5.1f}° | "
                      f"Hata: {error:5.1f}°", end='\r')
                
                time.sleep(0.02)
            
            self.recording = False
            
            # Sonuçları analiz et
            final_error = abs(step_data[-1]["error"]) if step_data else 999
            settling_time = self._calculate_settling_time(step_data, test_scenario["setpoint"])
            
            print(f"\n   Final Hata: {final_error:.2f}°")
            print(f"   Settling Time: {settling_time:.2f}s")
            print(f"   ✅ {test_scenario['description']} tamamlandı")
            
            # Nötre dön
            self.stabilizer.pid_controller.set_setpoints({"roll": 0, "pitch": 0, "yaw": 0})
            time.sleep(3)  # Stabilizasyon için bekle
        
        self.stabilizer.disable_stabilization()
        print("\n✅ Step yanıt testleri tamamlandı")
    
    def _calculate_settling_time(self, data, setpoint, tolerance=0.5):
        """Settling time hesapla (%5 tolerance)"""
        if not data:
            return float('inf')
        
        tolerance_band = abs(setpoint) * 0.05  # %5 tolerans
        
        for i in reversed(range(len(data))):
            if abs(data[i]["error"]) > tolerance_band:
                return data[i]["time"] if i < len(data) - 1 else data[-1]["time"]
        
        return 0.0  # Hemen yerleşti
    
    def test_disturbance_rejection(self):
        """Bozucu etki reddi testi"""
        print("\nBOZUCU ETKİ REDDİ TESİ")
        print("Harici bozuculara karşı stabilizasyon testi...")
        print("-"*60)
        
        self.stabilizer.enable_stabilization(mode=1)
        
        print("Stabilizasyon sistemi çalışıyor...")
        print("Pixhawk'ı manuel olarak farklı yönlere zorlayın!")
        print("Sistem size karşı koyacak ve eski konumuna dönmeye çalışacak.")
        print("30 saniye test süresi...")
        
        self.recording = True
        disturbance_data = []
        
        start_time = time.time()
        last_print_time = start_time
        
        while self.running and (time.time() - start_time) < 30:
            current_time = time.time()
            attitude = self.mav.get_attitude()
            
            self.stabilizer.update_stabilization(attitude)
            
            # Bozucu etki verisi kaydet
            disturbance_data.append({
                "time": current_time - start_time,
                "roll": attitude["roll"],
                "pitch": attitude["pitch"], 
                "yaw": attitude["yaw"]
            })
            
            # Her saniye durum yazdır
            if current_time - last_print_time >= 1.0:
                remaining = 30 - (current_time - start_time)
                print(f"⏱️ {remaining:4.1f}s | "
                      f"Roll: {attitude['roll']:5.1f}° | "
                      f"Pitch: {attitude['pitch']:5.1f}° | "
                      f"Yaw: {attitude['yaw']:5.1f}°")
                last_print_time = current_time
            
            time.sleep(0.02)
        
        self.recording = False
        self.stabilizer.disable_stabilization()
        
        # Performans analizi
        if disturbance_data:
            roll_std = self._calculate_std([d["roll"] for d in disturbance_data])
            pitch_std = self._calculate_std([d["pitch"] for d in disturbance_data])
            yaw_std = self._calculate_std([d["yaw"] for d in disturbance_data])
            
            print(f"\nPerformans Analizi:")
            print(f"   Roll Std Dev: {roll_std:.2f}°")
            print(f"   Pitch Std Dev: {pitch_std:.2f}°")
            print(f"   Yaw Std Dev: {yaw_std:.2f}°")
        
        print("✅ Bozucu etki reddi testi tamamlandı")
    
    def _calculate_std(self, data):
        """Standart sapma hesapla"""
        if not data:
            return 0
        
        mean = sum(data) / len(data)
        variance = sum((x - mean) ** 2 for x in data) / len(data)
        return variance ** 0.5
    
    def test_pid_tuning(self):
        """PID ayar testi"""
        print("\nPID AYAR TESİ")
        print("Mevcut PID parametreleri test ediliyor...")
        print("-"*60)
        
        # Mevcut PID değerlerini göster
        print("Mevcut PID Parametreleri:")
        for axis in ["roll", "pitch", "yaw"]:
            config = getattr(PIDConfig, f"{axis.upper()}_PID")
            print(f"   {axis.upper()}: Kp={config['kp']}, Ki={config['ki']}, Kd={config['kd']}")
        
        print("\nPID performansını değerlendirmek için manuel test yapın...")
        print("Pixhawk'ı hareket ettirin ve sistemin yanıtını gözlemleyin.")
        
        # İsteğe bağlı PID ayarı
        tune_pid = input("\nPID değerlerini ayarlamak ister misiniz? (e/h): ").lower()
        if tune_pid == 'e':
            self._interactive_pid_tuning()
        
        print("✅ PID ayar testi tamamlandı")
    
    def _interactive_pid_tuning(self):
        """İnteraktif PID ayarlama"""
        print("\nİNTERAKTİF PID AYARLAMA")
        print("Hangi ekseni ayarlamak istiyorsunuz?")
        print("1. Roll")
        print("2. Pitch") 
        print("3. Yaw")
        
        choice = input("Seçiminiz (1-3): ")
        axis_map = {"1": "roll", "2": "pitch", "3": "yaw"}
        
        if choice in axis_map:
            axis = axis_map[choice]
            current_config = getattr(PIDConfig, f"{axis.upper()}_PID")
            
            print(f"\n{axis.upper()} PID Ayarlama:")
            print(f"Mevcut değerler - Kp:{current_config['kp']}, Ki:{current_config['ki']}, Kd:{current_config['kd']}")
            
            try:
                kp = float(input(f"Yeni Kp değeri ({current_config['kp']}): ") or current_config['kp'])
                ki = float(input(f"Yeni Ki değeri ({current_config['ki']}): ") or current_config['ki'])
                kd = float(input(f"Yeni Kd değeri ({current_config['kd']}): ") or current_config['kd'])
                
                # PID değerlerini güncelle
                self.stabilizer.pid_controller.update_gains(axis, kp, ki, kd)
                print(f"✅ {axis.upper()} PID değerleri güncellendi")
                
                # Test için kısa süre çalıştır
                test_new = input("Yeni değerleri test etmek ister misiniz? (e/h): ").lower()
                if test_new == 'e':
                    self._test_updated_pid(axis, 15)
                    
            except ValueError:
                print("❌ Geçersiz değer girdiniz!")
    
    def _test_updated_pid(self, axis, duration):
        """Güncellenen PID'yi test et"""
        print(f"\n{axis.upper()} PID test ediliyor ({duration}s)...")
        
        self.stabilizer.enable_stabilization(mode=1)
        
        start_time = time.time()
        while self.running and (time.time() - start_time) < duration:
            attitude = self.mav.get_attitude()
            self.stabilizer.update_stabilization(attitude)
            
            print(f"Roll: {attitude['roll']:5.1f}° | "
                  f"Pitch: {attitude['pitch']:5.1f}° | "
                  f"Yaw: {attitude['yaw']:5.1f}°", end='\r')
            
            time.sleep(0.02)
        
        self.stabilizer.disable_stabilization()
        print(f"\n✅ {axis.upper()} PID testi tamamlandı")
    
    def run_full_test(self):
        """Tam stabilizasyon test senaryosu"""
        try:
            # Sistemleri ayarla
            if not self.setup_systems():
                return
            
            # Attitude okuma testi
            self.test_attitude_reading()
            
            if not self.running:
                return
            
            # Manuel stabilizasyon testi
            self.test_manual_stabilization()
            
            if not self.running:
                return
            
            # Step yanıt testi
            self.test_step_response()
            
            if not self.running:
                return
            
            # Bozucu etki testi
            self.test_disturbance_rejection()
            
            if not self.running:
                return
            
            # PID ayar testi
            self.test_pid_tuning()
            
            print("\n" + "="*60)
            print("🎉 X WING STABİLİZASYON TESTLERİ BAŞARIYLA TAMAMLANDI!")
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
    tester = XWingStabilizationTester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
