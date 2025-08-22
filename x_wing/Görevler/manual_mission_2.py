"""
TEKNOFEST 2025 Su Altı Roket Aracı
X Wing - Manuel Görev 2: Servo Kalibrasyonu ve Fine-Tuning

Bu manuel görev X Wing servo kalibrasyonu ve PID fine-tuning
için kullanılır. Terminal üzerinden detaylı ayarlamalar yapabilir.
"""

import os
import sys
import time
import json
import threading

# Proje modüllerini import et
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.pid_controller import SubmarineStabilizer
from common.d300_sensor import D300Sensor
from common.gpio_helper import GPIOController
from x_wing.hardware_pinmap import (
    PixhawkConfig, FinControlConfig, PIDConfig, 
    FinMixingConfig, SensorConfig, RaspberryPiConfig
)

class ManualMission2:
    """X Wing Manuel Görev 2 - Kalibrasyon ve Fine-Tuning"""
    
    def __init__(self):
        self.running = False
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        self.stabilizer = None
        self.depth_sensor = None
        self.gpio = None
        
        # Kalibrasyon verileri
        self.calibration_data = {
            "servo_limits": {},
            "pid_values": {},
            "mixing_matrix": {},
            "calibration_date": None
        }
        
        # Test durumu
        self.test_running = False
        self.test_thread = None
        
        print("X Wing Manuel Görev 2 - Servo Kalibrasyonu ve Fine-Tuning")
        print("="*65)
    
    def setup_systems(self):
        """Tüm sistemleri ayarla"""
        print("Kalibrasyon sistemleri ayarlanıyor...")
        
        try:
            # GPIO ayarla
            self.gpio = GPIOController(
                button_pin=RaspberryPiConfig.BUTTON_PIN,
                led_pin=RaspberryPiConfig.LED_PIN,
                buzzer_pin=RaspberryPiConfig.BUZZER_PIN
            )
            self.gpio.setup_gpio()
            print("✅ GPIO sistemi hazır")
            
            # MAVLink bağlantısı
            self.mav = MAVLinkController(
                PixhawkConfig.MAVLINK_PORT,
                PixhawkConfig.MAVLINK_BAUD
            )
            if not self.mav.connect():
                raise Exception("MAVLink bağlantısı başarısız")
            print("✅ MAVLink bağlantısı kuruldu")
            
            # Servo kontrolcüsü
            self.servo_controller = ServoController(self.mav, FinControlConfig.FINS)
            print("✅ Servo kontrolcüsü hazır")
            
            # Derinlik sensörü (opsiyonel)
            self.depth_sensor = D300Sensor(
                bus_number=SensorConfig.D300_BUS,
                address=SensorConfig.D300_I2C_ADDRESS
            )
            if self.depth_sensor.connect():
                self.depth_sensor.start_continuous_reading()
                print("✅ D300 sensörü bağlandı")
            else:
                print("⚠️  D300 sensörü bağlanamadı (opsiyonel)")
            
            # Stabilizasyon sistemi
            pid_configs = {
                "roll": PIDConfig.ROLL_PID,
                "pitch": PIDConfig.PITCH_PID,
                "yaw": PIDConfig.YAW_PID
            }
            
            self.stabilizer = SubmarineStabilizer(
                pid_configs,
                FinMixingConfig.MIXING_MATRIX,
                FinMixingConfig.FIN_EFFECTIVENESS
            )
            
            self.stabilizer.set_callbacks(self._servo_callback, None)
            print("✅ Stabilizasyon sistemi hazır")
            
            # Kalibrasyon verilerini yükle
            self.load_calibration_data()
            
            # Başlangıç sequence
            if self.gpio:
                self.gpio.startup_sequence()
            
            print("✅ Kalibrasyon sistemleri hazır!")
            return True
            
        except Exception as e:
            print(f"❌ Sistem ayarlama hatası: {e}")
            return False
    
    def _servo_callback(self, servo_outputs):
        """Servo çıkış callback'i"""
        for fin_name, pwm_value in servo_outputs.items():
            if fin_name in FinControlConfig.FINS:
                aux_port = FinControlConfig.FINS[fin_name]["aux_port"]
                self.mav.set_servo_pwm(aux_port, pwm_value)
    
    def load_calibration_data(self):
        """Önceki kalibrasyon verilerini yükle"""
        calibration_file = os.path.join(os.path.dirname(__file__), "calibration_data.json")
        
        if os.path.exists(calibration_file):
            try:
                with open(calibration_file, 'r') as f:
                    self.calibration_data = json.load(f)
                print(f"✅ Kalibrasyon verileri yüklendi: {self.calibration_data.get('calibration_date', 'Bilinmiyor')}")
            except Exception as e:
                print(f"⚠️  Kalibrasyon verileri yüklenemedi: {e}")
        else:
            print("ℹ️  Önceki kalibrasyon verisi bulunamadı")
    
    def save_calibration_data(self):
        """Kalibrasyon verilerini kaydet"""
        calibration_file = os.path.join(os.path.dirname(__file__), "calibration_data.json")
        
        self.calibration_data["calibration_date"] = time.strftime("%Y-%m-%d %H:%M:%S")
        
        try:
            with open(calibration_file, 'w') as f:
                json.dump(self.calibration_data, f, indent=4)
            print(f"✅ Kalibrasyon verileri kaydedildi: {calibration_file}")
        except Exception as e:
            print(f"❌ Kalibrasyon verileri kaydedilemedi: {e}")
    
    def show_help(self):
        """Yardım menüsünü göster"""
        print("\n" + "="*70)
        print("X WING KALİBRASYON - KOMUT LİSTESİ")
        print("="*70)
        print("SİSTEM:")
        print("  help, h      - Bu yardım menüsü")
        print("  quit, q      - Çıkış")
        print("  status       - Sistem durumu")
        print("  save         - Kalibrasyon verilerini kaydet")
        print("  load         - Kalibrasyon verilerini yükle")
        print()
        print("SERVO KALİBRASYON:")
        print("  servo_cal <fin>     - Belirli fin'i kalibre et")
        print("  servo_test <fin>    - Belirli fin'i test et")
        print("  servo_limits <fin>  - Fin limitlerini ayarla")
        print("  servo_neutral <fin> - Fin nötr pozisyonunu ayarla")
        print("  servo_all_test      - Tüm servolar test")
        print()
        print("FİN İSİMLERİ (X Wing):")
        print("  upper_right  - Üst sağ fin")
        print("  upper_left   - Üst sol fin") 
        print("  lower_left   - Alt sol fin")
        print("  lower_right  - Alt sağ fin")
        print()
        print("PID KALİBRASYON:")
        print("  pid_tune <axis>     - PID manuel ayarlama")
        print("  pid_auto <axis>     - PID otomatik ayarlama")
        print("  pid_test <axis>     - PID yanıt testi")
        print("  pid_show <axis>     - PID değerlerini göster")
        print("    Eksenler: roll, pitch, yaw")
        print()
        print("MİXİNG MATRİSİ:")
        print("  mixing_show         - Mevcut mixing matrisini göster")
        print("  mixing_test         - Mixing matrisini test et")
        print("  mixing_calibrate    - Mixing matrisini kalibre et")
        print()
        print("TEST MODLARI:")
        print("  stability_test      - Stabilite testi (manuel hareket)")
        print("  response_test       - Yanıt süresi testi")
        print("  precision_test      - Hassasiyet testi")
        print("="*70)
    
    def calibrate_servo(self, fin_name: str):
        """Belirli servo kalibrasyonu"""
        if fin_name not in FinControlConfig.FINS:
            print(f"❌ Geçersiz fin adı: {fin_name}")
            return
        
        fin_config = FinControlConfig.FINS[fin_name]
        print(f"\n{fin_config['name']} Servo Kalibrasyonu")
        print("="*50)
        
        # Mevcut kalibrasyon verisi
        if fin_name in self.calibration_data.get("servo_limits", {}):
            current_cal = self.calibration_data["servo_limits"][fin_name]
            print(f"Mevcut kalibrasyon: Min={current_cal.get('min', 1000)}, "
                  f"Neutral={current_cal.get('neutral', 1500)}, "
                  f"Max={current_cal.get('max', 2000)}")
        
        print("\nKalibrasyon adımları:")
        print("1. Minimum pozisyon ayarı")
        print("2. Maksimum pozisyon ayarı") 
        print("3. Nötr pozisyon ayarı")
        print("4. Doğrulama testi")
        
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
        
        # 4. Doğrulama testi
        print(f"\n4. {fin_config['name']} Doğrulama Testi:")
        self._verify_servo_calibration(fin_name, servo_cal)
        
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
                print("Test süresi: 3 saniye")
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
    
    def _verify_servo_calibration(self, fin_name: str, servo_cal: dict):
        """Servo kalibrasyonunu doğrula"""
        print("Kalibrasyon doğrulaması yapılıyor...")
        
        test_sequence = [
            ("neutral", servo_cal.get("neutral", 1500)),
            ("min", servo_cal.get("min", 1000)),
            ("neutral", servo_cal.get("neutral", 1500)),
            ("max", servo_cal.get("max", 2000)),
            ("neutral", servo_cal.get("neutral", 1500))
        ]
        
        for pos_name, pwm_value in test_sequence:
            if pwm_value:
                print(f"  → {pos_name.capitalize()} pozisyon: PWM {pwm_value}")
                self.servo_controller.set_servo_position(fin_name, pwm_value)
                time.sleep(1.5)
        
        verify = input("Kalibrasyon doğru görünüyor mu? (e/h): ").lower()
        if verify == 'e':
            print("✅ Kalibrasyon doğrulandı")
        else:
            print("⚠️  Kalibrasyon tekrar gözden geçirilmeli")
    
    def tune_pid(self, axis: str):
        """Manuel PID ayarlama"""
        if axis not in self.stabilizer.pid_controller.controllers:
            print(f"❌ Geçersiz eksen: {axis}")
            return
        
        controller = self.stabilizer.pid_controller.controllers[axis]
        
        print(f"\n{axis.upper()} PID Ayarlama")
        print("="*40)
        
        # Mevcut değerleri göster
        current_gains = (controller.state.kp, controller.state.ki, controller.state.kd)
        print(f"Mevcut değerler: Kp={current_gains[0]}, Ki={current_gains[1]}, Kd={current_gains[2]}")
        
        # Yeni değerleri al
        try:
            print("\nYeni değerler (boş bırakırsanız mevcut değer korunur):")
            
            kp_str = input(f"Kp ({current_gains[0]}): ").strip()
            kp = float(kp_str) if kp_str else current_gains[0]
            
            ki_str = input(f"Ki ({current_gains[1]}): ").strip()
            ki = float(ki_str) if ki_str else current_gains[1]
            
            kd_str = input(f"Kd ({current_gains[2]}): ").strip()  
            kd = float(kd_str) if kd_str else current_gains[2]
            
            # PID değerlerini güncelle
            controller.set_gains(kp, ki, kd)
            
            # Kalibrasyon verilerine kaydet
            if "pid_values" not in self.calibration_data:
                self.calibration_data["pid_values"] = {}
            
            self.calibration_data["pid_values"][axis] = {
                "kp": kp, "ki": ki, "kd": kd
            }
            
            print(f"✅ {axis.upper()} PID değerleri güncellendi")
            
            # Test yapmak istiyor mu?
            test_it = input("Yeni değerlerle test yapmak ister misiniz? (e/h): ").lower()
            if test_it == 'e':
                self._test_pid_response(axis)
                
        except ValueError:
            print("❌ Geçersiz sayı formatı")
        except Exception as e:
            print(f"❌ PID ayarlama hatası: {e}")
    
    def _test_pid_response(self, axis: str):
        """PID yanıt testi"""
        print(f"\n{axis.upper()} PID Yanıt Testi")
        print("Pixhawk'ı hareket ettirin, PID yanıtını gözlemleyin")
        print("Test süresi: 15 saniye")
        print("-"*40)
        
        # Stabilizasyonu etkinleştir
        self.stabilizer.enable_stabilization(mode=1)
        
        start_time = time.time()
        max_error = 0
        error_sum = 0
        sample_count = 0
        
        try:
            while time.time() - start_time < 15:
                attitude = self.mav.get_attitude()
                
                # PID güncelle
                self.stabilizer.update_stabilization(attitude)
                
                # Hata analizi
                current_value = attitude.get(axis, 0)
                error = abs(current_value)  # Nötr pozisyondan sapma
                max_error = max(max_error, error)
                error_sum += error
                sample_count += 1
                
                # Durum göster
                remaining = 15 - (time.time() - start_time)
                print(f"Kalan: {remaining:4.1f}s | {axis.capitalize()}: {current_value:6.2f}° | "
                      f"Max Hata: {max_error:5.2f}°", end='\r')
                
                time.sleep(0.1)
            
            # Test sonuçları
            avg_error = error_sum / sample_count if sample_count > 0 else 0
            print(f"\n\nTest Sonuçları:")
            print(f"  Maksimum Hata: {max_error:.2f}°")
            print(f"  Ortalama Hata: {avg_error:.2f}°")
            
            if avg_error < 2.0:
                print("✅ İyi PID performansı")
            elif avg_error < 5.0:
                print("⚠️  Orta PID performansı")
            else:
                print("❌ PID ayarları optimize edilmeli")
                
        finally:
            self.stabilizer.disable_stabilization()
    
    def test_mixing_matrix(self):
        """Mixing matrisi testi"""
        print("\nMIXING MATRİSİ TESİ")
        print("X Wing çapraz fin kontrolü test ediliyor...")
        print("="*50)
        
        # Test hareketleri
        test_movements = [
            ("Roll Sağ", {"roll": 1.0, "pitch": 0, "yaw": 0}),
            ("Roll Sol", {"roll": -1.0, "pitch": 0, "yaw": 0}),
            ("Pitch Yukarı", {"roll": 0, "pitch": 1.0, "yaw": 0}),
            ("Pitch Aşağı", {"roll": 0, "pitch": -1.0, "yaw": 0}),
            ("Yaw Sağ", {"roll": 0, "pitch": 0, "yaw": 1.0}),
            ("Yaw Sol", {"roll": 0, "pitch": 0, "yaw": -1.0})
        ]
        
        for movement_name, control_input in test_movements:
            print(f"\n{movement_name} Testi:")
            
            # Mixing matrisini uygula
            mixed_outputs = self._apply_mixing_test(control_input)
            
            # Sonuçları göster
            print("Fin PWM Çıkışları:")
            for fin_name, pwm_output in mixed_outputs.items():
                fin_desc = FinControlConfig.FINS[fin_name]["name"]
                print(f"  {fin_desc}: {pwm_output}")
            
            # Servolar uygula
            self.servo_controller.set_multiple_servos(mixed_outputs)
            
            input("Devam etmek için Enter basın...")
        
        # Nötr pozisyona getir
        self.servo_controller.all_servos_neutral()
        print("✅ Mixing matrisi testi tamamlandı")
    
    def _apply_mixing_test(self, control_input):
        """Test için mixing matrisi uygulama"""
        mixed_outputs = {}
        
        for fin_name in FinControlConfig.FINS.keys():
            output_value = 1500  # Nötr başlangıç
            
            # Her kontrol ekseni için mixing uygula
            for axis, axis_value in control_input.items():
                if axis in FinMixingConfig.MIXING_MATRIX:
                    if fin_name in FinMixingConfig.MIXING_MATRIX[axis]:
                        contribution = (axis_value * 
                                      FinMixingConfig.MIXING_MATRIX[axis][fin_name] * 
                                      200)  # 200 PWM maksimum katkı
                        output_value += contribution
            
            # PWM limitlerini uygula
            mixed_outputs[fin_name] = max(1000, min(2000, int(output_value)))
        
        return mixed_outputs
    
    def stability_test(self):
        """Stabilite testi - manuel hareket ile"""
        print("\nSTABİLİTE TESİ")
        print("Pixhawk'ı manuel olarak hareket ettirin")
        print("Sistem stabilizasyon için finleri otomatik kontrole alacak")
        print("Test süresi: 30 saniye")
        print("-"*50)
        
        # Stabilizasyonu etkinleştir
        self.stabilizer.enable_stabilization(mode=1)
        
        stability_data = []
        start_time = time.time()
        
        try:
            while time.time() - start_time < 30:
                attitude = self.mav.get_attitude()
                
                # Stabilizasyon güncelle
                self.stabilizer.update_stabilization(attitude)
                
                # Stabilite verisi kaydet
                stability_metric = (attitude["roll"]**2 + attitude["pitch"]**2)**0.5
                stability_data.append(stability_metric)
                
                # Durum göster
                remaining = 30 - (time.time() - start_time)
                print(f"Kalan: {remaining:4.1f}s | Stabilite: {stability_metric:5.2f}°", end='\r')
                
                time.sleep(0.1)
            
            # Stabilite analizi
            if stability_data:
                avg_stability = sum(stability_data) / len(stability_data)
                max_deviation = max(stability_data)
                
                print(f"\n\nStabilite Test Sonuçları:")
                print(f"  Ortalama Sapma: {avg_stability:.2f}°")
                print(f"  Maksimum Sapma: {max_deviation:.2f}°")
                
                if avg_stability < 3.0:
                    print("✅ Mükemmel stabilite")
                elif avg_stability < 6.0:
                    print("✅ İyi stabilite")
                elif avg_stability < 10.0:
                    print("⚠️  Orta stabilite - PID ayarları iyileştirilebilir")
                else:
                    print("❌ Zayıf stabilite - sistem kalibrasyonu gerekli")
                    
        finally:
            self.stabilizer.disable_stabilization()
            self.servo_controller.all_servos_neutral()
    
    def process_command(self, command: str):
        """Komut işleme"""
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
            
            # Servo kalibrasyon komutları
            elif cmd == 'servo_cal':
                if len(parts) == 2:
                    self.calibrate_servo(parts[1])
                else:
                    print("❌ Kullanım: servo_cal <fin>")
                    
            elif cmd == 'servo_test':
                if len(parts) == 2:
                    fin_name = parts[1]
                    if fin_name in FinControlConfig.FINS:
                        self.servo_controller.test_servo_range(fin_name, 2.0)
                    else:
                        print(f"❌ Geçersiz fin: {fin_name}")
                else:
                    print("❌ Kullanım: servo_test <fin>")
                    
            elif cmd == 'servo_all_test':
                self.servo_controller.test_all_servos(2.0)
            
            # PID komutları
            elif cmd == 'pid_tune':
                if len(parts) == 2:
                    self.tune_pid(parts[1])
                else:
                    print("❌ Kullanım: pid_tune <axis>")
                    
            elif cmd == 'pid_test':
                if len(parts) == 2:
                    self._test_pid_response(parts[1])
                else:
                    print("❌ Kullanım: pid_test <axis>")
                    
            elif cmd == 'pid_show':
                if len(parts) == 2:
                    axis = parts[1]
                    if axis in self.stabilizer.pid_controller.controllers:
                        stats = self.stabilizer.pid_controller.controllers[axis].get_stats()
                        print(f"\n{axis.upper()} PID İstatistikleri:")
                        print(f"  Kp: {stats['kp']}, Ki: {stats['ki']}, Kd: {stats['kd']}")
                        print(f"  Setpoint: {stats['setpoint']}")
                        print(f"  Integral: {stats['integral']:.3f}")
                        print(f"  Son Çıkış: {stats['last_output']:.2f}")
                    else:
                        print(f"❌ Geçersiz eksen: {axis}")
                else:
                    print("❌ Kullanım: pid_show <axis>")
            
            # Mixing matrisi komutları
            elif cmd == 'mixing_show':
                print("\nX WING MIXİNG MATRİSİ:")
                for axis, fins in FinMixingConfig.MIXING_MATRIX.items():
                    print(f"{axis.upper()}:")
                    for fin_name, contribution in fins.items():
                        fin_desc = FinControlConfig.FINS[fin_name]["name"]
                        print(f"  {fin_desc}: {contribution:+.1f}")
                    print()
                    
            elif cmd == 'mixing_test':
                self.test_mixing_matrix()
            
            # Test komutları
            elif cmd == 'stability_test':
                self.stability_test()
                
            elif cmd == 'response_test':
                print("Yanıt süresi testi henüz implement edilmedi")
                
            elif cmd == 'precision_test':
                print("Hassasiyet testi henüz implement edilmedi")
            
            else:
                print(f"❌ Bilinmeyen komut: {cmd}")
                print("Yardım için 'help' yazın")
                
        except Exception as e:
            print(f"❌ Komut işleme hatası: {e}")
        
        return True
    
    def show_system_status(self):
        """Sistem durumunu göster"""
        print("\n" + "="*60)
        print("KALİBRASYON SİSTEMİ DURUMU")
        print("="*60)
        
        # Bağlantı durumları
        print(f"MAVLink: {'🟢 Bağlı' if self.mav and self.mav.is_connected() else '🔴 Kopuk'}")
        print(f"D300 Sensör: {'🟢 Aktif' if self.depth_sensor and self.depth_sensor.is_connected() else '🔴 Pasif'}")
        print(f"GPIO: {'🟢 Hazır' if self.gpio and self.gpio.setup_complete else '🔴 Hazır değil'}")
        
        # Kalibrasyon durumu
        print(f"\nKalibrasyon Verileri:")
        print(f"  Servo Limitler: {len(self.calibration_data.get('servo_limits', {}))} fin kalibre edildi")
        print(f"  PID Değerleri: {len(self.calibration_data.get('pid_values', {}))} eksen ayarlandı")
        print(f"  Son Kalibrasyon: {self.calibration_data.get('calibration_date', 'Henüz yok')}")
        
        # Servo durumları
        if hasattr(self.servo_controller, 'current_positions'):
            print(f"\nServo Pozisyonları:")
            for fin_name, position in self.servo_controller.get_current_positions().items():
                fin_desc = FinControlConfig.FINS[fin_name]["name"]
                print(f"  {fin_desc}: PWM {position}")
        
        print("="*60)
    
    def run_mission(self):
        """Ana kalibrasyon döngüsü"""
        if not self.setup_systems():
            return False
        
        self.running = True
        
        print("\nX Wing Kalibrasyon Sistemi Başlatıldı!")
        print("Yardım için 'help', çıkmak için 'quit' yazın")
        print("-"*60)
        
        try:
            while self.running:
                try:
                    command = input("\nKalibrasyon komutu: ").strip()
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
        """Sistem temizliği"""
        print("Sistem temizliği yapılıyor...")
        
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
        
        if self.depth_sensor:
            self.depth_sensor.stop_continuous_reading()
            self.depth_sensor.disconnect()
        
        if self.gpio:
            self.gpio.cleanup_gpio()
        
        print("✅ Kalibrasyon sistemi temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    mission = ManualMission2()
    
    try:
        mission.run_mission()
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")
    
    print("Kalibrasyon görevi sonlandı.")

if __name__ == "__main__":
    main()
