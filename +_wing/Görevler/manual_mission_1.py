"""
TEKNOFEST 2025 Su Altı Roket Aracı
+ Wing - Manuel Görev 1: İnteraktif Kontrol ve Test

Bu manuel görev + Wing konfigürasyonu ile interaktif kontrol sağlar.
Kullanıcı terminalden komutlar verebilir ve sistemi test edebilir.
+ Wing'in bağımsız eksen kontrol avantajı gösterilebilir.
"""

import os
import sys
import time
import threading
from typing import Dict, Any

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

class PlusWingManualMission1:
    """+ Wing Manuel Görev 1 - İnteraktif Kontrol"""
    
    def __init__(self):
        self.running = False
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        self.stabilizer = None
        
        # Kontrol durumu
        self.manual_mode = True
        self.stabilization_enabled = False
        self.current_motor_speed = 0
        self.data_monitoring = False
        
        # Veri monitörleme thread'i
        self._monitor_thread = None
        self._stop_monitor = threading.Event()
        
        print("+ Wing Manuel Görev 1 - İnteraktif Kontrol Sistemi")
        print("="*60)
        print("+ Wing Avantajı: Bağımsız pitch ve roll kontrolü!")
    
    def setup_systems(self):
        """Tüm sistemleri ayarla"""
        print("+ Wing sistemler ayarlanıyor...")
        
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
            print("✅ Servo kontrolcüsü hazır")
            
            # Stabilizasyon sistemi
            pid_configs = {
                "roll": plus_wing_config.PIDConfig.ROLL_PID,
                "pitch": plus_wing_config.PIDConfig.PITCH_PID,
                "yaw": plus_wing_config.PIDConfig.YAW_PID,
            }
            
            self.stabilizer = SubmarineStabilizer(
                pid_configs,
                plus_wing_config.FinMixingConfig.MIXING_MATRIX,
                plus_wing_config.FinMixingConfig.FIN_EFFECTIVENESS
            )
            
            # Callback'leri ayarla
            self.stabilizer.set_callbacks(self._servo_callback, self._motor_callback)
            print("✅ Stabilizasyon sistemi hazır")
            
            print("✅ + Wing tüm sistemler hazır!")
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
    
    def _motor_callback(self, motor_speed):
        """Motor çıkış callback'i"""
        self.mav.set_motor_speed(motor_speed)
        self.current_motor_speed = motor_speed
    
    def start_data_monitoring(self):
        """Veri monitörleme thread'ini başlat"""
        if self._monitor_thread and self._monitor_thread.is_alive():
            return
        
        self.data_monitoring = True
        self._stop_monitor.clear()
        self._monitor_thread = threading.Thread(target=self._data_monitor_loop)
        self._monitor_thread.daemon = True
        self._monitor_thread.start()
        print("📊 + Wing veri monitörleme başlatıldı")
    
    def stop_data_monitoring(self):
        """Veri monitörleme thread'ini durdur"""
        self.data_monitoring = False
        self._stop_monitor.set()
        if self._monitor_thread:
            self._monitor_thread.join(timeout=1)
        print("📊 Veri monitörleme durduruldu")
    
    def _data_monitor_loop(self):
        """Veri monitörleme döngüsü"""
        while not self._stop_monitor.is_set() and self.data_monitoring:
            try:
                # Sensör verilerini topla
                attitude = self.mav.get_attitude()
                distance = self.mav.get_distance()
                battery = self.mav.get_battery_status()
                
                # Terminal temizle ve + Wing spesifik verileri göster
                os.system('clear' if os.name == 'posix' else 'cls')
                
                print("="*70)
                print("+ WING MANUEL KONTROL - CANLI VERİ MONİTÖRLEME")
                print("="*70)
                print(f"⏰ Zaman: {time.strftime('%H:%M:%S')}")
                print("-"*70)
                print("ATTITUDE VERİLERİ (+ Wing Bağımsız Kontrol):")
                print(f"  Roll:  {attitude['roll']:8.2f}° (Sol/Sağ finler)")
                print(f"  Pitch: {attitude['pitch']:8.2f}° (Üst/Alt finler)")
                print(f"  Yaw:   {attitude['yaw']:8.2f}° (Tüm finler)")
                print("-"*70)
                print("SENSÖR VERİLERİ:")
                print(f"  Mesafe:   {distance:6.2f} m (Pixhawk)")
                print(f"  Batarya:  {battery['voltage']:5.1f}V ({battery['remaining']:3d}%)")
                print("-"*70)
                print("SİSTEM DURUMU:")
                print(f"  Motor Hızı:     {self.current_motor_speed:3d}%")
                print(f"  Stabilizasyon:  {'AÇIK' if self.stabilization_enabled else 'KAPALI'}")
                print(f"  Mod:           {'STABILIZE' if not self.manual_mode else 'MANUEL'}")
                print(f"  Konfigürasyon: + WING (Bağımsız Eksen)")
                print("-"*70)
                print("+ WING ÖZEL KOMUTLAR:")
                print("  pitch_test   - Sadece pitch kontrolü demo")
                print("  roll_test    - Sadece roll kontrolü demo") 
                print("  isolation    - Eksen bağımsızlığı demo")
                print("-"*70)
                print("KONTROLLER:")
                print("  'h' - Yardım    'q' - Çıkış    'm' - Monitör durdur")
                print("="*70)
                
                # Stabilizasyon aktifse güncelle
                if self.stabilization_enabled:
                    sensor_data = {
                        **attitude,
                        "distance": distance
                    }
                    self.stabilizer.update_stabilization(sensor_data)
                
                time.sleep(1)  # 1Hz güncelleme
                
            except Exception as e:
                print(f"Monitörleme hatası: {e}")
                time.sleep(1)
    
    def show_help(self):
        """+ Wing özel yardım menüsü"""
        print("\n" + "="*70)
        print("+ WING MANUEL KONTROL - KOMUT LİSTESİ")
        print("="*70)
        print("SİSTEM KONTROL:")
        print("  h, help     - Bu yardım menüsünü göster")
        print("  q, quit     - Programdan çık")
        print("  status      - Sistem durumunu göster")
        print("  monitor     - Canlı veri monitörlemeyi başlat/durdur")
        print()
        print("SERVO KONTROL (+ Wing Spesifik):")
        print("  servo test  - Tüm servoları test et")
        print("  servo <fin> <pwm> - Belirli finı belirli PWM'e ayarla")
        print("    + Wing Finler: upper, lower, left, right")
        print("    PWM: 1000-2000 arası")
        print("  neutral     - Tüm servoları nötr konuma getir")
        print()
        print("HAREKET KOMUTLARI (+ Wing Ortogonal):")
        print("  yukarı      - Yukarı hareket (sadece üst/alt finler)")
        print("  aşağı       - Aşağı hareket (sadece üst/alt finler)")
        print("  sola        - Sola hareket (sadece sol/sağ finler)")
        print("  sağa        - Sağa hareket (sadece sol/sağ finler)")
        print("  nötr        - Nötr pozisyon")
        print()
        print("+ WING ÖZEL TESTLER:")
        print("  pitch_test  - Yalnız pitch kontrol testi")
        print("  roll_test   - Yalnız roll kontrol testi")
        print("  isolation   - Eksen bağımsızlığı demonstrasyonu")
        print("  precision   - Hassas kontrol testi")
        print()
        print("MOTOR KONTROL:")
        print("  motor <hız> - Motor hızını ayarla (0-100%)")
        print("  stop        - Motoru durdur")
        print()
        print("STABİLİZASYON:")
        print("  stab on/off - Stabilizasyonu aç/kapat")
        print("  pid <eksen> - PID değerlerini göster (roll/pitch/yaw)")
        print()
        print("TEST KOMUTLARI:")
        print("  test attitude - Attitude verilerini test et")
        print("  test sensors  - Tüm sensörleri test et")
        print("  test plus_wing - + Wing spesifik testler")
        print("="*70)
    
    def test_pitch_only(self):
        """Yalnız pitch kontrol testi (+ Wing avantajı)"""
        print("\n+ WING PITCH KONTROL TESİ")
        print("Sadece üst ve alt finler hareket edecek, sol/sağ sabit kalacak!")
        print("-"*60)
        
        # Sadece pitch hareketleri
        pitch_tests = [
            ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
            ("Pitch Yukarı", {"upper": 1700, "lower": 1300, "left": 1500, "right": 1500}),
            ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
            ("Pitch Aşağı", {"upper": 1300, "lower": 1700, "left": 1500, "right": 1500}),
            ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
        ]
        
        for test_name, servo_positions in pitch_tests:
            print(f"🎯 {test_name}")
            print(f"   Üst: {servo_positions['upper']}, Alt: {servo_positions['lower']}")
            print(f"   Sol: {servo_positions['left']}, Sağ: {servo_positions['right']} (sabit)")
            
            self.servo_controller.set_multiple_servos(servo_positions)
            time.sleep(2.5)
        
        print("✅ + Wing pitch kontrol testi tamamlandı!")
    
    def test_roll_only(self):
        """Yalnız roll kontrol testi (+ Wing avantajı)"""
        print("\n+ WING ROLL KONTROL TESİ")
        print("Sadece sol ve sağ finler hareket edecek, üst/alt sabit kalacak!")
        print("-"*60)
        
        # Sadece roll hareketleri
        roll_tests = [
            ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
            ("Roll Sola", {"upper": 1500, "lower": 1500, "left": 1700, "right": 1300}),
            ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
            ("Roll Sağa", {"upper": 1500, "lower": 1500, "left": 1300, "right": 1700}),
            ("Nötr", {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
        ]
        
        for test_name, servo_positions in roll_tests:
            print(f"🎯 {test_name}")
            print(f"   Üst: {servo_positions['upper']}, Alt: {servo_positions['lower']} (sabit)")
            print(f"   Sol: {servo_positions['left']}, Sağ: {servo_positions['right']}")
            
            self.servo_controller.set_multiple_servos(servo_positions)
            time.sleep(2.5)
        
        print("✅ + Wing roll kontrol testi tamamlandı!")
    
    def test_axis_isolation(self):
        """Eksen bağımsızlığı demonstrasyonu"""
        print("\n+ WING EKSEN BAĞIMSIZLIĞI DEMONSTRASYONu")
        print("Roll ve pitch hareketlerinin birbirini etkilemediği gösterilecek!")
        print("-"*60)
        
        isolation_demo = [
            ("Başlangıç - Tümü Nötr", 
             {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
            
            ("Sadece Pitch (Roll etkilenmiyor)", 
             {"upper": 1600, "lower": 1400, "left": 1500, "right": 1500}),
             
            ("Pitch + Roll (Bağımsız kombinasyon)", 
             {"upper": 1600, "lower": 1400, "left": 1600, "right": 1400}),
             
            ("Sadece Roll (Pitch etkilenmiyor)",
             {"upper": 1500, "lower": 1500, "left": 1600, "right": 1400}),
             
            ("Karşıt Yönler (Pitch yukarı + Roll sol)", 
             {"upper": 1650, "lower": 1350, "left": 1650, "right": 1350}),
             
            ("Nötr - Demo Bitişi",
             {"upper": 1500, "lower": 1500, "left": 1500, "right": 1500}),
        ]
        
        for demo_name, servo_positions in isolation_demo:
            print(f"\n🌟 {demo_name}")
            print(f"   Üst: {servo_positions['upper']:<4} | Alt: {servo_positions['lower']:<4}")
            print(f"   Sol: {servo_positions['left']:<4} | Sağ: {servo_positions['right']:<4}")
            
            self.servo_controller.set_multiple_servos(servo_positions)
            time.sleep(3)
        
        print("\n✅ + Wing eksen bağımsızlığı demonstrasyonu tamamlandı!")
        print("🎉 + Wing'in avantajı: Pitch ve roll birbirini etkilemiyor!")
    
    def test_precision_control(self):
        """Hassas kontrol testi"""
        print("\n+ WING HASSASİYET TESTİ")
        print("+ Wing'in hassas kontrol kabiliyeti test ediliyor...")
        print("-"*60)
        
        # Çok hassas PWM adımları
        precision_values = [1500, 1505, 1510, 1515, 1520, 1515, 1510, 1505, 1500,
                           1495, 1490, 1485, 1480, 1485, 1490, 1495, 1500]
        
        print("Üst fin hassas hareket testi...")
        for i, pwm_val in enumerate(precision_values):
            precision_cmd = {
                "upper": pwm_val,
                "lower": 1500,
                "left": 1500, 
                "right": 1500
            }
            
            self.servo_controller.set_multiple_servos(precision_cmd)
            print(f"  Adım {i+1:2d}: {pwm_val} PWM", end='\r')
            time.sleep(0.4)
        
        print("\n✅ Hassasiyet testi tamamlandı!")
    
    def test_plus_wing_specific(self):
        """+ Wing spesifik test menüsü"""
        print("\n+ WING SPESİFİK TEST MENÜSÜ")
        print("="*50)
        print("1. Pitch Kontrol Testi")
        print("2. Roll Kontrol Testi")
        print("3. Eksen Bağımsızlığı Demo")
        print("4. Hassasiyet Testi")
        print("5. Tüm Testler")
        
        choice = input("\nSeçiminiz (1-5): ").strip()
        
        if choice == '1':
            self.test_pitch_only()
        elif choice == '2':
            self.test_roll_only()
        elif choice == '3':
            self.test_axis_isolation()
        elif choice == '4':
            self.test_precision_control()
        elif choice == '5':
            print("Tüm + Wing testleri çalıştırılıyor...")
            self.test_pitch_only()
            time.sleep(1)
            self.test_roll_only()
            time.sleep(1)
            self.test_axis_isolation()
            time.sleep(1)
            self.test_precision_control()
        else:
            print("Geçersiz seçim!")
    
    def process_command(self, command: str):
        """+ Wing komut işleme"""
        try:
            parts = command.strip().lower().split()
            if not parts:
                return True
            
            cmd = parts[0]
            
            # Sistem komutları
            if cmd in ['h', 'help']:
                self.show_help()
                
            elif cmd in ['q', 'quit']:
                print("Çıkılıyor...")
                return False
                
            elif cmd == 'status':
                self.show_system_status()
                
            elif cmd == 'monitor':
                if self.data_monitoring:
                    self.stop_data_monitoring()
                else:
                    self.start_data_monitoring()
                    print("Monitörlemeyi durdurmak için 'm' tuşuna basın")
                    input("Devam etmek için Enter...")
                    self.stop_data_monitoring()
            
            # Servo komutları
            elif cmd == 'servo':
                if len(parts) == 2 and parts[1] == 'test':
                    self.servo_controller.test_all_servos(2.0)
                elif len(parts) == 3:
                    fin_name = parts[1]
                    try:
                        pwm_value = int(parts[2])
                        self.servo_controller.set_servo_position(fin_name, pwm_value)
                        print(f"✅ {fin_name} servo PWM {pwm_value} olarak ayarlandı")
                    except ValueError:
                        print("❌ Geçersiz PWM değeri")
                else:
                    print("❌ Kullanım: servo test veya servo <fin> <pwm>")
            
            elif cmd == 'neutral':
                self.servo_controller.all_servos_neutral()
                print("✅ Tüm + Wing servolar nötr konuma getirildi")
            
            # + Wing hareket komutları
            elif cmd in plus_wing_config.FinControlConfig.MOVEMENT_COMMANDS:
                movement_cmd = plus_wing_config.FinControlConfig.MOVEMENT_COMMANDS[cmd]
                self.servo_controller.execute_movement_command(movement_cmd)
                print(f"✅ + Wing {cmd.capitalize()} hareketi çalıştırıldı")
            
            # + Wing özel testler
            elif cmd == 'pitch_test':
                self.test_pitch_only()
                
            elif cmd == 'roll_test':
                self.test_roll_only()
                
            elif cmd == 'isolation':
                self.test_axis_isolation()
                
            elif cmd == 'precision':
                self.test_precision_control()
            
            # Motor komutları
            elif cmd == 'motor':
                if len(parts) == 2:
                    try:
                        speed = int(parts[1])
                        self.mav.set_motor_speed(speed)
                        self.current_motor_speed = speed
                        print(f"✅ Motor hızı %{speed} olarak ayarlandı")
                    except ValueError:
                        print("❌ Geçersiz hız değeri")
                else:
                    print("❌ Kullanım: motor <hız>")
            
            elif cmd == 'stop':
                self.mav.set_motor_speed(0)
                self.current_motor_speed = 0
                print("✅ Motor durduruldu")
            
            # Stabilizasyon komutları
            elif cmd == 'stab':
                if len(parts) == 2:
                    if parts[1] == 'on':
                        self.stabilizer.enable_stabilization(mode=1)
                        self.stabilization_enabled = True
                        self.manual_mode = False
                        print("✅ + Wing stabilizasyon etkinleştirildi")
                    elif parts[1] == 'off':
                        self.stabilizer.disable_stabilization()
                        self.stabilization_enabled = False
                        self.manual_mode = True
                        print("✅ Stabilizasyon devre dışı bırakıldı")
                    else:
                        print("❌ Kullanım: stab on/off")
                else:
                    print("❌ Kullanım: stab on/off")
            
            # Test komutları
            elif cmd == 'test':
                if len(parts) == 2:
                    test_type = parts[1]
                    if test_type == 'attitude':
                        self.test_attitude_data()
                    elif test_type == 'sensors':
                        self.test_all_sensors()
                    elif test_type == 'plus_wing':
                        self.test_plus_wing_specific()
                    else:
                        print("❌ Bilinmeyen test türü")
                else:
                    print("❌ Kullanım: test <attitude|sensors|plus_wing>")
            
            else:
                print(f"❌ Bilinmeyen komut: {cmd}")
                print("Yardım için 'h' yazın")
                
        except Exception as e:
            print(f"❌ Komut işleme hatası: {e}")
        
        return True
    
    def show_system_status(self):
        """+ Wing sistem durumu"""
        print("\n" + "="*60)
        print("+ WING SİSTEM DURUMU")
        print("="*60)
        
        # Bağlantı durumları
        print(f"MAVLink: {'🟢 Bağlı' if self.mav and self.mav.is_connected() else '🔴 Kopuk'}")
        print(f"Konfigürasyon: 🔵 + Wing (Ortogonal Kontrol)")
        
        # Kontrol durumu
        print(f"Mod: {'🔵 Stabilize' if not self.manual_mode else '🟡 Manuel'}")
        print(f"Motor: %{self.current_motor_speed}")
        
        # + Wing servo durumları
        print(f"\n+ Wing Servo Durumları:")
        if hasattr(self.servo_controller, 'current_positions'):
            for fin_name, position in self.servo_controller.get_current_positions().items():
                fin_desc = plus_wing_config.FinControlConfig.FINS[fin_name]["name"]
                print(f"  {fin_desc}: PWM {position}")
        
        # + Wing avantajları
        print(f"\n🌟 + Wing Avantajları:")
        print(f"  ✅ Bağımsız Pitch Kontrolü (Üst/Alt finler)")
        print(f"  ✅ Bağımsız Roll Kontrolü (Sol/Sağ finler)")
        print(f"  ✅ Yüksek Stabilite (4 nokta kontrol)")
        print(f"  ✅ Hassas Manevrabilite")
        
        print("="*60)
    
    def test_attitude_data(self):
        """Attitude veri testi"""
        print("\n+ WING ATTITUDE VERİ TESİ (10 saniye)")
        print("-"*50)
        
        for i in range(100):  # 10 saniye, 10Hz
            attitude = self.mav.get_attitude()
            print(f"Roll: {attitude['roll']:6.1f}° (Sol/Sağ) | "
                  f"Pitch: {attitude['pitch']:6.1f}° (Üst/Alt) | "
                  f"Yaw: {attitude['yaw']:6.1f}° (Tümü)", end='\r')
            time.sleep(0.1)
        
        print("\n✅ + Wing attitude test tamamlandı")
    
    def test_all_sensors(self):
        """Tüm sensör testi"""
        print("\n+ WING SENSÖR TESİ")
        print("-"*40)
        
        # MAVLink sensörleri
        attitude = self.mav.get_attitude()
        distance = self.mav.get_distance()
        battery = self.mav.get_battery_status()
        
        print(f"Attitude: Roll={attitude['roll']:.1f}°, Pitch={attitude['pitch']:.1f}°, Yaw={attitude['yaw']:.1f}°")
        print(f"Mesafe: {distance:.2f} m (Pixhawk)")
        print(f"Batarya: {battery['voltage']:.1f}V (%{battery['remaining']})")
        
        print("✅ + Wing sensör testi tamamlandı")
    
    def run_mission(self):
        """Ana + Wing manuel görev döngüsü"""
        if not self.setup_systems():
            return False
        
        self.running = True
        
        print("\n+ Wing Manuel Kontrol Sistemi Başlatıldı!")
        print("+ Wing Avantajı: Bağımsız eksen kontrolü için özel komutlar!")
        print("Yardım için 'h', çıkmak için 'q' yazın")
        print("-"*60)
        
        try:
            while self.running:
                try:
                    command = input("\n+ Wing Komut: ").strip()
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
        print("+ Wing sistem temizliği yapılıyor...")
        
        self.running = False
        self.stop_data_monitoring()
        
        if self.stabilizer:
            self.stabilizer.disable_stabilization()
        
        if self.servo_controller:
            self.servo_controller.emergency_stop()
        
        if self.mav:
            self.mav.set_motor_speed(0)
            self.mav.disconnect()
        
        print("✅ + Wing sistem temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    mission = PlusWingManualMission1()
    
    try:
        mission.run_mission()
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")
    
    print("+ Wing manuel görev sonlandı.")

if __name__ == "__main__":
    main()
