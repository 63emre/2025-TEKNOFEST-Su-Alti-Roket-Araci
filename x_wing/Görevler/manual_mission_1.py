"""
TEKNOFEST 2025 Su Altı Roket Aracı
X Wing - Manuel Görev 1: İnteraktif Kontrol ve Test

Bu manuel görev X Wing konfigürasyonu ile interaktif kontrol sağlar.
Kullanıcı terminalden komutlar verebilir ve sistemi test edebilir.
"""

import os
import sys
import time
import threading
from typing import Dict, Any

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

class ManualMission1:
    """X Wing Manuel Görev 1 - İnteraktif Kontrol"""
    
    def __init__(self):
        self.running = False
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        self.stabilizer = None
        self.depth_sensor = None
        self.gpio = None
        
        # Kontrol durumu
        self.manual_mode = True
        self.stabilization_enabled = False
        self.current_motor_speed = 0
        self.data_monitoring = False
        
        # Veri monitörleme thread'i
        self._monitor_thread = None
        self._stop_monitor = threading.Event()
        
        print("X Wing Manuel Görev 1 - İnteraktif Kontrol Sistemi")
        print("="*60)
    
    def setup_systems(self):
        """Tüm sistemleri ayarla"""
        print("Sistemler ayarlanıyor...")
        
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
            
            # Derinlik sensörü
            self.depth_sensor = D300Sensor(
                bus_number=SensorConfig.D300_BUS,
                address=SensorConfig.D300_I2C_ADDRESS
            )
            if not self.depth_sensor.connect():
                print("⚠️  D300 sensörü bağlanamadı - devam ediliyor")
            else:
                self.depth_sensor.calibrate_surface_level()
                self.depth_sensor.start_continuous_reading()
                print("✅ D300 derinlik sensörü hazır")
            
            # Stabilizasyon sistemi
            pid_configs = {
                "roll": PIDConfig.ROLL_PID,
                "pitch": PIDConfig.PITCH_PID,
                "yaw": PIDConfig.YAW_PID,
                "depth": PIDConfig.DEPTH_PID
            }
            
            self.stabilizer = SubmarineStabilizer(
                pid_configs,
                FinMixingConfig.MIXING_MATRIX,
                FinMixingConfig.FIN_EFFECTIVENESS
            )
            
            # Callback'leri ayarla
            self.stabilizer.set_callbacks(self._servo_callback, self._motor_callback)
            print("✅ Stabilizasyon sistemi hazır")
            
            # Başlangıç sequence
            if self.gpio:
                self.gpio.startup_sequence()
            
            print("✅ Tüm sistemler hazır!")
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
        print("📊 Veri monitörleme başlatıldı")
    
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
                
                depth = 0.0
                if self.depth_sensor:
                    depth = self.depth_sensor.get_depth()
                
                # Terminal temizle ve verileri yazdır
                os.system('clear' if os.name == 'posix' else 'cls')
                
                print("="*70)
                print("X WING MANUEL KONTROL - CANLI VERİ MONİTÖRLEME")
                print("="*70)
                print(f"⏰ Zaman: {time.strftime('%H:%M:%S')}")
                print("-"*70)
                print("ATTITUDE VERİLERİ:")
                print(f"  Roll:  {attitude['roll']:8.2f}°")
                print(f"  Pitch: {attitude['pitch']:8.2f}°")
                print(f"  Yaw:   {attitude['yaw']:8.2f}°")
                print("-"*70)
                print("SENSÖR VERİLERİ:")
                print(f"  Derinlik: {depth:6.2f} m")
                print(f"  Mesafe:   {distance:6.2f} m")
                print("-"*70)
                print("SİSTEM DURUMU:")
                print(f"  Batarya:     {battery['voltage']:5.1f}V ({battery['remaining']:3d}%)")
                print(f"  Motor Hızı:  {self.current_motor_speed:3d}%")
                print(f"  Stabilizasyon: {'AÇIK' if self.stabilization_enabled else 'KAPALI'}")
                print(f"  Mod: {'STABILIZE' if not self.manual_mode else 'MANUEL'}")
                print("-"*70)
                print("KONTROLLER:")
                print("  'h' - Yardım    'q' - Çıkış    'm' - Monitör durdur")
                print("="*70)
                
                # Stabilizasyon aktifse güncelle
                if self.stabilization_enabled:
                    sensor_data = {
                        **attitude,
                        "depth": depth,
                        "distance": distance
                    }
                    self.stabilizer.update_stabilization(sensor_data)
                
                time.sleep(1)  # 1Hz güncelleme
                
            except Exception as e:
                print(f"Monitörleme hatası: {e}")
                time.sleep(1)
    
    def show_help(self):
        """Yardım menüsünü göster"""
        print("\n" + "="*70)
        print("X WING MANUEL KONTROL - KOMUT LİSTESİ")
        print("="*70)
        print("SİSTEM KONTROL:")
        print("  h, help     - Bu yardım menüsünü göster")
        print("  q, quit     - Programdan çık")
        print("  status      - Sistem durumunu göster")
        print("  monitor     - Canlı veri monitörlemeyi başlat/durdur")
        print()
        print("SERVO KONTROL:")
        print("  servo test  - Tüm servoları test et")
        print("  servo <fin> <pwm> - Belirli finı belirli PWM'e ayarla")
        print("    Finler: upper_right, upper_left, lower_left, lower_right")
        print("    PWM: 1000-2000 arası")
        print("  neutral     - Tüm servoları nötr konuma getir")
        print()
        print("HAREKET KOMUTLARI (X Wing):")
        print("  yukarı      - Yukarı hareket")
        print("  aşağı       - Aşağı hareket")
        print("  sola        - Sola hareket")
        print("  sağa        - Sağa hareket")
        print("  roll_sağ    - Sağa roll")
        print("  roll_sol    - Sola roll")
        print("  nötr        - Nötr pozisyon")
        print()
        print("MOTOR KONTROL:")
        print("  motor <hız> - Motor hızını ayarla (0-100%)")
        print("  stop        - Motoru durdur")
        print()
        print("STABİLİZASYON:")
        print("  stab on/off - Stabilizasyonu aç/kapat")
        print("  pid <eksen> - PID değerlerini göster")
        print("  setpoint <eksen> <değer> - PID hedefini ayarla")
        print()
        print("TEST KOMUTLARI:")
        print("  test attitude - Attitude verilerini test et")
        print("  test sensors  - Tüm sensörleri test et")
        print("  test movement - Hareket komutlarını test et")
        print("="*70)
    
    def process_command(self, command: str):
        """Komut işleme"""
        try:
            parts = command.strip().lower().split()
            if not parts:
                return
            
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
                print("✅ Tüm servolar nötr konuma getirildi")
            
            # Hareket komutları
            elif cmd in FinControlConfig.MOVEMENT_COMMANDS:
                movement_cmd = FinControlConfig.MOVEMENT_COMMANDS[cmd]
                self.servo_controller.execute_movement_command(movement_cmd)
                print(f"✅ {cmd.capitalize()} hareketi çalıştırıldı")
            
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
                        print("✅ Stabilizasyon etkinleştirildi")
                    elif parts[1] == 'off':
                        self.stabilizer.disable_stabilization()
                        self.stabilization_enabled = False
                        self.manual_mode = True
                        print("✅ Stabilizasyon devre dışı bırakıldı")
                    else:
                        print("❌ Kullanım: stab on/off")
                else:
                    print("❌ Kullanım: stab on/off")
            
            elif cmd == 'pid':
                if len(parts) == 2:
                    axis = parts[1]
                    if axis in self.stabilizer.pid_controller.controllers:
                        stats = self.stabilizer.pid_controller.controllers[axis].get_stats()
                        print(f"\n{axis.upper()} PID İstatistikleri:")
                        print(f"  Kp: {stats['kp']}, Ki: {stats['ki']}, Kd: {stats['kd']}")
                        print(f"  Setpoint: {stats['setpoint']:.2f}")
                        print(f"  Son Çıkış: {stats['last_output']:.2f}")
                        print(f"  Ortalama Hata: {stats['avg_error']:.3f}")
                    else:
                        print(f"❌ Bilinmeyen eksen: {axis}")
                else:
                    print("❌ Kullanım: pid <eksen>")
            
            # Test komutları
            elif cmd == 'test':
                if len(parts) == 2:
                    test_type = parts[1]
                    if test_type == 'attitude':
                        self.test_attitude_data()
                    elif test_type == 'sensors':
                        self.test_all_sensors()
                    elif test_type == 'movement':
                        self.test_movement_commands()
                    else:
                        print("❌ Bilinmeyen test türü")
                else:
                    print("❌ Kullanım: test <attitude|sensors|movement>")
            
            else:
                print(f"❌ Bilinmeyen komut: {cmd}")
                print("Yardım için 'h' yazın")
                
        except Exception as e:
            print(f"❌ Komut işleme hatası: {e}")
        
        return True
    
    def show_system_status(self):
        """Sistem durumunu göster"""
        print("\n" + "="*50)
        print("SİSTEM DURUMU")
        print("="*50)
        
        # MAVLink durumu
        print(f"MAVLink: {'🟢 Bağlı' if self.mav.is_connected() else '🔴 Kopuk'}")
        
        # Sensör durumları
        print(f"D300 Sensör: {'🟢 Aktif' if self.depth_sensor and self.depth_sensor.is_connected() else '🔴 Pasif'}")
        
        # GPIO durumu
        gpio_status = self.gpio.get_status() if self.gpio else {"setup_complete": False}
        print(f"GPIO: {'🟢 Hazır' if gpio_status['setup_complete'] else '🔴 Hazır değil'}")
        
        # Kontrol durumu
        print(f"Mod: {'🔵 Stabilize' if not self.manual_mode else '🟡 Manuel'}")
        print(f"Motor: %{self.current_motor_speed}")
        
        # Servo durumları
        print("\nServo Durumları:")
        for fin_name, position in self.servo_controller.get_current_positions().items():
            fin_desc = FinControlConfig.FINS[fin_name]["name"]
            print(f"  {fin_desc}: PWM {position}")
        
        print("="*50)
    
    def test_attitude_data(self):
        """Attitude verilerini test et"""
        print("\nATTITUDE VERİ TESİ (10 saniye)")
        print("-"*40)
        
        for i in range(100):  # 10 saniye, 10Hz
            attitude = self.mav.get_attitude()
            print(f"Roll: {attitude['roll']:6.1f}° | "
                  f"Pitch: {attitude['pitch']:6.1f}° | "
                  f"Yaw: {attitude['yaw']:6.1f}°", end='\r')
            time.sleep(0.1)
        
        print("\n✅ Attitude test tamamlandı")
    
    def test_all_sensors(self):
        """Tüm sensörleri test et"""
        print("\nTÜM SENSÖR TESİ")
        print("-"*40)
        
        # MAVLink sensörleri
        attitude = self.mav.get_attitude()
        distance = self.mav.get_distance()
        battery = self.mav.get_battery_status()
        
        print(f"Attitude: Roll={attitude['roll']:.1f}°, Pitch={attitude['pitch']:.1f}°, Yaw={attitude['yaw']:.1f}°")
        print(f"Mesafe: {distance:.2f} m")
        print(f"Batarya: {battery['voltage']:.1f}V (%{battery['remaining']})")
        
        # D300 sensörü
        if self.depth_sensor:
            depth_data = self.depth_sensor.get_all_data()
            print(f"Derinlik: {depth_data['depth_m']:.2f} m")
            print(f"Su Sıcaklığı: {depth_data['temp_celsius']:.1f}°C")
            print(f"Basınç: {depth_data['pressure_mbar']:.1f} mbar")
        
        print("✅ Sensör testi tamamlandı")
    
    def test_movement_commands(self):
        """Hareket komutlarını test et"""
        print("\nHAREKET KOMUTLARI TESİ")
        print("X Wing hareket komutları test ediliyor...")
        print("-"*40)
        
        movements = ["nötr", "yukarı", "nötr", "aşağı", "nötr", "sola", "nötr", "sağa", "nötr"]
        
        for movement in movements:
            if movement in FinControlConfig.MOVEMENT_COMMANDS:
                movement_cmd = FinControlConfig.MOVEMENT_COMMANDS[movement]
                self.servo_controller.execute_movement_command(movement_cmd)
                print(f"✅ {movement.capitalize()} hareketi")
                time.sleep(2)
        
        print("✅ Hareket komutları testi tamamlandı")
    
    def run_mission(self):
        """Ana görev döngüsü"""
        if not self.setup_systems():
            return False
        
        self.running = True
        
        print("\nX Wing Manuel Kontrol Sistemi Başlatıldı!")
        print("Yardım için 'h', çıkmak için 'q' yazın")
        print("-"*60)
        
        try:
            while self.running:
                try:
                    command = input("\nKomut: ").strip()
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
        self.stop_data_monitoring()
        
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
        
        print("✅ Sistem temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    mission = ManualMission1()
    
    try:
        mission.run_mission()
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")
    
    print("Manuel görev sonlandı.")

if __name__ == "__main__":
    main()
