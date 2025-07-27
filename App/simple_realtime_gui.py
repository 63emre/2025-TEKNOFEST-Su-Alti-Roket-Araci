#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Simple Real-Time GUI
TCP MAVLink Real-Time Data & Control (No Curses)
"""

import os
import sys
import time
import threading
import math
import json
from datetime import datetime

# Local imports
try:
    from mavlink_handler import MAVLinkHandler
    from depth_sensor import D300DepthSensor
except ImportError as e:
    print(f"❌ Import hatası: {e}")
    sys.exit(1)

class SimpleRealTimeGUI:
    def __init__(self):
        """Basit Real-Time GUI"""
        self.mavlink = None
        self.depth_sensor = None
        
        # Kontrol durumu
        self.armed = False
        self.running = True
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0
        
        # Live data
        self.live_imu = {
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'connected': False, 'update_count': 0
        }
        
        self.depth_data = {
            'depth_m': 0.0, 'temperature_c': 0.0, 'pressure_mbar': 0.0,
            'connected': False
        }
        
        # Thread control
        self.data_thread = None
        self.input_thread = None
        self.display_thread = None
        self.data_lock = threading.Lock()
        
        # Config
        self.load_config()
    
    def load_config(self):
        """Config yükle"""
        try:
            with open("config/hardware_config.json", 'r') as f:
                self.config = json.load(f)
        except:
            self.config = {
                "mavlink": {"connection_string": "tcp:127.0.0.1:5777"},
                "raspberry_pi": {"i2c": {"depth_sensor_address": "0x76", "bus_number": 1}}
            }
    
    def clear_screen(self):
        """Ekranı temizle"""
        os.system('clear' if os.name == 'posix' else 'cls')
    
    def init_systems(self):
        """Sistemleri başlat"""
        print("🚀 TEKNOFEST ROV - Simple Real-Time GUI başlatılıyor...")
        
        # MAVLink TCP bağlantısı
        try:
            self.mavlink = MAVLinkHandler()
            if self.mavlink.connect():
                print("✅ TCP MAVLink bağlantısı kuruldu (127.0.0.1:5777)!")
                self.live_imu['connected'] = True
            else:
                print("❌ TCP MAVLink bağlantısı başarısız!")
                return False
        except Exception as e:
            print(f"❌ MAVLink hatası: {e}")
            return False
        
        # Depth sensor (opsiyonel)
        try:
            self.depth_sensor = D300DepthSensor()
            if self.depth_sensor.connect():
                print("✅ I2C Depth sensörü bağlandı!")
                self.depth_data['connected'] = True
            else:
                print("⚠️ I2C Depth sensörü bağlanamadı (MAVLink'den denenecek)")
        except Exception as e:
            print(f"⚠️ Depth sensör hatası: {e}")
        
        return True
    
    def start_threads(self):
        """Thread'leri başlat"""
        # Data thread
        self.data_thread = threading.Thread(target=self.data_loop, daemon=True)
        self.data_thread.start()
        
        # Input thread  
        self.input_thread = threading.Thread(target=self.input_loop, daemon=True)
        self.input_thread.start()
        
        # Display thread
        self.display_thread = threading.Thread(target=self.display_loop, daemon=True)
        self.display_thread.start()
        
        print("✅ Tüm thread'ler başlatıldı!")
    
    def data_loop(self):
        """Veri güncelleme döngüsü - 20Hz"""
        while self.running:
            try:
                # IMU verilerini al
                if self.mavlink and self.mavlink.connected:
                    raw_imu = self.mavlink.get_imu_data()
                    if raw_imu:
                        with self.data_lock:
                            self.update_imu_data(raw_imu)
                            self.live_imu['update_count'] += 1
                
                # Depth verilerini al
                if self.depth_sensor:
                    try:
                        depth_reading = self.depth_sensor.read_data()
                        if depth_reading:
                            with self.data_lock:
                                self.depth_data.update(depth_reading)
                    except:
                        pass
                
                time.sleep(0.05)  # 20Hz
                
            except Exception as e:
                print(f"❌ Veri döngüsü hatası: {e}")
                time.sleep(0.1)
    
    def update_imu_data(self, raw_imu):
        """IMU verilerini güncelle"""
        try:
            accel_x, accel_y, accel_z = raw_imu[0], raw_imu[1], raw_imu[2]
            gyro_x, gyro_y, gyro_z = raw_imu[3], raw_imu[4], raw_imu[5]
            
            # Roll ve Pitch hesapla
            if abs(accel_z) > 0.001:
                roll_rad = math.atan2(accel_y, accel_z)
                pitch_rad = math.atan2(-accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))
                
                self.live_imu['roll'] = math.degrees(roll_rad)
                self.live_imu['pitch'] = math.degrees(pitch_rad)
            
            # YAW integration
            dt = 0.05  # 20Hz
            gyro_z_deg = math.degrees(gyro_z)
            if abs(gyro_z_deg) > 0.3:  # Noise threshold
                self.live_imu['yaw'] += gyro_z_deg * dt
                
                # Normalize yaw
                while self.live_imu['yaw'] > 180:
                    self.live_imu['yaw'] -= 360
                while self.live_imu['yaw'] < -180:
                    self.live_imu['yaw'] += 360
            
            self.live_imu['connected'] = True
            
        except Exception as e:
            print(f"❌ IMU güncelleme hatası: {e}")
    
    def input_loop(self):
        """Klavye girişi döngüsü"""
        print("\n📋 KONTROL KOMUTLARI:")
        print("  W/S: Pitch ±    A/D: Roll ±     Q/E: Yaw ±")
        print("  O/L: Motor ±    SPACE: ARM/DISARM")
        print("  R: RAW mod     F: PID mod     X: Çıkış")
        print("\n💡 Komut girin (ENTER ile onaylayın):")
        
        while self.running:
            try:
                command = input().lower().strip()
                
                if command == 'x':
                    self.running = False
                    break
                elif command == 'w':
                    self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + 5)
                    self.send_servo_commands()
                elif command == 's':
                    self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - 5)
                    self.send_servo_commands()
                elif command == 'a':
                    self.servo_values['roll'] = min(45, self.servo_values['roll'] + 5)
                    self.send_servo_commands()
                elif command == 'd':
                    self.servo_values['roll'] = max(-45, self.servo_values['roll'] - 5)
                    self.send_servo_commands()
                elif command == 'q':
                    self.servo_values['yaw'] = min(45, self.servo_values['yaw'] + 5)
                    self.send_servo_commands()
                elif command == 'e':
                    self.servo_values['yaw'] = max(-45, self.servo_values['yaw'] - 5)
                    self.send_servo_commands()
                elif command == 'o':
                    self.motor_value = min(100, self.motor_value + 10)
                    self.send_motor_command()
                elif command == 'l':
                    self.motor_value = max(-100, self.motor_value - 10)
                    self.send_motor_command()
                elif command == ' ':
                    self.toggle_arm()
                elif command == '0':
                    # Tümünü sıfırla
                    self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
                    self.motor_value = 0
                    self.send_servo_commands()
                    self.send_motor_command()
                    print("🔄 Tüm kontroller sıfırlandı!")
                
            except KeyboardInterrupt:
                self.running = False
                break
            except:
                pass
    
    def display_loop(self):
        """Ekran görüntüleme döngüsü - 2Hz"""
        while self.running:
            try:
                self.clear_screen()
                self.display_status()
                time.sleep(0.5)  # 2Hz display update
            except Exception as e:
                print(f"❌ Display hatası: {e}")
                time.sleep(1)
    
    def display_status(self):
        """Durum bilgilerini göster"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        
        print("="*80)
        print("🚀 TEKNOFEST ROV - REAL-TIME DATA & CONTROL")
        print(f"📅 {timestamp}     TCP: {'✅ BAĞLI' if self.live_imu['connected'] else '❌ BAĞLI DEĞİL'}")
        print(f"🔴 {'ARMED' if self.armed else 'DISARMED'}     Updates: {self.live_imu['update_count']}")
        print("="*80)
        
        # LIVE IMU DATA
        with self.data_lock:
            if self.live_imu['connected']:
                print("📊 LIVE IMU DATA:")
                print(f"  ROLL:  {self.live_imu['roll']:+8.1f}°")
                print(f"  PITCH: {self.live_imu['pitch']:+8.1f}°") 
                print(f"  YAW:   {self.live_imu['yaw']:+8.1f}°")
            else:
                print("📊 LIVE IMU DATA: ❌ TCP BAĞLANTISI YOK")
        
        print()
        
        # DEPTH DATA
        if self.depth_data['connected']:
            print("🌊 DEPTH SENSOR DATA:")
            print(f"  Derinlik:  {self.depth_data['depth_m']:6.2f} m")
            print(f"  Sıcaklık:  {self.depth_data['temperature_c']:6.1f} °C")
            print(f"  Basınç:    {self.depth_data['pressure_mbar']:6.1f} mbar")
        else:
            print("🌊 DEPTH SENSOR: ❌ I2C BAĞLANTISI YOK")
        
        print()
        
        # MANUAL CONTROL STATUS
        print("🎮 MANUEL KONTROL DURUMU:")
        print(f"  Roll:  {self.servo_values['roll']:+4.0f}° [A/D]")
        print(f"  Pitch: {self.servo_values['pitch']:+4.0f}° [W/S]")
        print(f"  Yaw:   {self.servo_values['yaw']:+4.0f}° [Q/E]")
        print(f"  Motor: {self.motor_value:+4.0f}% [O/L]")
        
        print()
        print("💡 Komutlar: W/S/A/D/Q/E (servo), O/L (motor), 0 (reset), X (çıkış)")
        print("-" * 80)
    
    def send_servo_commands(self):
        """Servo komutlarını gönder"""
        if not self.mavlink or not self.mavlink.connected:
            print("⚠️ MAVLink bağlantısı yok!")
            return
        
        if not self.armed:
            print("⚠️ Sistem DISARMED - servo komutları gönderilmiyor")
            return
        
        try:
            self.mavlink.control_servos_raw(
                self.servo_values['roll'],
                self.servo_values['pitch'], 
                self.servo_values['yaw']
            )
            print(f"✅ Servo: R={self.servo_values['roll']}° P={self.servo_values['pitch']}° Y={self.servo_values['yaw']}°")
        except Exception as e:
            print(f"❌ Servo komut hatası: {e}")
    
    def send_motor_command(self):
        """Motor komutunu gönder"""
        if not self.mavlink or not self.mavlink.connected:
            print("⚠️ MAVLink bağlantısı yok!")
            return
        
        if not self.armed:
            print("⚠️ Sistem DISARMED - motor komutları gönderilmiyor")
            return
        
        try:
            # Motor PWM hesapla
            pwm_limits = self.config.get("pixhawk", {}).get("pwm_limits", {})
            neutral = pwm_limits.get("motor_stop", 1500)
            pwm_range = 400  # ±400 range
            
            pwm_value = neutral + (self.motor_value * pwm_range // 100)
            pwm_value = max(1100, min(1900, pwm_value))
            
            self.mavlink.send_raw_motor_pwm(int(pwm_value))
            print(f"✅ Motor: {self.motor_value}% (PWM: {pwm_value})")
        except Exception as e:
            print(f"❌ Motor komut hatası: {e}")
    
    def toggle_arm(self):
        """ARM/DISARM toggle"""
        if not self.mavlink or not self.mavlink.connected:
            print("❌ TCP MAVLink bağlantısı yok!")
            return
        
        try:
            if self.armed:
                if self.mavlink.disarm_system():
                    self.armed = False
                    # Reset controls
                    self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
                    self.motor_value = 0
                    print("🟢 Sistem DISARM edildi!")
                else:
                    print("❌ DISARM başarısız!")
            else:
                if self.mavlink.arm_system():
                    self.armed = True
                    print("🔴 Sistem ARM edildi - DİKKAT!")
                    print("⚠️ Servo ve motor komutları artık aktif!")
                else:
                    print("❌ ARM başarısız!")
        except Exception as e:
            print(f"❌ ARM/DISARM hatası: {e}")
    
    def cleanup(self):
        """Temizlik"""
        print("\n🔄 Sistem kapatılıyor...")
        self.running = False
        
        if self.mavlink:
            try:
                self.mavlink.emergency_stop()
                self.mavlink.disconnect()
            except:
                pass
        
        if self.depth_sensor:
            try:
                self.depth_sensor.disconnect()
            except:
                pass
        
        print("✅ Sistem temizlendi!")
    
    def run(self):
        """Ana program"""
        try:
            if not self.init_systems():
                print("❌ Sistem başlatılamadı!")
                return
            
            self.start_threads()
            
            # Ana döngü
            while self.running:
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n👋 Kullanıcı tarafından durduruldu!")
        except Exception as e:
            print(f"❌ Kritik hata: {e}")
        finally:
            self.cleanup()

if __name__ == "__main__":
    print("🚀 TEKNOFEST ROV - Simple Real-Time GUI başlatılıyor...")
    
    # Çalışma dizini kontrolü
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı! App/ klasörünün içinden çalıştırın.")
        sys.exit(1)
    
    # GUI'yi başlat
    gui = SimpleRealTimeGUI()
    gui.run() 