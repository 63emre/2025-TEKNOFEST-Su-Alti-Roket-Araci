#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - D300 Derinlik Kontrolü
X-Wing Konfigürasyonu ile Stabilize Derinlik Değişimi
Araç Ağırlığı: 4-4.5 kg için optimize edilmiş PID değerleri

Hardware:
- D300 Derinlik Sensörü (I2C: 0x76)
- X-Wing Fin Konfigürasyonu (AUX1,3,4,5)
- GPIO 23: Kontrol LED'i
- Ana Motor: AUX6
"""

import time
import threading
import math
import json
import os
import sys
from datetime import datetime
from pymavlink import mavutil

# D300 Depth Sensor - I2C Support
try:
    import smbus2
    HAS_I2C = True
    print("✅ I2C support loaded for D300")
except ImportError:
    HAS_I2C = False
    print("⚠️ I2C not available - using MAVLink depth data")

# GPIO Support for LED
try:
    import RPi.GPIO as GPIO
    HAS_GPIO = True
    print("✅ GPIO support loaded")
except ImportError:
    try:
        from gpiozero import LED
        HAS_GPIOZERO = True
        HAS_GPIO = False
        print("✅ gpiozero support loaded (Pi5 compatible)")
    except ImportError:
        HAS_GPIO = False
        HAS_GPIOZERO = False
        print("⚠️ GPIO not available")

# MAVLink Connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

# Hardware Configuration - X-Wing Setup
SERVO_CHANNELS = {
    'front_left': 9,   # AUX1 - Ön Sol Fin
    'front_right': 11, # AUX3 - Ön Sağ Fin  
    'rear_left': 12,   # AUX4 - Arka Sol Fin
    'rear_right': 13,  # AUX5 - Arka Sağ Fin
    'main_motor': 14   # AUX6 - Ana Motor
}

# D300 Sensor Configuration
D300_I2C_BUS = 1
D300_I2C_ADDRESS = 0x76  # Kullanıcı tarafından belirtilen adres
GPIO_LED_PIN = 23        # Kullanıcı tarafından belirtilen LED pin

# PWM Limits
PWM_MIN = 1000
PWM_NEUTRAL = 1500
PWM_MAX = 2000

# PID Configuration - 4-4.5kg araç için optimize edilmiş
DEPTH_PID = {
    'kp': 1.2,    # Proportional gain - ağır araç için artırıldı
    'ki': 0.05,   # Integral gain - yavaş artış için
    'kd': 0.25,   # Derivative gain - stabilite için
    'max_integral': 100.0,
    'max_output': 400     # PWM değişim limiti
}

# Stabilization PID - X-Wing konfigürasyonu için
STABILIZATION_PID = {
    'roll': {'kp': 1.0, 'ki': 0.03, 'kd': 0.15},
    'pitch': {'kp': 1.0, 'ki': 0.03, 'kd': 0.15},
    'yaw': {'kp': 0.9, 'ki': 0.02, 'kd': 0.12}
}

# Movement Settings
DEPTH_CHANGE_RATE = 0.5  # m/s maksimum derinlik değişim hızı
STABILIZATION_FREQUENCY = 50  # Hz

class D300DepthSensor:
    """D300 Derinlik Sensörü I2C Kontrolü"""
    
    def __init__(self, bus_num=D300_I2C_BUS, address=D300_I2C_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self.connected = False
        self.depth_m = 0.0
        self.temperature_c = 20.0
        self.surface_pressure = 1013.25  # mbar
        
    def connect(self):
        """D300 sensörüne bağlan"""
        if not HAS_I2C:
            print("⚠️ I2C desteği yok - simülasyon modu")
            self.connected = True
            return True
            
        try:
            print(f"🔌 D300 sensörüne bağlanılıyor (I2C: 0x{self.address:02x})")
            self.bus = smbus2.SMBus(self.bus_num)
            
            # Test okuma
            test_data = self.bus.read_byte(self.address)
            self.connected = True
            print(f"✅ D300 sensörü bağlandı (Test data: 0x{test_data:02x})")
            return True
            
        except Exception as e:
            print(f"❌ D300 bağlantı hatası: {e}")
            print("💡 Simülasyon modunda devam ediliyor...")
            self.connected = True  # Simülasyon için
            return True
    
    def read_depth(self):
        """Derinlik verisi oku"""
        if not self.connected or not HAS_I2C:
            # Simülasyon - gerçekçi veri üret
            import random
            self.depth_m = max(0, self.depth_m + random.uniform(-0.05, 0.05))
            self.temperature_c = 20.0 + random.uniform(-2, 2)
            return self.depth_m, self.temperature_c
            
        try:
            # D300 için gerçek okuma protokolü
            # Bu bölüm gerçek D300 protokolüne göre güncellenmeli
            pressure_raw = self.bus.read_word_data(self.address, 0x00)
            temp_raw = self.bus.read_word_data(self.address, 0x02)
            
            # Basınçtan derinlik hesapla (1 mbar ≈ 1 cm su)
            pressure_mbar = pressure_raw / 100.0  # Raw'dan mbar'a çevir
            self.depth_m = max(0, (pressure_mbar - self.surface_pressure) / 100.0)
            
            # Sıcaklık hesapla
            self.temperature_c = temp_raw / 100.0  # Raw'dan derece'ye çevir
            
            return self.depth_m, self.temperature_c
            
        except Exception as e:
            print(f"⚠️ D300 okuma hatası: {e}")
            return self.depth_m, self.temperature_c
    
    def calibrate_surface(self):
        """Yüzey basıncını kalibre et"""
        print("🔧 Yüzey kalibrasyonu yapılıyor...")
        readings = []
        
        for i in range(10):
            if HAS_I2C and self.connected:
                try:
                    pressure_raw = self.bus.read_word_data(self.address, 0x00)
                    pressure_mbar = pressure_raw / 100.0
                    readings.append(pressure_mbar)
                except:
                    readings.append(1013.25)
            else:
                readings.append(1013.25)
            time.sleep(0.1)
        
        self.surface_pressure = sum(readings) / len(readings)
        print(f"✅ Yüzey basıncı: {self.surface_pressure:.2f} mbar")
        return True

class XWingDepthController:
    """X-Wing Konfigürasyonlu Derinlik Kontrolü"""
    
    def __init__(self):
        # MAVLink bağlantısı
        self.master = None
        self.connected = False
        
        # D300 Sensörü
        self.depth_sensor = D300DepthSensor()
        
        # GPIO LED
        self.led = None
        self.gpio_initialized = False
        
        # Navigation data
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Target depth
        self.target_depth = 0.0
        
        # PID Controllers
        self.depth_pid = PIDController(**DEPTH_PID)
        self.roll_pid = PIDController(**STABILIZATION_PID['roll'])
        self.pitch_pid = PIDController(**STABILIZATION_PID['pitch'])
        self.yaw_pid = PIDController(**STABILIZATION_PID['yaw'])
        
        # Current servo positions
        self.servo_positions = {channel: PWM_NEUTRAL for channel in SERVO_CHANNELS.values()}
        
        # Control state
        self.running = False
        self.control_thread = None
        
        print("🚀 X-Wing Derinlik Kontrolü başlatıldı")
        print(f"📏 D300 Sensör Adresi: 0x{D300_I2C_ADDRESS:02x}")
        print(f"💡 LED Pin: GPIO {GPIO_LED_PIN}")
        
    def setup_gpio(self):
        """GPIO LED'ini başlat"""
        if HAS_GPIO:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(GPIO_LED_PIN, GPIO.OUT)
                GPIO.output(GPIO_LED_PIN, GPIO.LOW)
                self.gpio_initialized = True
                print(f"✅ GPIO {GPIO_LED_PIN} LED başlatıldı")
                return True
            except Exception as e:
                print(f"⚠️ GPIO hatası: {e}")
                
        elif HAS_GPIOZERO:
            try:
                from gpiozero import LED
                self.led = LED(GPIO_LED_PIN)
                self.led.off()
                self.gpio_initialized = True
                print(f"✅ gpiozero GPIO {GPIO_LED_PIN} LED başlatıldı")
                return True
            except Exception as e:
                print(f"⚠️ gpiozero hatası: {e}")
        
        print("⚠️ GPIO desteği yok - LED simülasyon modu")
        return False
    
    def set_led(self, state):
        """LED'i aç/kapat"""
        if not self.gpio_initialized:
            return
            
        try:
            if HAS_GPIO:
                GPIO.output(GPIO_LED_PIN, GPIO.HIGH if state else GPIO.LOW)
            elif HAS_GPIOZERO and self.led:
                if state:
                    self.led.on()
                else:
                    self.led.off()
                    
            print(f"💡 LED: {'AÇIK' if state else 'KAPALI'}")
        except Exception as e:
            print(f"⚠️ LED kontrol hatası: {e}")
    
    def connect_pixhawk(self):
        """Pixhawk'a bağlan"""
        try:
            print(f"📡 Pixhawk bağlantısı: {MAV_ADDRESS}@{MAV_BAUD}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS, baud=MAV_BAUD)
            self.master.wait_heartbeat(timeout=10)
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı")
            return True
        except Exception as e:
            print(f"❌ MAVLink bağlantı hatası: {e}")
            return False
    
    def get_telemetry(self):
        """Telemetri verilerini al"""
        if not self.connected:
            return
            
        try:
            # IMU verilerini al
            msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                self.current_roll = math.degrees(msg.roll)
                self.current_pitch = math.degrees(msg.pitch)
                self.current_yaw = math.degrees(msg.yaw)
            
            # Derinlik verisi - D300'den al
            depth, temp = self.depth_sensor.read_depth()
            self.current_depth = depth
            
        except Exception as e:
            print(f"⚠️ Telemetri hatası: {e}")
    
    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        if not self.connected:
            return
            
        pwm_value = max(PWM_MIN, min(PWM_MAX, int(pwm_value)))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm_value, 0, 0, 0, 0, 0
            )
            self.servo_positions[channel] = pwm_value
        except Exception as e:
            print(f"⚠️ Servo {channel} ayar hatası: {e}")
    
    def calculate_x_wing_control(self, depth_output, roll_output, pitch_output, yaw_output):
        """X-Wing konfigürasyonu için servo değerlerini hesapla"""
        # Base PWM değerleri
        front_left = PWM_NEUTRAL
        front_right = PWM_NEUTRAL
        rear_left = PWM_NEUTRAL
        rear_right = PWM_NEUTRAL
        motor = PWM_NEUTRAL
        
        # Derinlik kontrolü - tüm finlerde pitch etkisi
        pitch_component = pitch_output + (depth_output * 0.3)
        
        # X-Wing konfigürasyonu
        front_left += pitch_component + roll_output + (yaw_output * 0.5)
        front_right += pitch_component - roll_output - (yaw_output * 0.5)
        rear_left += -pitch_component + roll_output - (yaw_output * 0.5)
        rear_right += -pitch_component - roll_output + (yaw_output * 0.5)
        
        # Motor - derinlik kontrolü için
        motor += depth_output * 0.1  # Çok hafif motor desteği
        
        return {
            SERVO_CHANNELS['front_left']: front_left,
            SERVO_CHANNELS['front_right']: front_right,
            SERVO_CHANNELS['rear_left']: rear_left,
            SERVO_CHANNELS['rear_right']: rear_right,
            SERVO_CHANNELS['main_motor']: motor
        }
    
    def control_loop(self):
        """Ana kontrol döngüsü"""
        dt = 1.0 / STABILIZATION_FREQUENCY
        
        while self.running:
            start_time = time.time()
            
            # Telemetri al
            self.get_telemetry()
            
            # PID hesaplamaları
            depth_error = self.target_depth - self.current_depth
            depth_output = self.depth_pid.update(depth_error, dt)
            
            # Stabilizasyon PID'leri
            roll_output = self.roll_pid.update(-self.current_roll, dt)  # Roll'u sıfırda tut
            pitch_output = self.pitch_pid.update(-self.current_pitch, dt)  # Pitch'i sıfırda tut
            yaw_output = self.yaw_pid.update(0, dt)  # Yaw değişimini minimal tut
            
            # X-Wing servo kontrolü
            servo_commands = self.calculate_x_wing_control(
                depth_output, roll_output, pitch_output, yaw_output
            )
            
            # Servo komutlarını gönder
            for channel, pwm in servo_commands.items():
                self.set_servo_pwm(channel, pwm)
            
            # Status gösterimi
            if int(time.time()) % 2 == 0:  # 2 saniyede bir
                print(f"🌊 Hedef: {self.target_depth:.2f}m | Mevcut: {self.current_depth:.2f}m | "
                      f"Hata: {depth_error:.2f}m | PID: {depth_output:.1f}")
            
            # LED kontrolü - hedef derinliğe yakınsa yanık
            led_state = abs(depth_error) < 0.2  # 20cm tolerans
            self.set_led(led_state)
            
            # Timing
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def set_target_depth(self, depth_input):
        """Hedef derinliği ayarla (metre veya cm)"""
        try:
            # Input kontrolü
            if isinstance(depth_input, str):
                depth_str = depth_input.lower().strip()
                
                if 'cm' in depth_str:
                    # Santimetre girişi
                    depth_val = float(depth_str.replace('cm', '').strip())
                    target_depth = depth_val / 100.0  # cm'yi m'ye çevir
                elif 'm' in depth_str:
                    # Metre girişi
                    depth_val = float(depth_str.replace('m', '').strip())
                    target_depth = depth_val
                else:
                    # Sadece sayı - metre olarak kabul et
                    target_depth = float(depth_str)
            else:
                # Direkt sayısal değer
                target_depth = float(depth_input)
            
            # Güvenlik limitleri
            target_depth = max(0.0, min(10.0, target_depth))  # 0-10m arası
            
            print(f"🎯 Yeni hedef derinlik: {target_depth:.2f}m")
            
            # Yavaş değişim için kademeli hedef güncelleme
            if abs(target_depth - self.target_depth) > 0.5:
                steps = int(abs(target_depth - self.target_depth) / 0.5)
                step_size = (target_depth - self.target_depth) / steps
                
                for i in range(steps):
                    self.target_depth += step_size
                    time.sleep(1.0 / DEPTH_CHANGE_RATE)  # Yavaş değişim
            else:
                self.target_depth = target_depth
                
            return True
            
        except ValueError as e:
            print(f"❌ Geçersiz derinlik değeri: {depth_input}")
            print("💡 Örnek kullanım: '2.5m', '150cm', '3.0'")
            return False
    
    def start_control(self):
        """Kontrol sistemini başlat"""
        if self.running:
            print("⚠️ Kontrol sistemi zaten çalışıyor")
            return False
            
        # GPIO başlat
        self.setup_gpio()
        
        # Pixhawk bağlantısı
        if not self.connect_pixhawk():
            return False
            
        # D300 sensörü başlat
        if not self.depth_sensor.connect():
            return False
            
        # Yüzey kalibrasyonu
        self.depth_sensor.calibrate_surface()
        
        # Kontrol döngüsünü başlat
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("✅ Derinlik kontrol sistemi başlatıldı")
        print("💡 Kullanım: set_target_depth('2.5m') veya set_target_depth('150cm')")
        self.set_led(True)  # Başlangıç sinyali
        time.sleep(1)
        self.set_led(False)
        
        return True
    
    def stop_control(self):
        """Kontrol sistemini durdur"""
        print("🛑 Kontrol sistemi durduruluyor...")
        self.running = False
        
        if self.control_thread:
            self.control_thread.join(timeout=2)
        
        # Servolar nötr pozisyona
        for channel in SERVO_CHANNELS.values():
            self.set_servo_pwm(channel, PWM_NEUTRAL)
        
        # LED'i kapat
        self.set_led(False)
        
        # GPIO temizle
        if HAS_GPIO and self.gpio_initialized:
            GPIO.cleanup()
        
        print("✅ Kontrol sistemi durduruldu")

class PIDController:
    """PID Kontrolör"""
    
    def __init__(self, kp, ki, kd, max_integral=100, max_output=400):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
        self.max_output = max_output
        
        self.prev_error = 0
        self.integral = 0
    
    def update(self, error, dt):
        # Proportional
        proportional = self.kp * error
        
        # Integral
        self.integral += error * dt
        self.integral = max(-self.max_integral, min(self.max_integral, self.integral))
        integral = self.ki * self.integral
        
        # Derivative
        derivative = self.kd * (error - self.prev_error) / dt if dt > 0 else 0
        self.prev_error = error
        
        # Total output
        output = proportional + integral + derivative
        output = max(-self.max_output, min(self.max_output, output))
        
        return output
    
    def reset(self):
        self.prev_error = 0
        self.integral = 0

def main():
    """Ana program"""
    print("🚀 TEKNOFEST D300 Derinlik Kontrolü")
    print("=" * 50)
    
    controller = XWingDepthController()
    
    try:
        # Sistemi başlat
        if not controller.start_control():
            print("❌ Sistem başlatılamadı")
            return
        
        print("\n📋 Komutlar:")
        print("  Hedef derinlik ayarla: '2.5m', '150cm', '3.0'")
        print("  LED aç/kapat: 'led on' / 'led off'")
        print("  Mevcut durum: 'status'")
        print("  Çıkış: 'quit' veya Ctrl+C")
        print()
        
        # Interaktif kontrol
        while True:
            try:
                user_input = input("💻 Komut: ").strip()
                
                if user_input.lower() in ['quit', 'exit', 'q']:
                    break
                elif user_input.lower() == 'status':
                    print(f"🌊 Mevcut derinlik: {controller.current_depth:.2f}m")
                    print(f"🎯 Hedef derinlik: {controller.target_depth:.2f}m")
                    print(f"📐 Roll: {controller.current_roll:.1f}° | "
                          f"Pitch: {controller.current_pitch:.1f}° | "
                          f"Yaw: {controller.current_yaw:.1f}°")
                elif user_input.lower() == 'led on':
                    controller.set_led(True)
                elif user_input.lower() == 'led off':
                    controller.set_led(False)
                elif user_input:
                    # Derinlik komutu olarak dene
                    controller.set_target_depth(user_input)
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"⚠️ Komut hatası: {e}")
        
    except KeyboardInterrupt:
        print("\n⚠️ Kullanıcı tarafından durduruldu")
    finally:
        controller.stop_control()
        print("👋 Program sonlandırıldı")

if __name__ == "__main__":
    main() 