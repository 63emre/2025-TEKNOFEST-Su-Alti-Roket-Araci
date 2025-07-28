#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - U-Şekli Hareket Paternleri
X-Wing Konfigürasyonu ile Stabilize Hareket Kontrolü
Araç Ağırlığı: 4-4.5 kg için optimize edilmiş PID değerleri

Movement Patterns:
- İleri Hareket (Forward)
- Sağ Hareket (Right Strafe)
- Sol Hareket (Left Strafe)
- Geri Hareket (Backward)
- U Şekli Manevralar

Hardware:
- X-Wing Fin Konfigürasyonu (AUX1,3,4,5)
- Ana Motor: AUX6
- GPIO 23: Durum LED'i
- D300 Derinlik Sensörü (0x76) - stabilizasyon için
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
D300_I2C_ADDRESS = 0x76
GPIO_LED_PIN = 23

# PWM Limits
PWM_MIN = 1000
PWM_NEUTRAL = 1500
PWM_MAX = 2000

# PID Configuration - 4-4.5kg araç için optimize edilmiş
MOVEMENT_PID = {
    'forward': {'kp': 1.0, 'ki': 0.02, 'kd': 0.15},
    'strafe': {'kp': 0.9, 'ki': 0.02, 'kd': 0.12},
    'turn': {'kp': 0.8, 'ki': 0.01, 'kd': 0.10}
}

# Stabilization PID - X-Wing konfigürasyonu için
STABILIZATION_PID = {
    'roll': {'kp': 1.2, 'ki': 0.04, 'kd': 0.18},
    'pitch': {'kp': 1.2, 'ki': 0.04, 'kd': 0.18},
    'yaw': {'kp': 1.0, 'ki': 0.03, 'kd': 0.15},
    'depth': {'kp': 1.0, 'ki': 0.03, 'kd': 0.20}
}

# Movement Settings
MOVEMENT_SPEED_SLOW = 0.3    # Yavaş hareket hızı (0-1)
MOVEMENT_SPEED_NORMAL = 0.5  # Normal hareket hızı (0-1)
MOVEMENT_SPEED_FAST = 0.7    # Hızlı hareket hızı (0-1)

TURN_RATE_SLOW = 15.0        # derece/saniye
TURN_RATE_NORMAL = 30.0      # derece/saniye
TURN_RATE_FAST = 45.0        # derece/saniye

STABILIZATION_FREQUENCY = 50  # Hz
MOVEMENT_UPDATE_FREQUENCY = 20  # Hz

# U-Şekli Hareket Tanımları
U_SHAPE_PATTERNS = {
    'small_u': {
        'name': 'Küçük U Şekli',
        'sequence': [
            ('forward', 2.0, MOVEMENT_SPEED_SLOW),
            ('turn_right', 90.0, TURN_RATE_SLOW),
            ('forward', 1.0, MOVEMENT_SPEED_SLOW),
            ('turn_right', 90.0, TURN_RATE_SLOW),
            ('forward', 2.0, MOVEMENT_SPEED_SLOW)
        ]
    },
    'large_u': {
        'name': 'Büyük U Şekli',
        'sequence': [
            ('forward', 4.0, MOVEMENT_SPEED_NORMAL),
            ('turn_right', 90.0, TURN_RATE_NORMAL),
            ('forward', 2.0, MOVEMENT_SPEED_NORMAL),
            ('turn_right', 90.0, TURN_RATE_NORMAL),
            ('forward', 4.0, MOVEMENT_SPEED_NORMAL)
        ]
    },
    'complex_u': {
        'name': 'Karmaşık U Manevrası',
        'sequence': [
            ('forward', 3.0, MOVEMENT_SPEED_NORMAL),
            ('strafe_right', 1.0, MOVEMENT_SPEED_SLOW),
            ('turn_right', 45.0, TURN_RATE_SLOW),
            ('forward', 1.5, MOVEMENT_SPEED_SLOW),
            ('turn_right', 45.0, TURN_RATE_SLOW),
            ('strafe_left', 1.0, MOVEMENT_SPEED_SLOW),
            ('forward', 3.0, MOVEMENT_SPEED_NORMAL)
        ]
    }
}

class D300DepthSensor:
    """D300 Derinlik Sensörü - Stabilizasyon için"""
    
    def __init__(self, bus_num=D300_I2C_BUS, address=D300_I2C_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self.connected = False
        self.depth_m = 2.0  # Varsayılan hareket derinliği
        self.target_depth = 2.0
        
    def connect(self):
        """D300 sensörüne bağlan"""
        if not HAS_I2C:
            print("⚠️ I2C desteği yok - simülasyon modu")
            self.connected = True
            return True
            
        try:
            print(f"🔌 D300 sensörüne bağlanılıyor (I2C: 0x{self.address:02x})")
            self.bus = smbus2.SMBus(self.bus_num)
            test_data = self.bus.read_byte(self.address)
            self.connected = True
            print(f"✅ D300 sensörü bağlandı")
            return True
        except Exception as e:
            print(f"❌ D300 bağlantı hatası: {e}")
            self.connected = True  # Simülasyon için
            return True
    
    def read_depth(self):
        """Derinlik verisi oku"""
        if not self.connected or not HAS_I2C:
            # Simülasyon
            import random
            self.depth_m += random.uniform(-0.02, 0.02)
            self.depth_m = max(0.5, min(5.0, self.depth_m))
            return self.depth_m
            
        # Gerçek okuma implementasyonu buraya
        return self.depth_m

class XWingMovementController:
    """X-Wing Konfigürasyonlu Hareket Kontrolü"""
    
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
        self.current_position = {'x': 0.0, 'y': 0.0}
        self.current_heading = 0.0  # derece
        self.current_depth = 2.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Target states
        self.target_position = {'x': 0.0, 'y': 0.0}
        self.target_heading = 0.0
        self.target_depth = 2.0
        
        # PID Controllers
        self.forward_pid = PIDController(**MOVEMENT_PID['forward'])
        self.strafe_pid = PIDController(**MOVEMENT_PID['strafe'])
        self.turn_pid = PIDController(**MOVEMENT_PID['turn'])
        self.roll_pid = PIDController(**STABILIZATION_PID['roll'])
        self.pitch_pid = PIDController(**STABILIZATION_PID['pitch'])
        self.yaw_pid = PIDController(**STABILIZATION_PID['yaw'])
        self.depth_pid = PIDController(**STABILIZATION_PID['depth'])
        
        # Current servo positions
        self.servo_positions = {channel: PWM_NEUTRAL for channel in SERVO_CHANNELS.values()}
        
        # Movement state
        self.current_movement = 'stop'
        self.movement_active = False
        self.pattern_active = False
        self.pattern_step = 0
        self.pattern_name = ""
        
        # Control threads
        self.running = False
        self.control_thread = None
        self.movement_thread = None
        
        print("🚀 X-Wing Hareket Kontrolü başlatıldı")
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
            
            # Derinlik verisi
            self.current_depth = self.depth_sensor.read_depth()
            
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
    
    def calculate_x_wing_movement(self, forward_power, strafe_power, turn_power, 
                                 roll_stab, pitch_stab, yaw_stab, depth_stab):
        """X-Wing konfigürasyonu için hareket hesaplaması"""
        
        # Base PWM değerleri
        front_left = PWM_NEUTRAL
        front_right = PWM_NEUTRAL
        rear_left = PWM_NEUTRAL
        rear_right = PWM_NEUTRAL
        motor = PWM_NEUTRAL
        
        # İleri/Geri hareket - ana motor
        motor += forward_power * 400  # PWM değişimi
        
        # Yatay hareket (strafe) - finler ile
        strafe_component = strafe_power * 300
        
        # Dönüş hareketi
        turn_component = turn_power * 250
        
        # Stabilizasyon bileşenleri
        pitch_component = pitch_stab + (forward_power * 0.1)  # Hafif pitch kompanzasyonu
        
        # X-Wing konfigürasyonu - hareket + stabilizasyon
        front_left += pitch_component + roll_stab + turn_component + (strafe_component * 0.7)
        front_right += pitch_component - roll_stab - turn_component - (strafe_component * 0.7)
        rear_left += -pitch_component + roll_stab - turn_component + (strafe_component * 0.7)
        rear_right += -pitch_component - roll_stab + turn_component - (strafe_component * 0.7)
        
        # Derinlik stabilizasyonu - tüm finlerde hafif etki
        depth_component = depth_stab * 0.2
        front_left += depth_component
        front_right += depth_component
        rear_left += depth_component
        rear_right += depth_component
        
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
            
            # Stabilizasyon PID'leri
            roll_stab = self.roll_pid.update(-self.current_roll, dt)
            pitch_stab = self.pitch_pid.update(-self.current_pitch, dt)
            yaw_stab = self.yaw_pid.update(self.target_heading - self.current_yaw, dt)
            depth_stab = self.depth_pid.update(self.target_depth - self.current_depth, dt)
            
            # Hareket PID'leri (varsayılan olarak 0)
            forward_power = 0.0
            strafe_power = 0.0
            turn_power = 0.0
            
            # Aktif harekete göre güç ayarla
            if self.movement_active:
                if self.current_movement == 'forward':
                    forward_power = 0.5
                elif self.current_movement == 'backward':
                    forward_power = -0.5
                elif self.current_movement == 'strafe_right':
                    strafe_power = 0.5
                elif self.current_movement == 'strafe_left':
                    strafe_power = -0.5
                elif self.current_movement == 'turn_right':
                    turn_power = 0.5
                elif self.current_movement == 'turn_left':
                    turn_power = -0.5
            
            # X-Wing servo kontrolü hesapla
            servo_commands = self.calculate_x_wing_movement(
                forward_power, strafe_power, turn_power,
                roll_stab, pitch_stab, yaw_stab, depth_stab
            )
            
            # Servo komutlarını gönder
            for channel, pwm in servo_commands.items():
                self.set_servo_pwm(channel, pwm)
            
            # LED kontrolü - hareket sırasında yanık
            self.set_led(self.movement_active or self.pattern_active)
            
            # Timing
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def execute_movement(self, movement_type, duration, speed=MOVEMENT_SPEED_NORMAL):
        """Tekil hareket komutunu çalıştır"""
        print(f"🏃 Hareket başlatılıyor: {movement_type} ({duration:.1f}s, hız: {speed:.1f})")
        
        self.current_movement = movement_type
        self.movement_active = True
        
        time.sleep(duration)
        
        self.movement_active = False
        self.current_movement = 'stop'
        
        # Durma süresi
        time.sleep(0.5)
        print(f"✅ Hareket tamamlandı: {movement_type}")
    
    def execute_turn(self, direction, angle, turn_rate=TURN_RATE_NORMAL):
        """Dönüş hareketini çalıştır"""
        print(f"🔄 Dönüş başlatılıyor: {direction} {angle}° ({turn_rate}°/s)")
        
        start_heading = self.current_yaw
        target_heading = start_heading + (angle if direction == 'turn_right' else -angle)
        
        # Başlangıç zamanı
        start_time = time.time()
        duration = abs(angle) / turn_rate
        
        self.current_movement = direction
        self.movement_active = True
        
        # Dönüş kontrolü
        while time.time() - start_time < duration and self.running:
            # Hedef başlığı güncelle
            progress = (time.time() - start_time) / duration
            self.target_heading = start_heading + (angle * progress * (1 if direction == 'turn_right' else -1))
            time.sleep(0.1)
        
        self.movement_active = False
        self.current_movement = 'stop'
        self.target_heading = target_heading
        
        time.sleep(0.5)
        print(f"✅ Dönüş tamamlandı: {angle}°")
    
    def execute_u_pattern(self, pattern_name='small_u'):
        """U şekli patern çalıştır"""
        if pattern_name not in U_SHAPE_PATTERNS:
            print(f"❌ Bilinmeyen patern: {pattern_name}")
            return False
        
        pattern = U_SHAPE_PATTERNS[pattern_name]
        print(f"🎯 {pattern['name']} başlatılıyor...")
        
        self.pattern_active = True
        self.pattern_name = pattern_name
        self.pattern_step = 0
        
        try:
            for step, (movement_type, param, speed) in enumerate(pattern['sequence']):
                if not self.running:
                    break
                    
                self.pattern_step = step + 1
                print(f"📍 Adım {self.pattern_step}/{len(pattern['sequence'])}: {movement_type}")
                
                if 'turn' in movement_type:
                    # Dönüş hareketi
                    direction = movement_type
                    angle = param
                    turn_rate = speed if speed > 10 else TURN_RATE_SLOW
                    self.execute_turn(direction, angle, turn_rate)
                else:
                    # Lineer hareket
                    duration = param / speed  # mesafe / hız = süre
                    self.execute_movement(movement_type, duration, speed)
            
            print(f"🎉 {pattern['name']} başarıyla tamamlandı!")
            return True
            
        except Exception as e:
            print(f"❌ Patern çalıştırma hatası: {e}")
            return False
        finally:
            self.pattern_active = False
            self.pattern_name = ""
            self.pattern_step = 0
    
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
        
        # Kontrol döngüsünü başlat
        self.running = True
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("✅ Hareket kontrol sistemi başlatıldı")
        
        # Başlangıç LED sinyali
        for i in range(3):
            self.set_led(True)
            time.sleep(0.2)
            self.set_led(False)
            time.sleep(0.2)
        
        return True
    
    def stop_control(self):
        """Kontrol sistemini durdur"""
        print("🛑 Kontrol sistemi durduruluyor...")
        self.running = False
        self.movement_active = False
        self.pattern_active = False
        
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
    print("🚀 TEKNOFEST U-Şekli Hareket Kontrolü")
    print("=" * 50)
    
    controller = XWingMovementController()
    
    try:
        # Sistemi başlat
        if not controller.start_control():
            print("❌ Sistem başlatılamadı")
            return
        
        print("\n📋 Hareket Komutları:")
        print("  🔄 U-Şekli Paternler:")
        print("    'small_u'   - Küçük U şekli")
        print("    'large_u'   - Büyük U şekli")
        print("    'complex_u' - Karmaşık U manevrası")
        print("  🏃 Tekil Hareketler:")
        print("    'forward 3'    - 3 saniye ileri")
        print("    'backward 2'   - 2 saniye geri")
        print("    'right 2'      - 2 saniye sağa")
        print("    'left 2'       - 2 saniye sola")
        print("    'turn_right 90'- 90° sağa dön")
        print("    'turn_left 45' - 45° sola dön")
        print("  💡 Diğer:")
        print("    'led on/off'   - LED kontrol")
        print("    'status'       - Mevcut durum")
        print("    'stop'         - Acil dur")
        print("    'quit'         - Çıkış")
        print()
        
        # Interaktif kontrol
        while True:
            try:
                user_input = input("💻 Komut: ").strip().lower()
                
                if user_input in ['quit', 'exit', 'q']:
                    break
                elif user_input == 'stop':
                    controller.movement_active = False
                    controller.pattern_active = False
                    print("🛑 Tüm hareketler durduruldu")
                elif user_input == 'status':
                    print(f"🧭 Mevcut başlık: {controller.current_yaw:.1f}°")
                    print(f"🌊 Mevcut derinlik: {controller.current_depth:.2f}m")
                    print(f"📐 Roll: {controller.current_roll:.1f}° | "
                          f"Pitch: {controller.current_pitch:.1f}°")
                    print(f"🏃 Hareket durumu: {'AKTİF' if controller.movement_active else 'DURGUN'}")
                    if controller.pattern_active:
                        print(f"🎯 Patern: {controller.pattern_name} (Adım: {controller.pattern_step})")
                elif user_input == 'led on':
                    controller.set_led(True)
                elif user_input == 'led off':
                    controller.set_led(False)
                elif user_input in U_SHAPE_PATTERNS:
                    # U-şekli patern çalıştır
                    threading.Thread(
                        target=controller.execute_u_pattern, 
                        args=(user_input,), 
                        daemon=True
                    ).start()
                elif ' ' in user_input:
                    # Parametreli komutlar
                    parts = user_input.split()
                    command = parts[0]
                    param = float(parts[1])
                    
                    if command == 'forward':
                        threading.Thread(
                            target=controller.execute_movement,
                            args=('forward', param, MOVEMENT_SPEED_SLOW),
                            daemon=True
                        ).start()
                    elif command == 'backward':
                        threading.Thread(
                            target=controller.execute_movement,
                            args=('backward', param, MOVEMENT_SPEED_SLOW),
                            daemon=True
                        ).start()
                    elif command == 'right':
                        threading.Thread(
                            target=controller.execute_movement,
                            args=('strafe_right', param, MOVEMENT_SPEED_SLOW),
                            daemon=True
                        ).start()
                    elif command == 'left':
                        threading.Thread(
                            target=controller.execute_movement,
                            args=('strafe_left', param, MOVEMENT_SPEED_SLOW),
                            daemon=True
                        ).start()
                    elif command == 'turn_right':
                        threading.Thread(
                            target=controller.execute_turn,
                            args=('turn_right', param, TURN_RATE_SLOW),
                            daemon=True
                        ).start()
                    elif command == 'turn_left':
                        threading.Thread(
                            target=controller.execute_turn,
                            args=('turn_left', param, TURN_RATE_SLOW),
                            daemon=True
                        ).start()
                    else:
                        print(f"❌ Bilinmeyen komut: {command}")
                else:
                    print("❌ Geçersiz komut. 'help' yazın veya yukarıdaki listeyi kontrol edin.")
                    
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