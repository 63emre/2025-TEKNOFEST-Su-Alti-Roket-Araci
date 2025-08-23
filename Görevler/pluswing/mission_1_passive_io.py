#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş
Pixhawk Pasif I/O Hub Yaklaşımı

Pixhawk Rolü (Pasif):
- PWM çıkışları (DO_SET_SERVO ile Pi kontrolü)
- ATTITUDE telemetrisi (20 Hz)
- SCALED_PRESSURE telemetrisi (10 Hz)
- MANUAL mode (tek güvenli mod)

Pi Rolü (Aktif):
- Tüm PID kontrolleri (derinlik, heading)
- Servo mixing (Plus-Wing)
- Odometri (PWM→hız→mesafe)
- Mission state machine
- D300 derinlik sensörü

Mission Flow:
1. DESCENT (2m derinlik)
2. STRAIGHT_COURSE (10m düz seyir)
3. OFFSHORE_CRUISE (50m uzaklaşma)
4. RETURN_NAVIGATION (başlangıça dönüş)
5. FINAL_APPROACH (±2m/5s pozisyon tutma)
6. SURFACE_AND_SHUTDOWN (yüzeye çık + enerji kes)
"""

import time
import threading
import math
import json
import os
import argparse
from datetime import datetime
from pymavlink import mavutil
import numpy as np

# GPIO kontrol sistemi
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("✅ RPi.GPIO modülü yüklendi")
except ImportError:
    print("⚠️ RPi.GPIO bulunamadı, LED/Buzzer devre dışı")
    GPIO_AVAILABLE = False

# Hardware config import
try:
    from hardware_config_passive_io import (
        PLUS_WING_CHANNELS, PWM_LIMITS, DEPTH_PID, HEADING_PID,
        D300_CONFIG, PRESSURE_CONFIG, MAVLINK_CONFIG, SAFETY_CONFIG,
        apply_plus_wing_mixing, calculate_motor_pwm, get_channel_for_actuator
    )
    print("✅ Pixhawk Pasif I/O hardware config yüklendi")
except ImportError:
    print("❌ Hardware config bulunamadı!")
    exit(1)

# D300 derinlik sensörü
try:
    import sys
    _BASE_DIR = os.path.dirname(__file__) if '__file__' in globals() else os.getcwd()
    _APP_DIR = os.path.normpath(os.path.join(_BASE_DIR, '../App'))
    if _APP_DIR not in sys.path:
        sys.path.append(_APP_DIR)
    from depth_sensor import D300DepthSensor
    D300_AVAILABLE = True
    print("✅ D300 derinlik sensörü modülü yüklendi")
except ImportError:
    print("⚠️ D300 derinlik sensörü modülü bulunamadı, SCALED_PRESSURE kullanılacak")
    D300_AVAILABLE = False

# Hız kalibrasyonu
try:
    with open('../config/cal_speed.json', 'r') as f:
        SPEED_CALIBRATION = json.load(f)
    print("✅ Hız kalibrasyonu yüklendi")
except:
    print("⚠️ Hız kalibrasyonu bulunamadı, default değerler kullanılacak")
    SPEED_CALIBRATION = {
        "plus_wing": {
            "1400": 0.5, "1500": 0.0, "1600": 0.8, "1700": 1.5, "1800": 2.2
        }
    }

# MAVLink bağlantı
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# Mission parametreleri
MISSION_PARAMS = {
    'target_depth': 2.0,           # 2m derinlik
    'straight_distance': 10.0,     # 10m düz seyir
    'min_offshore_distance': 50.0, # 50m uzaklaşma
    'timeout_seconds': 300,        # 5 dakika
    'position_tolerance': 2.0,     # ±2m tolerans
    'depth_tolerance': 0.2,        # ±0.2m derinlik toleransı
    'final_hold_duration': 5.0     # 5s pozisyon tutma
}

class PIDController:
    """Basit PID Controller"""
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        current_time = time.time()
        dt = max(0.01, current_time - self.last_time)
        
        error = setpoint - measurement
        self.integral += error * dt
        
        # Integral windup protection
        integral_limit = self.max_output / self.ki if self.ki > 0 else float('inf')
        self.integral = np.clip(self.integral, -integral_limit, integral_limit)
        
        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = np.clip(output, -self.max_output, self.max_output)
        
        self.previous_error = error
        self.last_time = current_time
        return output
    
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

class PixhawkPassiveIOMission1:
    """
    TEKNOFEST Görev 1 - Pixhawk Pasif I/O Hub
    
    Pixhawk sadece PWM çıkış + telemetri hub olarak kullanılır.
    Tüm kontrol Pi tarafından DO_SET_SERVO ile yapılır.
    """
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.mission_active = False
        
        # GPIO sistemi
        self.gpio_initialized = False
        self._init_gpio_system()
        
        # D300 derinlik sensörü
        self.d300_sensor = None
        self.d300_connected = False
        if D300_AVAILABLE:
            try:
                self.d300_sensor = D300DepthSensor(i2c_address=D300_CONFIG['i2c_address'])
                self.d300_connected = self.d300_sensor.initialize()
                print("✅ D300 derinlik sensörü başlatıldı" if self.d300_connected else "⚠️ D300 başlatılamadı")
            except Exception as e:
                print(f"⚠️ D300 hatası: {e}")
                self.d300_connected = False
        
        # Mission state
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.mission_completion_time = None
        
        # Sensör verileri
        self.current_depth = 0.0
        self.depth_source = "none"
        self.current_heading = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Odometri (GPS'siz navigasyon)
        self.start_heading = 0.0
        self.traveled_distance = 0.0
        self.current_position_estimate = {'x': 0.0, 'y': 0.0}  # Başlangıçtan relatif
        self.current_motor_pwm = PWM_LIMITS['motor_neutral']
        self.last_odometry_time = time.time()
        
        # PID Controllers (sadece desteklenen parametreleri kullan)
        depth_pid_params = {k: DEPTH_PID[k] for k in ['kp', 'ki', 'kd', 'max_output'] if k in DEPTH_PID}
        heading_pid_params = {k: HEADING_PID[k] for k in ['kp', 'ki', 'kd', 'max_output'] if k in HEADING_PID}
        self.depth_pid = PIDController(**depth_pid_params)
        self.heading_pid = PIDController(**heading_pid_params)
        
        # Mission metrics
        self.max_offshore_distance = 0.0
        self.final_position_error = float('inf')
        self.leak_detected = False
        self.latched_fault = None
        
        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False
        
        # Telemetri
        self.telemetry_data = []
        self.last_attitude_time = 0
        self.last_pressure_time = 0
        
        # Arming sistemi (90s kuralı)
        self.arming_start_time = None
        self.arming_completed = False
        
        print("✅ Pixhawk Pasif I/O Mission 1 başlatıldı")
        print("🎯 Amaç: Su altında GPS'siz navigasyon + derinlik kontrolü")
    
    def _init_gpio_system(self):
        """GPIO sistemi başlatma"""
        if not GPIO_AVAILABLE:
            return
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            GPIO.setup(4, GPIO.OUT)   # Status LED
            GPIO.setup(13, GPIO.OUT)  # Buzzer
            GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Power button
            GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Emergency stop
            GPIO.setup(26, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Leak sensor (aktif-low)
            
            GPIO.output(4, GPIO.LOW)
            GPIO.output(13, GPIO.LOW)
            
            self.gpio_initialized = True
            print("✅ GPIO sistemi başlatıldı")
        except Exception as e:
            print(f"⚠️ GPIO hatası: {e}")
            self.gpio_initialized = False
    
    def _signal_status(self, status):
        """Durum sinyali (LED/Buzzer)"""
        if not self.gpio_initialized:
            return
            
        try:
            if status == "running":
                GPIO.output(4, GPIO.HIGH)
            elif status == "success":
                for _ in range(3):
                    GPIO.output(13, GPIO.HIGH)
                    time.sleep(0.2)
                    GPIO.output(13, GPIO.LOW)
                    time.sleep(0.2)
                GPIO.output(4, GPIO.HIGH)
            elif status == "fault":
                for _ in range(6):
                    GPIO.output(4, GPIO.HIGH)
                    GPIO.output(13, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(4, GPIO.LOW)
                    GPIO.output(13, GPIO.LOW)
                    time.sleep(0.1)
        except:
            pass
    
    def connect_pixhawk(self):
        """Pixhawk bağlantısı (Pasif I/O Hub)"""
        try:
            print("🔌 Pixhawk Pasif I/O Hub bağlantısı...")
            
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=15)
            
            if heartbeat:
                self.connected = True
                print("✅ MAVLink bağlantısı başarılı!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                
                # MANUAL mode'a geç (tek güvenli mod)
                self._set_manual_mode()
                
                # Telemetri stream rate'leri ayarla
                self._setup_telemetry_streams()
                
                return True
            else:
                print("❌ Heartbeat timeout!")
                return False
                
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def _set_manual_mode(self):
        """MANUAL mode'a geç (tek güvenli mod)"""
        try:
            mode_map = self.master.mode_mapping()
            if 'MANUAL' not in mode_map:
                print("❌ MANUAL mode id bulunamadı")
                return False
            manual_mode_id = mode_map['MANUAL']
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                manual_mode_id
            )
            
            # Mode değişimini doğrula
            time.sleep(2.0)
            heartbeat = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3.0)
            if heartbeat and heartbeat.custom_mode == manual_mode_id:
                print("✅ MANUAL mode aktif")
                return True
            else:
                print("❌ MANUAL mode geçiş doğrulanamadı")
                return False
                
        except Exception as e:
            print(f"❌ Mode geçiş hatası: {e}")
            return False
    
    def _setup_telemetry_streams(self):
        """Telemetri stream rate'lerini ayarla"""
        try:
            # Sadece ATTITUDE için data stream iste (20 Hz)
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # IMU/ATTITUDE
                20,
                1
            )
            print("📊 Telemetri stream: ATTITUDE 20 Hz (request_data_stream)")
        except Exception as e:
            print(f"⚠️ Stream rate hatası: {e}")
    
    def read_sensors(self):
        """Tüm sensörleri oku"""
        if not self.connected or self.latched_fault:
            return False
        
        try:
            current_time = time.time()
            
            # ATTITUDE mesajı (Pixhawk IMU)
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw
                if self.current_heading < 0:
                    self.current_heading += 360
                self.last_attitude_time = current_time
            
            # Derinlik sensörü (D300 zorunlu - saha)
            depth_read_success = False
            
            if self.d300_connected and self.d300_sensor:
                try:
                    depth_data = self.d300_sensor.read_depth()
                    if depth_data['success']:
                        self.current_depth = max(0.0, depth_data['depth'])
                        self.depth_source = "d300"
                        self._last_d300_time = current_time
                        depth_read_success = True
                except Exception as e:
                    print(f"⚠️ D300 okuma hatası: {e}")
            
            if not depth_read_success:
                # Saha: fallback devre dışı → fault
                print("🚨 D300 derinlik verisi yok! Mission abort.")
                self._trigger_latched_fault("DEPTH_UNAVAILABLE")
                return False
            
            # Acil stop ve Leak GPIO kontrolü
            if self.gpio_initialized:
                try:
                    if GPIO.input(19) == GPIO.LOW:  # Emergency stop (aktif-low)
                        self._trigger_latched_fault("EMERGENCY_STOP")
                        return False
                    if GPIO.input(26) == GPIO.LOW:  # Leak (aktif-low)
                        self.leak_detected = True
                        self._trigger_latched_fault("LEAK_DETECTED")
                        return False
                except Exception:
                    pass

            # Leak detection (STATUSTEXT monitoring - ikincil)
            statustext = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if statustext and statustext.text:
                text_lower = statustext.text.lower()
                if "leak" in text_lower or "water" in text_lower:
                    self.leak_detected = True
                    self._trigger_latched_fault("LEAK_DETECTED")
                    return False
            
            # Odometri güncelle
            self._update_odometry()
            
            # Watchdog kontrolü
            if not self._check_watchdog():
                return False
            
            # Telemetri kaydet
            self.telemetry_data.append({
                'timestamp': current_time,
                'mission_stage': self.mission_stage,
                'depth': self.current_depth,
                'depth_source': self.depth_source,
                'heading': self.current_heading,
                'attitude': {'roll': self.current_roll, 'pitch': self.current_pitch, 'yaw': self.current_yaw},
                'position_estimate': self.current_position_estimate.copy(),
                'traveled_distance': self.traveled_distance,
                'motor_pwm': self.current_motor_pwm,
                'leak_detected': self.leak_detected
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            self._trigger_latched_fault(f"SENSOR_READ_ERROR: {e}")
            return False
    
    def _check_watchdog(self):
        """Watchdog kontrolü (0.5s telemetri timeout)"""
        current_time = time.time()
        
        # ATTITUDE timeout
        if current_time - self.last_attitude_time > SAFETY_CONFIG['watchdog_timeout']:
            print(f"🚨 WATCHDOG: ATTITUDE timeout ({current_time - self.last_attitude_time:.1f}s)")
            self._trigger_latched_fault("ATTITUDE_TIMEOUT")
            return False
        
        # Derinlik timeout (yalnız D300)
        timeout_threshold = SAFETY_CONFIG['watchdog_timeout']
        if not hasattr(self, '_last_d300_time'):
            self._last_d300_time = current_time
        if current_time - self._last_d300_time > timeout_threshold:
            print(f"🚨 WATCHDOG: D300 timeout")
            self._trigger_latched_fault("DEPTH_SENSOR_TIMEOUT")
            return False
        
        return True
    
    def _trigger_latched_fault(self, fault_reason):
        """Latched fault tetikle (mission abort)"""
        if not self.latched_fault:
            self.latched_fault = fault_reason
            print(f"🚨 LATCHED FAULT: {fault_reason}")
            print("🚨 Emergency neutral + mission abort...")
            
            self.mission_stage = "MISSION_ABORT"
            self.mission_active = False
            
            # Tüm servolar neutral
            self._emergency_neutral_all()
            
            # Fault sinyali
            self._signal_status("fault")
    
    def _emergency_neutral_all(self):
        """Acil durum: Tüm servolar neutral"""
        try:
            print("🚨 Emergency: Tüm servolar neutral...")
            
            # Motor neutral
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], PWM_LIMITS['motor_neutral'])
            
            # Tüm finler neutral
            for fin_name, channel in PLUS_WING_CHANNELS.items():
                if fin_name != 'motor':
                    self._send_servo_command(channel, PWM_LIMITS['servo_neutral'])
                    time.sleep(0.05)  # Kısa delay
            
            print("✅ Emergency neutral tamamlandı")
        except Exception as e:
            print(f"❌ Emergency neutral hatası: {e}")
    
    def _update_odometry(self):
        """Odometri güncelle (PWM→hız→mesafe)"""
        current_time = time.time()
        dt = current_time - self.last_odometry_time
        
        if dt < 0.01:  # Minimum 10ms
            return
        
        try:
            # PWM'den hız hesapla (cal_speed.json)
            speed = self._pwm_to_speed(self.current_motor_pwm)
            
            # Mesafe entegrasyonu (v * dt)
            distance_increment = speed * dt
            self.traveled_distance += abs(distance_increment)
            
            # Pozisyon entegrasyonu (heading tabanlı)
            heading_rad = math.radians(self.current_heading)
            dx = distance_increment * math.cos(heading_rad)
            dy = distance_increment * math.sin(heading_rad)
            
            self.current_position_estimate['x'] += dx
            self.current_position_estimate['y'] += dy
            
            self.last_odometry_time = current_time
            
        except Exception as e:
            print(f"⚠️ Odometri hatası: {e}")
    
    def _pwm_to_speed(self, pwm):
        """PWM değerini hıza çevir (kalibrasyondan)"""
        try:
            cal_data = SPEED_CALIBRATION.get("plus_wing", {})
            
            # En yakın PWM değerlerini bul
            pwm_values = sorted([int(k) for k in cal_data.keys()])
            
            if pwm <= pwm_values[0]:
                return cal_data[str(pwm_values[0])]
            elif pwm >= pwm_values[-1]:
                return cal_data[str(pwm_values[-1])]
            else:
                # Linear interpolation
                for i in range(len(pwm_values) - 1):
                    if pwm_values[i] <= pwm <= pwm_values[i + 1]:
                        pwm1, pwm2 = pwm_values[i], pwm_values[i + 1]
                        speed1, speed2 = cal_data[str(pwm1)], cal_data[str(pwm2)]
                        
                        # Linear interpolation
                        ratio = (pwm - pwm1) / (pwm2 - pwm1)
                        return speed1 + ratio * (speed2 - speed1)
            
            return 0.0
            
        except:
            return 0.0  # Fallback
    
    def _send_servo_command(self, channel, pwm_value):
        """DO_SET_SERVO komutu gönder"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,     # param1: servo channel
                pwm_value,   # param2: PWM value
                0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Servo komut hatası: {e}")
            return False
    
    def _get_distance_from_start(self):
        """Başlangıç noktasından mesafe (odometri tabanlı)"""
        return math.sqrt(self.current_position_estimate['x']**2 + self.current_position_estimate['y']**2)
    
    def _get_bearing_to_start(self):
        """Başlangıç noktasına bearing"""
        dx = -self.current_position_estimate['x']
        dy = -self.current_position_estimate['y']
        
        bearing = math.degrees(math.atan2(dy, dx))
        if bearing < 0:
            bearing += 360
        
        return bearing
    
    def display_mission_status(self):
        """Mission durumu göster"""
        print("\n" + "="*80)
        print("🚀 TEKNOFEST - GÖREV 1 (Pixhawk Pasif I/O)")
        print("="*80)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        mission_time = (time.time() - self.mission_start_time) if self.mission_start_time else 0
        remaining_time = max(0, MISSION_PARAMS['timeout_seconds'] - mission_time)
        
        print(f"⏰ Zaman: {timestamp} | Süre: {mission_time:.0f}s | Kalan: {remaining_time:.0f}s")
        print(f"🎯 Aşama: {self.mission_stage}")
        
        distance_from_start = self._get_distance_from_start()
        bearing_to_start = self._get_bearing_to_start()
        
        print(f"📍 Pozisyon (odometri): x={self.current_position_estimate['x']:.1f}m, y={self.current_position_estimate['y']:.1f}m")
        print(f"🏠 Başlangıçtan uzaklık: {distance_from_start:.1f}m")
        print(f"🧭 Başlangıca bearing: {bearing_to_start:.0f}° | Heading: {self.current_heading:.0f}°")
        
        print(f"🌊 Derinlik: {self.current_depth:.2f}m ({self.depth_source}) | Hedef: {MISSION_PARAMS['target_depth']:.1f}m")
        print(f"📊 Attitude: R={self.current_roll:+.1f}° P={self.current_pitch:+.1f}° Y={self.current_yaw:+.1f}°")
        print(f"⚙️ Motor PWM: {self.current_motor_pwm} | Mesafe: {self.traveled_distance:.1f}m")
        
        if self.latched_fault:
            print(f"🚨 FAULT: {self.latched_fault}")
        
        print("="*80)
    
    # Mission stage implementations continue...
    # (I'll continue with the control loop and mission stages in the next part)
    
    def control_loop(self):
        """Ana kontrol döngüsü"""
        print("🎯 Pixhawk Pasif I/O Control Loop başlatıldı")
        
        loop_count = 0
        last_heartbeat = time.time()
        
        try:
            while self.running and self.mission_active:
                cycle_start = time.time()
                
                # Heartbeat
                if cycle_start - last_heartbeat > 5.0:
                    print(f"💓 Control heartbeat - {self.mission_stage}")
                    last_heartbeat = cycle_start
                
                # Sensör okuma
                if not self.read_sensors():
                    continue
                
                # Arming kontrolü (90s kuralı)
                if not self.arming_completed:
                    if not self._handle_arming_sequence():
                        time.sleep(0.1)
                        continue
                
                # Mission stage execution
                if self.mission_stage == "DESCENT":
                    self.execute_descent()
                elif self.mission_stage == "STRAIGHT_COURSE":
                    self.execute_straight_course()
                elif self.mission_stage == "OFFSHORE_CRUISE":
                    self.execute_offshore_cruise()
                elif self.mission_stage == "RETURN_NAVIGATION":
                    self.execute_return_navigation()
                elif self.mission_stage == "FINAL_APPROACH":
                    self.execute_final_approach()
                elif self.mission_stage == "SURFACE_AND_SHUTDOWN":
                    self.execute_surface_shutdown()
                elif self.mission_stage == "MISSION_COMPLETE":
                    break
                elif self.mission_stage == "MISSION_ABORT":
                    break
                
                # Timing kontrolü
                cycle_time = time.time() - cycle_start
                target_cycle_time = 1.0 / 20  # 20 Hz
                sleep_time = max(0.01, target_cycle_time - cycle_time)
                time.sleep(sleep_time)
                
                loop_count += 1
                
        except Exception as e:
            print(f"🚨 Control loop exception: {e}")
            self._trigger_latched_fault(f"CONTROL_LOOP_EXCEPTION: {e}")
        finally:
            print("🔄 Control loop sonlandırılıyor...")
            self._emergency_neutral_all()
    
    def _handle_arming_sequence(self):
        """90 saniye arming sequence (motor/servo neutral)"""
        if self.arming_start_time is None:
            self.arming_start_time = time.time()
            print("🔒 90 saniye arming sequence başladı (motor/servo neutral)")
        
        elapsed = time.time() - self.arming_start_time
        remaining = SAFETY_CONFIG['arming_duration'] - elapsed
        
        if remaining > 0:
            # Arming süresince tüm çıkışlar neutral
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], PWM_LIMITS['motor_neutral'])
            for fin_name, channel in PLUS_WING_CHANNELS.items():
                if fin_name != 'motor':
                    self._send_servo_command(channel, PWM_LIMITS['servo_neutral'])
            
            # Her 10 saniyede durum göster
            if int(elapsed) % 10 == 0 and int(elapsed) != int(elapsed - 1):
                print(f"🔒 Arming: {remaining:.0f}s kaldı (motor/servo neutral)")
            
            return False
        else:
            if not self.arming_completed:
                self.arming_completed = True
                print("✅ 90 saniye arming tamamlandı - mission başlıyor!")
            return True
    
    def execute_descent(self):
        """2m derinliğe iniş"""
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            # Derinlik PID → pitch komutu
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)
            yaw_cmd = 0.0  # inişte heading tutmayı zorlamıyoruz

            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, yaw_cmd)

            # Motor baz ileri hız
            motor_pwm = 1600
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))

            # Komutları gönder (20 Hz)
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
        else:
            print("✅ Hedef derinlik ulaşıldı! Düz seyire geçiliyor...")
            self.mission_stage = "STRAIGHT_COURSE"
            self.start_heading = self.current_heading  # Başlangıç heading'i kaydet
            self.depth_pid.reset()
    
    def execute_straight_course(self):
        """10m düz seyir"""
        if self.traveled_distance < MISSION_PARAMS['straight_distance']:
            # Derinlik kontrolü → pitch
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)

            # Heading kontrolü (düz gitmek için) → yaw
            heading_error = self.start_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_output = self.heading_pid.update(0, heading_error)
            yaw_cmd = np.clip(heading_output / max(1.0, HEADING_PID.get('max_output', 100)), -1.0, 1.0)

            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, yaw_cmd)

            # Motor PWM (sabit baz hız)
            motor_pwm = 1600
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))
            
            # Komutları gönder
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
        else:
            print("✅ 10m düz seyir tamamlandı! Uzaklaşmaya başlanıyor...")
            self.mission_stage = "OFFSHORE_CRUISE"
    
    def execute_offshore_cruise(self):
        """50m uzaklaşma"""
        distance_from_start = self._get_distance_from_start()
        self.max_offshore_distance = max(self.max_offshore_distance, distance_from_start)
        
        if distance_from_start < MISSION_PARAMS['min_offshore_distance']:
            # Derinlik kontrolü → pitch
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)

            # Heading kontrolü (düz gitmek için) → yaw
            heading_error = self.start_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_output = self.heading_pid.update(0, heading_error)
            yaw_cmd = np.clip(heading_output / max(1.0, HEADING_PID.get('max_output', 100)), -1.0, 1.0)

            # Plus-Wing mixing
            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, yaw_cmd)
            
            # Motor PWM (sabit baz hız)
            motor_pwm = 1600
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))
            
            # Komutları gönder
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
        else:
            print("✅ 50m uzaklaşma tamamlandı! Geri dönüş başlıyor...")
            self.mission_stage = "RETURN_NAVIGATION"
    
    def execute_return_navigation(self):
        """Başlangıç noktasına geri dönüş"""
        distance_from_start = self._get_distance_from_start()
        bearing_to_start = self._get_bearing_to_start()
        
        if distance_from_start > MISSION_PARAMS['position_tolerance']:
            # Derinlik kontrolü → pitch
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)

            # Bearing kontrolü (başlangıça dönmek için) → yaw
            heading_error = bearing_to_start - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_output = self.heading_pid.update(0, heading_error)
            yaw_cmd = np.clip(heading_output / max(1.0, HEADING_PID.get('max_output', 100)), -1.0, 1.0)

            # Plus-Wing mixing
            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, yaw_cmd)
            
            # Motor PWM (geri dönüş hızı - sabit baz)
            motor_pwm = 1650
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))
            
            # Komutları gönder
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
        else:
            print("✅ Başlangıç noktasına yaklaşıldı! Final yaklaşım...")
            self.mission_stage = "FINAL_APPROACH"
            self.final_position_error = distance_from_start
    
    def execute_final_approach(self):
        """Final yaklaşım ve 5s pozisyon tutma"""
        distance_from_start = self._get_distance_from_start()
        self.final_position_error = distance_from_start
        
        if distance_from_start <= MISSION_PARAMS['position_tolerance']:
            if not hasattr(self, '_final_hold_start_time'):
                self._final_hold_start_time = time.time()
                print("🎯 Pozisyon toleransı içinde! 5 saniye tutma başlatıldı...")
            
            hold_duration = time.time() - self._final_hold_start_time
            
            # Pozisyon tutma (yavaş ileri + pitch ile derinlik tut)
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)
            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, 0.0)

            motor_pwm = 1550
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))
            
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
            
            if hold_duration >= MISSION_PARAMS['final_hold_duration']:
                print("✅ 5 saniye pozisyon tutma tamamlandı! Yüzeye çıkış...")
                self.mission_stage = "SURFACE_AND_SHUTDOWN"
        else:
            # Tolerans dışına çıktık, sayacı sıfırla
            if hasattr(self, '_final_hold_start_time'):
                delattr(self, '_final_hold_start_time')
            
            # Pozisyona geri dön
            bearing_to_start = self._get_bearing_to_start()
            heading_error = bearing_to_start - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_output = self.heading_pid.update(0, heading_error)
            yaw_cmd = np.clip(heading_output / max(1.0, HEADING_PID.get('max_output', 100)), -1.0, 1.0)
            
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)
            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, yaw_cmd)
            
            motor_pwm = 1550  # Yavaş yaklaşım hızı
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))
            
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
    
    def execute_surface_shutdown(self):
        """Yüzeye çıkış ve enerji kesme"""
        if self.current_depth > 0.5:
            # Yüzeye çıkış: derinlik PID ile pitch yukarı, düşük baz hız
            depth_output = self.depth_pid.update(0.2, self.current_depth)  # 0.2m hedef
            pitch_cmd = np.clip(depth_output / max(1.0, DEPTH_PID.get('max_output', 200)), -1.0, 1.0)
            fin_pwms = apply_plus_wing_mixing(0.0, pitch_cmd, 0.0)

            motor_pwm = 1450
            motor_pwm = int(np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max']))
            
            self._send_servo_command(PLUS_WING_CHANNELS['motor'], motor_pwm)
            self.current_motor_pwm = motor_pwm
            
            for fin_name, pwm in fin_pwms.items():
                channel = PLUS_WING_CHANNELS[fin_name]
                self._send_servo_command(channel, pwm)
        else:
            # Yüzeye çıktık - enerji kesme simülasyonu
            print("🌊 Yüzeye çıkış tamamlandı!")
            print("⚡ Sistem enerjisi kesiliyor...")
            
            # Tüm çıkışlar neutral
            self._emergency_neutral_all()
            
            self.mission_completion_time = time.time()
            self.mission_stage = "MISSION_COMPLETE"
            print("✅ GÖREV 1 TAMAMLANDI!")
    
    def monitoring_loop(self):
        """İzleme döngüsü"""
        while self.running and self.mission_active:
            if self.latched_fault:
                break
            
            # Durum göstergesi
            self._signal_status("running")
            
            # Her 3 saniyede durum göster
            if len(self.telemetry_data) % 60 == 0:  # 20Hz * 3s = 60 sample
                self.display_mission_status()
            
            # Süre kontrolü
            if self.mission_start_time:
                elapsed = time.time() - self.mission_start_time
                if elapsed > MISSION_PARAMS['timeout_seconds']:
                    print("⏰ Süre doldu! Görev sonlandırılıyor...")
                    self.mission_stage = "MISSION_TIMEOUT"
                    break
            
            time.sleep(1.0)
    
    def generate_mission_report(self):
        """Mission raporu oluştur"""
        mission_duration = (self.mission_completion_time - self.mission_start_time) if (self.mission_completion_time and self.mission_start_time) else 0
        
        print("\n" + "="*80)
        print("📋 GÖREV 1 RAPORU - Pixhawk Pasif I/O Hub")
        print("="*80)
        
        print(f"📅 Görev Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Görev Süresi: {mission_duration:.1f} saniye")
        print(f"🎯 Mission Stage: {self.mission_stage}")
        
        # Performans metrikleri
        print(f"\n📊 PERFORMANS METRİKLERİ:")
        print("-"*60)
        print(f"🌊 Maksimum Uzaklık: {self.max_offshore_distance:.1f}m / {MISSION_PARAMS['min_offshore_distance']}m")
        print(f"🎯 Final Pozisyon Hatası: {self.final_position_error:.1f}m / {MISSION_PARAMS['position_tolerance']}m")
        print(f"📏 Toplam Mesafe: {self.traveled_distance:.1f}m")
        print(f"💧 Sızdırmazlık: {'✅ BAŞARILI' if not self.leak_detected else '❌ SIZINTI'}")
        print(f"🔧 Sistem: Pixhawk Pasif I/O Hub")
        print(f"📊 Derinlik Kaynağı: {self.depth_source}")
        
        # Puanlama
        print(f"\n🏆 PUANLAMA:")
        print("-"*40)
        
        # Seyir puanı
        time_factor = max(0, (MISSION_PARAMS['timeout_seconds'] - mission_duration) / MISSION_PARAMS['timeout_seconds']) if mission_duration > 0 else 0
        cruise_success = self.max_offshore_distance >= MISSION_PARAMS['min_offshore_distance']
        cruise_points = int(150 * time_factor) if cruise_success else 0
        
        # Pozisyon puanı
        position_success = self.final_position_error <= MISSION_PARAMS['position_tolerance'] and self.mission_stage == "MISSION_COMPLETE"
        position_points = 90 if position_success else 0
        
        # Sızdırmazlık puanı
        waterproof_points = 60 if not (self.leak_detected or self.latched_fault) else 0
        
        total_points = cruise_points + position_points + waterproof_points
        
        print(f"  🚀 Seyir Yapma: {cruise_points}/150 puan")
        print(f"  🎯 Pozisyon Tutma: {position_points}/90 puan")
        print(f"  💧 Sızdırmazlık: {waterproof_points}/60 puan")
        print(f"\n📈 TOPLAM PUAN: {total_points}/300")
        
        # Rapor kaydet
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        mission_report = {
            'timestamp': datetime.now().isoformat(),
            'mission_type': 'Pixhawk Passive I/O Mission 1',
            'mission_completed': self.mission_stage == "MISSION_COMPLETE",
            'mission_duration': mission_duration,
            'system_info': {
                'approach': 'Pixhawk Passive I/O Hub',
                'depth_source': self.depth_source,
                'latched_fault': self.latched_fault
            },
            'performance_metrics': {
                'max_offshore_distance': self.max_offshore_distance,
                'final_position_error': self.final_position_error,
                'traveled_distance': self.traveled_distance,
                'total_points': total_points
            },
            'telemetry_summary': {
                'total_samples': len(self.telemetry_data),
                'depth_source': self.depth_source,
                'leak_detected': self.leak_detected
            }
        }
        
        with open(f'mission_1_passive_io_report_{timestamp_str}.json', 'w') as f:
            json.dump(mission_report, f, indent=2)
        
        print(f"\n💾 Görev raporu kaydedildi: mission_1_passive_io_report_{timestamp_str}.json")
        
        return total_points >= 180  # %60 başarı şartı
    
    def cleanup(self):
        """Sistem temizleme"""
        print("\n🧹 Sistem temizleniyor...")
        
        self.running = False
        self.mission_active = False
        
        # Tüm servolar neutral
        if self.connected:
            self._emergency_neutral_all()
        
        # Thread'leri sonlandır
        if hasattr(self, 'control_thread') and self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=3.0)
        
        if hasattr(self, 'monitoring_thread') and self.monitoring_thread and self.monitoring_thread.is_alive():
            self.monitoring_thread.join(timeout=2.0)
        
        # D300 sensörü kapat
        if self.d300_connected and self.d300_sensor:
            try:
                self.d300_sensor.close()
                print("   ✅ D300 sensörü kapatıldı")
            except:
                pass
        
        # MAVLink bağlantı kapat
        if self.connected and self.master:
            try:
                self.master.close()
                print("   ✅ MAVLink bağlantısı kapatıldı")
            except:
                pass
        
        # GPIO temizle
        if self.gpio_initialized:
            try:
                GPIO.output(4, GPIO.LOW)
                GPIO.output(13, GPIO.LOW)
                GPIO.cleanup()
                print("   ✅ GPIO temizlendi")
            except:
                pass
        
        print("✅ Sistem temizleme tamamlandı")
    
    def run_mission_1(self):
        """Ana Mission 1 execution"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - GÖREV 1 (Pixhawk Pasif I/O)")
        print("="*80)
        print("🎯 Görev: Seyir Yapma & Başlangıç Noktasına Geri Dönüş")
        print("🔧 Sistem: Pixhawk Pasif I/O Hub")
        print("🌊 Navigasyon: GPS'siz odometri")
        print("⏱️ Süre Limiti: 5 dakika")
        print("🏆 Maksimum Puan: 300")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        if not self.d300_connected:
            print("❌ D300 derinlik sensörü bağlı değil! Saha koşullarında zorunlu.")
            return False
        
        print("\n⚠️ GÖREV HAZIRLIĞI:")
        print("- Pixhawk MANUAL mode'da mı?")
        print("- SERVOx_FUNCTION = 0 ayarlandı mı?")
        print("- D300 derinlik sensörü çalışıyor mu?")
        print("- Pozitif sephiye ayarlandı mı?")
        print("- Şamandıra takıldı mı?")
        
        try:
            ready = input("\n✅ Görev 1 başlasın mı? (y/n): ").lower()
        except EOFError:
            ready = 'y'
        
        if ready != 'y':
            print("❌ Görev iptal edildi")
            return False
        
        self.mission_start_time = time.time()
        self.mission_active = True
        self.running = True
        self.mission_stage = "DESCENT"
        
        # Thread'leri başlat
        try:
            self.control_thread = threading.Thread(target=self.control_loop, name="PassiveIOControl")
            self.control_thread.daemon = False
            self.control_thread.start()
            
            self.monitoring_thread = threading.Thread(target=self.monitoring_loop, name="Monitoring")
            self.monitoring_thread.daemon = False
            self.monitoring_thread.start()
            
        except Exception as e:
            print(f"❌ Thread başlatma hatası: {e}")
            self.cleanup()
            return False
        
        try:
            print("\n🚀 GÖREV 1 BAŞLADI! (Pixhawk Pasif I/O)")
            
            self.control_thread.join()
            
            success = self.generate_mission_report()
            
            if success:
                self._signal_status("success")
                print("\n🎉 GÖREV 1 BAŞARILI!")
            else:
                print("\n⚠️ GÖREV 1 KISMÎ BAŞARILI!")
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Görev kullanıcı tarafından durduruldu")
            self.mission_active = False
            self.running = False
            return False
        except Exception as e:
            print(f"\n❌ Görev hatası: {e}")
            self.mission_active = False
            self.running = False
            return False
        finally:
            self.cleanup()

def main():
    """Ana fonksiyon"""
    parser = argparse.ArgumentParser(description='TEKNOFEST Görev 1: Pixhawk Pasif I/O Hub')
    args = parser.parse_args()
    
    mission = PixhawkPassiveIOMission1()
    
    try:
        success = mission.run_mission_1()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
