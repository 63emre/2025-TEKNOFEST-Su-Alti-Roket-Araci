#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş
ArduPlane GUIDED MODE - X-Wing Configuration

ArduPlane'e Bırakılan:
- Attitude Stabilization (Roll/Pitch/Yaw)
- Servo Mixing (X-Wing configuration)
- Motor Control (Throttle/Speed)
- Safety Systems (Arming, failsafe)

Bizde Kalan:
- Mission State Machine
- Depth Control (Su altı özel)
- Waypoint Navigation Commands
- Telemetry & Logging
"""

import time
import threading
import math
import json
import os
import argparse
from datetime import datetime
from pymavlink import mavutil

# GPIO kontrol sistemi (LED & Buzzer)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("✅ RPi.GPIO modülü yüklendi")
except ImportError:
    print("⚠️ RPi.GPIO bulunamadı, LED/Buzzer devre dışı")
    GPIO_AVAILABLE = False

# D300 derinlik sensörü import
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

# MAVLink bağlantı adresi
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# GPIO Pin tanımları
GPIO_STATUS_LED = 4
GPIO_BUZZER_PWM = 13
GPIO_POWER_BUTTON = 18
GPIO_EMERGENCY_STOP = 19

# I2C Devices
I2C_D300_ADDRESS = 0x76

# Görev parametreleri
MISSION_PARAMS = {
    'target_depth': 2.0,
    'straight_distance': 10.0,
    'min_outbound_distance': 50.0,
    'cruise_speed': 1.5,
    'return_speed': 1.8,
    'timeout_seconds': 300,
    'position_tolerance': 2.0,
    'depth_tolerance': 0.2
}

# ArduPlane GUIDED Mode Parametreleri
GUIDED_MODE_CONFIG = {
    'default_airspeed': 12.0,
    'waypoint_radius': 5.0,
    'loiter_radius': 10.0,
    'guided_timeout': 30.0,
    'max_bank_angle': 30.0,
    'max_pitch_angle': 20.0
}

# Kontrol parametreleri
CONTROL_PARAMS = {
    'depth_pid':   {'kp': 120.0, 'ki': 6.0,  'kd': 35.0, 'max_output': 200}
}

# Kontrol frekansı
CONTROL_FREQUENCY = 10  # Hz

# Basit PID Controller (sadece derinlik için)
class PIDController:
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

    def update(self, setpoint, measurement):
        now = time.time()
        dt = max(0.01, now - self.last_time)

        error = setpoint - measurement
        self.integral += error * dt
        if self.ki > 0:
            integral_limit = self.max_output / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))

        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.max_output, min(self.max_output, output))

        self.previous_error = error
        self.last_time = now
        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

class Mission1Navigator:
    """
    TEKNOFEST Su Altı Roket Aracı - ArduPlane GUIDED Mode Mission Controller
    X-Wing Configuration
    
    Simplified approach:
    - ArduPlane handles stabilization and servo mixing
    - We handle high-level mission logic and depth control
    """
    def __init__(self):
        self.master = None
        self.connected = False
        
        # GPIO sistemi başlat
        self.gpio_initialized = False
        self._init_gpio_system()

        # D300 derinlik sensörü
        self.d300_sensor = None
        self.d300_connected = False
        if D300_AVAILABLE:
            try:
                self.d300_sensor = D300DepthSensor(i2c_address=I2C_D300_ADDRESS)
                self.d300_connected = self.d300_sensor.initialize()
                if self.d300_connected:
                    print("✅ D300 derinlik sensörü başlatıldı")
                else:
                    print("⚠️ D300 sensörü başlatılamadı, SCALED_PRESSURE kullanılacak")
            except Exception as e:
                print(f"⚠️ D300 sensörü hatası: {e}, SCALED_PRESSURE kullanılacak")
                self.d300_connected = False

        # Mission state
        self.mission_active = False
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.mission_completion_time = None

        # Position tracking
        self.start_position = {'lat': 0.0, 'lon': 0.0}
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Dead-reckoning
        self.initial_heading = None
        self.traveled_distance = 0.0
        self.current_pwm = 1500
        self.estimated_speed = 0.0
        self.filtered_speed = 0.0
        
        # Mission metrics
        self.straight_distance_completed = 0.0
        self.outbound_distance = 0.0
        self.inbound_distance = 0.0
        self.max_offshore_distance = 0.0
        self.final_position_error_est = float('inf')
        self.leak_detected = False
        
        # Status tracking
        self.depth_source = "unknown"
        self._latched_fault = None

        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False

        # Telemetry
        self.telemetry_data = []

        # PID Controllers
        self.depth_pid = PIDController(**CONTROL_PARAMS['depth_pid'])

        print("✅ ArduPlane GUIDED X-Wing Mission Controller başlatıldı")
    
    def _init_gpio_system(self):
        """GPIO sistemi başlatma (LED & Buzzer)"""
        if not GPIO_AVAILABLE:
            return
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Output pinleri
            GPIO.setup(GPIO_STATUS_LED, GPIO.OUT)
            GPIO.setup(GPIO_BUZZER_PWM, GPIO.OUT)
            
            # Input pinleri (pull-up dirençli)
            GPIO.setup(GPIO_POWER_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(GPIO_EMERGENCY_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Başlangıçta kapalı
            GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
            GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
            
            self.gpio_initialized = True
            print("✅ GPIO sistemi başlatıldı")
            
        except Exception as e:
            print(f"⚠️ GPIO başlatma hatası: {e}")
            self.gpio_initialized = False
    
    def _update_status_indicators(self, stage, fault=None):
        """Durum göstergelerini güncelle (LED & Buzzer)"""
        if not self.gpio_initialized:
            return
            
        try:
            if fault:
                # FAULT: Hızlı flash LED + uzun bip×3
                for _ in range(6):
                    GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                    time.sleep(0.1)
                return
                
                # Normal operasyon: Stage'e göre LED
                if stage in ["DESCENT", "STRAIGHT_COURSE", "OFFSHORE_CRUISE"]:
                    GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)  # Sabit yanar
                elif stage in ["RETURN_NAVIGATION", "FINAL_APPROACH"]:
                    # Yavaş flash (0.5 Hz)
                    led_state = int(time.time() * 2) % 2
                    GPIO.output(GPIO_STATUS_LED, led_state)
                elif stage == "MISSION_COMPLETE":
                    # Başarı: 3 kısa bip
                    for _ in range(3):
                        GPIO.output(GPIO_BUZZER_PWM, GPIO.HIGH)
                        time.sleep(0.2)
                        GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                        time.sleep(0.2)
                    GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)  # LED sabit yanar
                    
        except Exception as e:
            print(f"⚠️ GPIO güncelleme hatası: {e}")

    # ---------------- MAVLink ----------------
    def connect_pixhawk(self):
        """ArduPlane bağlantısı kur"""
        try:
            print("🔌 ArduPlane bağlantısı kuruluyor...")
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                print(f"🌐 TCP/UDP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)

            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            print(f"   System ID: {self.master.target_system}")
            print(f"   Component ID: {self.master.target_component}")
            
            # Stream rate istekleri
            self._request_data_streams()
            
            # GUIDED mode'a geç
            self._set_flight_mode("GUIDED")
            
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def _request_data_streams(self):
        """MAVLink data stream hızlarını ayarla"""
        try:
            # Position data
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                10,  # 10 Hz
                1    # Enable
            )
            
            # Attitude data
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
                10,  # 10 Hz
                1    # Enable
            )
            
            # Pressure data
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,  # SCALED_PRESSURE
                5,   # 5 Hz
                1    # Enable
            )
            
            print("📊 Stream rate istekleri gönderildi")
        except Exception as e:
            print(f"⚠️ Stream rate ayarlama hatası: {e}")

    def _set_flight_mode(self, mode_name):
        """ArduPlane flight mode ayarla"""
        try:
            # Get mode number
            if mode_name not in self.master.mode_mapping():
                print(f"❌ Bilinmeyen flight mode: {mode_name}")
                return False
            
            mode_id = self.master.mode_mapping()[mode_name]
            
            # Set mode
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            
            print(f"✅ Flight mode set: {mode_name}")
            return True
            
        except Exception as e:
            print(f"❌ Flight mode ayarlama hatası: {e}")
            return False

    def read_sensors(self):
        """GPS YOK: Sadece ATTITUDE ve SCALED_PRESSURE, ayrıca leak için STATUSTEXT dinlenir."""
        if not self.connected:
            return False
        
        # Latched fault kontrolü
        if self._latched_fault:
            return False
            
        try:
            current_time = time.time()
            
            # GPS Position (if available)
            gps_msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if gps_msg:
                self.current_position['lat'] = gps_msg.lat / 1e7
                self.current_position['lon'] = gps_msg.lon / 1e7
                self.current_position['alt'] = gps_msg.alt / 1000.0
            
            # Attitude
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw
                if self.current_heading < 0:
                    self.current_heading += 360

            # Derinlik sensörü (D300 öncelikli, yoksa SCALED_PRESSURE)
            depth_read_success = False
            if self.d300_connected and self.d300_sensor:
                try:
                    depth_data = self.d300_sensor.read_depth()
                    if depth_data['success']:
                        self.current_depth = max(0.0, depth_data['depth'])
                        self.depth_source = "d300"
                        depth_read_success = True
                except Exception as e:
                    print(f"⚠️ D300 okuma hatası: {e}")
                    
            if not depth_read_success:
                # D300 yok veya hatalı, SCALED_PRESSURE kullan
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    depth_pressure = pressure_msg.press_abs - 1013.25
                    self.current_depth = max(0.0, depth_pressure * 0.0102)
                    self.depth_source = "scaled_pressure"

            # Leak uyarısı (metin üzerinden basit tespit)
            statustext = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if statustext and statustext.text:
                txt = statustext.text.lower()
                if "leak" in txt or "water" in txt:
                    self.leak_detected = True
                    self._trigger_latched_fault("LEAK_DETECTED")

            # Telemetri log
            self.telemetry_data.append({
                't': time.time(),
                'position': self.current_position.copy(),
                'depth': self.current_depth,
                'heading': self.current_heading,
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'stage': self.mission_stage,
                'leak_detected': self.leak_detected
            })
            return True
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            self._trigger_latched_fault(f"SENSOR_READ_ERROR: {e}")
            return False
    
    def _trigger_latched_fault(self, fault_reason):
        """Kalıcı hata durumu tetikle"""
        if not self._latched_fault:  # Sadece ilk hata için
            self._latched_fault = fault_reason
            print(f"🚨 LATCHED FAULT: {fault_reason}")
            print("🚨 Sistem güvenli duruma geçiyor...")
            
            # Mission'ı abort et
            self.mission_stage = "MISSION_ABORT"
            self.mission_active = False
            
            # Emergency RTL
            self._set_flight_mode("RTL")
            
    def set_guided_waypoint(self, lat, lon, alt):
        """ArduPlane GUIDED mode waypoint komutu gönder"""
        if not self.connected:
            return False
            
        try:
            # SET_POSITION_TARGET_GLOBAL_INT komutu
            self.master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                0b0000111111111000,  # type_mask (position only)
                int(lat * 1e7),      # lat_int
                int(lon * 1e7),      # lon_int
                alt,                 # alt
                0, 0, 0,            # vx, vy, vz
                0, 0, 0,            # afx, afy, afz
                0, 0                # yaw, yaw_rate
            )
            
            print(f"✅ GUIDED waypoint gönderildi: {lat:.6f}, {lon:.6f}, {alt:.1f}m")
            return True
            
        except Exception as e:
            print(f"❌ GUIDED waypoint hatası: {e}")
            return False

    def set_depth_correction(self, depth_correction):
        """Derinlik düzeltmesi için throttle override"""
        if not self.connected:
            return False
        
        try:
            # Throttle override (1000-2000 range)
            throttle_pwm = 1500 + int(depth_correction)
            throttle_pwm = max(1000, min(2000, throttle_pwm))
            
            # Manual throttle override
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                3,  # Throttle channel (usually channel 3)
                throttle_pwm,
                0, 0, 0, 0, 0
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Depth correction hatası: {e}")
            return False
    
    def calculate_distance_bearing(self, lat1, lon1, lat2, lon2):
        """İki GPS koordinatı arası mesafe ve bearing hesaplama"""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula for distance
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters
        
        # Bearing calculation
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        bearing = (bearing + 360) % 360  # Normalize to 0-360
        
        return distance, bearing

    def get_current_distance_from_start(self):
        """Başlangıç noktasından mevcut uzaklık"""
        if self.start_position['lat'] == 0.0 and self.start_position['lon'] == 0.0:
            return 0.0
        
        distance, _ = self.calculate_distance_bearing(
            self.start_position['lat'], self.start_position['lon'],
            self.current_position['lat'], self.current_position['lon']
        )
        return distance

    def update_pwm_based_odometry(self):
        """PWM tabanlı hız kestirimi ve mesafe güncelleme - Basitleştirilmiş"""
        # ArduPlane GUIDED mode'da bu fonksiyon basitleştirildi
        # Gerçek mesafe GPS'den alınır
        pass

    def display_status(self):
        print("\n" + "="*80)
        print("🚀 TEKNOFEST - GÖREV 1 (ArduPlane GUIDED - X-Wing)")
        print("="*80)
        t_now = datetime.now().strftime("%H:%M:%S")
        elapsed = (time.time() - self.mission_start_time) if self.mission_start_time else 0.0
        remaining = max(0, MISSION_PARAMS['timeout_seconds'] - elapsed)
        print(f"⏰ Zaman: {t_now} | Süre: {elapsed:.0f}s | Kalan: {remaining:.0f}s")
        print(f"🎯 Aşama: {self.mission_stage}")
        print(f"🌊 Derinlik: {self.current_depth:.2f} m (hedef {MISSION_PARAMS['target_depth']:.2f} m)")
        print(f"🧭 Heading: {self.current_heading:.1f}° | Stabilizasyon: ArduPlane GUIDED")
        print(f"📍 Pozisyon: {self.current_position['lat']:.6f}, {self.current_position['lon']:.6f}")
        distance_from_start = self.get_current_distance_from_start()
        print(f"🏠 Başlangıçtan uzaklık: {distance_from_start:.1f}m")
        print("="*80)

    # --------------- Aşamalar ---------------
    def control_loop(self):
        """Ana kontrol döngüsü - ArduPlane GUIDED komutları"""
        loop_count = 0
        
        while self.running and self.mission_active:
            cycle_start = time.time()
            
            self.read_sensors()

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

            # Timing kontrolü
            cycle_time = time.time() - cycle_start
            target_cycle_time = 1.0 / CONTROL_FREQUENCY
            sleep_time = max(0.01, target_cycle_time - cycle_time)
            time.sleep(sleep_time)
            
            loop_count += 1

    def execute_descent(self):
        """2m derinliğe iniş - ArduPlane GUIDED + Manuel derinlik kontrolü"""
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            # Derinlik kontrolü için throttle correction
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            self.set_depth_correction(depth_output)
            
            # Mevcut pozisyonda loiter (ArduPlane GUIDED)
            if self.current_position['lat'] != 0.0:
                self.set_guided_waypoint(
                    self.current_position['lat'],
                    self.current_position['lon'],
                    -self.current_depth  # Negative altitude for underwater
                )
        else:
            print("✅ Hedef derinlik ulaşıldı! Düz seyire geçiliyor...")
            self.mission_stage = "STRAIGHT_COURSE"
            self.initial_heading = self.current_heading
            self.depth_pid.reset()

    def execute_straight_course(self):
        """10m düz seyir - ArduPlane GUIDED waypoint"""
        if not hasattr(self, '_straight_waypoint_set'):
            # İleri doğru 10m waypoint hesapla
            bearing = self.current_heading
            distance = MISSION_PARAMS['straight_distance']
            
            # Yeni koordinat hesapla
            lat_rad = math.radians(self.current_position['lat'])
            lon_rad = math.radians(self.current_position['lon'])
            bearing_rad = math.radians(bearing)
            
            new_lat_rad = math.asin(
                math.sin(lat_rad) * math.cos(distance / 6371000) +
                math.cos(lat_rad) * math.sin(distance / 6371000) * math.cos(bearing_rad)
            )
            new_lon_rad = lon_rad + math.atan2(
                math.sin(bearing_rad) * math.sin(distance / 6371000) * math.cos(lat_rad),
                math.cos(distance / 6371000) - math.sin(lat_rad) * math.sin(new_lat_rad)
            )
            
            target_lat = math.degrees(new_lat_rad)
            target_lon = math.degrees(new_lon_rad)
            
            # ArduPlane'e waypoint gönder
            self.set_guided_waypoint(target_lat, target_lon, -MISSION_PARAMS['target_depth'])
            self._straight_waypoint_set = True
            self._straight_target = {'lat': target_lat, 'lon': target_lon}
            
            print(f"✅ Düz seyir waypoint set: {target_lat:.6f}, {target_lon:.6f}")
        
        # Derinlik kontrolü
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            self.set_depth_correction(depth_output)
        
        # Waypoint'e ulaşıldı mı kontrol et
        if hasattr(self, '_straight_target'):
            distance_to_waypoint, _ = self.calculate_distance_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self._straight_target['lat'], self._straight_target['lon']
            )
            
            if distance_to_waypoint < GUIDED_MODE_CONFIG['waypoint_radius']:
                print("✅ 10m düz seyir tamamlandı! İleri mesafe hedefi için devam...")
                self.mission_stage = "OFFSHORE_CRUISE"
                delattr(self, '_straight_waypoint_set')

    def execute_offshore_cruise(self):
        """Kıyıdan uzaklaşma - ArduPlane GUIDED"""
        distance_from_start = self.get_current_distance_from_start()
        self.max_offshore_distance = max(self.max_offshore_distance, distance_from_start)
        
        if distance_from_start < MISSION_PARAMS['min_outbound_distance']:
            if not hasattr(self, '_offshore_waypoint_set'):
                # İleri doğru 50m waypoint hesapla
                bearing = self.current_heading
                remaining_distance = MISSION_PARAMS['min_outbound_distance'] - distance_from_start
                
                # Yeni koordinat hesapla
                lat_rad = math.radians(self.current_position['lat'])
                lon_rad = math.radians(self.current_position['lon'])
                bearing_rad = math.radians(bearing)
                
                new_lat_rad = math.asin(
                    math.sin(lat_rad) * math.cos(remaining_distance / 6371000) +
                    math.cos(lat_rad) * math.sin(remaining_distance / 6371000) * math.cos(bearing_rad)
                )
                new_lon_rad = lon_rad + math.atan2(
                    math.sin(bearing_rad) * math.sin(remaining_distance / 6371000) * math.cos(lat_rad),
                    math.cos(remaining_distance / 6371000) - math.sin(lat_rad) * math.sin(new_lat_rad)
                )
                
                target_lat = math.degrees(new_lat_rad)
                target_lon = math.degrees(new_lon_rad)
                
                # ArduPlane'e waypoint gönder
                self.set_guided_waypoint(target_lat, target_lon, -MISSION_PARAMS['target_depth'])
                self._offshore_waypoint_set = True
                
                print(f"✅ Offshore waypoint set: {target_lat:.6f}, {target_lon:.6f}")
            
            # Derinlik kontrolü
            depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
            if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
                depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
                self.set_depth_correction(depth_output)
        else:
            print("✅ İleri toplam mesafe tamamlandı! Geri dönüş başlıyor...")
            self.mission_stage = "RETURN_NAVIGATION"
            if hasattr(self, '_offshore_waypoint_set'):
                delattr(self, '_offshore_waypoint_set')

    def execute_return_navigation(self):
        """Başlangıç noktasına geri dönüş - ArduPlane GUIDED"""
        if not hasattr(self, '_return_waypoint_set'):
            # Başlangıç noktasına waypoint gönder
            self.set_guided_waypoint(
                self.start_position['lat'],
                self.start_position['lon'],
                -MISSION_PARAMS['target_depth']
            )
            self._return_waypoint_set = True
            print(f"✅ Return waypoint set: {self.start_position['lat']:.6f}, {self.start_position['lon']:.6f}")
        
        # Derinlik kontrolü
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            self.set_depth_correction(depth_output)
        
        # Başlangıç noktasına yaklaştık mı?
        distance_from_start = self.get_current_distance_from_start()
        if distance_from_start <= MISSION_PARAMS['position_tolerance'] * 2:
            print("✅ Başlangıça tahmini dönüş tamamlandı! Final yaklaşım...")
            self.mission_stage = "FINAL_APPROACH"
            delattr(self, '_return_waypoint_set')

    def execute_final_approach(self):
        """Final yaklaşım ve pozisyon tutma"""
        distance_from_start = self.get_current_distance_from_start()
        self.final_position_error_est = distance_from_start
        
        # Başlangıç noktasında loiter
        if not hasattr(self, '_final_loiter_set'):
            self.set_guided_waypoint(
                self.start_position['lat'],
                self.start_position['lon'],
                -MISSION_PARAMS['target_depth']
            )
            self._final_loiter_set = True
        
        # Derinlik kontrolü
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            self.set_depth_correction(depth_output)
        
        # 5 saniye pozisyon tutma sayacı
        if distance_from_start <= MISSION_PARAMS['position_tolerance']:
            if not hasattr(self, '_final_hold_start'):
                self._final_hold_start = time.time()
            elif (time.time() - self._final_hold_start) >= 5.0:
                print("✅ Final pozisyon tahmini stabil! Yüzeye çıkış ve enerji kesme...")
                self.mission_stage = "SURFACE_AND_SHUTDOWN"
        else:
            self._final_hold_start = None

    def execute_surface_shutdown(self):
        """Yüzeye çıkış ve sistem kapatma"""
        # Yüzeye çıkış waypoint
        self.set_guided_waypoint(
            self.current_position['lat'],
            self.current_position['lon'],
            0.0  # Surface level
        )

        # Yüzeye çıkana kadar bekle (yaklaşık derinlik < 0.5 m)
        if self.current_depth > 0.5:
            return

        print("🌊 Yüzeye çıkış tamamlandı!")
        print("⚡ Sistem enerjisi kesiliyor...")

        # RTL mode'a geç (safe landing)
        self._set_flight_mode("RTL")

        self.mission_completion_time = time.time()
        self.mission_stage = "MISSION_COMPLETE"
        print("✅ GÖREV 1 TAMAMLANDI!")

    # --------------- İzleme ---------------
    def monitoring_loop(self):
        while self.running and self.mission_active:
            # Durumu periyodik göster
            if len(self.telemetry_data) % 30 == 0:
                self.display_status()

            # Süre limiti
            if self.mission_start_time and (time.time() - self.mission_start_time) > MISSION_PARAMS['timeout_seconds']:
                print("⏰ Süre doldu! Görev sonlandırılıyor...")
                self.mission_stage = "MISSION_COMPLETE"
                break
            time.sleep(1.0)

    # --------------- Rapor ---------------
    def generate_mission_report(self):
        """Detaylı görev raporu oluştur"""
        duration = (self.mission_completion_time - self.mission_start_time) if (self.mission_completion_time and self.mission_start_time) else 0.0

        print("\n" + "="*80)
        print("📋 GÖREV 1 RAPORU (ArduPlane GUIDED - X-Wing)")
        print("="*80)
        print(f"📅 Tarih: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Süre: {duration:.1f} s")
        print("\n📊 METRİKLER:")
        print("-"*60)
        distance_from_start = self.get_current_distance_from_start()
        print(f"🌊 Maksimum Kıyı Uzaklığı: {self.max_offshore_distance:.1f} m / {MISSION_PARAMS['min_outbound_distance']} m")
        print(f"🎯 Final Pozisyon Hatası: {self.final_position_error_est:.1f} m (tolerans {MISSION_PARAMS['position_tolerance']} m)")
        print(f"💧 Sızdırmazlık: {'✅ BAŞARILI' if not self.leak_detected else '❌ SIZINTI'}")
        print(f"🛩️ Stabilizasyon: ArduPlane GUIDED Mode")

        print("\n🏆 PUANLAMA (yaklaşık):")
        print("-"*40)
        # Hız/seyir puanı (koşullar sağlandıysa süreye göre)
        time_factor = max(0, (300 - duration) / 300) if duration > 0 else 0
        cruise_ok = self.max_offshore_distance >= MISSION_PARAMS['min_outbound_distance']
        cruise_points = int(150 * time_factor) if cruise_ok else 0
        print(f"  🚀 Seyir Yapma (hız): {cruise_points}/150")

        position_points = 90 if self.final_position_error_est <= MISSION_PARAMS['position_tolerance'] else 0
        print(f"  🎯 Başlangıçta Enerji Kesme: {position_points}/90")

        waterproof_points = 60 if not self.leak_detected else 0
        print(f"  💧 Sızdırmazlık: {waterproof_points}/60")

        total_points = cruise_points + position_points + waterproof_points
        print(f"\n📈 TOPLAM: {total_points}/300")

        report = {
            'timestamp': datetime.now().isoformat(),
            'mission_type': 'ArduPlane GUIDED X-Wing Mission 1',
            'duration_s': duration,
            'system_info': {
                'stabilization': 'ArduPlane GUIDED Mode',
                'configuration': 'X-Wing',
                'depth_control': 'Manual PID'
            },
            'metrics': {
                'max_offshore_distance_m': self.max_offshore_distance,
                'final_position_error_est_m': self.final_position_error_est,
                'leak_detected': self.leak_detected,
                'latched_fault': self._latched_fault
            },
            'scoring': {
                'cruise_points': cruise_points,
                'position_points': position_points,
                'waterproof_points': waterproof_points,
                'total_points': total_points
            }
        }

        fname = f'mission_1_guided_x_wing_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(fname, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"\n💾 Rapor kaydedildi: {fname}")

        return total_points >= 180  # %60 başarı eşiği

    def cleanup(self):
        """Sistem temizleme"""
        self.mission_active = False
        self.running = False
        print("\n🧹 Sistem temizleniyor...")

        # ArduPlane'i güvenli moda al
        if self.connected:
            self._set_flight_mode("LOITER")

        # D300 sensörünü kapat
        if self.d300_connected and self.d300_sensor:
            try:
                self.d300_sensor.close()
                print("🔌 D300 derinlik sensörü kapatıldı")
            except:
                pass

        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

        print("✅ Sistem temizleme tamamlandı")

    # --------------- Yaşam Döngüsü ---------------
    def run_mission_1(self):
        """Ana Mission 1 execution fonksiyonu - ArduPlane GUIDED"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - GÖREV 1 (ArduPlane GUIDED - X-Wing)")
        print("="*80)
        print("🎯 Görev: Seyir Yapma & Başlangıç Noktasına Geri Dönüş")
        print("🛩️ Stabilizasyon: ArduPlane GUIDED Mode")
        print("⏱️ Süre Limiti: 5 dakika")
        print("🏆 Maksimum Puan: 300")

        if not self.connect_pixhawk():
            print("❌ ArduPlane bağlantısı başarısız!")
            return False

        # GPS pozisyon al (başlangıç noktası)
        print("📍 GPS pozisyon bekleniyor...")
        for i in range(30):  # 30 saniye bekle
            if self.read_sensors():
                if self.current_position['lat'] != 0.0 and self.current_position['lon'] != 0.0:
                    self.start_position = self.current_position.copy()
                    print(f"✅ Başlangıç pozisyonu: {self.start_position['lat']:.6f}, {self.start_position['lon']:.6f}")
                    break
            time.sleep(1.0)
        else:
            print("❌ GPS pozisyon alınamadı!")
            return False

        print("\n⚠️ GÖREV HAZIRLIĞI:")
        print("- ArduPlane GUIDED mode aktif mi?")
        print("- Tüm sistemler hazır mı?")
        print("- Güvenlik kontrolleri tamam mı?")
        print("- Şamandıra takılı mı?")

        try:
            ready = input("\n✅ Görev 1 başlasın mı? (y/n): ").lower()
        except EOFError:
            ready = 'y'  # non-interaktif çalıştırma için otomatik onay
        if ready != 'y':
            print("❌ Görev iptal edildi")
            return False

        self.mission_start_time = time.time()
        self.mission_active = True
        self.running = True
        self.mission_stage = "DESCENT"

        # Threadler
        self.control_thread = threading.Thread(target=self.control_loop, daemon=False)
        self.control_thread.start()
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop, daemon=False)
        self.monitoring_thread.start()

        try:
            print("\n🚀 GÖREV 1 BAŞLADI! (ArduPlane GUIDED)")
            self.control_thread.join()  # kontrol tamamlanana kadar
            success = self.generate_mission_report()
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
            # Her durumda güvenli kapanış
            self.cleanup()

# ------------------ main ------------------
def main():
    parser = argparse.ArgumentParser(description='TEKNOFEST Görev 1 ArduPlane GUIDED (X-Wing)')
    args = parser.parse_args()

    mission = Mission1Navigator()
    try:
        ok = mission.run_mission_1()
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
