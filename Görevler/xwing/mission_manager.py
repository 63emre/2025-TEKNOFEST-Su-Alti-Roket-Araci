#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Mission Manager
ArduPlane GUIDED MODE - Unified Mission Control

Bu mission manager tüm görevleri ArduPlane GUIDED modu ile yönetir.
ArduPlane stabilizasyon yapar, biz sadece high-level mission logic yürütürüz.
"""

import time
import threading
import math
import json
import os
import argparse
from datetime import datetime
from pymavlink import mavutil

# GPIO kontrol sistemi
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

# D300 derinlik sensörü
try:
    import sys
    _BASE_DIR = os.path.dirname(__file__) if '__file__' in globals() else os.getcwd()
    _APP_DIR = os.path.normpath(os.path.join(_BASE_DIR, '../App'))
    if _APP_DIR not in sys.path:
        sys.path.append(_APP_DIR)
    from depth_sensor import D300DepthSensor
    D300_AVAILABLE = True
except ImportError:
    D300_AVAILABLE = False

# MAVLink bağlantı adresi
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# GPIO Pin tanımları
GPIO_STATUS_LED = 4
GPIO_BUZZER_PWM = 13
GPIO_POWER_BUTTON = 18
GPIO_EMERGENCY_STOP = 19
I2C_D300_ADDRESS = 0x76

# ArduPlane GUIDED Mode Parametreleri
GUIDED_MODE_CONFIG = {
    'default_airspeed': 12.0,
    'waypoint_radius': 5.0,
    'loiter_radius': 10.0,
    'guided_timeout': 30.0
}

# Mission parametreleri
MISSION_PARAMS = {
    'mission_1': {
        'target_depth': 2.0,
        'straight_distance': 10.0,
        'min_offshore_distance': 50.0,
        'timeout_seconds': 300,
        'position_tolerance': 2.0,
        'depth_tolerance': 0.2
    },
    'mission_2': {
        'target_depth': 1.5,
        'approach_distance': 30.0,
        'launch_distance': 10.0,
        'timeout_seconds': 180,
        'position_tolerance': 3.0,
        'depth_tolerance': 0.3,
        'rocket_count': 2
    }
}

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

class ArduPlaneMissionManager:
    """ArduPlane GUIDED Mode Mission Manager"""
    
    def __init__(self, mission_type="mission_1", configuration="x_wing"):
        self.mission_type = mission_type
        self.configuration = configuration
        
        # MAVLink connection
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
                print("✅ D300 derinlik sensörü başlatıldı" if self.d300_connected else "⚠️ D300 sensörü başlatılamadı")
                except Exception as e:
                print(f"⚠️ D300 sensörü hatası: {e}")
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
        
        # Mission-specific data
        self.mission_data = {
            'max_offshore_distance': 0.0,
            'rockets_launched': 0,
            'final_position_error': float('inf'),
            'leak_detected': False,
            'latched_fault': None
        }
        
        # Controllers
        self.depth_pid = PIDController(kp=120.0, ki=5.0, kd=35.0, max_output=200)
        
        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False
        
        # Telemetry
        self.telemetry_data = []
        
        print(f"✅ ArduPlane GUIDED Mission Manager başlatıldı")
        print(f"   - Mission Type: {mission_type}")
        print(f"   - Configuration: {configuration}")
        print(f"   - Stabilizasyon: ArduPlane GUIDED mode")
    
    def _init_gpio_system(self):
        """GPIO sistemi başlatma"""
        if not GPIO_AVAILABLE:
            return
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            GPIO.setup(GPIO_STATUS_LED, GPIO.OUT)
            GPIO.setup(GPIO_BUZZER_PWM, GPIO.OUT)
            GPIO.setup(GPIO_POWER_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(GPIO_EMERGENCY_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
            GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
            
            self.gpio_initialized = True
            print("✅ GPIO sistemi başlatıldı")
            
        except Exception as e:
            print(f"⚠️ GPIO başlatma hatası: {e}")
            self.gpio_initialized = False
    
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
            
            # Stream rate istekleri ve GUIDED mode
            self._request_data_streams()
            self._set_flight_mode("GUIDED")
            
                return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
        return False
    
    def _request_data_streams(self):
        """MAVLink data stream hızlarını ayarla"""
        try:
            streams = [
                (mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 10),
                (mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 5)
            ]
            
            for stream_type, rate in streams:
                self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                    stream_type,
                    rate,
                    1  # Enable
                )
            
            print("📊 Stream rate istekleri gönderildi")
        except Exception as e:
            print(f"⚠️ Stream rate ayarlama hatası: {e}")
    
    def _set_flight_mode(self, mode_name):
        """ArduPlane flight mode ayarla"""
        try:
            if mode_name not in self.master.mode_mapping():
                print(f"❌ Bilinmeyen flight mode: {mode_name}")
                return False
            
            mode_id = self.master.mode_mapping()[mode_name]
            
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
        """Tüm sensör verilerini oku"""
        if not self.connected or self.mission_data['latched_fault']:
                return False
        
        try:
            # GPS Position
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
                        depth_read_success = True
                except Exception as e:
                    print(f"⚠️ D300 okuma hatası: {e}")
            
            if not depth_read_success:
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    depth_pressure = pressure_msg.press_abs - 1013.25
                    self.current_depth = max(0.0, depth_pressure * 0.0102)
            
            # Leak tespiti
            statustext = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if statustext and statustext.text:
                text_lower = statustext.text.lower()
                if "leak" in text_lower or "water" in text_lower:
                    self.mission_data['leak_detected'] = True
                    self._trigger_latched_fault("LEAK_DETECTED")
            return False
        
            # Telemetri kaydet
            self.telemetry_data.append({
                'timestamp': time.time(),
                'position': self.current_position.copy(),
                'depth': self.current_depth,
                'heading': self.current_heading,
                'mission_stage': self.mission_stage
            })
            
        return True
    
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            self._trigger_latched_fault(f"SENSOR_READ_ERROR: {e}")
            return False
    
    def _trigger_latched_fault(self, fault_reason):
        """Kalıcı hata durumu tetikle"""
        if not self.mission_data['latched_fault']:
            self.mission_data['latched_fault'] = fault_reason
            print(f"🚨 LATCHED FAULT: {fault_reason}")
            print("🚨 Emergency RTL aktif...")
            
            self.mission_stage = "MISSION_ABORT"
            self.mission_active = False
            self._set_flight_mode("RTL")
    
    def set_guided_waypoint(self, lat, lon, alt):
        """ArduPlane GUIDED mode waypoint komutu"""
        if not self.connected:
        return False
    
        try:
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
            
            return True
            
        except Exception as e:
            print(f"❌ GUIDED waypoint hatası: {e}")
            return False
    
    def set_depth_correction(self, depth_correction):
        """Derinlik düzeltmesi için throttle override"""
        if not self.connected:
            return False
        
        try:
            throttle_pwm = 1500 + int(depth_correction)
            throttle_pwm = max(1000, min(2000, throttle_pwm))
            
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                3,  # Throttle channel
                throttle_pwm,
                0, 0, 0, 0, 0
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Depth correction hatası: {e}")
            return False
    
    def calculate_distance_bearing(self, lat1, lon1, lat2, lon2):
        """GPS koordinatları arası mesafe ve bearing hesaplama"""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c
        
        # Bearing
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        bearing = (bearing + 360) % 360
        
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
    
    def cleanup(self):
        """Sistem temizleme"""
        print("\n🧹 Sistem temizleniyor...")
        
        self.running = False
        self.mission_active = False
        
        if self.connected:
            self._set_flight_mode("LOITER")
        
        if self.d300_connected and self.d300_sensor:
            try:
                self.d300_sensor.close()
            except:
                pass
        
        if self.connected and self.master:
            try:
                self.master.close()
            except:
                pass
            self.connected = False
        
        if self.gpio_initialized:
            try:
                GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
                GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                GPIO.cleanup()
            except:
                pass
        
        print("✅ Sistem temizleme tamamlandı")
    
    def run_mission(self):
        """Ana görev çalıştırma fonksiyonu"""
        print(f"🚀 TEKNOFEST - {self.mission_type.upper()} (ArduPlane GUIDED)")
        
        if not self.connect_pixhawk():
            return False
        
        # GPS pozisyon al
        print("📍 GPS pozisyon bekleniyor...")
        for i in range(30):
            if self.read_sensors():
                if self.current_position['lat'] != 0.0 and self.current_position['lon'] != 0.0:
                    self.start_position = self.current_position.copy()
                    print(f"✅ Başlangıç pozisyonu: {self.start_position['lat']:.6f}, {self.start_position['lon']:.6f}")
                    break
            time.sleep(1.0)
        else:
            print("❌ GPS pozisyon alınamadı!")
            return False
        
        try:
            ready = input(f"\n✅ {self.mission_type.upper()} başlasın mı? (y/n): ").lower()
        except EOFError:
            ready = 'y'
        
        if ready != 'y':
            print("❌ Görev iptal edildi")
            return False
        
        self.mission_start_time = time.time()
        self.mission_active = True
        self.running = True
        self.mission_stage = "DESCENT"
        
        print(f"\n🚀 {self.mission_type.upper()} BAŞLADI!")
        
        # Basit mission loop (detaylar için önceki implementation'a bakın)
        try:
            while self.running and self.mission_active:
                if not self.read_sensors():
                    continue
                
                # Mission stages burada implement edilecek
                # (Önceki implementation'dan alınabilir)
                
                time.sleep(1.0 / CONTROL_FREQUENCY)
                
        except KeyboardInterrupt:
            print(f"\n⚠️ {self.mission_type.upper()} kullanıcı tarafından durduruldu")
            return False
            except Exception as e:
            print(f"\n❌ {self.mission_type.upper()} hatası: {e}")
            return False
            finally:
            self.cleanup()
        
        return True
    
def main():
    """Ana fonksiyon"""
    parser = argparse.ArgumentParser(description='TEKNOFEST ArduPlane GUIDED Mission Manager')
    parser.add_argument('--mission', choices=['mission_1', 'mission_2'], default='mission_1',
                       help='Mission type to execute')
    parser.add_argument('--config', choices=['x_wing', 'plus_wing'], default='x_wing',
                       help='Aircraft configuration')
    
    args = parser.parse_args()
    
    mission_manager = ArduPlaneMissionManager(mission_type=args.mission, configuration=args.config)
    
    try:
        success = mission_manager.run_mission()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())