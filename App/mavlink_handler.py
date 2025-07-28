#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - MAVLink Handler
Pixhawk PX4 PIX 2.4.8 MAVLink Serial Communication Handler
"""

import time
import math
import json
import threading
import os
from collections import deque
from pymavlink import mavutil, mavwp

class MAVLinkHandler:
    """MAVLink protokolü ile Pixhawk bağlantısı"""
    
    def __init__(self, config_path="config/hardware_config.json"):
        """MAVLink handler başlat"""
        # Environment variable support
        self.serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
        self.baud_rate = int(os.getenv("MAV_BAUD", "115200"))
        
        # Config'den ayarları yükle
        self.config = self.load_config(config_path)
        
        # MAVLink connection
        self.master = None
        self.connected = False
        self.armed = False
        
        # Control modes
        self.control_mode = "raw"  # "raw" veya "pid"
        self.navigation_mode = "gps_only"
        
        # Data buffers
        self.imu_data_buffer = deque(maxlen=100)
        self.gps_data_buffer = deque(maxlen=50)
        self.depth_data_buffer = deque(maxlen=50)
        
        # Thread safety
        self.data_lock = threading.Lock()
    
    def load_config(self, config_path):
        """Config dosyasını yükle"""
        default_config = {
            "mavlink": {"connection_string": "/dev/ttyACM0", "baudrate": 115200}
        }
        
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # Environment variables'dan connection string güncelle
            if "mavlink" in config:
                # Environment variable varsa onu kullan
                config["mavlink"]["connection_string"] = self.serial_port
                config["mavlink"]["baudrate"] = self.baud_rate
            
            return config
        except Exception as e:
            print(f"⚠️ Config yükleme hatası: {e}")
            return default_config
    
    def connect(self):
        """Pixhawk'a serial bağlantı kur"""
        try:
            print(f"🔌 Pixhawk serial bağlantısı kuruluyor...")
            print(f"   Port: {self.serial_port}")
            print(f"   Baud: {self.baud_rate}")
            
            # Serial connection string oluştur
            connection_string = f"{self.serial_port},{self.baud_rate}"
            print(f"   Connection: {connection_string}")
            
            # MAVLink bağlantısı kur
            self.master = mavutil.mavlink_connection(
                self.serial_port,
                baud=self.baud_rate,
                autoreconnect=True
            )
            
            # Heartbeat bekle (timeout 20 saniye)
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=20)
            
            if heartbeat:
                self.connected = True
                print("✅ Serial MAVLink bağlantısı kuruldu!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                
                # IMU stream'lerini etkinleştir
                self.request_imu_streams()
                
                # Sistem durumunu kontrol et
                self.check_system_status()
                
                return True
            else:
                print("❌ Heartbeat alınamadı!")
                return False
                
        except Exception as e:
            print(f"❌ Serial bağlantı hatası: {e}")
            print("💡 Pixhawk serial portu ve baud rate'ini kontrol edin")
            self.connected = False
            return False
    
    def request_imu_streams(self):
        """IMU veri akışlarını etkinleştir"""
        try:
            # Request IMU, attitude, ve diğer önemli mesajlar
            message_types = [
                (mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 50),
                (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU, 50),
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 20),
                (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 10),
                (mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE, 10),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 5),
                (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)
            ]
            
            for msg_id, rate in message_types:
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    msg_id,
                    rate,
                    1  # start_stop (1 = start, 0 = stop)
                )
            
            print("📊 IMU veri akışları etkinleştirildi")
            time.sleep(1)  # Stream'lerin başlaması için bekle
            
        except Exception as e:
            print(f"⚠️ Stream etkinleştirme hatası: {e}")
    
    def disconnect(self):
        """MAVLink bağlantısını kapat"""
        try:
            if self.master:
                self.master.close()
            self.connected = False
            self.armed = False
            print("🔌 Serial MAVLink bağlantısı kapatıldı")
        except Exception as e:
            print(f"⚠️ Bağlantı kapatma hatası: {e}")
    
    def check_system_status(self):
        """Sistem durumunu kontrol et"""
        try:
            # Heartbeat'ten sistem durumunu al
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print(f"🔍 Sistem durumu: {'ARMED' if self.armed else 'DISARMED'}")
                print(f"🔍 Sistem tipi: {msg.type}")
                return True
            return False
        except Exception as e:
            print(f"⚠️ Sistem durum kontrolü hatası: {e}")
            return False
    
    def arm_system(self):
        """Sistemi arm et"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            
            time.sleep(1)
            self.check_system_status()
            
            if self.armed:
                print("✅ Sistem ARM edildi")
                return True
            else:
                print("❌ ARM işlemi başarısız")
                return False
                
        except Exception as e:
            print(f"❌ ARM hatası: {e}")
            return False
    
    def disarm_system(self):
        """Sistemi disarm et"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0
            )
            
            time.sleep(1)
            self.check_system_status()
            
            if not self.armed:
                print("✅ Sistem DISARM edildi")
                return True
            else:
                print("❌ DISARM işlemi başarısız")
                return False
                
        except Exception as e:
            print(f"❌ DISARM hatası: {e}")
            return False
    
    def set_control_mode(self, mode):
        """Kontrol modunu ayarla"""
        if mode in ["raw", "pid"]:
            self.control_mode = mode
            print(f"🎛️ Kontrol modu: {mode.upper()}")
    
    def set_navigation_mode(self, mode):
        """Navigation modunu ayarla"""
        if mode in ["gps_only", "imu_only", "hybrid"]:
            self.navigation_mode = mode
            print(f"🧭 Navigation modu: {mode}")
    
    def send_raw_servo_pwm(self, channel, pwm_value):
        """Raw PWM servo komutu gönder"""
        if not self.connected or not self.armed:
            return False
        
        # PWM limitlerini kontrol et
        pwm_value = max(1000, min(2000, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, channel, pwm_value, 0, 0, 0, 0, 0
            )
            return True
            
        except Exception as e:
            print(f"❌ Servo PWM hatası: {e}")
            return False
    
    def send_raw_motor_pwm(self, pwm_value):
        """Motor PWM komutu gönder (AUX6 = Channel 14)"""
        if not self.connected or not self.armed:
            return False
            
        motor_channel = 14  # AUX6 = MAVLink channel 14
        pwm_value = max(1000, min(2000, pwm_value))
        
        # Motor debug logging
        if abs(pwm_value - 1500) > 50:
            direction = "İLERİ" if pwm_value > 1500 else "GERİ"
            power = abs(pwm_value - 1500) * 100 // 500
            print(f"🚁 MOTOR: {direction} - PWM:{pwm_value}µs ({power}%)")
        
        return self.send_raw_servo_pwm(motor_channel, pwm_value)
    
    def control_servos_raw(self, roll, pitch, yaw):
        """X-Fin servo kontrolü (Raw PWM)"""
        if not self.armed:
            return False
            
        # X-Wing Matrix Hesaplaması
        neutral = 1500
        
        # Hardware channels: AUX1=9, AUX3=11, AUX4=12, AUX5=13
        front_left_pwm = neutral + int((pitch * 8) + (roll * 10) + (yaw * 6))    # AUX1
        front_right_pwm = neutral + int((pitch * 8) - (roll * 10) - (yaw * 6))   # AUX3
        rear_left_pwm = neutral + int((-pitch * 8) + (roll * 10) - (yaw * 6))    # AUX4
        rear_right_pwm = neutral + int((-pitch * 8) - (roll * 10) + (yaw * 6))   # AUX5
        
        # PWM limit kontrolü
        front_left_pwm = max(1000, min(2000, front_left_pwm))
        front_right_pwm = max(1000, min(2000, front_right_pwm))
        rear_left_pwm = max(1000, min(2000, rear_left_pwm))
        rear_right_pwm = max(1000, min(2000, rear_right_pwm))
        
        # Servo komutlarını gönder
        results = []
        results.append(self.send_raw_servo_pwm(9, front_left_pwm))    # AUX1
        results.append(self.send_raw_servo_pwm(11, front_right_pwm))  # AUX3
        results.append(self.send_raw_servo_pwm(12, rear_left_pwm))    # AUX4
        results.append(self.send_raw_servo_pwm(13, rear_right_pwm))   # AUX5
        
        # Debug logging
        if abs(roll) > 1 or abs(pitch) > 1 or abs(yaw) > 1:
            print(f"🎮 X-WING: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°")
            print(f"   PWM → AUX1:{front_left_pwm} AUX3:{front_right_pwm} AUX4:{rear_left_pwm} AUX5:{rear_right_pwm}")
        
        return all(results)
    
    def control_servos_pid(self, target_roll, target_pitch, target_yaw):
        """PID servo kontrolü (IMU feedback ile)"""
        if not self.armed:
            return False
        
        # Bu implementasyon ileride eklenecek
        # Şimdilik raw kontrolü kullan
        return self.control_servos_raw(target_roll, target_pitch, target_yaw)
    
    def get_imu_data(self):
        """IMU verilerini al"""
        if not self.connected or not self.master:
            return None
            
        try:
            # ATTITUDE mesajından veri al
            msg = self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
            if msg:
                # Angle rates
                gyro_x = msg.rollspeed   # rad/s
                gyro_y = msg.pitchspeed  # rad/s
                gyro_z = msg.yawspeed    # rad/s
                
                # Gravity vector estimate
                roll_rad = msg.roll
                pitch_rad = msg.pitch
                
                accel_x = -9.81 * math.sin(pitch_rad)
                accel_y = 9.81 * math.sin(roll_rad) * math.cos(pitch_rad)
                accel_z = 9.81 * math.cos(roll_rad) * math.cos(pitch_rad)
                
                with self.data_lock:
                    self.imu_data_buffer.append((accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z))
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
                    
        except Exception as e:
            pass
        
        return None
    
    def get_imu_data_alternative(self):
        """Alternative IMU data getter"""
        return self.get_imu_data()
    
    def get_gps_data(self):
        """GPS verilerini al"""
        if not self.connected:
            return None
            
        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            if msg:
                lat = msg.lat / 1e7  # degrees
                lon = msg.lon / 1e7  # degrees
                alt = msg.alt / 1000.0  # mm to meters
                satellites = getattr(msg, 'satellites_visible', 0)
                
                with self.data_lock:
                    self.gps_data_buffer.append((lat, lon, alt, satellites))
                
                return lat, lon, alt, satellites
        except:
            pass
        return None
    
    def get_depth_data(self):
        """Depth sensor verilerini al (MAVLink üzerinden)"""
        if not self.connected:
            return None
        
        try:
            # SCALED_PRESSURE2 mesajını dene (D300 için)
            msg = self.master.recv_match(type='SCALED_PRESSURE2', blocking=False)
            if msg:
                pressure_mbar = msg.press_abs
                temperature_c = msg.temperature / 100.0
                depth_m = max(0.0, (pressure_mbar - 1013.25) / 100.0)
                
                data = {
                    'depth_m': depth_m,
                    'temperature_c': temperature_c,
                    'pressure_mbar': pressure_mbar,
                    'timestamp': time.time()
                }
                
                with self.data_lock:
                    self.depth_data_buffer.append(data)
                
                return data
            
            # Alternatif SCALED_PRESSURE
            msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if msg:
                pressure_mbar = msg.press_abs
                temperature_c = msg.temperature / 100.0
                depth_m = max(0.0, (pressure_mbar - 1013.25) / 100.0)
                
                data = {
                    'depth_m': depth_m,
                    'temperature_c': temperature_c,
                    'pressure_mbar': pressure_mbar,
                    'timestamp': time.time()
                }
                
                with self.data_lock:
                    self.depth_data_buffer.append(data)
                
                return data
            
            return None
            
        except Exception as e:
            return None
    
    def get_all_sensor_data(self):
        """Tüm sensör verilerini toplu olarak al"""
        data = {}
        
        # IMU verisi
        imu_data = self.get_imu_data()
        if imu_data:
            data['imu'] = {
                'accel_x': imu_data[0],
                'accel_y': imu_data[1], 
                'accel_z': imu_data[2],
                'gyro_x': imu_data[3],
                'gyro_y': imu_data[4],
                'gyro_z': imu_data[5]
            }
        
        # Depth verisi
        depth_data = self.get_depth_data()
        if depth_data:
            data['depth'] = depth_data
        
        # GPS verisi
        gps_data = self.get_gps_data()
        if gps_data:
            data['gps'] = {
                'lat': gps_data[0],
                'lon': gps_data[1],
                'alt': gps_data[2],
                'satellites': gps_data[3]
            }
        
        return data
    
    def emergency_stop(self):
        """Acil durum - tüm kontroller durdur"""
        print("🚨 ACİL DURUM - TÜMÜ DURDURULUYOR!")
        
        if not self.connected:
            return
        
        try:
            # Tüm servo'ları neutral'a
            neutral = 1500
            motor_stop = 1500
            
            # X-Fin servo'ları (AUX1,3,4,5)
            for channel in [9, 11, 12, 13]:  # AUX1, AUX3, AUX4, AUX5
                self.send_raw_servo_pwm(channel, neutral)
            
            # Motor durdur (AUX6)
            self.send_raw_servo_pwm(14, motor_stop)
            
            print("✅ Acil durum protokolü tamamlandı")
            
        except Exception as e:
            print(f"❌ Acil durum hatası: {e}")
    
    def get_system_status(self):
        """Sistem durumu bilgilerini döndür"""
        try:
            return {
                'connection_status': self.connected,
                'connection_type': 'Serial',
                'serial_port': self.serial_port,
                'baud_rate': self.baud_rate,
                'armed_status': self.armed,
                'control_mode': self.control_mode,
                'navigation_mode': self.navigation_mode,
                'system_time': time.time(),
                'heartbeat_active': self.connected
            }
        except Exception as e:
            print(f"System status error: {e}")
            return {
                'connection_status': False,
                'connection_type': 'Serial',
                'serial_port': self.serial_port,
                'baud_rate': self.baud_rate,
                'armed_status': False,
                'control_mode': 'raw',
                'navigation_mode': 'gps_only',
                'system_time': time.time(),
                'heartbeat_active': False
            }

# IMU Filter ve PID Controller sınıfları (değişmedi)
class IMUFilter:
    """Basit IMU filtresi - Complementary Filter"""
    
    def __init__(self):
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0
        self.yaw_filtered = 0.0
        self.last_time = time.time()
        
        # Filtre parametreleri
        self.gyro_weight = 0.98
        self.accel_weight = 0.02
    
    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        """IMU verilerini filtrele"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 0.1 or dt <= 0:  # Sanity check
            dt = 0.02
        
        # Accelerometer'dan açıları hesapla
        accel_roll = math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))
        accel_pitch = math.atan2(-accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))
        
        # Complementary filter
        gyro_roll = self.roll_filtered + gyro_x * dt
        gyro_pitch = self.pitch_filtered + gyro_y * dt
        
        self.roll_filtered = self.gyro_weight * gyro_roll + self.accel_weight * accel_roll
        self.pitch_filtered = self.gyro_weight * gyro_pitch + self.accel_weight * accel_pitch
        self.yaw_filtered += gyro_z * dt  # Yaw sadece gyro'dan
        
        # Radyan'dan dereceye çevir
        roll_deg = math.degrees(self.roll_filtered)
        pitch_deg = math.degrees(self.pitch_filtered)
        yaw_deg = math.degrees(self.yaw_filtered)
        
        return roll_deg, pitch_deg, yaw_deg

class PIDController:
    """Basit PID Controller"""
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Anti-windup
        self.max_integral = 50.0
        self.dead_zone = 5.0
    
    def update(self, setpoint, current_value):
        """PID hesapla"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.02
            
        error = setpoint - current_value
        
        # Dead zone
        if abs(error) < self.dead_zone:
            error = 0.0
        
        # Proportional
        p_term = self.kp * error
        
        # Integral with anti-windup
        if abs(self.integral) < self.max_integral:
            self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Derivative
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error
        
        output = p_term + i_term + d_term
        return max(-45, min(45, output))  # Limit output

if __name__ == "__main__":
    # Test kodu
    print("🔧 MAVLink Handler Serial Test")
    
    handler = MAVLinkHandler()
    if handler.connect():
        print("✅ Serial bağlantı test başarılı!")
        
        # Test IMU data
        imu_data = handler.get_imu_data()
        if imu_data:
            print(f"📊 IMU: {imu_data}")
        
        handler.disconnect()
    else:
        print("❌ Serial bağlantı test başarısız!")
        print(f"💡 Kontrol et: {handler.serial_port} @ {handler.baud_rate} baud") 