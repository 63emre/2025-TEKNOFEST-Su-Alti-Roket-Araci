#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - MAVLink Handler
Raw PWM vs PID Control Sistemi
"""

import time
import math
import json
import threading
from pymavlink import mavutil
from collections import deque
import numpy as np

class MAVLinkHandler:
    def __init__(self, config_path="config/hardware_config.json"):
        """MAVLink bağlantı ve kontrol sistemi"""
        self.load_config(config_path)
        self.master = None
        self.connected = False
        self.armed = False
        
        # Kontrol durumu
        self.control_mode = "raw"  # "raw" veya "pid"
        self.navigation_mode = "imu_only"  # "gps_only", "imu_only", "hybrid"
        
        # Raw kontrol için
        self.last_pwm_values = {}
        self.pwm_hysteresis = 3  # µs
        
        # PID kontrol için
        self.imu_filter = IMUFilter()
        self.pid_controllers = {
            'roll': PIDController(0.8, 0.02, 0.12),
            'pitch': PIDController(0.8, 0.02, 0.12),
            'yaw': PIDController(0.8, 0.02, 0.12)
        }
        
        # Thread güvenliği
        self.control_lock = threading.Lock()
        self.running = False
        
    def load_config(self, config_path):
        """Konfigürasyon yükle"""
        try:
            with open(config_path, 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            print(f"❌ Config yükleme hatası: {e}")
            # Varsayılan config
            self.config = {
                "pixhawk": {
                    "servos": {"front_left": 1, "rear_left": 3, "rear_right": 4, "front_right": 5},
                    "motor": 6,
                    "pwm_limits": {"servo_min": 1100, "servo_max": 1900, "servo_neutral": 1500}
                },
                "mavlink": {"connection_string": "tcp:127.0.0.1:5777"}
            }
    
    def connect(self):
        """Pixhawk'a bağlan"""
        try:
            connection_string = self.config["mavlink"]["connection_string"]
            print(f"🔌 Pixhawk'a bağlanıyor: {connection_string}")
            
            self.master = mavutil.mavlink_connection(connection_string)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Sistem durumunu kontrol et
            self.check_system_status()
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        self.running = False
        if self.master:
            # Tüm servolar neutral'a
            self.emergency_stop()
            self.master.close()
            self.connected = False
            print("🔌 MAVLink bağlantısı kapatıldı")
    
    def check_system_status(self):
        """Sistem durumunu kontrol et"""
        if not self.connected:
            return False
            
        try:
            # Heartbeat bekle
            msg = self.master.recv_match(type='HEARTBEAT', timeout=5)
            if msg:
                self.armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                return True
        except:
            pass
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
            with self.control_lock:
                self.control_mode = mode
                print(f"🎛️ Kontrol modu: {mode.upper()}")
    
    def set_navigation_mode(self, mode):
        """Navigation modunu ayarla"""
        if mode in ["gps_only", "imu_only", "hybrid"]:
            with self.control_lock:
                self.navigation_mode = mode
                print(f"🧭 Navigation modu: {mode}")
    
    def send_raw_servo_pwm(self, channel, pwm_value):
        """RAW PWM gönder - titreşim önleyici"""
        if not self.connected or not self.armed:
            return False
        
        # PWM limitlerini kontrol et
        limits = self.config["pixhawk"]["pwm_limits"]
        pwm_value = max(limits["servo_min"], min(limits["servo_max"], pwm_value))
        
        # PWM Hysteresis - gereksiz gönderimi önle
        last_pwm = self.last_pwm_values.get(channel, None)
        if last_pwm is not None and abs(pwm_value - last_pwm) < self.pwm_hysteresis:
            return True  # Değişiklik çok küçük, gönderme
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, channel, pwm_value, 0, 0, 0, 0, 0
            )
            
            self.last_pwm_values[channel] = pwm_value
            return True
            
        except Exception as e:
            print(f"❌ Servo PWM hatası: {e}")
            return False

    def send_servo_test_pwm(self, channel, pwm_value):
        """Test için servo PWM gönder - ARM kontrolü olmadan"""
        if not self.connected:
            print("❌ MAVLink bağlı değil!")
            return False
        
        # PWM limitlerini kontrol et
        limits = self.config["pixhawk"]["pwm_limits"]
        pwm_value = max(limits["servo_min"], min(limits["servo_max"], pwm_value))
        
        try:
            # Test modu için ARM kontrolü yapmadan gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, channel, pwm_value, 0, 0, 0, 0, 0
            )
            
            # Test modu için response bekle
            time.sleep(0.1)
            
            self.last_pwm_values[channel] = pwm_value
            return True
            
        except Exception as e:
            print(f"❌ Test Servo PWM hatası: {e}")
            return False

    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM ayarla (test için alias)"""
        return self.send_servo_test_pwm(channel, pwm_value)
    
    def send_raw_motor_pwm(self, pwm_value):
        """Motor PWM gönder"""
        if not self.connected or not self.armed:
            return False
            
        motor_channel = self.config["pixhawk"]["motor"]
        limits = self.config["pixhawk"]["pwm_limits"]
        
        # Motor PWM limitlerini kontrol et
        pwm_value = max(limits["motor_min"], min(limits["motor_max"], pwm_value))
        
        return self.send_raw_servo_pwm(motor_channel, pwm_value)
    
    def control_servos_raw(self, roll, pitch, yaw):
        """RAW servo kontrolü - direkt PWM"""
        if not self.armed:
            return False
            
        with self.control_lock:
            # X-fin matrix hesaplaması
            servos = self.config["pixhawk"]["servos"]
            neutral = self.config["pixhawk"]["pwm_limits"]["servo_neutral"]
            
            # X konfigürasyonu servo kontrolü
            front_left_pwm = neutral + (roll * 10) + (pitch * 8) + (yaw * 5)
            front_right_pwm = neutral - (roll * 10) + (pitch * 8) - (yaw * 5)
            rear_left_pwm = neutral + (roll * 10) - (pitch * 8) - (yaw * 5)
            rear_right_pwm = neutral - (roll * 10) - (pitch * 8) + (yaw * 5)
            
            # Servo komutlarını gönder
            results = []
            results.append(self.send_raw_servo_pwm(servos["front_left"], int(front_left_pwm)))
            results.append(self.send_raw_servo_pwm(servos["front_right"], int(front_right_pwm)))
            results.append(self.send_raw_servo_pwm(servos["rear_left"], int(rear_left_pwm)))
            results.append(self.send_raw_servo_pwm(servos["rear_right"], int(rear_right_pwm)))
            
            return all(results)
    
    def control_servos_pid(self, target_roll, target_pitch, target_yaw):
        """PID servo kontrolü - IMU feedback ile"""
        if not self.armed:
            return False
        
        with self.control_lock:
            # IMU verilerini al
            imu_data = self.get_imu_data()
            if not imu_data:
                return False
            
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
            
            # IMU filtresi uygula
            current_roll, current_pitch, current_yaw = self.imu_filter.update(
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            )
            
            # PID kontrolcülerini güncelle
            roll_output = self.pid_controllers['roll'].update(target_roll, current_roll)
            pitch_output = self.pid_controllers['pitch'].update(target_pitch, current_pitch)
            yaw_output = self.pid_controllers['yaw'].update(target_yaw, current_yaw)
            
            # PID çıktılarını servo komutlarına çevir
            return self.control_servos_raw(roll_output, pitch_output, yaw_output)
    
    def get_imu_data(self):
        """IMU verilerini al (Roll, Pitch, Yaw)"""
        if not self.connected:
            return None
            
        try:
            # ATTITUDE mesajını al (Roll, Pitch, Yaw için)
            msg = self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
            if msg:
                # Radians to degrees
                roll = math.degrees(msg.roll)
                pitch = math.degrees(msg.pitch) 
                yaw = math.degrees(msg.yaw)
                
                return roll, pitch, yaw
                
        except Exception as e:
            print(f"❌ IMU veri hatası: {e}")
            pass
            
        return None
    
    def get_raw_imu_data(self):
        """RAW IMU verilerini al (Accelerometer, Gyro)"""
        if not self.connected:
            return None
            
        try:
            msg = self.master.recv_match(type='RAW_IMU', blocking=False, timeout=0.1)
            if msg:
                # Convert to SI units
                accel_x = msg.xacc / 1000.0  # mg to m/s²
                accel_y = msg.yacc / 1000.0
                accel_z = msg.zacc / 1000.0
                gyro_x = math.radians(msg.xgyro / 1000.0)  # mrad/s to rad/s
                gyro_y = math.radians(msg.ygyro / 1000.0)
                gyro_z = math.radians(msg.zgyro / 1000.0)
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        except:
            pass
        return None
    
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
                
                return lat, lon, alt, satellites
        except:
            pass
        return None
    
    def emergency_stop(self):
        """Acil durum - tüm servo/motor durdur"""
        print("🚨 ACİL DURUM - TÜMÜ DURDURULUYOR!")
        
        if not self.connected:
            return
        
        try:
            # Tüm servoları neutral'a
            servos = self.config["pixhawk"]["servos"]
            neutral = self.config["pixhawk"]["pwm_limits"]["servo_neutral"]
            motor_stop = self.config["pixhawk"]["pwm_limits"]["motor_stop"]
            
            for servo_name, channel in servos.items():
                self.send_raw_servo_pwm(channel, neutral)
            
            # Motoru durdur
            motor_channel = self.config["pixhawk"]["motor"]
            self.send_raw_servo_pwm(motor_channel, motor_stop)
            
            print("✅ Acil durum protokolü tamamlandı")
            
        except Exception as e:
            print(f"❌ Acil durum hatası: {e}")

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
    # Test
    handler = MAVLinkHandler()
    if handler.connect():
        print("Test başarılı!")
        handler.disconnect() 