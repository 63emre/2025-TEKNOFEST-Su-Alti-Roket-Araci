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
        """Pixhawk'a TCP bağlantısı - Pi5 + PiOS OPTİMİZE"""
        try:
            connection_string = self.config["mavlink"]["connection_string"]
            print(f"🔌 TCP MAVLink bağlantısı kuruluyor: {connection_string}")
            
            # Pi5 için TCP bağlantı optimize
            self.master = mavutil.mavlink_connection(
                connection_string,
                baud=57600,  # TCP için önemli değil ama set edelim
                source_system=255,  # Ground station ID
                use_native=False  # Pi5 uyumluluk için
            )
            
            # Heartbeat bekle - GUI için timeout artırıldı
            print("⏳ Heartbeat bekleniyor (20s timeout)...")
            self.master.wait_heartbeat(timeout=20)
            
            self.connected = True
            print("✅ TCP MAVLink bağlantısı başarılı!")
            print(f"📡 Target System: {self.master.target_system}")
            print(f"📡 Target Component: {self.master.target_component}")
            
            # Sistem durumunu kontrol et
            self.check_system_status()
            
            # IMU data stream'lerini request et
            self.request_imu_streams()
            
            return True
    
    def request_imu_streams(self):
        """IMU veri akışlarını başlat"""
        try:
            print("📡 IMU veri akışları başlatılıyor...")
            
            # 20 Hz = 50,000 microseconds
            stream_rate = 50000
            
            message_intervals = [
                (mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, stream_rate),
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, stream_rate),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 100000),  # 10 Hz for speed
            ]
            
            for msg_id, interval in message_intervals:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0, msg_id, interval, 0, 0, 0, 0, 0
                )
            
            print("✅ IMU streams başlatıldı (HIGHRES_IMU 20Hz, ATTITUDE 20Hz)")
            
        except Exception as e:
            print(f"❌ IMU stream request hatası: {e}")
            
        except Exception as e:
            print(f"❌ TCP MAVLink bağlantı hatası: {e}")
            print(f"🔧 Exception türü: {type(e).__name__}")
            print("💡 Kontrol et: ArduSub çalışıyor mu? TCP port açık mı?")
            print(f"🔍 Connection string: {connection_string}")
            import traceback
            print(f"📄 Traceback: {traceback.format_exc()}")
            self.connected = False
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
    
    def send_raw_motor_pwm(self, pwm_value):
        """Motor PWM gönder - GERÇEK HARDWARE AUX6"""
        if not self.connected or not self.armed:
            return False
            
        # GERÇEK HARDWARE: Motor AUX6 → MAVLink Channel 14
        motor_channel = 14  # AUX6 = MAVLink channel 14
        limits = self.config["pixhawk"]["pwm_limits"]
        
        # Motor PWM limitlerini kontrol et
        pwm_value = max(limits["motor_min"], min(limits["motor_max"], pwm_value))
        
        # Real-time motor debug logging
        if abs(pwm_value - limits["motor_neutral"]) > 50:
            direction = "İLERİ" if pwm_value > limits["motor_neutral"] else "GERİ"
            power = abs(pwm_value - limits["motor_neutral"]) * 100 // 500
            print(f"🚁 MOTOR CONTROL: {direction} - PWM:{pwm_value}µs ({power}%)")
        
        return self.send_raw_servo_pwm(motor_channel, pwm_value)
    
    def control_servos_raw(self, roll, pitch, yaw):
        """RAW servo kontrolü - GERÇEK HARDWARE X-Mixing"""
        if not self.armed:
            return False
            
        with self.control_lock:
            # GERÇEK HARDWARE CONFIG
            servos = self.config["pixhawk"]["servos"]
            neutral = self.config["pixhawk"]["pwm_limits"]["servo_neutral"]
            
            # GERÇEK X-Wing Matrix Hesaplaması
            # Hardware: AUX1(Ön Sol), AUX3(Ön Sağ), AUX4(Arka Sol), AUX5(Arka Sağ)
            
            # X-Mixing formülü (optimize edilmiş multiplier'lar)
            front_left_pwm = neutral + (pitch * 8) + (roll * 10) + (yaw * 6)    # AUX1: Ön Sol
            front_right_pwm = neutral + (pitch * 8) - (roll * 10) - (yaw * 6)   # AUX3: Ön Sağ
            rear_left_pwm = neutral - (pitch * 8) + (roll * 10) - (yaw * 6)     # AUX4: Arka Sol  
            rear_right_pwm = neutral - (pitch * 8) - (roll * 10) + (yaw * 6)    # AUX5: Arka Sağ
            
            # PWM limit kontrolü
            servo_min = self.config["pixhawk"]["pwm_limits"]["servo_min"]
            servo_max = self.config["pixhawk"]["pwm_limits"]["servo_max"]
            
            front_left_pwm = max(servo_min, min(servo_max, int(front_left_pwm)))
            front_right_pwm = max(servo_min, min(servo_max, int(front_right_pwm)))
            rear_left_pwm = max(servo_min, min(servo_max, int(rear_left_pwm)))
            rear_right_pwm = max(servo_min, min(servo_max, int(rear_right_pwm)))
            
            # GERÇEK HARDWARE - Servo komutlarını MAVLink channel'lara gönder
            results = []
            results.append(self.send_raw_servo_pwm(9, front_left_pwm))   # AUX1 → MAVLink 9 (Ön Sol)
            results.append(self.send_raw_servo_pwm(11, front_right_pwm)) # AUX3 → MAVLink 11 (Ön Sağ)
            results.append(self.send_raw_servo_pwm(12, rear_left_pwm))   # AUX4 → MAVLink 12 (Arka Sol)
            results.append(self.send_raw_servo_pwm(13, rear_right_pwm))  # AUX5 → MAVLink 13 (Arka Sağ)
            
            # Real-time debug logging
            if abs(roll) > 1 or abs(pitch) > 1 or abs(yaw) > 1:
                print(f"🎮 X-WING RAW CONTROL: R={roll:.1f}° P={pitch:.1f}° Y={yaw:.1f}°")
                print(f"   PWM → AUX1:{front_left_pwm} AUX3:{front_right_pwm} AUX4:{rear_left_pwm} AUX5:{rear_right_pwm}")
            
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
        """IMU verilerini al - HIGHRES_IMU kullanarak (ArduSub uyumlu)"""
        if not self.connected or not self.master:
            return None
            
        try:
            # HIGHRES_IMU mesajını dene (ArduSub'da bu var!)
            msg = self.master.recv_match(type='HIGHRES_IMU', blocking=False, timeout=0.05)
            if msg:
                # HIGHRES_IMU formatı - direkt SI units
                accel_x = msg.xacc  # m/s²
                accel_y = msg.yacc  # m/s²
                accel_z = msg.zacc  # m/s²
                gyro_x = msg.xgyro  # rad/s
                gyro_y = msg.ygyro  # rad/s
                gyro_z = msg.zgyro  # rad/s
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            
            # Alternatif: SCALED_IMU mesajını dene
            msg_scaled = self.master.recv_match(type='SCALED_IMU', blocking=False, timeout=0.05)
            if msg_scaled:
                # SCALED_IMU formatı
                accel_x = msg_scaled.xacc / 1000.0  # mg to m/s²
                accel_y = msg_scaled.yacc / 1000.0
                accel_z = msg_scaled.zacc / 1000.0
                gyro_x = msg_scaled.xgyro / 1000.0 * (math.pi / 180.0)  # mrad/s to rad/s
                gyro_y = msg_scaled.ygyro / 1000.0 * (math.pi / 180.0)
                gyro_z = msg_scaled.zgyro / 1000.0 * (math.pi / 180.0)
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            
            # Son alternatif: RAW_IMU
            msg_raw = self.master.recv_match(type='RAW_IMU', blocking=False, timeout=0.05)
            if msg_raw:
                # RAW_IMU formatı
                accel_x = msg_raw.xacc / 1000.0 * 9.81  # mg to m/s²
                accel_y = msg_raw.yacc / 1000.0 * 9.81
                accel_z = msg_raw.zacc / 1000.0 * 9.81
                gyro_x = msg_raw.xgyro / 1000.0 * (math.pi / 180.0)  # mdeg/s to rad/s
                gyro_y = msg_raw.ygyro / 1000.0 * (math.pi / 180.0)
                gyro_z = msg_raw.zgyro / 1000.0 * (math.pi / 180.0)
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            
            # DEBUG: TCP'de hangi mesajlar var kontrol et
            if not hasattr(self, 'imu_debug_counter'):
                self.imu_debug_counter = 0
            self.imu_debug_counter += 1
            
            if self.imu_debug_counter % 500 == 1:  # Her 500 call'da bir debug
                print(f"🔍 IMU DEBUG #{self.imu_debug_counter}: HIGHRES_IMU, SCALED_IMU, RAW_IMU bulunamadı")
                # Herhangi bir mesaj var mı?
                any_msg = self.master.recv_match(blocking=False, timeout=0.05)
                if any_msg:
                    print(f"🔍 TCP'de bulunan mesaj tipi: {any_msg.get_type()}")
                else:
                    print("🔍 TCP'de hiç mesaj yok!")
                    
        except Exception as e:
            # Debug için hata göster
            if not hasattr(self, 'imu_error_counter'):
                self.imu_error_counter = 0
            self.imu_error_counter += 1
            
            if self.imu_error_counter % 100 == 1:  # Her 100 hatada bir log
                print(f"❌ IMU data exception #{self.imu_error_counter}: {e}")
        
        return None
    
    def get_imu_data_alternative(self):
        """ATTITUDE mesajından IMU verisi al - TCP için alternatif"""
        if not self.connected or not self.master:
            return None
            
        try:
            # ATTITUDE mesajını dene - ArduSub'da her zaman var
            msg = self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.05)
            if msg:
                # ATTITUDE mesajından angle rates
                gyro_x = msg.rollspeed   # rad/s (roll rate)
                gyro_y = msg.pitchspeed  # rad/s (pitch rate)
                gyro_z = msg.yawspeed    # rad/s (yaw rate)
                
                # Accelerometer verisi yok, gravity estimate yapalım
                import math
                roll_rad = msg.roll
                pitch_rad = msg.pitch
                
                # Gravity vector estimate
                accel_x = -9.81 * math.sin(pitch_rad)
                accel_y = 9.81 * math.sin(roll_rad) * math.cos(pitch_rad)
                accel_z = 9.81 * math.cos(roll_rad) * math.cos(pitch_rad)
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
            
        except Exception as e:
            if not hasattr(self, 'alt_imu_error_counter'):
                self.alt_imu_error_counter = 0
            self.alt_imu_error_counter += 1
            
            if self.alt_imu_error_counter % 100 == 1:
                print(f"❌ Alternative IMU data exception #{self.alt_imu_error_counter}: {e}")
        
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
    
    def get_depth_data(self):
        """Depth sensor verilerini al (MAVLink üzerinden)"""
        if not self.connected:
            return None
        
        try:
            # ArduSub'da depth sensor verisi genellikle SCALED_PRESSURE2 mesajında gelir
            msg = self.master.recv_match(type='SCALED_PRESSURE2', blocking=False)
            if msg:
                # D300 depth sensor data
                pressure_mbar = msg.press_abs  # Absolute pressure in millibar
                temperature_c = msg.temperature / 100.0  # Temperature in celsius (from centidegrees)
                
                # Convert pressure to depth (rough approximation)
                # 1 mbar ≈ 1 cm water depth
                depth_m = max(0.0, (pressure_mbar - 1013.25) / 100.0)  # Sea level correction
                
                return {
                    'depth_m': depth_m,
                    'temperature_c': temperature_c,
                    'pressure_mbar': pressure_mbar,
                    'timestamp': time.time()
                }
            
            # Alternatif olarak SCALED_PRESSURE mesajını da dene
            msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if msg:
                pressure_mbar = msg.press_abs
                temperature_c = msg.temperature / 100.0
                depth_m = max(0.0, (pressure_mbar - 1013.25) / 100.0)
                
                return {
                    'depth_m': depth_m,
                    'temperature_c': temperature_c,
                    'pressure_mbar': pressure_mbar,
                    'timestamp': time.time()
                }
            
            return None
            
        except Exception as e:
            print(f"❌ Depth data alma hatası: {e}")
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
    
    def get_system_status(self):
        """Sistem durumu bilgilerini döndür"""
        try:
            return {
                'connection_status': self.connected,
                'armed_status': getattr(self, 'armed', False),
                'flight_mode': getattr(self, 'current_mode', 'MANUAL'),
                'battery_voltage': getattr(self, 'battery_voltage', 0.0),
                'battery_level': getattr(self, 'battery_level', 0),
                'gps_status': getattr(self, 'gps_fix_type', 0),
                'satellites': getattr(self, 'satellites_visible', 0),
                'depth': getattr(self, 'depth', 0.0),
                'temperature': getattr(self, 'temperature', 20.0),
                'pressure': getattr(self, 'pressure', 1013.25),
                'system_time': time.time(),
                'uptime': time.time() - getattr(self, 'start_time', time.time()),
                'last_heartbeat': getattr(self, 'last_heartbeat', 0),
                'errors': getattr(self, 'error_count', 0)
            }
        except Exception as e:
            print(f"System status error: {e}")
            return {
                'connection_status': False,
                'armed_status': False,
                'flight_mode': 'UNKNOWN',
                'battery_voltage': 0.0,
                'battery_level': 0,
                'gps_status': 0,
                'satellites': 0,
                'depth': 0.0,
                'temperature': 0.0,
                'pressure': 0.0,
                'system_time': time.time(),
                'uptime': 0,
                'last_heartbeat': 0,
                'errors': 0
            }

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

    def get_system_status(self):
        """Sistem durumu bilgilerini döndür"""
        try:
            return {
                'connection_status': self.connected,
                'armed_status': getattr(self, 'armed', False),
                'flight_mode': getattr(self, 'current_mode', 'MANUAL'),
                'battery_voltage': getattr(self, 'battery_voltage', 0.0),
                'battery_level': getattr(self, 'battery_level', 0),
                'gps_status': getattr(self, 'gps_fix_type', 0),
                'satellites': getattr(self, 'satellites_visible', 0),
                'depth': getattr(self, 'depth', 0.0),
                'temperature': getattr(self, 'temperature', 20.0),
                'pressure': getattr(self, 'pressure', 1013.25),
                'system_time': time.time(),
                'uptime': time.time() - getattr(self, 'start_time', time.time()),
                'last_heartbeat': getattr(self, 'last_heartbeat', 0),
                'errors': getattr(self, 'error_count', 0)
            }
        except Exception as e:
            print(f"System status error: {e}")
            return {
                'connection_status': False,
                'armed_status': False,
                'flight_mode': 'UNKNOWN',
                'battery_voltage': 0.0,
                'battery_level': 0,
                'gps_status': 0,
                'satellites': 0,
                'depth': 0.0,
                'temperature': 0.0,
                'pressure': 0.0,
                'system_time': time.time(),
                'uptime': 0,
                'last_heartbeat': 0,
                'errors': 0
            }

if __name__ == "__main__":
    # Test
    handler = MAVLinkHandler()
    if handler.connect():
        print("Test başarılı!")
        handler.disconnect() 