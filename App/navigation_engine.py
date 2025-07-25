#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Navigation Engine
GPS + IMU + Hibrit Navigation Sistemi
"""

import time
import math
import json
import threading
from collections import deque
import numpy as np

class NavigationEngine:
    def __init__(self, mavlink_handler, sensor_manager=None, depth_sensor=None, control_settings_path="config/control_settings.json"):
        """Navigation sistemi"""
        self.mavlink = mavlink_handler
        self.sensor_manager = sensor_manager
        self.depth_sensor = depth_sensor
        self.load_control_settings(control_settings_path)
        
        # Navigation modları
        self.gps_nav = GPSNavigation(mavlink_handler, self.sensor_manager)
        self.imu_nav = IMUNavigation(mavlink_handler, self.sensor_manager)
        self.hybrid_nav = HybridNavigation(mavlink_handler, self.gps_nav, self.imu_nav)
        
        # Aktif navigation
        self.current_mode = "imu_only"
        self.mission_active = False
        self.mission_thread = None
        
        # Thread güvenliği
        self.nav_lock = threading.Lock()
        
    def load_control_settings(self, config_path):
        """Kontrol ayarlarını yükle"""
        try:
            with open(config_path, 'r') as f:
                self.settings = json.load(f)
        except Exception as e:
            print(f"❌ Control settings yükleme hatası: {e}")
            # Varsayılan ayarlar
            self.settings = {
                "navigation_modes": {
                    "gps_only": {"accuracy_meters": 1.0},
                    "imu_only": {"accuracy_meters": 0.5},
                    "hybrid": {"gps_timeout_seconds": 5}
                }
            }
    
    def set_navigation_mode(self, mode):
        """Navigation modunu ayarla"""
        if mode in ["gps_only", "imu_only", "hybrid"]:
            with self.nav_lock:
                self.current_mode = mode
                self.mavlink.set_navigation_mode(mode)
                print(f"🧭 Navigation modu: {mode}")
    
    def get_navigation_handler(self):
        """Aktif navigation handler'ı döndür"""
        if self.current_mode == "gps_only":
            return self.gps_nav
        elif self.current_mode == "imu_only":
            return self.imu_nav
        else:  # hybrid
            return self.hybrid_nav
    
    def get_depth_data(self):
        """Derinlik sensöründen veri al"""
        if self.depth_sensor and self.depth_sensor.connected:
            return self.depth_sensor.get_sensor_data()
        return {
            'depth_m': 0.0,
            'temperature_c': 20.0,
            'pressure_mbar': 1013.25,
            'connected': False
        }
    
    def emergency_surface(self):
        """Acil durum yüzeye çıkış"""
        depth_data = self.get_depth_data()
        current_depth = depth_data['depth_m']
        
        if current_depth < 0.5:
            print("🌊 Zaten yüzeyde!")
            return True
        
        print(f"🚨 ACİL YÜZEY ÇIKIŞI! Mevcut derinlik: {current_depth:.1f}m")
        
        # Tam güç yukarı çık
        success = self.mavlink.emergency_surface()
        
        if success:
            # Derinlik takibi
            start_time = time.time()
            while time.time() - start_time < 30:  # 30 saniye max
                depth_data = self.get_depth_data()
                current_depth = depth_data['depth_m']
                
                print(f"Yüzeye çıkıyor... Derinlik: {current_depth:.1f}m")
                
                if current_depth < 0.5:
                    print("✅ Yüzeye ulaşıldı!")
                    return True
                
                time.sleep(1.0)
        
        print("❌ Yüzey çıkışı başarısız!")
        return False
    
    def start_movement_mission(self, command_type, parameter, control_mode):
        """Hareket görevini başlat"""
        if self.mission_active:
            print("⚠️ Zaten bir görev aktif!")
            return False
        
        def mission_worker():
            try:
                with self.nav_lock:
                    self.mission_active = True
                    
                print(f"🎯 Görev başlatıldı: {command_type} {parameter}")
                
                # Navigation handler'ı al
                nav_handler = self.get_navigation_handler()
                
                # Komut tipine göre hareket
                if command_type == "FORWARD":
                    success = nav_handler.move_forward(parameter, control_mode)
                elif command_type == "YAW_ROTATION":
                    success = nav_handler.rotate_yaw(parameter, control_mode)
                elif command_type == "ASCEND":
                    success = nav_handler.move_vertical(parameter, control_mode)
                elif command_type == "STRAFE_LEFT":
                    success = nav_handler.move_left(parameter, control_mode)
                elif command_type == "STRAFE_RIGHT":
                    success = nav_handler.move_right(parameter, control_mode)
                else:
                    print(f"❌ Bilinmeyen komut: {command_type}")
                    success = False
                
                if success:
                    print(f"✅ Görev tamamlandı: {command_type}")
                else:
                    print(f"❌ Görev başarısız: {command_type}")
                    
            except Exception as e:
                print(f"❌ Görev hatası: {e}")
            finally:
                with self.nav_lock:
                    self.mission_active = False
        
        # Mission thread'i başlat
        self.mission_thread = threading.Thread(target=mission_worker, daemon=True)
        self.mission_thread.start()
        return True
    
    def stop_mission(self):
        """Aktif görevi durdur"""
        if self.mission_active:
            with self.nav_lock:
                self.mission_active = False
            print("🛑 Görev durduruldu")
    
    def get_navigation_status(self):
        """Navigation durumunu döndür"""
        nav_handler = self.get_navigation_handler()
        return {
            "mode": self.current_mode,
            "mission_active": self.mission_active,
            "position": nav_handler.get_current_position(),
            "accuracy": nav_handler.get_accuracy()
        }

class GPSNavigation:
    """GPS bazlı navigation"""
    
    def __init__(self, mavlink_handler, sensor_manager=None):
        self.mavlink = mavlink_handler
        self.sensor_manager = sensor_manager
        self.start_position = None
        self.current_position = None
        self.gps_quality = 0
    
    def get_current_position(self):
        """Mevcut GPS pozisyonunu al"""
        # Önce Adafruit GPS'den dene
        if self.sensor_manager and self.sensor_manager.sensors_initialized:
            sensor_data = self.sensor_manager.get_all_sensor_data()
            gps_data = sensor_data.get('gps', {})
            
            if gps_data.get('connected', False):
                position = gps_data.get('position', {})
                
                lat = position.get('latitude', 0.0)
                lon = position.get('longitude', 0.0) 
                alt = position.get('altitude', 0.0)
                satellites = position.get('satellites', 0)
                
                if lat != 0.0 and lon != 0.0:  # Valid GPS fix
                    self.current_position = (lat, lon, alt)
                    self.gps_quality = satellites
                    return self.current_position
        
        # Fallback: MAVLink GPS
        gps_data = self.mavlink.get_gps_data()
        if gps_data:
            lat, lon, alt, satellites = gps_data
            self.current_position = (lat, lon, alt)
            self.gps_quality = satellites
            return self.current_position
        return None
    
    def get_accuracy(self):
        """GPS doğruluğunu döndür"""
        if self.gps_quality >= 4:
            return 1.0  # meters
        elif self.gps_quality >= 2:
            return 3.0
        else:
            return 10.0
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """İki GPS koordinatı arası mesafe (Haversine)"""
        R = 6371000  # Earth radius in meters
        
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat/2) * math.sin(dlat/2) +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon/2) * math.sin(dlon/2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """İki nokta arası bearing hesapla"""
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        
        y = math.sin(dlon) * math.cos(math.radians(lat2))
        x = (math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) -
             math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon))
        
        bearing = math.atan2(y, x)
        return math.degrees(bearing)
    
    def calculate_target_coordinates(self, lat, lon, distance_m, bearing_deg):
        """Hedef koordinatları hesapla"""
        R = 6371000  # Earth radius
        
        bearing_rad = math.radians(bearing_deg)
        lat1_rad = math.radians(lat)
        lon1_rad = math.radians(lon)
        
        lat2_rad = math.asin(math.sin(lat1_rad) * math.cos(distance_m/R) +
                            math.cos(lat1_rad) * math.sin(distance_m/R) * math.cos(bearing_rad))
        
        lon2_rad = lon1_rad + math.atan2(math.sin(bearing_rad) * math.sin(distance_m/R) * math.cos(lat1_rad),
                                        math.cos(distance_m/R) - math.sin(lat1_rad) * math.sin(lat2_rad))
        
        return math.degrees(lat2_rad), math.degrees(lon2_rad)
    
    def move_forward(self, distance_meters, control_mode):
        """GPS ile ileri hareket"""
        print(f"🧭 GPS Navigation: İleri {distance_meters}m")
        
        # Başlangıç pozisyonu
        start_pos = self.get_current_position()
        if not start_pos:
            print("❌ GPS sinyal yok!")
            return False
        
        start_lat, start_lon, start_alt = start_pos
        
        # Mevcut heading'i al (compass)
        current_heading = 0  # TODO: Compass'dan al
        
        # Hedef koordinatları hesapla
        target_lat, target_lon = self.calculate_target_coordinates(
            start_lat, start_lon, distance_meters, current_heading
        )
        
        print(f"📍 Hedef: {target_lat:.6f}, {target_lon:.6f}")
        
        # Hedefe doğru hareket et
        tolerance = 1.0  # 1 metre tolerans
        max_attempts = 100
        attempt = 0
        
        while attempt < max_attempts:
            current_pos = self.get_current_position()
            if not current_pos:
                print("❌ GPS kayboldu!")
                return False
            
            current_lat, current_lon, _ = current_pos
            
            # Hedefe olan mesafe
            distance_to_target = self.calculate_distance(
                current_lat, current_lon, target_lat, target_lon
            )
            
            if distance_to_target <= tolerance:
                print(f"✅ Hedefe ulaşıldı! (±{distance_to_target:.1f}m)")
                return True
            
            # Hedefe doğru bearing hesapla
            bearing_to_target = self.calculate_bearing(
                current_lat, current_lon, target_lat, target_lon
            )
            
            # Servo kontrolü
            if control_mode == "raw":
                # Raw kontrol ile hedefe doğru git
                self.control_to_bearing_raw(bearing_to_target, distance_to_target)
            else:
                # PID kontrol ile hedefe doğru git
                self.control_to_bearing_pid(bearing_to_target, distance_to_target)
            
            time.sleep(0.1)
            attempt += 1
        
        print("❌ Zaman aşımı - hedefe ulaşılamadı")
        return False
    
    def control_to_bearing_raw(self, target_bearing, distance):
        """Raw kontrol ile bearing'e doğru git"""
        # Basit bearing kontrolü
        bearing_error = target_bearing
        if bearing_error > 180:
            bearing_error -= 360
        elif bearing_error < -180:
            bearing_error += 360
        
        # Servo komutları
        yaw_command = bearing_error / 10.0  # Scale down
        pitch_command = min(10.0, distance / 2.0)  # İleri thrust
        
        self.mavlink.control_servos_raw(0, pitch_command, yaw_command)
    
    def control_to_bearing_pid(self, target_bearing, distance):
        """PID kontrol ile bearing'e doğru git"""
        # PID kontrol ile daha stabil hareket
        yaw_target = target_bearing / 10.0
        pitch_target = min(10.0, distance / 2.0)
        
        self.mavlink.control_servos_pid(0, pitch_target, yaw_target)
    
    def rotate_yaw(self, angle_degrees, control_mode):
        """GPS ile yaw dönme (Compass based)"""
        print(f"🧭 GPS Navigation: Yaw {angle_degrees}°")
        # TODO: Compass sensor integration gerekli
        return True
    
    def move_vertical(self, distance_meters, control_mode):
        """Dikey hareket"""
        print(f"🧭 GPS Navigation: Dikey {distance_meters}m")
        # TODO: Depth sensor integration gerekli
        return True
    
    def move_left(self, distance_meters, control_mode):
        """Sol hareket"""
        current_heading = 0  # TODO: Compass'dan al
        return self.move_to_relative_position(-distance_meters, 0, control_mode)
    
    def move_right(self, distance_meters, control_mode):
        """Sağ hareket"""
        current_heading = 0  # TODO: Compass'dan al
        return self.move_to_relative_position(distance_meters, 0, control_mode)
    
    def move_to_relative_position(self, x_offset, y_offset, control_mode):
        """Relative pozisyona git"""
        start_pos = self.get_current_position()
        if not start_pos:
            return False
        
        start_lat, start_lon, _ = start_pos
        
        # X,Y offset'i GPS koordinatlarına çevir
        # Basit approximation (small distances için)
        lat_per_meter = 1.0 / 111000.0  # degrees per meter
        lon_per_meter = 1.0 / (111000.0 * math.cos(math.radians(start_lat)))
        
        target_lat = start_lat + (y_offset * lat_per_meter)
        target_lon = start_lon + (x_offset * lon_per_meter)
        
        # Hedefe git
        distance = math.sqrt(x_offset**2 + y_offset**2)
        bearing = math.degrees(math.atan2(x_offset, y_offset))
        
        return self.move_to_coordinates(target_lat, target_lon, control_mode)
    
    def move_to_coordinates(self, target_lat, target_lon, control_mode):
        """Belirli koordinatlara git"""
        tolerance = 1.0
        max_attempts = 100
        
        for attempt in range(max_attempts):
            current_pos = self.get_current_position()
            if not current_pos:
                return False
            
            current_lat, current_lon, _ = current_pos
            distance = self.calculate_distance(current_lat, current_lon, target_lat, target_lon)
            
            if distance <= tolerance:
                return True
            
            bearing = self.calculate_bearing(current_lat, current_lon, target_lat, target_lon)
            
            if control_mode == "raw":
                self.control_to_bearing_raw(bearing, distance)
            else:
                self.control_to_bearing_pid(bearing, distance)
            
            time.sleep(0.1)
        
        return False

class IMUNavigation:
    """IMU Dead Reckoning Navigation"""
    
    def __init__(self, mavlink_handler, sensor_manager=None):
        self.mavlink = mavlink_handler
        self.sensor_manager = sensor_manager
        
        # Relative position (başlangıca göre)
        self.position = [0.0, 0.0, 0.0]  # X, Y, Z meters
        self.velocity = [0.0, 0.0, 0.0]  # X, Y, Z m/s
        self.heading = 0.0  # degrees
        
        # IMU integration
        self.last_time = time.time()
        self.accel_buffer = deque(maxlen=10)
        
    def get_current_position(self):
        """Mevcut relative pozisyonu döndür"""
        return tuple(self.position)
    
    def get_accuracy(self):
        """IMU accuracy (drift ile artar)"""
        # Zamanla accuracy azalır
        elapsed = time.time() - self.last_time
        drift_factor = elapsed / 60.0  # dakika başı drift
        return 0.5 + drift_factor * 0.1
    
    def update_position_from_imu(self):
        """IMU'dan pozisyonu güncelle"""
        # Önce Adafruit sensörden dene
        if self.sensor_manager and self.sensor_manager.sensors_initialized:
            sensor_data = self.sensor_manager.get_all_sensor_data()
            imu_data = sensor_data.get('imu', {})
            
            if imu_data.get('connected', False):
                euler = imu_data.get('euler', {})
                accel = imu_data.get('acceleration', {})
                
                # IMU data formatı
                formatted_data = {
                    'roll': euler.get('roll', 0.0),
                    'pitch': euler.get('pitch', 0.0), 
                    'yaw': euler.get('heading', 0.0),
                    'xacc': accel.get('x', 0.0),
                    'yacc': accel.get('y', 0.0),
                    'zacc': accel.get('z', 0.0)
                }
                
                return self._integrate_imu_data(formatted_data)
        
        # Fallback: MAVLink IMU
        imu_data = self.mavlink.get_imu_data()
        if not imu_data:
            return False
        
        return self._integrate_imu_data(imu_data)
    
    def _integrate_imu_data(self, imu_data):
        """IMU verisini pozisyona çevir"""
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 0.1 or dt <= 0:
            dt = 0.02
        
        # IMU data extract
        if isinstance(imu_data, dict):
            # Formatted data from Adafruit or MAVLink
            accel_x = imu_data.get('xacc', 0.0)
            accel_y = imu_data.get('yacc', 0.0) 
            accel_z = imu_data.get('zacc', 0.0)
            yaw = imu_data.get('yaw', 0.0)
        else:
            # Tuple format (legacy)
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
            yaw = self.heading
        
        # Gravity compensation (basit)
        accel_x_corrected = accel_x - 0.0  # TODO: gravity vector çıkar
        accel_y_corrected = accel_y - 0.0
        accel_z_corrected = accel_z - 9.81
        
        # Velocity integration
        self.velocity[0] += accel_x_corrected * dt
        self.velocity[1] += accel_y_corrected * dt
        self.velocity[2] += accel_z_corrected * dt
        
        # Position integration
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.position[2] += self.velocity[2] * dt
        
        # Heading güncelle
        if isinstance(imu_data, dict):
            self.heading = yaw  # Adafruit'ten gelen yaw direkt kullan
        else:
            self.heading += math.degrees(gyro_z) * dt  # Gyro entegrasyonu
        
        # Drift correction (basit low-pass filter)
        damping = 0.99
        self.velocity[0] *= damping
        self.velocity[1] *= damping
        self.velocity[2] *= damping
        
        return True
    
    def reset_position(self):
        """Pozisyonu sıfırla"""
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.heading = 0.0
        print("📍 IMU pozisyonu sıfırlandı")
    
    def move_forward(self, distance_meters, control_mode):
        """IMU ile ileri hareket"""
        print(f"🧭 IMU Navigation: İleri {distance_meters}m")
        
        start_position = self.position.copy()
        target_distance = distance_meters
        tolerance = 0.5  # 50cm tolerans
        
        max_time = 60  # 60 saniye max
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            # IMU pozisyonunu güncelle
            if not self.update_position_from_imu():
                continue
            
            # Mevcut mesafeyi hesapla
            current_distance = math.sqrt(
                (self.position[0] - start_position[0])**2 +
                (self.position[1] - start_position[1])**2
            )
            
            # Hedefe ulaştık mı?
            if current_distance >= target_distance - tolerance:
                print(f"✅ IMU: Hedefe ulaşıldı! ({current_distance:.1f}m)")
                return True
            
            # İleri git
            remaining_distance = target_distance - current_distance
            thrust_power = min(20.0, remaining_distance * 2.0)
            
            if control_mode == "raw":
                self.mavlink.control_servos_raw(0, thrust_power, 0)
            else:
                self.mavlink.control_servos_pid(0, thrust_power, 0)
            
            time.sleep(0.1)
        
        print("❌ IMU: Zaman aşımı")
        return False
    
    def rotate_yaw(self, angle_degrees, control_mode):
        """IMU ile yaw dönme"""
        print(f"🧭 IMU Navigation: Yaw {angle_degrees}°")
        
        start_heading = self.heading
        target_heading = start_heading + angle_degrees
        tolerance = 5.0  # 5 derece tolerans
        
        max_time = 30
        start_time = time.time()
        
        while time.time() - start_time < max_time:
            if not self.update_position_from_imu():
                continue
            
            heading_error = target_heading - self.heading
            
            # Normalize angle
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            if abs(heading_error) <= tolerance:
                print(f"✅ IMU: Yaw tamamlandı! (±{heading_error:.1f}°)")
                return True
            
            # Yaw kontrolü
            yaw_power = max(-20.0, min(20.0, heading_error / 5.0))
            
            if control_mode == "raw":
                self.mavlink.control_servos_raw(0, 0, yaw_power)
            else:
                self.mavlink.control_servos_pid(0, 0, yaw_power)
            
            time.sleep(0.1)
        
        print("❌ IMU: Yaw zaman aşımı")
        return False
    
    def move_vertical(self, distance_meters, control_mode):
        """Dikey hareket"""
        print(f"🧭 IMU Navigation: Dikey {distance_meters}m")
        # TODO: Depth sensor ile integration
        return True
    
    def move_left(self, distance_meters, control_mode):
        """Sol hareket"""
        # 90 derece dön, ileri git, -90 derece dön
        if not self.rotate_yaw(-90, control_mode):
            return False
        if not self.move_forward(distance_meters, control_mode):
            return False
        return self.rotate_yaw(90, control_mode)
    
    def move_right(self, distance_meters, control_mode):
        """Sağ hareket"""
        # 90 derece dön, ileri git, -90 derece dön
        if not self.rotate_yaw(90, control_mode):
            return False
        if not self.move_forward(distance_meters, control_mode):
            return False
        return self.rotate_yaw(-90, control_mode)

class HybridNavigation:
    """GPS + IMU Hibrit Navigation"""
    
    def __init__(self, mavlink_handler, gps_nav, imu_nav):
        self.mavlink = mavlink_handler
        self.gps_nav = gps_nav
        self.imu_nav = imu_nav
        
        # Hibrit parametreler
        self.gps_timeout = 5.0  # saniye
        self.last_gps_time = 0
        self.gps_available = False
        
    def get_current_position(self):
        """Mevcut pozisyonu en iyi kaynaktan al"""
        gps_pos = self.gps_nav.get_current_position()
        if gps_pos and time.time() - self.last_gps_time < self.gps_timeout:
            self.gps_available = True
            self.last_gps_time = time.time()
            return gps_pos
        else:
            self.gps_available = False
            return self.imu_nav.get_current_position()
    
    def get_accuracy(self):
        """Mevcut accuracy"""
        if self.gps_available:
            return self.gps_nav.get_accuracy()
        else:
            return self.imu_nav.get_accuracy()
    
    def move_forward(self, distance_meters, control_mode):
        """Hibrit ileri hareket"""
        print(f"🧭 Hybrid Navigation: İleri {distance_meters}m")
        
        # GPS mevcut mu kontrol et
        if self.get_current_position() and self.gps_available:
            print("📡 GPS mevcut - GPS navigation kullanılıyor")
            return self.gps_nav.move_forward(distance_meters, control_mode)
        else:
            print("🔄 GPS yok - IMU navigation kullanılıyor")
            return self.imu_nav.move_forward(distance_meters, control_mode)
    
    def rotate_yaw(self, angle_degrees, control_mode):
        """Hibrit yaw dönme"""
        # Yaw için IMU daha güvenilir
        return self.imu_nav.rotate_yaw(angle_degrees, control_mode)
    
    def move_vertical(self, distance_meters, control_mode):
        """Dikey hareket"""
        # Dikey hareket için IMU kullan
        return self.imu_nav.move_vertical(distance_meters, control_mode)
    
    def move_left(self, distance_meters, control_mode):
        """Sol hareket"""
        if self.gps_available:
            return self.gps_nav.move_left(distance_meters, control_mode)
        else:
            return self.imu_nav.move_left(distance_meters, control_mode)
    
    def move_right(self, distance_meters, control_mode):
        """Sağ hareket"""
        if self.gps_available:
            return self.gps_nav.move_right(distance_meters, control_mode)
        else:
            return self.imu_nav.move_right(distance_meters, control_mode)

if __name__ == "__main__":
    # Test
    from mavlink_handler import MAVLinkHandler
    
    mavlink = MAVLinkHandler()
    if mavlink.connect():
        nav_engine = NavigationEngine(mavlink)
        print("Navigation Engine test başarılı!")
        mavlink.disconnect() 