#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Sensor Manager
Adafruit BNO055 IMU ve GPS sensörleri entegrasyonu
"""

import time
import math
import threading
from collections import deque
import json

# Adafruit sensör imports
try:
    import board
    import busio
    import adafruit_bno055
    import adafruit_gps
    ADAFRUIT_AVAILABLE = True
except ImportError:
    print("⚠️ Adafruit sensörleri bulunamadı! Test modunda çalışıyor...")
    ADAFRUIT_AVAILABLE = False

class BNO055Handler:
    """Adafruit BNO055 IMU sensörü"""
    
    def __init__(self):
        """BNO055 IMU başlat"""
        self.sensor = None
        self.connected = False
        
        # IMU data
        self.euler_angles = {'heading': 0.0, 'roll': 0.0, 'pitch': 0.0}
        self.quaternion = {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.accelerometer = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.gyroscope = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.magnetometer = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.temperature = 0.0
        
        # Data history
        self.euler_history = deque(maxlen=200)
        self.accel_history = deque(maxlen=200)
        
        # Calibration status
        self.calibration_status = {'system': 0, 'gyro': 0, 'accel': 0, 'mag': 0}
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Monitoring
        self.monitoring = False
        self.monitor_thread = None
        
    def connect(self):
        """BNO055'e bağlan"""
        if not ADAFRUIT_AVAILABLE:
            print("⚠️ BNO055 simülasyon modunda çalışıyor")
            self.connected = True
            return True
            
        try:
            print("🔌 BNO055 IMU'ya bağlanılıyor...")
            
            # I2C bus oluştur
            i2c = busio.I2C(board.SCL, board.SDA)
            
            # BNO055 sensörü oluştur
            self.sensor = adafruit_bno055.BNO055_I2C(i2c)
            
            # Sensör modunu ayarla
            self.sensor.mode = adafruit_bno055.NDOF_MODE
            
            # Bağlantı test et
            temp = self.sensor.temperature
            
            self.connected = True
            print("✅ BNO055 IMU başarıyla bağlandı!")
            return True
            
        except Exception as e:
            print(f"❌ BNO055 bağlantı hatası: {e}")
            return False
    
    def disconnect(self):
        """Sensör bağlantısını kapat"""
        self.stop_monitoring()
        self.connected = False
        print("🔌 BNO055 bağlantısı kapatıldı")
    
    def read_sensor_data(self):
        """Sensörden tüm verileri oku"""
        if not self.connected:
            return False
        
        try:
            if not ADAFRUIT_AVAILABLE:
                # Simülasyon verisi
                import random
                with self.data_lock:
                    self.euler_angles = {
                        'heading': random.uniform(0, 360),
                        'roll': random.uniform(-45, 45),
                        'pitch': random.uniform(-45, 45)
                    }
                    self.accelerometer = {
                        'x': random.uniform(-2, 2),
                        'y': random.uniform(-2, 2),
                        'z': random.uniform(8, 12)
                    }
                return True
            
            # Gerçek sensör verisi
            euler = self.sensor.euler
            quat = self.sensor.quaternion
            accel = self.sensor.acceleration
            gyro = self.sensor.gyro
            mag = self.sensor.magnetic
            temp = self.sensor.temperature
            calib = self.sensor.calibration_status
            
            with self.data_lock:
                # Euler angles (None check)
                if euler[0] is not None:
                    self.euler_angles = {
                        'heading': euler[0],
                        'roll': euler[1] if euler[1] is not None else 0.0,
                        'pitch': euler[2] if euler[2] is not None else 0.0
                    }
                
                # Quaternion
                if quat[0] is not None:
                    self.quaternion = {
                        'w': quat[0],
                        'x': quat[1] if quat[1] is not None else 0.0,
                        'y': quat[2] if quat[2] is not None else 0.0,
                        'z': quat[3] if quat[3] is not None else 0.0
                    }
                
                # Accelerometer
                if accel[0] is not None:
                    self.accelerometer = {
                        'x': accel[0],
                        'y': accel[1] if accel[1] is not None else 0.0,
                        'z': accel[2] if accel[2] is not None else 0.0
                    }
                
                # Gyroscope
                if gyro[0] is not None:
                    self.gyroscope = {
                        'x': gyro[0],
                        'y': gyro[1] if gyro[1] is not None else 0.0,
                        'z': gyro[2] if gyro[2] is not None else 0.0
                    }
                
                # Magnetometer
                if mag[0] is not None:
                    self.magnetometer = {
                        'x': mag[0],
                        'y': mag[1] if mag[1] is not None else 0.0,
                        'z': mag[2] if mag[2] is not None else 0.0
                    }
                
                # Temperature
                if temp is not None:
                    self.temperature = temp
                
                # Calibration
                self.calibration_status = {
                    'system': calib[0] if calib[0] is not None else 0,
                    'gyro': calib[1] if calib[1] is not None else 0,
                    'accel': calib[2] if calib[2] is not None else 0,
                    'mag': calib[3] if calib[3] is not None else 0
                }
                
                # History'ye ekle
                self.euler_history.append(self.euler_angles.copy())
                self.accel_history.append(self.accelerometer.copy())
            
            return True
            
        except Exception as e:
            print(f"❌ BNO055 veri okuma hatası: {e}")
            return False
    
    def get_euler_angles(self):
        """Euler açıları döndür"""
        with self.data_lock:
            return self.euler_angles.copy()
    
    def get_acceleration(self):
        """İvme değerleri döndür"""
        with self.data_lock:
            return self.accelerometer.copy()
    
    def get_calibration_status(self):
        """Kalibrasyon durumu döndür"""
        with self.data_lock:
            return self.calibration_status.copy()
    
    def start_monitoring(self, interval=0.05):
        """Sürekli sensör okuma başlat"""
        if self.monitoring:
            return
        
        self.monitoring = True
        
        def monitor_worker():
            while self.monitoring:
                if self.connected:
                    self.read_sensor_data()
                time.sleep(interval)
        
        self.monitor_thread = threading.Thread(target=monitor_worker, daemon=True)
        self.monitor_thread.start()
        print("🔄 BNO055 sürekli monitoring başlatıldı")
    
    def stop_monitoring(self):
        """Sürekli okuma durdur"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        print("⏹️ BNO055 monitoring durduruldu")

class GPSHandler:
    """Adafruit GPS sensörü"""
    
    def __init__(self, uart_device='/dev/ttyUSB0', baudrate=9600):
        """GPS sensörü başlat"""
        self.uart_device = uart_device
        self.baudrate = baudrate
        self.gps = None
        self.connected = False
        
        # GPS data
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.speed = 0.0
        self.course = 0.0
        self.satellites = 0
        self.fix_quality = 0
        self.timestamp = None
        
        # Data history
        self.position_history = deque(maxlen=100)
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Monitoring
        self.monitoring = False
        self.monitor_thread = None
        
    def connect(self):
        """GPS'e bağlan"""
        if not ADAFRUIT_AVAILABLE:
            print("⚠️ GPS simülasyon modunda çalışıyor")
            self.connected = True
            return True
            
        try:
            print(f"🔌 GPS'e bağlanılıyor: {self.uart_device}")
            
            # UART oluştur
            import serial
            uart = serial.Serial(self.uart_device, baudrate=self.baudrate, timeout=10)
            
            # GPS objesi oluştur
            self.gps = adafruit_gps.GPS(uart, debug=False)
            
            # GPS ayarları
            self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')  # RMC + GGA
            self.gps.send_command(b'PMTK220,1000')  # 1Hz update rate
            
            self.connected = True
            print("✅ GPS başarıyla bağlandı!")
            return True
            
        except Exception as e:
            print(f"❌ GPS bağlantı hatası: {e}")
            return False
    
    def disconnect(self):
        """GPS bağlantısını kapat"""
        self.stop_monitoring()
        self.connected = False
        print("🔌 GPS bağlantısı kapatıldı")
    
    def read_gps_data(self):
        """GPS verilerini oku"""
        if not self.connected:
            return False
        
        try:
            if not ADAFRUIT_AVAILABLE:
                # Simülasyon verisi
                import random
                with self.data_lock:
                    self.latitude = 41.0082 + random.uniform(-0.001, 0.001)  # İstanbul
                    self.longitude = 28.9784 + random.uniform(-0.001, 0.001)
                    self.satellites = random.randint(4, 12)
                    self.fix_quality = 1
                return True
            
            # Gerçek GPS verisi
            self.gps.update()
            
            with self.data_lock:
                if self.gps.has_fix:
                    self.latitude = self.gps.latitude
                    self.longitude = self.gps.longitude
                    self.altitude = self.gps.altitude_m if self.gps.altitude_m else 0.0
                    self.speed = self.gps.speed_knots if self.gps.speed_knots else 0.0
                    self.course = self.gps.track_angle_deg if self.gps.track_angle_deg else 0.0
                    self.fix_quality = self.gps.fix_quality
                    self.timestamp = self.gps.timestamp_utc
                    
                    # Position history'ye ekle
                    self.position_history.append({
                        'lat': self.latitude,
                        'lon': self.longitude,
                        'timestamp': time.time()
                    })
                
                self.satellites = self.gps.satellites if self.gps.satellites else 0
            
            return True
            
        except Exception as e:
            print(f"❌ GPS veri okuma hatası: {e}")
            return False
    
    def get_position(self):
        """GPS pozisyonu döndür"""
        with self.data_lock:
            return {
                'latitude': self.latitude,
                'longitude': self.longitude,
                'altitude': self.altitude,
                'fix_quality': self.fix_quality,
                'satellites': self.satellites
            }
    
    def start_monitoring(self, interval=1.0):
        """Sürekli GPS okuma başlat"""
        if self.monitoring:
            return
        
        self.monitoring = True
        
        def monitor_worker():
            while self.monitoring:
                if self.connected:
                    self.read_gps_data()
                time.sleep(interval)
        
        self.monitor_thread = threading.Thread(target=monitor_worker, daemon=True)
        self.monitor_thread.start()
        print("🔄 GPS sürekli monitoring başlatıldı")
    
    def stop_monitoring(self):
        """Sürekli okuma durdur"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        print("⏹️ GPS monitoring durduruldu")

class SensorManager:
    """Tüm sensörleri yöneten ana sınıf"""
    
    def __init__(self, config):
        """Sensor Manager başlat"""
        self.config = config
        
        # Sensör objeler
        self.imu = BNO055Handler()
        self.gps = GPSHandler()
        
        # Sensör durumları
        self.sensors_initialized = False
        
    def initialize_sensors(self):
        """Tüm sensörleri başlat"""
        print("🔄 Sensörler başlatılıyor...")
        
        success_count = 0
        
        # IMU başlat
        if self.imu.connect():
            self.imu.start_monitoring()
            success_count += 1
        
        # GPS başlat
        if self.gps.connect():
            self.gps.start_monitoring()
            success_count += 1
        
        if success_count > 0:
            self.sensors_initialized = True
            print(f"✅ {success_count}/2 sensör başarıyla başlatıldı")
        else:
            print("❌ Hiçbir sensör başlatılamadı!")
        
        return self.sensors_initialized
    
    def shutdown_sensors(self):
        """Tüm sensörleri kapat"""
        print("🔄 Sensörler kapatılıyor...")
        
        self.imu.disconnect()
        self.gps.disconnect()
        
        self.sensors_initialized = False
        print("✅ Tüm sensörler kapatıldı")
    
    def get_all_sensor_data(self):
        """Tüm sensör verilerini döndür"""
        return {
            'imu': {
                'connected': self.imu.connected,
                'euler': self.imu.get_euler_angles(),
                'acceleration': self.imu.get_acceleration(),
                'calibration': self.imu.get_calibration_status()
            },
            'gps': {
                'connected': self.gps.connected,
                'position': self.gps.get_position()
            }
        }

if __name__ == "__main__":
    # Test kodu
    print("🔬 Sensor Manager Test")
    
    # Dummy config
    config = {}
    
    manager = SensorManager(config)
    
    if manager.initialize_sensors():
        try:
            for i in range(20):
                data = manager.get_all_sensor_data()
                
                imu_data = data['imu']
                gps_data = data['gps']
                
                print(f"\n--- Ölçüm {i+1} ---")
                print(f"IMU: {imu_data['euler']}")
                print(f"GPS: {gps_data['position']['latitude']:.6f}, {gps_data['position']['longitude']:.6f}")
                print(f"Uydu: {gps_data['position']['satellites']}")
                
                time.sleep(1.0)
                
        except KeyboardInterrupt:
            print("\n🛑 Test durduruldu")
        
        manager.shutdown_sensors()
    else:
        print("❌ Sensör başlatma başarısız!") 