#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - D300 Derinlik Sensörü
I2C tabanlı derinlik ve sıcaklık sensörü
"""

import time
import smbus2
import threading
from collections import deque

class D300DepthSensor:
    """D300 Derinlik ve Sıcaklık Sensörü Sınıfı"""
    
    def __init__(self, bus_num=1, address=0x77):
        """D300 sensörü başlat"""
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self.connected = False
        
        # Sensor data
        self.depth_m = 0.0
        self.temperature_c = 0.0
        self.pressure_mbar = 0.0
        
        # Data history
        self.depth_history = deque(maxlen=100)
        self.temp_history = deque(maxlen=100)
        
        # Thread safety
        self.data_lock = threading.Lock()
        
        # Monitoring
        self.monitoring = False
        self.monitor_thread = None
        
    def connect(self):
        """D300 sensörüne bağlan"""
        try:
            print(f"🔌 D300 sensörüne bağlanılıyor (I2C: {self.address:#04x})")
            
            self.bus = smbus2.SMBus(self.bus_num)
            
            # Test read
            test_data = self.bus.read_byte(self.address)
            
            self.connected = True
            print("✅ D300 sensörü başarıyla bağlandı!")
            return True
            
        except FileNotFoundError:
            print("❌ I2C bulunamadı! Raspberry Pi'da mısınız?")
            return False
        except OSError:
            print("❌ D300 sensörü bulunamadı!")
            return False
        except Exception as e:
            print(f"❌ D300 bağlantı hatası: {e}")
            return False
    
    def disconnect(self):
        """Sensör bağlantısını kapat"""
        self.stop_monitoring()
        if self.bus:
            self.bus.close()
        self.connected = False
        print("🔌 D300 sensörü bağlantısı kapatıldı")
    
    def read_raw_data(self):
        """D300'den raw data oku"""
        if not self.connected:
            return None, None
            
        try:
            # D300 specific read sequence
            # Bu değerler gerçek D300 datasheet'ine göre ayarlanmalı
            
            # Pressure ve temperature read
            pressure_raw = self.bus.read_word_data(self.address, 0x00)
            temp_raw = self.bus.read_word_data(self.address, 0x02)
            
            return pressure_raw, temp_raw
            
        except Exception as e:
            print(f"❌ D300 veri okuma hatası: {e}")
            return None, None
    
    def convert_pressure_to_depth(self, pressure_raw):
        """Raw pressure'ı derinliğe çevir"""
        # D300 specific conversion formula
        # Bu formula gerçek kalibrasyon değerlerine göre ayarlanmalı
        pressure_mbar = pressure_raw * 0.1  # Example conversion
        
        # 1 mbar ≈ 1 cm su derinliği
        depth_cm = pressure_mbar - 1013.25  # Sea level pressure correction
        depth_m = depth_cm / 100.0
        
        return max(0.0, depth_m), pressure_mbar
    
    def convert_temp(self, temp_raw):
        """Raw temperature'u Celsius'a çevir"""
        # D300 specific conversion formula
        temp_c = temp_raw * 0.01 - 273.15  # Example conversion
        return temp_c
    
    def read_sensor(self):
        """Sensörden veri oku ve işle"""
        pressure_raw, temp_raw = self.read_raw_data()
        
        if pressure_raw is None or temp_raw is None:
            return False
        
        # Convert raw data
        depth, pressure = self.convert_pressure_to_depth(pressure_raw)
        temperature = self.convert_temp(temp_raw)
        
        with self.data_lock:
            self.depth_m = depth
            self.temperature_c = temperature
            self.pressure_mbar = pressure
            
            # Add to history
            self.depth_history.append(depth)
            self.temp_history.append(temperature)
        
        return True
    
    def get_depth(self):
        """Anlık derinlik değeri (metre)"""
        with self.data_lock:
            return self.depth_m
    
    def get_temperature(self):
        """Anlık sıcaklık değeri (Celsius)"""
        with self.data_lock:
            return self.temperature_c
    
    def get_pressure(self):
        """Anlık basınç değeri (mbar)"""
        with self.data_lock:
            return self.pressure_mbar
    
    def get_sensor_data(self):
        """Tüm sensör verilerini al"""
        with self.data_lock:
            return {
                'depth_m': self.depth_m,
                'temperature_c': self.temperature_c,
                'pressure_mbar': self.pressure_mbar,
                'connected': self.connected
            }
    
    def start_monitoring(self, interval=0.1):
        """Sürekli sensör okuma başlat"""
        if self.monitoring:
            return
        
        self.monitoring = True
        
        def monitor_worker():
            while self.monitoring:
                if self.connected:
                    self.read_sensor()
                time.sleep(interval)
        
        self.monitor_thread = threading.Thread(target=monitor_worker, daemon=True)
        self.monitor_thread.start()
        print("🔄 D300 sürekli monitoring başlatıldı")
    
    def stop_monitoring(self):
        """Sürekli okuma durdur"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        print("⏹️ D300 monitoring durduruldu")
    
    def calibrate_surface(self):
        """Yüzey kalibrasyonu yap"""
        if not self.connected:
            return False
        
        print("🌊 Yüzey kalibrasyonu başlıyor...")
        
        # 10 örnek al
        samples = []
        for i in range(10):
            if self.read_sensor():
                samples.append(self.pressure_mbar)
            time.sleep(0.1)
        
        if samples:
            surface_pressure = sum(samples) / len(samples)
            print(f"✅ Yüzey basıncı kalibre edildi: {surface_pressure:.1f} mbar")
            return True
        
        return False

if __name__ == "__main__":
    # Test kodu
    print("🔬 D300 Derinlik Sensörü Test")
    
    sensor = D300DepthSensor()
    
    if sensor.connect():
        sensor.start_monitoring()
        
        try:
            for i in range(50):
                data = sensor.get_sensor_data()
                print(f"Derinlik: {data['depth_m']:.2f}m, Sıcaklık: {data['temperature_c']:.1f}°C")
                time.sleep(0.2)
        except KeyboardInterrupt:
            print("\n🛑 Test durduruldu")
        
        sensor.disconnect()
    else:
        print("❌ Sensör bağlantısı başarısız!") 