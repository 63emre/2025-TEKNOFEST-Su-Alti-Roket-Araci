"""
TEKNOFEST 2025 Su Altı Roket Aracı  
D300 Derinlik ve Su Sıcaklığı Sensörü Helper Modülü

Bu modül D300 sensörü ile I2C üzerinden haberleşme sağlar.
I2C Adresi: 0x76
"""

import time
import logging
import smbus2
import threading
from typing import Dict, Optional

class D300Sensor:
    """D300 Derinlik ve Sıcaklık Sensörü Kontrol Sınıfı"""
    
    def __init__(self, bus_number: int = 1, address: int = 0x76):
        """
        D300 sensörünü başlat
        
        Args:
            bus_number: I2C bus numarası (Raspberry Pi'de genellikle 1)
            address: I2C adresi (0x76)
        """
        self.bus_number = bus_number
        self.address = address
        self.bus = None
        self.connected = False
        
        # Veri depolama
        self.depth_data = {"depth_m": 0.0, "pressure_mbar": 0.0}
        self.temperature_data = {"temp_celsius": 0.0}
        self.last_reading_time = 0
        
        # Logging ayarları
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Sürekli okuma için thread kontrolü
        self._stop_reading = threading.Event()
        self._reading_thread = None
        self.reading_interval = 0.1  # 10Hz okuma hızı
        
        # Kalibrasyon değerleri (sensör özelinde ayarlanmalı)
        self.depth_offset = 0.0  # Yüzey seviyesi offset'i
        self.pressure_sea_level = 1013.25  # Deniz seviyesi basıncı (mbar)
    
    def connect(self) -> bool:
        """
        D300 sensörüne bağlan
        
        Returns:
            bool: Bağlantı durumu
        """
        try:
            self.logger.info(f"D300 sensörüne bağlanılıyor (Bus: {self.bus_number}, Adres: 0x{self.address:02X})")
            
            # I2C bus bağlantısını aç
            self.bus = smbus2.SMBus(self.bus_number)
            
            # Sensör varlığını test et
            if self._test_sensor_connection():
                self.connected = True
                self.logger.info("D300 sensörü bağlantısı başarılı!")
                return True
            else:
                self.logger.error("D300 sensörü tespit edilemedi!")
                self.connected = False
                return False
                
        except Exception as e:
            self.logger.error(f"D300 bağlantı hatası: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Sensör bağlantısını kapat"""
        self.connected = False
        self.stop_continuous_reading()
        
        if self.bus:
            self.bus.close()
            self.logger.info("D300 sensörü bağlantısı kapatıldı")
    
    def _test_sensor_connection(self) -> bool:
        """Sensör bağlantısını test et"""
        try:
            # Basit bir okuma testi yap
            self.bus.read_byte(self.address)
            return True
        except Exception:
            return False
    
    def start_continuous_reading(self):
        """Sürekli veri okuma thread'ini başlat"""
        if not self._reading_thread or not self._reading_thread.is_alive():
            self._stop_reading.clear()
            self._reading_thread = threading.Thread(target=self._continuous_read_loop)
            self._reading_thread.daemon = True
            self._reading_thread.start()
            self.logger.info("D300 sürekli okuma başlatıldı")
    
    def stop_continuous_reading(self):
        """Sürekli veri okuma thread'ini durdur"""
        self._stop_reading.set()
        if self._reading_thread:
            self._reading_thread.join(timeout=2)
            self.logger.info("D300 sürekli okuma durduruldu")
    
    def _continuous_read_loop(self):
        """Sürekli okuma döngüsü"""
        while not self._stop_reading.is_set() and self.connected:
            try:
                self.read_sensor_data()
                time.sleep(self.reading_interval)
            except Exception as e:
                self.logger.error(f"D300 sürekli okuma hatası: {e}")
                time.sleep(1)  # Hata durumunda daha uzun bekle
    
    def read_sensor_data(self) -> Dict[str, float]:
        """
        Sensörden basınç ve sıcaklık verilerini oku
        
        Returns:
            Dict[str, float]: Sensor verileri
        """
        if not self.connected:
            self.logger.warning("D300 sensörü bağlı değil!")
            return {"depth_m": 0.0, "pressure_mbar": 0.0, "temp_celsius": 0.0}
        
        try:
            # D300 sensörü için özel okuma protokolü
            # Not: Gerçek D300 sensörünün protokolüne göre bu kısım güncellenmelidir
            
            # Basınç verisi oku (örnek protokol)
            pressure_raw = self._read_pressure_raw()
            temperature_raw = self._read_temperature_raw()
            
            # Ham verileri işle
            pressure_mbar = self._process_pressure_data(pressure_raw)
            temperature_celsius = self._process_temperature_data(temperature_raw)
            depth_m = self._calculate_depth_from_pressure(pressure_mbar)
            
            # Verileri güncelle
            self.depth_data = {
                "depth_m": depth_m,
                "pressure_mbar": pressure_mbar
            }
            self.temperature_data = {"temp_celsius": temperature_celsius}
            self.last_reading_time = time.time()
            
            return {
                "depth_m": depth_m,
                "pressure_mbar": pressure_mbar, 
                "temp_celsius": temperature_celsius
            }
            
        except Exception as e:
            self.logger.error(f"D300 veri okuma hatası: {e}")
            return {"depth_m": 0.0, "pressure_mbar": 0.0, "temp_celsius": 0.0}
    
    def _read_pressure_raw(self) -> int:
        """Ham basınç verisini oku"""
        # D300 sensörüne özel basınç okuma komutu
        # Bu kısım sensörün gerçek protokolüne göre güncellenmelidir
        try:
            # Örnek: 2 byte basınç verisi oku
            data = self.bus.read_i2c_block_data(self.address, 0x00, 2)
            return (data[0] << 8) | data[1]
        except Exception:
            return 0
    
    def _read_temperature_raw(self) -> int:
        """Ham sıcaklık verisini oku"""
        # D300 sensörüne özel sıcaklık okuma komutu
        try:
            # Örnek: 2 byte sıcaklık verisi oku
            data = self.bus.read_i2c_block_data(self.address, 0x02, 2)
            return (data[0] << 8) | data[1]
        except Exception:
            return 0
    
    def _process_pressure_data(self, raw_data: int) -> float:
        """Ham basınç verisini işle ve mbar cinsinden döndür"""
        # D300 sensörünün kalibrasyonuna göre bu formül güncellenmelidir
        # Örnek hesaplama (sensör datasheetinden alınmalı)
        pressure_mbar = (raw_data / 4096.0) * 1000.0  # Örnek çevirme
        return pressure_mbar
    
    def _process_temperature_data(self, raw_data: int) -> float:
        """Ham sıcaklık verisini işle ve Celsius cinsinden döndür"""
        # D300 sensörünün kalibrasyonuna göre bu formül güncellenmelidir
        # Örnek hesaplama
        temp_celsius = (raw_data / 100.0) - 40.0  # Örnek çevirme
        return temp_celsius
    
    def _calculate_depth_from_pressure(self, pressure_mbar: float) -> float:
        """Basınçtan derinlik hesapla"""
        # Su altı derinlik hesaplama formülü
        # 1 mbar = yaklaşık 1 cm su derinliği
        pressure_diff = pressure_mbar - self.pressure_sea_level
        depth_m = (pressure_diff / 100.0) + self.depth_offset  # mbar to metre
        return max(0.0, depth_m)  # Negatif derinlik olmasın
    
    def get_depth(self) -> float:
        """Güncel derinlik verisini al (metre)"""
        return self.depth_data["depth_m"]
    
    def get_pressure(self) -> float:
        """Güncel basınç verisini al (mbar)"""
        return self.depth_data["pressure_mbar"]
    
    def get_temperature(self) -> float:
        """Güncel sıcaklık verisini al (Celsius)"""
        return self.temperature_data["temp_celsius"]
    
    def get_all_data(self) -> Dict[str, float]:
        """Tüm sensör verilerini al"""
        return {
            "depth_m": self.get_depth(),
            "pressure_mbar": self.get_pressure(),
            "temp_celsius": self.get_temperature(),
            "timestamp": self.last_reading_time
        }
    
    def calibrate_surface_level(self):
        """Yüzey seviyesi kalibrasyonu yap"""
        if not self.connected:
            self.logger.warning("Kalibrasyon için sensör bağlı olmalı!")
            return
        
        self.logger.info("Yüzey seviyesi kalibrasyonu başlatılıyor...")
        
        # 10 ölçüm al ve ortalamasını kullan
        measurements = []
        for i in range(10):
            data = self.read_sensor_data()
            measurements.append(data["pressure_mbar"])
            time.sleep(0.1)
        
        # Ortalama basıncı yüzey seviyesi olarak ayarla
        self.pressure_sea_level = sum(measurements) / len(measurements)
        self.depth_offset = 0.0
        
        self.logger.info(f"Kalibrasyon tamamlandı. Yüzey basıncı: {self.pressure_sea_level:.2f} mbar")
    
    def is_connected(self) -> bool:
        """Bağlantı durumunu kontrol et"""
        if not self.connected:
            return False
        
        try:
            self._test_sensor_connection()
            return True
        except:
            return False
    
    def get_status_summary(self) -> Dict:
        """Durum özetini al"""
        return {
            "connected": self.is_connected(),
            "last_reading": time.time() - self.last_reading_time if self.last_reading_time > 0 else None,
            "data": self.get_all_data()
        }

# Test fonksiyonu
if __name__ == "__main__":
    # D300 sensörünü test et
    sensor = D300Sensor()
    
    try:
        if sensor.connect():
            print("D300 sensörüne başarıyla bağlanıldı!")
            
            # Yüzey kalibrasyonu yap
            print("Yüzey kalibrasyonu yapılıyor...")
            sensor.calibrate_surface_level()
            
            # Sürekli okuma başlat
            sensor.start_continuous_reading()
            
            # 30 saniye boyunca veri oku
            start_time = time.time()
            while time.time() - start_time < 30:
                data = sensor.get_all_data()
                print(f"Derinlik: {data['depth_m']:.2f}m, "
                      f"Basınç: {data['pressure_mbar']:.2f}mbar, "
                      f"Sıcaklık: {data['temp_celsius']:.1f}°C")
                time.sleep(1)
        else:
            print("D300 sensörü bağlantısı başarısız!")
            
    finally:
        sensor.disconnect()
