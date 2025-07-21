#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
D300 Derinlik ve Sıcaklık Sensörü Test Scripti

D300 sensörü I2C protokolü ile haberleşir ve hem derinlik hem de su sıcaklığı ölçümü yapar.
Bu script sensörün doğru çalıştığını ve kalibrasyonunu test eder.

Hardware:
- D300 Derinlik ve Sıcaklık Sensörü
- I2C bağlantısı (GPIO 2/3 - Raspberry Pi)
- 0x77 I2C address (default)

Referans: https://www.mucif.com/urunler/d300-derinlik-ve-su-sicakligi-sensoru
"""

import smbus2
import time
import json
import math
import numpy as np
from datetime import datetime, timedelta
import sys
import threading
import signal

# D300 Sensor Configuration  
I2C_BUS = 1                    # Raspberry Pi I2C bus
I2C_D300_ADDRESS = 0x77        # Default D300 address
SENSOR_DATA_RATE = 10          # Maximum 10Hz sampling rate

# Calibration Constants (D300 specific)
DEPTH_RESOLUTION = 0.01        # 0.01m resolution
TEMP_RESOLUTION = 0.01         # 0.01°C resolution
DEPTH_RANGE_MAX = 300.0        # 300m maximum depth
TEMP_RANGE_MIN = -20.0         # -20°C minimum 
TEMP_RANGE_MAX = 85.0          # +85°C maximum

# Test Parameters
CALIBRATION_SAMPLES = 50       # Surface pressure calibration sample count
DEPTH_ACCURACY_THRESHOLD = 0.05 # ±5cm accuracy threshold
TEMP_ACCURACY_THRESHOLD = 0.5  # ±0.5°C accuracy threshold
SENSOR_TIMEOUT = 5             # Sensor read timeout (seconds)

class D300DepthSensor:
    """D300 Derinlik ve Sıcaklık Sensörü Sınıfı"""
    
    def __init__(self, bus_num=I2C_BUS, address=I2C_D300_ADDRESS):
        self.bus_num = bus_num
        self.address = address
        self.bus = None
        self.connected = False
        
        # Sensor data
        self.current_depth = 0.0
        self.current_temperature = 0.0
        self.current_pressure = 1013.25  # mbar
        self.surface_pressure = 1013.25  # mbar (calibrated)
        
        # Calibration data
        self.calibrated = False
        self.calibration_offset = 0.0
        self.temperature_offset = 0.0
        
        # Statistics
        self.total_readings = 0
        self.failed_readings = 0
        self.last_reading_time = None
        
    def connect_sensor(self):
        """D300 sensörüne bağlan"""
        try:
            print(f"🔌 D300 sensörüne bağlanılıyor (I2C: {self.address:#04x})")
            self.bus = smbus2.SMBus(self.bus_num)
            
            # Sensor presence test
            if self.test_sensor_presence():
                self.connected = True
                print("✅ D300 sensörü başarıyla bağlandı!")
                return True
            else:
                print("❌ D300 sensörü bulunamadı!")
                return False
                
        except Exception as e:
            print(f"❌ D300 bağlantı hatası: {e}")
            return False
    
    def disconnect_sensor(self):
        """Sensör bağlantısını kapat"""
        if self.bus:
            self.bus.close()
            self.connected = False
            print("🔌 D300 sensörü bağlantısı kapatıldı")
    
    def test_sensor_presence(self):
        """Sensörün varlığını test et"""
        try:
            # D300 için basit read test
            data = self.bus.read_i2c_block_data(self.address, 0x00, 1)
            return True
        except Exception as e:
            print(f"⚠️ Sensör presence test hatası: {e}")
            return False
    
    def read_raw_data(self):
        """D300'den raw data oku"""
        if not self.connected:
            return None, None, None
        
        try:
            # D300 specific read sequence
            # Register 0x00: Trigger measurement
            self.bus.write_byte(self.address, 0x00)
            time.sleep(0.1)  # Conversion time
            
            # Register 0x01-0x06: Read pressure and temperature data
            data = self.bus.read_i2c_block_data(self.address, 0x01, 6)
            
            # Parse pressure data (24-bit)
            pressure_raw = (data[0] << 16) | (data[1] << 8) | data[2]
            
            # Parse temperature data (24-bit)  
            temperature_raw = (data[3] << 16) | (data[4] << 8) | data[5]
            
            # Convert to actual values (D300 specific conversion)
            pressure_mbar = self.convert_pressure(pressure_raw)
            temperature_celsius = self.convert_temperature(temperature_raw)
            
            # Calculate depth from pressure
            depth_meters = self.calculate_depth(pressure_mbar)
            
            self.current_pressure = pressure_mbar
            self.current_temperature = temperature_celsius
            self.current_depth = depth_meters
            self.last_reading_time = time.time()
            self.total_readings += 1
            
            return depth_meters, temperature_celsius, pressure_mbar
            
        except Exception as e:
            print(f"❌ D300 veri okuma hatası: {e}")
            self.failed_readings += 1
            return None, None, None
    
    def convert_pressure(self, raw_value):
        """Raw pressure değerini mbar'a çevir"""
        # D300 specific conversion formula
        # This would need to be adjusted based on actual D300 datasheet
        pressure_mbar = (raw_value / 16384.0) + 1000.0  # Example conversion
        return pressure_mbar
    
    def convert_temperature(self, raw_value):
        """Raw temperature değerini Celsius'a çevir"""
        # D300 specific conversion formula
        # This would need to be adjusted based on actual D300 datasheet
        temperature_celsius = (raw_value / 256.0) - 40.0  # Example conversion
        return temperature_celsius + self.temperature_offset
    
    def calculate_depth(self, pressure_mbar):
        """Basınçtan derinlik hesapla"""
        if not self.calibrated:
            return 0.0
        
        # Hydrostatic pressure formula: P = P0 + ρgh
        # Where: ρ = 1025 kg/m³ (seawater density)
        #        g = 9.81 m/s²
        #        h = depth in meters
        pressure_diff = pressure_mbar - self.surface_pressure
        depth = (pressure_diff * 100) / (1025 * 9.81)  # Convert mbar to Pa and calculate
        
        return max(0.0, depth + self.calibration_offset)
    
    def calibrate_surface_pressure(self, sample_count=CALIBRATION_SAMPLES):
        """Yüzey basıncı kalibrasyonu"""
        print(f"🔧 Yüzey basıncı kalibrasyonu başlatılıyor ({sample_count} örnek)...")
        
        if not self.connected:
            print("❌ Sensör bağlı değil!")
            return False
        
        pressure_samples = []
        
        for i in range(sample_count):
            depth, temp, pressure = self.read_raw_data()
            
            if pressure is not None:
                pressure_samples.append(pressure)
                print(f"   Örnek {i+1}/{sample_count}: {pressure:.2f} mbar")
            else:
                print(f"   Örnek {i+1}/{sample_count}: Hata!")
            
            time.sleep(0.1)
        
        if len(pressure_samples) < sample_count * 0.8:  # En az %80 başarı
            print("❌ Kalibrasyon başarısız - yetersiz örnek!")
            return False
        
        # İstatistiksel analiz
        self.surface_pressure = np.mean(pressure_samples)
        pressure_std = np.std(pressure_samples)
        
        print(f"✅ Yüzey basıncı kalibrasyonu tamamlandı!")
        print(f"   Ortalama basınç: {self.surface_pressure:.2f} mbar")
        print(f"   Standart sapma: {pressure_std:.2f} mbar")
        print(f"   Min: {min(pressure_samples):.2f} mbar")
        print(f"   Max: {max(pressure_samples):.2f} mbar")
        
        self.calibrated = True
        return True
    
    def calibrate_temperature_offset(self, reference_temp):
        """Sıcaklık offset kalibrasyonu"""
        print(f"🌡️ Sıcaklık kalibrasyonu (Referans: {reference_temp}°C)")
        
        temp_samples = []
        
        for i in range(20):
            depth, temp, pressure = self.read_raw_data()
            if temp is not None:
                temp_samples.append(temp)
            time.sleep(0.5)
        
        if len(temp_samples) < 15:
            print("❌ Sıcaklık kalibrasyonu başarısız!")
            return False
        
        measured_avg = np.mean(temp_samples)
        self.temperature_offset = reference_temp - measured_avg
        
        print(f"✅ Sıcaklık offset: {self.temperature_offset:.2f}°C")
        return True
    
    def run_accuracy_test(self, known_depth=0.0, duration=30):
        """Doğruluk testi"""
        print(f"📏 Doğruluk testi başlatılıyor (Bilinen derinlik: {known_depth}m)")
        
        if not self.calibrated:
            print("❌ Sensör kalibre edilmemiş!")
            return False
        
        depth_readings = []
        temp_readings = []
        start_time = time.time()
        
        print("📊 Veri toplama başladı...")
        
        while time.time() - start_time < duration:
            depth, temp, pressure = self.read_raw_data()
            
            if depth is not None and temp is not None:
                depth_readings.append(depth)
                temp_readings.append(temp)
                
                print(f"   Derinlik: {depth:.3f}m, Sıcaklık: {temp:.2f}°C, Basınç: {pressure:.1f}mbar")
            
            time.sleep(1.0)
        
        if len(depth_readings) < 10:
            print("❌ Yetersiz veri!")
            return False
        
        # İstatistiksel analiz
        depth_mean = np.mean(depth_readings)
        depth_std = np.std(depth_readings)
        depth_error = abs(depth_mean - known_depth)
        
        temp_mean = np.mean(temp_readings)
        temp_std = np.std(temp_readings)
        
        print("\n📊 DOĞRULUK TESTİ SONUÇLARI")
        print("=" * 50)
        print(f"Derinlik Ortalaması: {depth_mean:.3f} ± {depth_std:.3f}m")
        print(f"Bilinen Derinlik:    {known_depth:.3f}m")
        print(f"Mutlak Hata:         {depth_error:.3f}m")
        print(f"Sıcaklık Ortalaması: {temp_mean:.2f} ± {temp_std:.2f}°C")
        print(f"Toplam Okuma:        {len(depth_readings)} adet")
        
        # Doğruluk değerlendirmesi
        depth_accurate = depth_error <= DEPTH_ACCURACY_THRESHOLD
        temp_stable = temp_std <= TEMP_ACCURACY_THRESHOLD
        
        print(f"\n✅ Derinlik Doğruluğu: {'GEÇTİ' if depth_accurate else 'BAŞARISIZ'}")
        print(f"✅ Sıcaklık Kararlılığı: {'GEÇTİ' if temp_stable else 'BAŞARISIZ'}")
        
        return depth_accurate and temp_stable
    
    def run_response_time_test(self):
        """Yanıt süresi testi"""
        print("⏱️ Yanıt süresi testi başlatılıyor...")
        
        response_times = []
        
        for i in range(20):
            start_time = time.time()
            depth, temp, pressure = self.read_raw_data()
            end_time = time.time()
            
            if depth is not None:
                response_time = (end_time - start_time) * 1000  # ms
                response_times.append(response_time)
                print(f"   Okuma {i+1}: {response_time:.1f}ms")
            
            time.sleep(0.5)
        
        if len(response_times) < 15:
            print("❌ Yanıt süresi testi başarısız!")
            return False
        
        avg_response = np.mean(response_times)
        max_response = max(response_times)
        min_response = min(response_times)
        
        print(f"\n📊 YANITLAMA SÜRESİ ANALİZİ")
        print(f"Ortalama: {avg_response:.1f}ms")
        print(f"Minimum:  {min_response:.1f}ms")  
        print(f"Maksimum: {max_response:.1f}ms")
        
        # 100ms altında olmalı (10Hz için)
        response_good = avg_response < 100.0
        print(f"✅ Yanıt Süresi: {'GEÇTİ' if response_good else 'BAŞARISIZ'}")
        
        return response_good
    
    def run_stability_test(self, duration=60):
        """Kararlılık testi"""
        print(f"🔄 Kararlılık testi başlatılıyor ({duration}s)...")
        
        depth_readings = []
        temp_readings = []
        timestamps = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            depth, temp, pressure = self.read_raw_data()
            
            if depth is not None and temp is not None:
                depth_readings.append(depth)
                temp_readings.append(temp)
                timestamps.append(time.time() - start_time)
                
                if len(depth_readings) % 10 == 0:
                    print(f"   {len(depth_readings)} okuma tamamlandı...")
            
            time.sleep(1.0)
        
        if len(depth_readings) < 30:
            print("❌ Kararlılık testi için yetersiz veri!")
            return False
        
        # Drift analizi
        depth_drift = abs(depth_readings[-1] - depth_readings[0])
        temp_drift = abs(temp_readings[-1] - temp_readings[0])
        
        depth_noise = np.std(depth_readings)
        temp_noise = np.std(temp_readings)
        
        print(f"\n📊 KARARLILIK ANALİZİ")
        print(f"Derinlik Drift:  {depth_drift:.3f}m")
        print(f"Sıcaklık Drift:  {temp_drift:.2f}°C")
        print(f"Derinlik Gürültü: {depth_noise:.3f}m")
        print(f"Sıcaklık Gürültü: {temp_noise:.2f}°C")
        
        stability_good = depth_drift < 0.1 and temp_drift < 1.0
        print(f"✅ Kararlılık: {'GEÇTİ' if stability_good else 'BAŞARISIZ'}")
        
        return stability_good
    
    def generate_test_report(self):
        """Test raporu oluştur"""
        report = {
            'sensor_type': 'D300 Derinlik ve Sıcaklık Sensörü',
            'test_timestamp': datetime.now().isoformat(),
            'connection_status': self.connected,
            'calibration_status': self.calibrated,
            'surface_pressure': self.surface_pressure,
            'temperature_offset': self.temperature_offset,
            'statistics': {
                'total_readings': self.total_readings,
                'failed_readings': self.failed_readings,
                'success_rate': (self.total_readings - self.failed_readings) / max(1, self.total_readings) * 100
            },
            'current_values': {
                'depth': self.current_depth,
                'temperature': self.current_temperature,
                'pressure': self.current_pressure
            }
        }
        
        return report

def main():
    print("🔬 TEKNOFEST 2025 - D300 Derinlik Sensörü Test Suite")
    print("=" * 60)
    
    # D300 sensör instance
    sensor = D300DepthSensor()
    
    try:
        # Sensöre bağlan
        if not sensor.connect_sensor():
            print("❌ Sensör bağlantısı başarısız!")
            return 1
        
        # Test menüsü
        while True:
            print("\n🔧 TEST MENÜSÜ")
            print("1. Yüzey Basıncı Kalibrasyonu")
            print("2. Sıcaklık Offset Kalibrasyonu")
            print("3. Doğruluk Testi")
            print("4. Yanıt Süresi Testi")
            print("5. Kararlılık Testi")
            print("6. Canlı Veri Monitörleme")
            print("7. Test Raporu Oluştur")
            print("0. Çıkış")
            
            choice = input("\nSeçiminiz (0-7): ").strip()
            
            if choice == '1':
                sensor.calibrate_surface_pressure()
            
            elif choice == '2':
                ref_temp = float(input("Referans sıcaklık (°C): "))
                sensor.calibrate_temperature_offset(ref_temp)
            
            elif choice == '3':
                known_depth = float(input("Bilinen derinlik (m, yüzey için 0): "))
                duration = int(input("Test süresi (saniye): "))
                sensor.run_accuracy_test(known_depth, duration)
            
            elif choice == '4':
                sensor.run_response_time_test()
            
            elif choice == '5':
                duration = int(input("Test süresi (saniye): "))
                sensor.run_stability_test(duration)
            
            elif choice == '6':
                print("📡 Canlı veri monitörleme (Ctrl+C ile durdurun)")
                try:
                    while True:
                        depth, temp, pressure = sensor.read_raw_data()
                        if depth is not None:
                            print(f"Derinlik: {depth:.3f}m | Sıcaklık: {temp:.2f}°C | Basınç: {pressure:.1f}mbar")
                        time.sleep(1.0)
                except KeyboardInterrupt:
                    print("\n⚠️ Monitörleme durduruldu")
            
            elif choice == '7':
                report = sensor.generate_test_report()
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"d300_test_report_{timestamp}.json"
                
                with open(filename, 'w', encoding='utf-8') as f:
                    json.dump(report, f, ensure_ascii=False, indent=2)
                
                print(f"📊 Test raporu kaydedildi: {filename}")
                print(json.dumps(report, ensure_ascii=False, indent=2))
            
            elif choice == '0':
                break
            
            else:
                print("❌ Geçersiz seçim!")
        
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
        return 1
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        return 1
    finally:
        sensor.disconnect_sensor()

if __name__ == "__main__":
    sys.exit(main()) 