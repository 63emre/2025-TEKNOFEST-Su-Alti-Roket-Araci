#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SENSORS AIR - Hava Yarışı için Sensör Yönetimi
D300 derinlik sensörü KALDIRILDI - Sadece attitude ve barometric altitude
Pluswing/sensors.py'den uyarlanmıştır
"""

import time
import math
import statistics
from collections import deque
from pymavlink import mavutil
from config_air import *

class Logger:
    """Basit logger sınıfı"""
    def __init__(self):
        pass
    
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warning(self, msg):
        print(f"[WARNING] {msg}")
    
    def error(self, msg):
        print(f"[ERROR] {msg}")
    
    def debug(self, msg):
        print(f"[DEBUG] {msg}")
    
    def critical(self, msg):
        print(f"[CRITICAL] {msg}")

class AltitudeSensor:
    """Barometric Altitude Sensörü - D300 yerine
    MAVLink SCALED_PRESSURE mesajları üzerinden altitude hesaplaması
    """
    
    def __init__(self, mavlink_connection, logger=None):
        self.mavlink = mavlink_connection
        self.logger = logger or Logger()
        
        self.altitude_offset = None  # Başlangıç altitude (referans)
        self.last_valid_altitude = None
        self.last_valid_time = None
        
        # Altitude hesaplama parametreleri
        self.pressure_queue = deque(maxlen=5)
        
        # Veri geçerlilik kontrolleri
        self.pressure_min = 700.0   # mbar
        self.pressure_max = 1200.0  # mbar
        self.temp_min = -10.0       # °C
        self.temp_max = 50.0        # °C
        
        self.consecutive_failures = 0
        self.max_failures = 10
        
        self.logger.info("Barometric altitude sensörü başlatıldı")
        
    def read_raw_data(self):
        """Ham barometric verisi oku"""
        try:
            msg = self.mavlink.recv_match(type='SCALED_PRESSURE', blocking=False, timeout=0.1)
            if msg is None:
                self.consecutive_failures += 1
                return None, None
                
            # Basınç ve sıcaklık verilerini al
            pressure_mbar = float(msg.press_abs)  # hPa = mbar
            temperature_c = float(msg.temperature) / 100.0  # Celsius
            
            # Veri geçerlilik kontrolü
            if not (self.pressure_min <= pressure_mbar <= self.pressure_max):
                self.consecutive_failures += 1
                return None, None
            if not (self.temp_min <= temperature_c <= self.temp_max):
                self.consecutive_failures += 1
                return None, None
                
            # Başarılı okuma
            self.consecutive_failures = 0
            self.last_valid_time = time.time()
            
            return pressure_mbar, temperature_c
            
        except Exception as e:
            self.consecutive_failures += 1
            self.logger.error(f"Altitude sensör okuma hatası: {e}")
            return None, None
    
    def _calculate_altitude(self, pressure_mbar):
        """Basınçtan altitude hesaplama (standart atmosfer modeli)"""
        if self.altitude_offset is None:
            return None
            
        # Standart atmosfer formülü: h = 44330 * (1 - (P/P0)^0.1903)
        # P0: referans basınç, P: mevcut basınç
        try:
            if pressure_mbar <= 0 or self.altitude_offset <= 0:
                return None
                
            altitude = 44330.0 * (1.0 - pow(pressure_mbar / self.altitude_offset, 0.1903))
            return max(0.0, altitude)  # Negatif altitude olmasın
            
        except Exception as e:
            self.logger.warning(f"Altitude hesaplama hatası: {e}")
            return None
    
    def get_altitude(self):
        """Altitude ölçümü (metre)"""
        pressure, _ = self.read_raw_data()
        
        if pressure is None or self.altitude_offset is None:
            return None
            
        # Medyan filtre uygula
        self.pressure_queue.append(pressure)
        if len(self.pressure_queue) >= 3:
            filtered_pressure = statistics.median(self.pressure_queue)
        else:
            filtered_pressure = pressure
            
        altitude = self._calculate_altitude(filtered_pressure)
        if altitude is not None:
            self.last_valid_altitude = altitude
            
        return altitude
    
    def get_altitude_safe(self):
        """Güvenli altitude ölçümü - fallback mekanizmalı"""
        altitude = self.get_altitude()
        
        if altitude is not None:
            return altitude, "CONNECTED", False
        
        # Fallback: Son geçerli altitude
        if self.last_valid_altitude is not None:
            self.logger.warning(f"⚠️ Altitude fallback: {self.last_valid_altitude:.2f}m")
            return self.last_valid_altitude, "FALLBACK", True
        
        # Hava testinde 0 altitude döndür
        if SIMULATE_DEPTH:
            return SIMULATED_DEPTH, "SIMULATED", True
            
        return None, "NO_DATA", True
    
    def calibrate_ground_level(self, duration=5):
        """Yer seviyesi kalibrasyonu"""
        try:
            self.logger.info(f"Yer seviyesi kalibrasyonu başlıyor... ({duration}s)")
            self.logger.info("📍 Aracı yer seviyesinde tutun")
                
            pressures = []
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                pressure, _ = self.read_raw_data()
                
                if pressure is not None:
                    pressures.append(pressure)
                    
                time.sleep(0.1)
                
            if len(pressures) < 5:
                self.logger.error("❌ Kalibrasyon için yeterli veri alınamadı")
                return False
                
            # Medyan değeri referans basınç olarak kullan
            self.altitude_offset = statistics.median(pressures)
            
            self.logger.info(f"✅ Yer seviyesi kalibrasyonu tamamlandı")
            self.logger.info(f"Referans basınç: {self.altitude_offset:.2f} mbar")
            self.logger.info(f"Kalibrasyon verisi: {len(pressures)} örnek")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            return False
    
    def is_connected(self):
        """Altitude sensörü bağlı mı"""
        return self.consecutive_failures < self.max_failures
    
    def get_all_data(self):
        """Tüm sensör verilerini döndür"""
        pressure, temperature = self.read_raw_data()
        altitude = self.get_altitude()
        
        return {
            'pressure_mbar': pressure,
            'temperature_c': temperature,
            'altitude_m': altitude,
            'is_valid': pressure is not None and altitude is not None
        }

class AttitudeSensor:
    """Pixhawk MAVLink Attitude Sensörü
    Roll, Pitch, Yaw verilerini alır
    """
    
    def __init__(self, mavlink_connection, logger=None):
        self.mavlink = mavlink_connection
        self.logger = logger or Logger()
        self.yaw_offset = None
        self.last_attitude = None
        self.last_update_time = 0
        
    def set_yaw_reference(self, yaw_angle=None):
        """Yaw referans noktasını ayarla"""
        if yaw_angle is not None:
            self.yaw_offset = yaw_angle
        else:
            # Mevcut yaw'ı referans olarak al
            attitude = self.get_attitude()
            if attitude:
                self.yaw_offset = attitude['yaw_raw']
                
        if self.yaw_offset is not None:
            self.logger.info(f"Yaw referansı ayarlandı: {math.degrees(self.yaw_offset):.1f}°")
            
    def normalize_yaw(self, yaw):
        """Yaw açısını -π ile +π arasına normalize et"""
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw
        
    def get_attitude(self, timeout=1.0):
        """Attitude verilerini al"""
        try:
            msg = self.mavlink.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
            
            if msg is None:
                return None
                
            # Ham değerler
            roll_raw = float(msg.roll)
            pitch_raw = float(msg.pitch) 
            yaw_raw = float(msg.yaw)
            
            # Relatif yaw hesapla
            yaw_relative = None
            if self.yaw_offset is not None:
                yaw_relative = self.normalize_yaw(yaw_raw - self.yaw_offset)
            
            attitude_data = {
                'roll': roll_raw,
                'pitch': pitch_raw,
                'yaw_raw': yaw_raw,
                'yaw_relative': yaw_relative,
                'roll_deg': math.degrees(roll_raw),
                'pitch_deg': math.degrees(pitch_raw),
                'yaw_raw_deg': math.degrees(yaw_raw),
                'yaw_relative_deg': math.degrees(yaw_relative) if yaw_relative is not None else None,
                'timestamp': time.time()
            }
            
            self.last_attitude = attitude_data
            self.last_update_time = time.time()
            
            return attitude_data
            
        except Exception as e:
            self.logger.error(f"Attitude okuma hatası: {e}")
            return None
            
    def get_last_attitude(self, max_age=2.0):
        """Son attitude verisini döndür (yaş kontrolü ile)"""
        if self.last_attitude and (time.time() - self.last_update_time) < max_age:
            return self.last_attitude
        return None
        
    def is_data_fresh(self, max_age=1.0):
        """Veri güncel mi kontrol et"""
        return (time.time() - self.last_update_time) < max_age

class SystemSensor:
    """Pixhawk sistem durumu sensörü"""
    
    def __init__(self, mavlink_connection, logger=None):
        self.mavlink = mavlink_connection
        self.logger = logger or Logger()
        self.last_system_status = None
        
    def get_system_status(self, timeout=1.0):
        """Sistem durumunu al"""
        try:
            msg = self.mavlink.recv_match(type='SYS_STATUS', blocking=True, timeout=timeout)
            
            if msg is None:
                return None
                
            status_data = {
                'load': msg.load / 10.0,                         # %
                'errors_count1': msg.errors_count1,
                'errors_count2': msg.errors_count2,
                'errors_count3': msg.errors_count3,
                'errors_count4': msg.errors_count4,
                'timestamp': time.time()
            }
            
            self.last_system_status = status_data
            return status_data
            
        except Exception as e:
            self.logger.error(f"Sistem durumu okuma hatası: {e}")
            return None

class SensorManager:
    """Hava yarışı için sensör yöneticisi - D300 YOK"""
    
    def __init__(self, mavlink_connection, logger=None):
        self.logger = logger or Logger()
        self.mavlink = mavlink_connection
        
        # Sensörleri başlat - D300 YOK!
        self.altitude = AltitudeSensor(mavlink_connection, self.logger)  # D300 yerine
        self.attitude = AttitudeSensor(mavlink_connection, self.logger)
        self.system = SystemSensor(mavlink_connection, self.logger)
        
        # Backward compatibility için depth property ekle (ama kullanma!)
        self.depth = None  # KULLANILMAYACAK!
        
        self.logger.info("Hava yarışı sensör yöneticisi başlatıldı (D300 YOK)")
    
    def cleanup(self):
        """Tüm sensörleri temizle"""
        self.logger.info("Sensör yöneticisi temizleniyor...")
        try:
            if hasattr(self.altitude, 'cleanup'):
                self.altitude.cleanup()
            if hasattr(self.attitude, 'cleanup'):
                self.attitude.cleanup()
            if hasattr(self.system, 'cleanup'):
                self.system.cleanup()
        except Exception as e:
            self.logger.warning(f"Sensör temizlik hatası: {e}")
        self.logger.info("✅ Sensör yöneticisi temizlendi")
        
    def calibrate_all(self):
        """Tüm sensörleri kalibre et - D300 YOK"""
        self.logger.info("Hava yarışı sensör kalibrasyonu başlatılıyor...")
        
        results = {}
        
        # Altitude sensörü kalibrasyonu
        results['altitude'] = self.altitude.calibrate_ground_level(duration=5)
        
        # Attitude referans ayarlama
        self.attitude.set_yaw_reference()
        results['attitude'] = self.attitude.yaw_offset is not None
        
        # Sistem durumu kontrolü
        system_status = self.system.get_system_status()
        results['system'] = system_status is not None
        
        # DEPTH YOK - simüle et
        results['depth'] = True  # Her zaman başarılı (simüle)
        
        success_count = sum(results.values())
        total_count = len(results)
        
        self.logger.info(f"Hava yarışı kalibrasyonu tamamlandı: {success_count}/{total_count} sensör başarılı")
        
        return results
        
    def get_all_sensor_data(self):
        """Tüm sensör verilerini al - D300 YOK"""
        # Simüle edilmiş depth verisi (backward compatibility için)
        simulated_depth_data = {
            'pressure_mbar': None,
            'temperature_c': None,
            'depth_m': SIMULATED_DEPTH,
            'is_valid': SIMULATE_DEPTH
        }
        
        data = {
            'depth': simulated_depth_data,  # Simüle edilmiş
            'altitude': self.altitude.get_all_data(),  # Gerçek altitude
            'attitude': self.attitude.get_attitude(timeout=0.1),
            'system': self.system.get_system_status(timeout=0.1),
            'timestamp': time.time()
        }
        
        return data
        
    def check_sensor_health(self):
        """Sensör sağlığını kontrol et"""
        health = {
            'depth_connected': SIMULATE_DEPTH,  # Simüle edilmiş
            'altitude_connected': self.altitude.is_connected(),
            'attitude_fresh': self.attitude.is_data_fresh(),
            'overall_healthy': True
        }
        
        # Genel sağlık durumu
        health['overall_healthy'] = all([
            health['altitude_connected'],
            health['attitude_fresh']
        ])
        
        return health
        
    def test_altitude_connection(self):
        """Altitude sensörü bağlantısını test et"""
        try:
            self.logger.info("Altitude sensör bağlantısı test ediliyor...")
            
            # 3 deneme yap
            for attempt in range(3):
                pressure, temperature = self.altitude.read_raw_data()
                
                if pressure is not None and temperature is not None:
                    self.logger.info(f"✅ Altitude test başarılı - Basınç: {pressure:.1f}mbar, Sıcaklık: {temperature:.1f}°C")
                    return True
                    
                if attempt < 2:  # Son denemede bekleme
                    time.sleep(0.5)
                    
            self.logger.error("❌ Altitude sensör bağlantısı başarısız")
            return False
            
        except Exception as e:
            self.logger.error(f"Altitude test hatası: {e}")
            return False
            
    def test_attitude_connection(self):
        """Attitude sensörü bağlantısını test et"""
        try:
            self.logger.info("Attitude sensör bağlantısı test ediliyor...")
            
            # 3 deneme yap
            for attempt in range(3):
                attitude = self.attitude.get_attitude(timeout=1.0)
                
                if attitude is not None:
                    self.logger.info(f"✅ Attitude test başarılı - Roll: {attitude['roll_deg']:.1f}°, "
                                   f"Pitch: {attitude['pitch_deg']:.1f}°, Yaw: {attitude['yaw_raw_deg']:.1f}°")
                    return True
                    
                if attempt < 2:  # Son denemede bekleme
                    time.sleep(0.5)
                    
            self.logger.error("❌ Attitude sensör bağlantısı başarısız")
            return False
            
        except Exception as e:
            self.logger.error(f"Attitude test hatası: {e}")
            return False

    def log_sensor_status(self):
        """Sensör durumunu logla"""
        health = self.check_sensor_health()
        data = self.get_all_sensor_data()
        
        # Altitude sensörü
        if data['altitude']['is_valid']:
            self.logger.info(f"Altitude: {data['altitude']['altitude_m']:.3f}m, "
                           f"Basınç: {data['altitude']['pressure_mbar']:.1f}mbar, "
                           f"Sıcaklık: {data['altitude']['temperature_c']:.1f}°C")
        else:
            self.logger.warning("Altitude sensörü verisi geçersiz")
            
        # Attitude sensörü
        if data['attitude']:
            att = data['attitude']
            self.logger.info(f"Attitude - Roll: {att['roll_deg']:.1f}°, "
                           f"Pitch: {att['pitch_deg']:.1f}°, "
                           f"Yaw: {att['yaw_relative_deg']:.1f}°")
        else:
            self.logger.warning("Attitude verisi alınamadı")
            
        # Sistem durumu
        if data['system']:
            sys = data['system']
            self.logger.info(f"Sistem - Yük: {sys['load']:.1f}%")
                           
        # Simüle edilmiş depth
        if SIMULATE_DEPTH:
            self.logger.info(f"Simüle Depth: {SIMULATED_DEPTH:.2f}m (Hava testi)")
                           
        # Genel sağlık
        if not health['overall_healthy']:
            self.logger.warning("Sensör sağlık problemi tespit edildi!")
            
        return health
