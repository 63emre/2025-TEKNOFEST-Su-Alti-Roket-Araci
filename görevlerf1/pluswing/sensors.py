#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SENSORS - Sensör Okuma ve Yönetimi
D300 derinlik sensörü ve Pixhawk MAVLink telemetri fonksiyonları
"""

import time
import math
import smbus2
from pymavlink import mavutil
from config import *
from utils import Logger

class DepthSensor:
    """D300 Derinlik Sensörü Sınıfı
    I2C üzerinden basınç ve sıcaklık okuması yapar
    """
    
    def __init__(self, logger=None):
        self.bus = None
        self.address = D300_I2C_ADDRESS
        self.pressure_offset = None
        self.temperature_offset = None
        self.logger = logger or Logger()
        
        self._init_i2c()
        
    def _init_i2c(self):
        """I2C bağlantısını başlat"""
        try:
            self.bus = smbus2.SMBus(D300_I2C_BUS)
            self.logger.info(f"D300 sensörü I2C bus {D300_I2C_BUS} üzerinde başlatıldı")
        except Exception as e:
            self.logger.error(f"I2C başlatma hatası: {e}")
            self.bus = None
            
    def read_raw_data(self):
        """Ham sensör verisi oku"""
        if not self.bus:
            return None, None
            
        try:
            # D300 sensörü için basit okuma (sensör protokolüne göre ayarlanabilir)
            # Bu örnek genel bir I2C okuma, gerçek D300 protokolü farklı olabilir
            pressure_data = self.bus.read_i2c_block_data(self.address, 0x00, 4)
            temp_data = self.bus.read_i2c_block_data(self.address, 0x04, 4)
            
            # Big-endian 32-bit değer olarak yorumla
            pressure_raw = (pressure_data[0] << 24 | pressure_data[1] << 16 | 
                           pressure_data[2] << 8 | pressure_data[3])
            temp_raw = (temp_data[0] << 24 | temp_data[1] << 16 | 
                       temp_data[2] << 8 | temp_data[3])
            
            # mbar ve Celsius'a çevir (sensör spesifikasyonuna göre)
            pressure_mbar = pressure_raw / 100.0
            temperature_c = temp_raw / 100.0
            
            return pressure_mbar, temperature_c
            
        except Exception as e:
            self.logger.error(f"D300 sensör okuma hatası: {e}")
            return None, None
    
    def calibrate_surface(self, samples=10):
        """Yüzey basıncını kalibre et"""
        self.logger.info("D300 yüzey kalibrasyonu başlatılıyor...")
        
        pressure_readings = []
        temp_readings = []
        
        for i in range(samples):
            pressure, temperature = self.read_raw_data()
            if pressure is not None and temperature is not None:
                pressure_readings.append(pressure)
                temp_readings.append(temperature)
                self.logger.debug(f"Kalibrasyon {i+1}/{samples}: {pressure:.2f} mbar, {temperature:.1f}°C")
            time.sleep(0.1)
        
        if pressure_readings:
            self.pressure_offset = sum(pressure_readings) / len(pressure_readings)
            self.temperature_offset = sum(temp_readings) / len(temp_readings)
            self.logger.info(f"Kalibrasyon tamamlandı - Yüzey basıncı: {self.pressure_offset:.2f} mbar, Sıcaklık: {self.temperature_offset:.1f}°C")
            return True
        else:
            self.pressure_offset = 1013.25  # Standart atmosfer basıncı
            self.temperature_offset = 20.0   # Varsayılan sıcaklık
            self.logger.warning("Kalibrasyon başarısız, standart değerler kullanılıyor")
            return False
    
    def get_depth(self):
        """Derinlik ölçümü (metre)"""
        pressure, _ = self.read_raw_data()
        
        if pressure is None or self.pressure_offset is None:
            return None
            
        # Basınç farkından derinlik hesaplama
        # 1 mbar ≈ 0.01 m su derinliği (yaklaşık)
        # Daha hassas: 1 metre su = 98.0665 mbar
        pressure_diff = pressure - self.pressure_offset
        depth = pressure_diff * 0.0101972  # mbar to metre çevirme faktörü
        
        return max(0, depth)  # Negatif derinlik olmasın
        
    def get_temperature(self):
        """Sıcaklık ölçümü (Celsius)"""
        _, temperature = self.read_raw_data()
        return temperature
        
    def get_all_data(self):
        """Tüm sensör verilerini döndür"""
        pressure, temperature = self.read_raw_data()
        depth = self.get_depth()
        
        return {
            'pressure_mbar': pressure,
            'temperature_c': temperature,
            'depth_m': depth,
            'is_valid': pressure is not None and depth is not None
        }
        
    def is_connected(self):
        """Sensör bağlı mı kontrol et"""
        try:
            if not self.bus:
                return False
            # Basit okuma testi
            self.bus.read_byte(self.address)
            return True
        except:
            return False

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
    """Tüm sensörleri yöneten ana sınıf"""
    
    def __init__(self, mavlink_connection, logger=None):
        self.logger = logger or Logger()
        self.mavlink = mavlink_connection
        
        # Sensörleri başlat
        self.depth = DepthSensor(self.logger)
        self.attitude = AttitudeSensor(mavlink_connection, self.logger)
        self.system = SystemSensor(mavlink_connection, self.logger)
        
        self.logger.info("Sensör yöneticisi başlatıldı")
        
    def calibrate_all(self):
        """Tüm sensörleri kalibre et"""
        self.logger.info("Sensör kalibrasyonu başlatılıyor...")
        
        results = {}
        
        # Derinlik sensörü kalibrasyonu
        results['depth'] = self.depth.calibrate_surface()
        
        # Attitude referans ayarlama
        self.attitude.set_yaw_reference()
        results['attitude'] = self.attitude.yaw_offset is not None
        
        # Sistem durumu kontrolü
        system_status = self.system.get_system_status()
        results['system'] = system_status is not None
        
        success_count = sum(results.values())
        total_count = len(results)
        
        self.logger.info(f"Kalibrasyon tamamlandı: {success_count}/{total_count} sensör başarılı")
        
        return results
        
    def get_all_sensor_data(self):
        """Tüm sensör verilerini al"""
        data = {
            'depth': self.depth.get_all_data(),
            'attitude': self.attitude.get_attitude(timeout=0.1),
            'system': self.system.get_system_status(timeout=0.1),
            'timestamp': time.time()
        }
        
        return data
        
    def check_sensor_health(self):
        """Sensör sağlığını kontrol et"""
        health = {
            'depth_connected': self.depth.is_connected(),
            'attitude_fresh': self.attitude.is_data_fresh(),
            'overall_healthy': True
        }
        
        # Genel sağlık durumu
        health['overall_healthy'] = all([
            health['depth_connected'],
            health['attitude_fresh']
        ])
        
        return health
        
    def log_sensor_status(self):
        """Sensör durumunu logla"""
        health = self.check_sensor_health()
        data = self.get_all_sensor_data()
        
        # Derinlik sensörü
        if data['depth']['is_valid']:
            self.logger.info(f"Derinlik: {data['depth']['depth_m']:.2f}m, "
                           f"Basınç: {data['depth']['pressure_mbar']:.1f}mbar")
        else:
            self.logger.warning("Derinlik sensörü verisi geçersiz")
            
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
                           
        # Genel sağlık
        if not health['overall_healthy']:
            self.logger.warning("Sensör sağlık problemi tespit edildi!")
            
        return health
