#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SENSORS - Sensör Okuma ve Yönetimi
D300 derinlik sensörü ve Pixhawk MAVLink telemetri fonksiyonları
"""

import time
import math
import statistics
from collections import deque
from pymavlink import mavutil
from config import *
from utils import Logger

class DepthSensor:
    """D300 Derinlik Sensörü Sınıfı
    MAVLink SCALED_PRESSURE mesajları üzerinden basınç ve sıcaklık okuması yapar
    """
    
    # D300 mesaj kaynaklarına göre mesaj tanımları
    MSG_NAME_BY_SRC = {2: 'SCALED_PRESSURE2', 3: 'SCALED_PRESSURE3'}
    MSG_ID_BY_SRC = {2: 137, 3: 142}
    
    def __init__(self, mavlink_connection, logger=None, src=None):
        self.mavlink = mavlink_connection
        self.src = src or D300_SOURCE
        self.msg_name = self.MSG_NAME_BY_SRC[self.src]
        self.msg_id = self.MSG_ID_BY_SRC[self.src]
        
        self.pressure_offset = None  # Yüzey basıncı (P0)
        self.logger = logger or Logger()
        
        # Derinlik hesaplama parametreleri (DENİZ SUYU - GÖREVLER DENİZDE)
        self.water_density = D300_SEAWATER_DENSITY  # kg/m³ (deniz suyu)
        self.gravity = D300_GRAVITY  # m/s²
        
        # Medyan filtre için kuyruk
        self.pressure_queue = deque(maxlen=5)
        
        # Veri geçerlilik kontrolleri
        self.pressure_min = 700.0   # mbar
        self.pressure_max = 1200.0  # mbar
        self.temp_min = -5.0        # °C
        self.temp_max = 60.0        # °C
        
        # FALLBACK SİSTEMİ
        self.last_valid_depth = None
        self.last_valid_pressure = None
        self.last_valid_time = None
        self.connection_lost_time = None
        self.is_connected = True
        self.consecutive_failures = 0
        self.max_failures_before_disconnect = 10  # 10 başarısız okuma sonrası bağlantı kesildi kabul et
        
        self._request_data_stream()
        self.logger.info(f"D300 sensörü MAVLink {self.msg_name} üzerinden başlatıldı")
        
    def _request_data_stream(self):
        """D300 veri akışını iste"""
        try:
            interval_us = int(1_000_000 / D300_DATA_RATE_HZ)
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, self.msg_id, interval_us, 0, 0, 0, 0, 0
            )
        except Exception as e:
            self.logger.warning(f"D300 veri akışı isteği başarısız: {e}")
            
    def read_raw_data(self):
        """Ham D300 sensör verisi oku"""
        try:
            msg = self.mavlink.recv_match(type=self.msg_name, blocking=False, timeout=0.1)
            if msg is None:
                self.consecutive_failures += 1
                self._check_connection_status()
                return None, None
                
            # Basınç ve sıcaklık verilerini al
            pressure_mbar = float(msg.press_abs)  # hPa = mbar
            temperature_c = float(msg.temperature) / 100.0  # Celsius
            
            # Veri geçerlilik kontrolü
            if not (self.pressure_min <= pressure_mbar <= self.pressure_max):
                self.consecutive_failures += 1
                self._check_connection_status()
                return None, None
            if not (self.temp_min <= temperature_c <= self.temp_max):
                self.consecutive_failures += 1
                self._check_connection_status()
                return None, None
                
            # Başarılı okuma
            self.consecutive_failures = 0
            self.last_valid_pressure = pressure_mbar
            self.last_valid_time = time.time()
            if not self.is_connected:
                self.is_connected = True
                self.connection_lost_time = None
                self.logger.info("✅ D300 sensör bağlantısı yeniden kuruldu")
                
            return pressure_mbar, temperature_c
            
        except Exception as e:
            self.consecutive_failures += 1
            self._check_connection_status()
            self.logger.error(f"D300 sensör okuma hatası: {e}")
            return None, None
            
    def _check_connection_status(self):
        """Bağlantı durumunu kontrol et ve güncelle"""
        if self.consecutive_failures >= self.max_failures_before_disconnect:
            if self.is_connected:
                self.is_connected = False
                self.connection_lost_time = time.time()
                self.logger.error(f"❌ D300 sensör bağlantısı kesildi! ({self.consecutive_failures} başarısız okuma)")
                
    def get_depth_safe(self, mission_phase=None):
        """Güvenli derinlik ölçümü - fallback mekanizmalı
        Args:
            mission_phase: Görev fazı ("PHASE_1", "PHASE_2", vb.)
        Returns:
            tuple: (depth_meters, connection_status, fallback_used)
        """
        pressure, _ = self.read_raw_data()
        
        if pressure is not None and self.pressure_offset is not None:
            # Normal derinlik hesaplama
            self.pressure_queue.append(pressure)
            if len(self.pressure_queue) >= 3:
                filtered_pressure = statistics.median(self.pressure_queue)
            else:
                filtered_pressure = pressure
                
            depth = self._calculate_depth(filtered_pressure)
            self.last_valid_depth = depth
            return depth, "CONNECTED", False
            
        # D300 verisi yok - fallback durumu
        if mission_phase == "PHASE_1":
            # İlk 10m içinde D300 kesilirse emergency
            self.logger.critical("🚨 FAZ 1'DE D300 SENSÖRü KESTİ - ACİL DURUM PROSEDÜRÜ!")
            return None, "EMERGENCY_PHASE1", True
            
        # Diğer fazlarda fallback ile devam et
        if self.last_valid_depth is not None:
            # Son geçerli derinliği kullan
            connection_lost_duration = time.time() - (self.connection_lost_time or time.time())
            self.logger.warning(f"⚠️ D300 fallback: Son geçerli derinlik kullanılıyor: {self.last_valid_depth:.2f}m "
                              f"(Bağlantı kesildi: {connection_lost_duration:.1f}s önce)")
            return self.last_valid_depth, "FALLBACK", True
            
        # Hiç veri yok
        self.logger.error("❌ D300 fallback başarısız: Hiç geçerli veri yok!")
        return None, "NO_DATA", True
        
    def _calculate_depth(self, pressure_mbar):
        """Basınçtan derinlik hesaplama"""
        if self.pressure_offset is None:
            return None
            
        # Derinlik hesaplama: h = (P - P0) * 100 / (ρ * g)
        # P, P0: mbar → Pa için *100, ρ: kg/m³, g: m/s²
        pressure_diff_pa = (pressure_mbar - self.pressure_offset) * 100.0
        depth = pressure_diff_pa / (self.water_density * self.gravity)
        
        return max(0.0, depth)  # Negatif derinlik olmasın
    
    def get_depth(self):
        """Derinlik ölçümü (metre) - Standart versiyon"""
        pressure, _ = self.read_raw_data()
        
        if pressure is None or self.pressure_offset is None:
            return None
            
        # Medyan filtre uygula
        self.pressure_queue.append(pressure)
        if len(self.pressure_queue) >= 3:
            filtered_pressure = statistics.median(self.pressure_queue)
        else:
            filtered_pressure = pressure
            
        return self._calculate_depth(filtered_pressure)
        
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
        
    def calibrate_surface(self, duration=10, use_water_surface=True):
        """Yüzey basıncını kalibre et (P0 değerini ayarla)
        
        Args:
            duration: Kalibrasyon süresi (saniye)
            use_water_surface: True=su yüzeyinde, False=havada
            
        Returns:
            bool: Kalibrasyon başarılı mı
        """
        try:
            self.logger.info(f"D300 yüzey kalibrasyonu başlıyor... ({duration}s)")
            
            if use_water_surface:
                self.logger.info("📍 Sensörü su yüzeyinde tutun")
            else:
                self.logger.info("📍 Sensörü havada tutun")
                
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
            self.pressure_offset = statistics.median(pressures)
            
            self.logger.info(f"✅ D300 kalibrasyonu tamamlandı")
            self.logger.info(f"Referans basınç: {self.pressure_offset:.2f} mbar")
            self.logger.info(f"Kalibrasyon verisi: {len(pressures)} örnek")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            return False

    def is_connected(self):
        """D300 sensörü bağlı mı kontrol et"""
        try:
            # Son 2 saniye içinde veri alabildiysek bağlı sayılır
            pressure, temperature = self.read_raw_data()
            return pressure is not None and temperature is not None
        except:
            return False
            
    def set_water_density(self, density):
        """Su yoğunluğunu ayarla (deniz suyu: 1025, tatlı su: 997 kg/m³)"""
        self.water_density = density
        self.logger.info(f"Su yoğunluğu ayarlandı: {density} kg/m³")
        
    def get_water_info(self):
        """Su ortamı bilgilerini döndür"""
        return {
            'density': self.water_density,
            'type': 'deniz suyu' if self.water_density >= 1020 else 'tatlı su',
            'gravity': self.gravity
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
    """Tüm sensörleri yöneten ana sınıf"""
    
    def __init__(self, mavlink_connection, logger=None):
        self.logger = logger or Logger()
        self.mavlink = mavlink_connection
        
        # Sensörleri başlat
        self.depth = DepthSensor(mavlink_connection, self.logger)
        self.attitude = AttitudeSensor(mavlink_connection, self.logger)
        self.system = SystemSensor(mavlink_connection, self.logger)
        
        self.logger.info("Sensör yöneticisi başlatıldı")
        
    def calibrate_all(self, use_water_surface_calib=None):
        """Tüm sensörleri kalibre et
        
        Args:
            use_water_surface_calib: True=su yüzeyinde, False=havada, None=config'den al
        """
        self.logger.info("Sensör kalibrasyonu başlatılıyor...")
        
        results = {}
        
        # D300 derinlik sensörü kalibrasyonu
        water_info = self.depth.get_water_info()
        self.logger.info(f"Kalibrasyon ortamı: {water_info['type']} (ρ={water_info['density']} kg/m³)")
        
        # Su yüzeyinde tutma ayarını belirle
        if use_water_surface_calib is None:
            use_water_surface_calib = D300_USE_WATER_SURFACE_CALIB
            
        calib_method_str = "su yüzeyinde tutarak" if use_water_surface_calib else "havada"
        self.logger.info(f"D300 kalibrasyon metodu: {calib_method_str}")
        
        # Kalibrasyon süresini su türüne göre ayarla
        duration = D300_CALIB_DURATION_SEAWATER if water_info['density'] >= 1020 else D300_CALIB_DURATION_FRESHWATER
        
        results['depth'] = self.depth.calibrate_surface(
            duration=duration,
            use_water_surface=use_water_surface_calib
        )
        
        # Attitude referans ayarlama
        self.attitude.set_yaw_reference()
        results['attitude'] = self.attitude.yaw_offset is not None
        
        # Sistem durumu kontrolü
        system_status = self.system.get_system_status()
        results['system'] = system_status is not None
        
        success_count = sum(results.values())
        total_count = len(results)
        
        self.logger.info(f"Kalibrasyon tamamlandı: {success_count}/{total_count} sensör başarılı")
        
        # Özel durumlar için ek bilgi
        if results['depth'] and not use_water_surface_calib:
            self.logger.info(f"ℹ️ D300 havada kalibre edildi - {water_info['type']} derinlik hesaplamaları için hazır")
        
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
        
    def test_d300_connection(self):
        """D300 derinlik sensörü bağlantısını test et"""
        try:
            self.logger.info("D300 sensör bağlantısı test ediliyor...")
            
            # 3 deneme yap
            for attempt in range(3):
                pressure, temperature = self.depth.read_raw_data()
                
                if pressure is not None and temperature is not None:
                    self.logger.info(f"✅ D300 test başarılı - Basınç: {pressure:.1f}mbar, Sıcaklık: {temperature:.1f}°C")
                    return True
                    
                if attempt < 2:  # Son denemede bekleme
                    time.sleep(0.5)
                    
            self.logger.error("❌ D300 sensör bağlantısı başarısız")
            return False
            
        except Exception as e:
            self.logger.error(f"D300 test hatası: {e}")
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
        
        # D300 derinlik sensörü
        if data['depth']['is_valid']:
            self.logger.info(f"D300 Derinlik: {data['depth']['depth_m']:.3f}m, "
                           f"Basınç: {data['depth']['pressure_mbar']:.1f}mbar, "
                           f"Sıcaklık: {data['depth']['temperature_c']:.1f}°C")
        else:
            self.logger.warning("D300 sensörü verisi geçersiz")
            
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
