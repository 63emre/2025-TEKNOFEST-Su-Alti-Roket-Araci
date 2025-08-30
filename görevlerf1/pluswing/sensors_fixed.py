#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SENSORS FIXED - Verilen koda göre düzeltilmiş D300 okuma
Tüm mevcut avantajlar korundu + verilen kodun çalışan kısmı entegre edildi
"""

import time
import math
import statistics
import threading
from collections import deque
from pymavlink import mavutil
from config import *
from utils import Logger

class DepthSensor:
    """D300 Derinlik Sensörü - Verilen kod + mevcut avantajlar hybrid"""
    
    def __init__(self, mavlink_connection, logger=None, src=None):
        self.mavlink = mavlink_connection
        self.primary_src = src or D300_SOURCE
        self.logger = logger or Logger()
        
        # Çoklu kaynak fallback (MEVCUT AVANTAJ)
        self.available_sources = D300_FALLBACK_SOURCES
        self.current_src = self.primary_src
        self.tried_sources = set()
        
        self._update_source_config()
        
        # Kalibrasyon sistemi (MEVCUT AVANTAJ)
        self.pressure_offset = None
        
        # Derinlik hesaplama - verilen kodun parametreleri
        self.SEA_LEVEL_PRESSURE_PA = 101325.0  # Verilen koddan
        self.water_density = D300_SEAWATER_DENSITY  # 1025.0
        self.gravity = D300_GRAVITY  # 9.80665
        
        # Medyan filtre (MEVCUT AVANTAJ)
        self.pressure_queue = deque(maxlen=5)
        
        # Veri geçerlilik kontrolleri (MEVCUT AVANTAJ)
        self.pressure_min = 700.0
        self.pressure_max = 1200.0
        self.temp_min = -5.0
        self.temp_max = 60.0
        
        # Fallback sistemi (MEVCUT AVANTAJ)
        self.last_valid_depth = None
        self.last_valid_pressure = None
        self.last_valid_time = None
        self.connection_lost_time = None
        self.is_connected = True
        self.consecutive_failures = 0
        self.max_failures_before_disconnect = D300_MAX_FAILURES_BEFORE_DISCONNECT
        self.max_failures_before_source_switch = D300_MAX_FAILURES_BEFORE_SOURCE_SWITCH
        
        # Yeniden bağlantı (MEVCUT AVANTAJ)
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = D300_MAX_RECONNECT_ATTEMPTS
        self.last_reconnect_attempt = 0
        self.reconnect_interval = D300_RECONNECT_INTERVAL
        
        # Sürekli arama thread (MEVCUT AVANTAJ)
        self.continuous_search_enabled = True
        self.search_thread = None
        self.search_lock = threading.Lock()
        self.search_interval = 1.0
        
        # Veri akışını başlat
        self._request_data_stream_fixed()
        self._start_continuous_search()
        self.logger.info(f"D300 sensörü başlatıldı: {self.msg_name} (Hybrid mode)")
    
    def _update_source_config(self):
        """Kaynak konfigürasyonunu güncelle"""
        if self.current_src == 2:
            self.msg_name = "SCALED_PRESSURE2"
            self.msg_id = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2
        else:
            self.msg_name = "SCALED_PRESSURE3" 
            self.msg_id = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3
    
    def _request_data_stream_fixed(self):
        """Verilen kodun exact veri akışı isteği"""
        try:
            # Verilen kodun exact implementasyonu
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system, 
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                self.msg_id,  # SCALED_PRESSURE2/3 ID
                int(1e6/5),   # 5Hz (verilen koddan)
                0, 0, 0, 0, 0
            )
            self.logger.info(f"✅ {self.msg_name} veri akışı istendi (5Hz)")
            
        except Exception as e:
            self.logger.warning(f"D300 veri akışı isteği hatası: {e}")
    
    def read_raw_data(self):
        """Ham veri okuma - verilen kod + fallback"""
        try:
            # Verilen kodun exact okuma metodu
            msg = self.mavlink.recv_match(type=self.msg_name, blocking=True, timeout=2)
            
            if not msg:
                self.consecutive_failures += 1
                self._check_connection_status()
                return None, None
            
            # Verilen kodun exact veri çıkarma
            pressure_hpa = float(getattr(msg, "press_abs", 0.0))
            temp_c = float(getattr(msg, "temperature", 0)) / 100.0
            
            # Veri geçerlilik kontrolü (MEVCUT AVANTAJ)
            if not (self.pressure_min <= pressure_hpa <= self.pressure_max):
                self.consecutive_failures += 1
                self._check_connection_status()
                return None, None
            if not (self.temp_min <= temp_c <= self.temp_max):
                self.consecutive_failures += 1
                self._check_connection_status()
                return None, None
            
            # Başarılı okuma (MEVCUT AVANTAJ)
            self.consecutive_failures = 0
            self.last_valid_pressure = pressure_hpa
            self.last_valid_time = time.time()
            
            if not self.is_connected:
                self.is_connected = True
                self.connection_lost_time = None
                self.logger.info("✅ D300 bağlantı yeniden kuruldu")
            
            return pressure_hpa, temp_c
            
        except Exception as e:
            self.consecutive_failures += 1
            self._check_connection_status()
            self.logger.error(f"D300 okuma hatası: {e}")
            return None, None
    
    def _calculate_depth(self, pressure_hpa):
        """Verilen kodun exact derinlik hesaplama + kalibrasyon seçeneği"""
        if pressure_hpa is None:
            return None
        
        # Kalibrasyon varsa kullan (MEVCUT AVANTAJ)
        if self.pressure_offset is not None:
            # Kalibrasyonlu hesaplama
            pressure_diff_pa = (pressure_hpa - self.pressure_offset) * 100.0
            depth = pressure_diff_pa / (self.water_density * self.gravity)
            return max(0.0, depth)
        else:
            # Verilen kodun exact hesaplama
            pa = pressure_hpa * 100.0
            dp = max(0.0, pa - self.SEA_LEVEL_PRESSURE_PA)
            return dp / (self.water_density * self.gravity)
    
    def get_depth_safe(self, mission_phase=None):
        """Güvenli derinlik okuma - hybrid versiyon"""
        pressure, temperature = self.read_raw_data()
        
        if pressure is not None:
            # Medyan filtre (MEVCUT AVANTAJ)
            self.pressure_queue.append(pressure)
            if len(self.pressure_queue) >= 3:
                filtered_pressure = statistics.median(self.pressure_queue)
            else:
                filtered_pressure = pressure
            
            # Derinlik hesapla
            depth = self._calculate_depth(filtered_pressure)
            self.last_valid_depth = depth
            
            return depth, "CONNECTED", False
        
        # Fallback sistemi (MEVCUT AVANTAJ)
        if mission_phase == "PHASE_1":
            if self.consecutive_failures < 20:
                self.logger.warning(f"⚠️ FAZ 1 D300 geçici sorun (fail: {self.consecutive_failures})")
                return 0.0, "TEMPORARY_ISSUE", True
            else:
                self.logger.critical("🚨 FAZ 1 D300 ACİL DURUM!")
                return None, "EMERGENCY_PHASE1", True
        
        # Diğer fazlarda fallback
        if self.last_valid_depth is not None:
            connection_lost_duration = time.time() - (self.connection_lost_time or time.time())
            self.logger.warning(f"⚠️ D300 fallback: {self.last_valid_depth:.2f}m (kesildi: {connection_lost_duration:.1f}s)")
            return self.last_valid_depth, "FALLBACK", True
        
        return None, "NO_DATA", True
    
    def _check_connection_status(self):
        """Bağlantı durumu kontrolü (MEVCUT AVANTAJ)"""
        if self.consecutive_failures >= self.max_failures_before_source_switch:
            if not self._try_switch_source():
                if self.consecutive_failures >= self.max_failures_before_disconnect:
                    if self.is_connected:
                        self.is_connected = False
                        self.connection_lost_time = time.time()
                        self.logger.error(f"❌ D300 bağlantı kesildi! ({self.consecutive_failures} başarısız)")
    
    def _try_switch_source(self):
        """Kaynak değiştirme (MEVCUT AVANTAJ)"""
        for src in self.available_sources:
            if src != self.current_src and src not in self.tried_sources:
                old_src = self.current_src
                self.current_src = src
                self._update_source_config()
                self.tried_sources.add(old_src)
                
                self.logger.warning(f"🔄 D300 kaynak değişimi: {old_src} -> {src}")
                self._request_data_stream_fixed()
                time.sleep(0.5)
                
                test_pressure, _ = self.read_raw_data()
                if test_pressure is not None:
                    self.consecutive_failures = 0
                    self.logger.info(f"✅ Kaynak değişimi başarılı: {self.msg_name}")
                    return True
        
        return False
    
    def reconnect_attempt(self):
        """Yeniden bağlantı (MEVCUT AVANTAJ)"""
        current_time = time.time()
        if current_time - self.last_reconnect_attempt < self.reconnect_interval:
            return False
        
        self.last_reconnect_attempt = current_time
        self.reconnect_attempts += 1
        
        if self.reconnect_attempts > self.max_reconnect_attempts:
            return False
        
        self.logger.info(f"🔄 D300 yeniden bağlantı denemesi {self.reconnect_attempts}/{self.max_reconnect_attempts}")
        
        self._request_data_stream_fixed()
        time.sleep(0.5)
        
        test_pressure, _ = self.read_raw_data()
        if test_pressure is not None:
            self.logger.info("✅ D300 yeniden bağlantı başarılı!")
            self.consecutive_failures = 0
            self.reconnect_attempts = 0
            if not self.is_connected:
                self.is_connected = True
                self.connection_lost_time = None
            return True
        
        return self._try_switch_source()
    
    def _start_continuous_search(self):
        """Sürekli arama (MEVCUT AVANTAJ)"""
        if self.search_thread is None or not self.search_thread.is_alive():
            self.continuous_search_enabled = True
            self.search_thread = threading.Thread(target=self._continuous_search_worker, daemon=True)
            self.search_thread.start()
            self.logger.info("🔍 D300 sürekli arama başlatıldı")
    
    def _continuous_search_worker(self):
        """Sürekli arama worker (MEVCUT AVANTAJ)"""
        while self.continuous_search_enabled:
            try:
                with self.search_lock:
                    if not self.is_connected or self.consecutive_failures > 3:
                        self._background_search_and_reconnect()
                time.sleep(self.search_interval)
            except Exception as e:
                self.logger.error(f"D300 sürekli arama hatası: {e}")
                time.sleep(2.0)
    
    def _background_search_and_reconnect(self):
        """Arka plan arama (MEVCUT AVANTAJ)"""
        try:
            # Mevcut kaynağı test et
            if self._test_current_source():
                if not self.is_connected:
                    self.logger.info("✅ D300 mevcut kaynak yeniden çalışır")
                    self.is_connected = True
                    self.consecutive_failures = 0
                    self.connection_lost_time = None
                return True
            
            # Alternatif kaynak dene
            return self._try_switch_source()
            
        except Exception as e:
            self.logger.warning(f"D300 arka plan arama hatası: {e}")
        return False
    
    def _test_current_source(self):
        """Mevcut kaynak testi (MEVCUT AVANTAJ)"""
        try:
            test_msg = self.mavlink.recv_match(type=self.msg_name, blocking=False, timeout=0.1)
            if test_msg:
                pressure = float(getattr(test_msg, "press_abs", 0.0))
                return self.pressure_min <= pressure <= self.pressure_max
        except:
            pass
        return False
    
    def get_all_data(self):
        """Tüm veri (MEVCUT AVANTAJ)"""
        pressure, temperature = self.read_raw_data()
        depth = self.get_depth()
        
        return {
            'pressure_mbar': pressure,
            'temperature_c': temperature, 
            'depth_m': depth,
            'is_valid': pressure is not None and depth is not None
        }
    
    def get_depth(self):
        """Standart derinlik okuma"""
        pressure, _ = self.read_raw_data()
        
        if pressure is None:
            return None
        
        # Medyan filtre (MEVCUT AVANTAJ)
        self.pressure_queue.append(pressure)
        if len(self.pressure_queue) >= 3:
            filtered_pressure = statistics.median(self.pressure_queue)
        else:
            filtered_pressure = pressure
        
        return self._calculate_depth(filtered_pressure)
    
    def _calculate_depth(self, pressure_hpa):
        """Verilen kodun exact derinlik hesaplama + kalibrasyon"""
        if pressure_hpa is None:
            return None
        
        # Kalibrasyon varsa kullan (MEVCUT AVANTAJ)
        if self.pressure_offset is not None:
            pressure_diff_pa = (pressure_hpa - self.pressure_offset) * 100.0
            depth = pressure_diff_pa / (self.water_density * self.gravity)
            return max(0.0, depth)
        else:
            # Verilen kodun exact hesaplama
            pa = pressure_hpa * 100.0
            dp = max(0.0, pa - self.SEA_LEVEL_PRESSURE_PA)
            return dp / (self.water_density * self.gravity)
    
    def calibrate_seawater(self, duration=10):
        """Deniz suyu kalibrasyonu (MEVCUT AVANTAJ)"""
        try:
            self.logger.info("🌊 D300 deniz suyu kalibrasyonu...")
            self.logger.info("📍 Sensörü su yüzeyinde tutun!")
            
            pressures = []
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                pressure, _ = self.read_raw_data()
                if pressure is not None:
                    pressures.append(pressure)
                    if len(pressures) % 10 == 0:  # Her 10 örnekte rapor
                        self.logger.info(f"Kalibrasyon: {len(pressures)} örnek, son: {pressure:.2f} hPa")
                time.sleep(0.2)
            
            if len(pressures) >= 10:
                self.pressure_offset = statistics.median(pressures)
                self.logger.info(f"✅ D300 kalibrasyon başarılı: P0={self.pressure_offset:.2f} hPa")
                self.logger.info(f"Kalibrasyon verisi: {len(pressures)} örnek")
                return True
            else:
                self.logger.error("❌ Kalibrasyon için yeterli veri alınamadı")
                return False
                
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            return False
    
    def cleanup(self):
        """Temizlik (MEVCUT AVANTAJ)"""
        self.logger.info("D300 sensör temizleniyor...")
        self._stop_continuous_search()
        self.continuous_search_enabled = False
    
    def _stop_continuous_search(self):
        """Sürekli arama durdur (MEVCUT AVANTAJ)"""
        self.continuous_search_enabled = False
        if self.search_thread and self.search_thread.is_alive():
            self.search_thread.join(timeout=2.0)
            self.logger.info("🛑 D300 sürekli arama durduruldu")

# Diğer sensör sınıfları aynı kalacak...
# (AttitudeSensor, SystemSensor, SensorManager)
