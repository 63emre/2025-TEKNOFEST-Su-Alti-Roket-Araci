#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SENSORS HYBRID - D300 okuma sorununu çözen hybrid implementasyon
Verilen kodun çalışan kısmı + mevcut avantajları korur
"""

import time
import math
import statistics
import threading
from collections import deque
from pymavlink import mavutil
from config import *
from utils import Logger

class DepthSensorHybrid:
    """D300 Hybrid Derinlik Sensörü - Verilen kod + mevcut avantajlar"""
    
    def __init__(self, mavlink_connection, logger=None, src=None):
        self.mavlink = mavlink_connection
        self.primary_src = src or D300_SOURCE
        self.logger = logger or Logger()
        
        # Çoklu kaynak sistemi (mevcut avantaj)
        self.available_sources = D300_FALLBACK_SOURCES
        self.current_src = self.primary_src
        self.tried_sources = set()
        
        # Verilen kodun sabitlerini kullan
        self.SEA_LEVEL_PRESSURE_PA = 101325.0
        self.FLUID_DENSITY = D300_SEAWATER_DENSITY  # 1025.0 deniz suyu
        self.G = D300_GRAVITY  # 9.80665
        
        # Kalibrasyon için offset (mevcut avantaj)
        self.pressure_offset = None
        
        # Fallback sistemi (mevcut avantaj)
        self.last_valid_depth = None
        self.last_valid_pressure = None
        self.last_valid_time = None
        self.connection_lost_time = None
        self.is_connected = True
        self.consecutive_failures = 0
        self.max_failures_before_disconnect = D300_MAX_FAILURES_BEFORE_DISCONNECT
        self.max_failures_before_source_switch = D300_MAX_FAILURES_BEFORE_SOURCE_SWITCH
        
        # Sürekli arama sistemi (mevcut avantaj)
        self.continuous_search_enabled = True
        self.search_thread = None
        self.search_lock = threading.Lock()
        self.search_interval = 1.0
        
        # Medyan filtre (mevcut avantaj)
        self.pressure_queue = deque(maxlen=5)
        
        # Verilen kodun mesaj sistemi
        self.msg_name = "SCALED_PRESSURE2" if self.current_src == 2 else "SCALED_PRESSURE3"
        self.msg_id = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2 if self.current_src == 2 else mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3
        
        self._request_data_stream_hybrid()
        self._start_continuous_search()
        self.logger.info(f"D300 Hybrid sensörü başlatıldı: {self.msg_name}")
    
    def _request_data_stream_hybrid(self):
        """Verilen kodun exact veri akışı isteği"""
        try:
            self.logger.info(f"D300 veri akışı isteniyor: {self.msg_name}")
            
            # Verilen kodun exact kopyası
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system, 
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                self.msg_id,  # SCALED_PRESSURE2 veya SCALED_PRESSURE3
                int(1e6/5),   # 5Hz (verilen koddan)
                0, 0, 0, 0, 0
            )
            
            self.logger.info(f"✅ {self.msg_name} veri akışı isteği gönderildi (5Hz)")
            
        except Exception as e:
            self.logger.error(f"D300 veri akışı isteği hatası: {e}")
    
    def read_raw_data_hybrid(self):
        """Verilen kodun exact okuma metodu + fallback"""
        try:
            # Verilen kodun exact kopyası
            msg = self.mavlink.recv_match(type=self.msg_name, blocking=True, timeout=2)
            
            if not msg:
                self.consecutive_failures += 1
                self.logger.warning(f"D300 mesaj boş veya gelmiyor (başarısız: {self.consecutive_failures})")
                self._check_connection_status()
                return None, None
            
            # Verilen kodun exact veri çıkarma
            press_hpa = float(getattr(msg, "press_abs", 0.0))
            temp_c = float(getattr(msg, "temperature", 0)) / 100.0
            
            # Başarılı okuma (mevcut avantaj)
            self.consecutive_failures = 0
            self.last_valid_pressure = press_hpa
            self.last_valid_time = time.time()
            
            if not self.is_connected:
                self.is_connected = True
                self.connection_lost_time = None
                self.logger.info("✅ D300 bağlantı yeniden kuruldu")
            
            return press_hpa, temp_c
            
        except Exception as e:
            self.consecutive_failures += 1
            self.logger.error(f"D300 okuma hatası: {e}")
            self._check_connection_status()
            return None, None
    
    def to_depth_m_hybrid(self, press_hpa: float) -> float:
        """Verilen kodun exact derinlik hesaplama + kalibrasyon"""
        if press_hpa is None:
            return None
            
        # Kalibrasyon varsa kullan (mevcut avantaj)
        if self.pressure_offset is not None:
            # Kalibrasyonlu hesaplama
            pressure_diff_pa = (press_hpa - self.pressure_offset) * 100.0
            depth = pressure_diff_pa / (self.FLUID_DENSITY * self.G)
            return max(0.0, depth)
        else:
            # Verilen kodun exact hesaplama
            pa = press_hpa * 100.0
            dp = max(0.0, pa - self.SEA_LEVEL_PRESSURE_PA)
            return dp / (self.FLUID_DENSITY * self.G)
    
    def get_depth_hybrid(self):
        """Hybrid derinlik okuma - verilen kod + mevcut avantajlar"""
        pressure, temperature = self.read_raw_data_hybrid()
        
        if pressure is not None:
            # Medyan filtre uygula (mevcut avantaj)
            self.pressure_queue.append(pressure)
            if len(self.pressure_queue) >= 3:
                filtered_pressure = statistics.median(self.pressure_queue)
            else:
                filtered_pressure = pressure
            
            # Derinlik hesapla
            depth = self.to_depth_m_hybrid(filtered_pressure)
            self.last_valid_depth = depth
            
            return depth, temperature, "CONNECTED", False
        
        # Fallback sistemi (mevcut avantaj)
        if self.last_valid_depth is not None:
            connection_lost_duration = time.time() - (self.connection_lost_time or time.time())
            self.logger.warning(f"⚠️ D300 fallback: {self.last_valid_depth:.2f}m (kesildi: {connection_lost_duration:.1f}s önce)")
            return self.last_valid_depth, None, "FALLBACK", True
        
        return None, None, "NO_DATA", True
    
    def _check_connection_status(self):
        """Bağlantı durumu kontrolü (mevcut avantaj)"""
        if self.consecutive_failures >= self.max_failures_before_source_switch:
            if not self._try_switch_source():
                if self.consecutive_failures >= self.max_failures_before_disconnect:
                    if self.is_connected:
                        self.is_connected = False
                        self.connection_lost_time = time.time()
                        self.logger.error(f"❌ D300 bağlantı kesildi! ({self.consecutive_failures} başarısız)")
    
    def _try_switch_source(self):
        """Kaynak değiştirme (mevcut avantaj)"""
        for src in self.available_sources:
            if src != self.current_src and src not in self.tried_sources:
                old_src = self.current_src
                self.current_src = src
                self.tried_sources.add(old_src)
                
                # Yeni kaynak konfigürasyonu
                self.msg_name = "SCALED_PRESSURE2" if src == 2 else "SCALED_PRESSURE3"
                self.msg_id = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2 if src == 2 else mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE3
                
                self.logger.warning(f"🔄 D300 kaynak değişimi: {old_src} -> {src}")
                
                # Yeni kaynak için veri akışı iste
                self._request_data_stream_hybrid()
                time.sleep(0.5)
                
                # Test et
                test_pressure, _ = self.read_raw_data_hybrid()
                if test_pressure is not None:
                    self.consecutive_failures = 0
                    self.logger.info(f"✅ D300 kaynak değişimi başarılı: {self.msg_name}")
                    return True
        
        return False
    
    def _start_continuous_search(self):
        """Sürekli arama thread (mevcut avantaj)"""
        if self.search_thread is None or not self.search_thread.is_alive():
            self.continuous_search_enabled = True
            self.search_thread = threading.Thread(target=self._continuous_search_worker, daemon=True)
            self.search_thread.start()
            self.logger.info("🔍 D300 sürekli arama thread başlatıldı")
    
    def _continuous_search_worker(self):
        """Sürekli arama worker (mevcut avantaj)"""
        while self.continuous_search_enabled:
            try:
                with self.search_lock:
                    if not self.is_connected or self.consecutive_failures > 3:
                        self._try_switch_source()
                time.sleep(self.search_interval)
            except Exception as e:
                self.logger.error(f"D300 sürekli arama hatası: {e}")
                time.sleep(2.0)
    
    def calibrate_seawater(self, duration=10):
        """10s deniz suyu kalibrasyonu (mevcut avantaj)"""
        try:
            self.logger.info("🌊 D300 deniz suyu kalibrasyonu başlıyor...")
            self.logger.info("📍 Sensörü su yüzeyinde tutun!")
            
            pressures = []
            start_time = time.time()
            
            while (time.time() - start_time) < duration:
                pressure, _ = self.read_raw_data_hybrid()
                if pressure is not None:
                    pressures.append(pressure)
                    print(f"Kalibrasyon verisi: {pressure:.2f} hPa")
                time.sleep(0.2)
            
            if len(pressures) >= 10:
                self.pressure_offset = statistics.median(pressures)
                self.logger.info(f"✅ D300 kalibrasyon başarılı: P0={self.pressure_offset:.2f} hPa")
                return True
            else:
                self.logger.error("❌ Kalibrasyon için yeterli veri alınamadı")
                return False
                
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            return False
    
    def cleanup(self):
        """Temizlik (mevcut avantaj)"""
        self.continuous_search_enabled = False
        if self.search_thread and self.search_thread.is_alive():
            self.search_thread.join(timeout=2.0)

def test_hybrid():
    """Hybrid sistemin testi"""
    print("🧪 D300 Hybrid Test")
    print("=" * 50)
    
    try:
        # MAVLink bağlantısı
        ports = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2"]
        mavlink = None
        
        for port in ports:
            try:
                print(f"Port deneniyor: {port}")
                mavlink = mavutil.mavlink_connection(port, baud=115200)
                if mavlink.wait_heartbeat(timeout=5):
                    print(f"✅ Bağlantı başarılı: {port}")
                    break
            except:
                continue
        
        if not mavlink:
            print("❌ Hiçbir port çalışmıyor")
            return False
        
        # Hybrid sensör
        sensor = DepthSensorHybrid(mavlink, Logger())
        
        # Kalibrasyon
        print("Kalibrasyon yapılıyor...")
        calib_ok = sensor.calibrate_seawater(duration=5)
        
        # Test
        print("Hybrid okuma testi...")
        for i in range(10):
            depth, temp, status, fallback = sensor.get_depth_hybrid()
            if depth is not None:
                print(f"✅ Derinlik: {depth:.3f}m | Sıcaklık: {temp:.1f}°C | Durum: {status}")
            else:
                print(f"❌ Veri alınamadı | Durum: {status}")
            time.sleep(0.5)
        
        sensor.cleanup()
        return True
        
    except Exception as e:
        print(f"❌ Hybrid test hatası: {e}")
        return False

if __name__ == "__main__":
    test_hybrid()
