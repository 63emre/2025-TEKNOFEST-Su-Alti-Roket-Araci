#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Vibration Monitor
Real-time Titreşim Analizi Sistemi
"""

import time
import math
import threading
from collections import deque
import numpy as np

class VibrationMonitor:
    def __init__(self, mavlink_handler, buffer_size=100):
        """Titreşim monitörü"""
        self.mavlink = mavlink_handler
        self.buffer_size = buffer_size
        
        # IMU data buffers (2 saniyelik data @50Hz)
        self.accel_x_buffer = deque(maxlen=buffer_size)
        self.accel_y_buffer = deque(maxlen=buffer_size) 
        self.accel_z_buffer = deque(maxlen=buffer_size)
        self.gyro_x_buffer = deque(maxlen=buffer_size)
        self.gyro_y_buffer = deque(maxlen=buffer_size)
        self.gyro_z_buffer = deque(maxlen=buffer_size)
        
        # Vibration metrics
        self.vibration_level = 0.0  # 0-100 scale
        self.vibration_color = "green"
        self.vibration_category = "low"
        
        # Frequency analysis
        self.dominant_frequency = 0.0
        self.frequency_bands = {
            "low": 0.0,      # 0-5 Hz
            "medium": 0.0,   # 5-15 Hz  
            "high": 0.0      # 15-25 Hz
        }
        
        # Monitoring thread
        self.monitoring = False
        self.monitor_thread = None
        self.data_lock = threading.Lock()
        
        # Callbacks
        self.callbacks = []
        
    def add_callback(self, callback_func):
        """Vibration güncellemesi için callback ekle"""
        self.callbacks.append(callback_func)
    
    def start_monitoring(self):
        """Titreşim monitörünü başlat"""
        if self.monitoring:
            return
            
        self.monitoring = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        print("📊 Vibration monitoring başlatıldı")
    
    def stop_monitoring(self):
        """Titreşim monitörünü durdur"""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)
        print("📊 Vibration monitoring durduruldu")
    
    def _monitor_loop(self):
        """Ana monitoring döngüsü"""
        update_rate = 20  # Hz
        dt = 1.0 / update_rate
        
        while self.monitoring:
            start_time = time.time()
            
            # IMU verilerini al
            imu_data = self.mavlink.get_imu_data()
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                
                with self.data_lock:
                    # Buffer'lara ekle
                    self.accel_x_buffer.append(accel_x)
                    self.accel_y_buffer.append(accel_y)
                    self.accel_z_buffer.append(accel_z)
                    self.gyro_x_buffer.append(gyro_x)
                    self.gyro_y_buffer.append(gyro_y)
                    self.gyro_z_buffer.append(gyro_z)
                    
                    # Titreşim analizi yap
                    self._analyze_vibration()
                    
                    # Callback'leri çağır
                    for callback in self.callbacks:
                        try:
                            callback(self.get_vibration_data())
                        except:
                            pass
            
            # Timing
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def _analyze_vibration(self):
        """Titreşim analizi yap"""
        if len(self.accel_x_buffer) < 20:
            return
        
        # Standard deviation bazlı titreşim seviyesi
        accel_std_x = np.std(list(self.accel_x_buffer))
        accel_std_y = np.std(list(self.accel_y_buffer))
        accel_std_z = np.std(list(self.accel_z_buffer))
        
        gyro_std_x = np.std(list(self.gyro_x_buffer))
        gyro_std_y = np.std(list(self.gyro_y_buffer))
        gyro_std_z = np.std(list(self.gyro_z_buffer))
        
        # Combined vibration magnitude
        accel_vibration = math.sqrt(accel_std_x**2 + accel_std_y**2 + accel_std_z**2)
        gyro_vibration = math.sqrt(gyro_std_x**2 + gyro_std_y**2 + gyro_std_z**2)
        
        # Weight accelerometer more (structural vibration)
        total_vibration = accel_vibration * 0.8 + gyro_vibration * 0.2
        
        # 0-100 scale'e normalize et
        self.vibration_level = min(100, total_vibration * 100)
        
        # Vibration kategorisi belirle
        if self.vibration_level < 20:
            self.vibration_category = "low"
            self.vibration_color = "green"
        elif self.vibration_level < 50:
            self.vibration_category = "medium"
            self.vibration_color = "yellow"
        else:
            self.vibration_category = "high"
            self.vibration_color = "red"
        
        # Frequency analysis (FFT)
        self._frequency_analysis()
    
    def _frequency_analysis(self):
        """Frekans domain analizi"""
        if len(self.accel_x_buffer) < 50:
            return
        
        try:
            # Accelerometer X eksenini analiz et (dominant axis usually)
            signal = np.array(list(self.accel_x_buffer))
            
            # Remove DC component
            signal = signal - np.mean(signal)
            
            # FFT
            fft = np.fft.fft(signal)
            freqs = np.fft.fftfreq(len(signal), d=0.05)  # 20Hz sampling
            
            # Power spectrum
            power = np.abs(fft)**2
            
            # Find dominant frequency
            positive_freqs = freqs[:len(freqs)//2]
            positive_power = power[:len(power)//2]
            
            if len(positive_power) > 0:
                dominant_idx = np.argmax(positive_power)
                self.dominant_frequency = abs(positive_freqs[dominant_idx])
            
            # Frequency bands analysis
            low_band = positive_power[(positive_freqs >= 0) & (positive_freqs < 5)]
            medium_band = positive_power[(positive_freqs >= 5) & (positive_freqs < 15)]
            high_band = positive_power[(positive_freqs >= 15) & (positive_freqs < 25)]
            
            total_power = np.sum(positive_power)
            if total_power > 0:
                self.frequency_bands["low"] = np.sum(low_band) / total_power * 100
                self.frequency_bands["medium"] = np.sum(medium_band) / total_power * 100
                self.frequency_bands["high"] = np.sum(high_band) / total_power * 100
                
        except Exception as e:
            # FFT hatası durumunda sessizce devam et
            pass
    
    def get_vibration_data(self):
        """Mevcut titreşim verilerini döndür"""
        with self.data_lock:
            return {
                "level": self.vibration_level,
                "category": self.vibration_category,
                "color": self.vibration_color,
                "dominant_frequency": self.dominant_frequency,
                "frequency_bands": self.frequency_bands.copy(),
                "buffer_size": len(self.accel_x_buffer)
            }
    
    def get_vibration_level(self):
        """Basit vibration level döndür (0-100)"""
        return self.vibration_level
    
    def get_vibration_color(self):
        """Vibration rengi döndür"""
        return self.vibration_color
    
    def get_vibration_category(self):
        """Vibration kategorisi döndür"""
        return self.vibration_category
    
    def is_high_vibration(self, threshold=50):
        """Yüksek titreşim var mı?"""
        return self.vibration_level > threshold
    
    def get_diagnostic_info(self):
        """Detaylı diagnostic bilgisi"""
        with self.data_lock:
            current_data = self.get_vibration_data()
            
            # Buffer statistics
            if len(self.accel_x_buffer) > 0:
                accel_stats = {
                    "x": {
                        "mean": np.mean(list(self.accel_x_buffer)),
                        "std": np.std(list(self.accel_x_buffer)),
                        "min": np.min(list(self.accel_x_buffer)),
                        "max": np.max(list(self.accel_x_buffer))
                    },
                    "y": {
                        "mean": np.mean(list(self.accel_y_buffer)),
                        "std": np.std(list(self.accel_y_buffer)),
                        "min": np.min(list(self.accel_y_buffer)),
                        "max": np.max(list(self.accel_y_buffer))
                    },
                    "z": {
                        "mean": np.mean(list(self.accel_z_buffer)),
                        "std": np.std(list(self.accel_z_buffer)),
                        "min": np.min(list(self.accel_z_buffer)),
                        "max": np.max(list(self.accel_z_buffer))
                    }
                }
                
                gyro_stats = {
                    "x": {
                        "mean": np.mean(list(self.gyro_x_buffer)),
                        "std": np.std(list(self.gyro_x_buffer)),
                        "min": np.min(list(self.gyro_x_buffer)),
                        "max": np.max(list(self.gyro_x_buffer))
                    },
                    "y": {
                        "mean": np.mean(list(self.gyro_y_buffer)),
                        "std": np.std(list(self.gyro_y_buffer)),
                        "min": np.min(list(self.gyro_y_buffer)),
                        "max": np.max(list(self.gyro_y_buffer))
                    },
                    "z": {
                        "mean": np.mean(list(self.gyro_z_buffer)),
                        "std": np.std(list(self.gyro_z_buffer)),
                        "min": np.min(list(self.gyro_z_buffer)),
                        "max": np.max(list(self.gyro_z_buffer))
                    }
                }
            else:
                accel_stats = gyro_stats = {}
            
            return {
                "vibration_data": current_data,
                "accelerometer_stats": accel_stats,
                "gyroscope_stats": gyro_stats,
                "sample_count": len(self.accel_x_buffer),
                "monitoring_active": self.monitoring
            }
    
    def reset_buffers(self):
        """Buffer'ları temizle"""
        with self.data_lock:
            self.accel_x_buffer.clear()
            self.accel_y_buffer.clear()
            self.accel_z_buffer.clear()
            self.gyro_x_buffer.clear()
            self.gyro_y_buffer.clear()
            self.gyro_z_buffer.clear()
            
            self.vibration_level = 0.0
            self.vibration_color = "green"
            self.vibration_category = "low"
        
        print("📊 Vibration buffers temizlendi")
    
    def set_buffer_size(self, size):
        """Buffer size değiştir"""
        if 10 <= size <= 500:
            with self.data_lock:
                self.buffer_size = size
                
                # Yeni buffer'lar oluştur
                old_data_x = list(self.accel_x_buffer)[-size:]
                old_data_y = list(self.accel_y_buffer)[-size:]
                old_data_z = list(self.accel_z_buffer)[-size:]
                
                self.accel_x_buffer = deque(old_data_x, maxlen=size)
                self.accel_y_buffer = deque(old_data_y, maxlen=size)
                self.accel_z_buffer = deque(old_data_z, maxlen=size)
                
                # Gyro için de aynı
                old_gyro_x = list(self.gyro_x_buffer)[-size:]
                old_gyro_y = list(self.gyro_y_buffer)[-size:]
                old_gyro_z = list(self.gyro_z_buffer)[-size:]
                
                self.gyro_x_buffer = deque(old_gyro_x, maxlen=size)
                self.gyro_y_buffer = deque(old_gyro_y, maxlen=size)
                self.gyro_z_buffer = deque(old_gyro_z, maxlen=size)
            
            print(f"📊 Buffer size: {size}")

class VibrationComparator:
    """PID vs Raw kontrol titreşim karşılaştırması"""
    
    def __init__(self):
        self.raw_control_data = []
        self.pid_control_data = []
        self.current_mode = None
        self.recording = False
        
    def start_recording(self, control_mode):
        """Belirli mod için kayıt başlat"""
        self.current_mode = control_mode
        self.recording = True
        print(f"📊 {control_mode} modu için vibration kaydı başlatıldı")
    
    def stop_recording(self):
        """Kaydı durdur"""
        self.recording = False
        print("📊 Vibration kaydı durduruldu")
    
    def add_vibration_sample(self, vibration_data):
        """Vibration sample ekle"""
        if not self.recording or not self.current_mode:
            return
        
        timestamp = time.time()
        sample = {
            "timestamp": timestamp,
            "level": vibration_data["level"],
            "category": vibration_data["category"],
            "dominant_frequency": vibration_data["dominant_frequency"],
            "frequency_bands": vibration_data["frequency_bands"]
        }
        
        if self.current_mode == "raw":
            self.raw_control_data.append(sample)
        elif self.current_mode == "pid":
            self.pid_control_data.append(sample)
    
    def get_comparison_report(self):
        """Karşılaştırma raporu oluştur"""
        if not self.raw_control_data or not self.pid_control_data:
            return None
        
        # Raw control statistics
        raw_levels = [sample["level"] for sample in self.raw_control_data]
        raw_stats = {
            "mean_level": np.mean(raw_levels),
            "std_level": np.std(raw_levels),
            "max_level": np.max(raw_levels),
            "sample_count": len(raw_levels)
        }
        
        # PID control statistics
        pid_levels = [sample["level"] for sample in self.pid_control_data]
        pid_stats = {
            "mean_level": np.mean(pid_levels),
            "std_level": np.std(pid_levels),
            "max_level": np.max(pid_levels),
            "sample_count": len(pid_levels)
        }
        
        # Comparison
        improvement = raw_stats["mean_level"] - pid_stats["mean_level"]
        improvement_percent = (improvement / raw_stats["mean_level"]) * 100 if raw_stats["mean_level"] > 0 else 0
        
        better_mode = "RAW" if raw_stats["mean_level"] < pid_stats["mean_level"] else "PID"
        
        return {
            "raw_control": raw_stats,
            "pid_control": pid_stats,
            "improvement": improvement,
            "improvement_percent": improvement_percent,
            "better_mode": better_mode,
            "recommendation": self._get_recommendation(raw_stats, pid_stats)
        }
    
    def _get_recommendation(self, raw_stats, pid_stats):
        """Öneri oluştur"""
        raw_mean = raw_stats["mean_level"]
        pid_mean = pid_stats["mean_level"]
        
        if abs(raw_mean - pid_mean) < 5:
            return "Her iki mod da benzer titreşim seviyesi gösteriyor. Tercihe göre seçilebilir."
        elif raw_mean < pid_mean:
            return f"RAW control {pid_mean - raw_mean:.1f} puan daha az titreşimli. RAW modu önerilir."
        else:
            return f"PID control {raw_mean - pid_mean:.1f} puan daha az titreşimli. PID modu önerilir."
    
    def clear_data(self):
        """Tüm verileri temizle"""
        self.raw_control_data.clear()
        self.pid_control_data.clear()
        print("📊 Comparison data temizlendi")

if __name__ == "__main__":
    # Test
    from mavlink_handler import MAVLinkHandler
    
    mavlink = MAVLinkHandler()
    if mavlink.connect():
        vibration_monitor = VibrationMonitor(mavlink)
        vibration_monitor.start_monitoring()
        
        print("Vibration Monitor test başarılı!")
        time.sleep(5)
        
        data = vibration_monitor.get_vibration_data()
        print(f"Vibration Level: {data['level']:.1f}")
        
        vibration_monitor.stop_monitoring()
        mavlink.disconnect() 