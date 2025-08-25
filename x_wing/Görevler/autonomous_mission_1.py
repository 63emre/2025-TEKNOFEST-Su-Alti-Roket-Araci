#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - X Wing Konfigürasyon
Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş

Şartname Görev Tanımı:
- Başlangıç bölgesinden 2m derinlikte düz istikamette 10m ilerle (süre başlatılır)
- Kıyıdan en az 50m uzaklaş
- Başlangıç noktasına otonom geri dön
- Pozitif sephiye ile yüzeye çıkıp enerjiyi kes

Puanlama:
- Seyir yapma (hız): 150 puan
- Başlangıç noktasında enerji kesme: 90 puan  
- Sızdırmazlık: 60 puan
- Süre limiti: 5 dakika
- Toplam: 300 puan

GPS YOK - Mesafe Pixhawk sensöründen algılanacak
"""

import os
import sys
import time
import json
import logging
import math
from datetime import datetime
from typing import Dict, Any

# Logging'i dosyaya yönlendir (otonom görev için)
log_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'logs')
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, 'x_wing_autonomous_mission_1.log')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
    ]
)

# Görev parametreleri dosyası (JSON ile değiştirilebilir)
MISSION_PARAMS_FILE = os.path.join(os.path.dirname(__file__), "mission_1_params.json")

# Varsayılan görev parametreleri (şartnameye uygun)
DEFAULT_MISSION_PARAMS = {
    'target_depth': 2.0,            # 2m derinlik (şartname)
    'straight_distance': 10.0,      # 10m düz seyir (şartname)
    'min_offshore_distance': 50.0,  # 50m kıyıdan uzaklık (şartname)
    'cruise_speed_percent': 25,     # Seyir hızı motor %
    'return_speed_percent': 30,     # Geri dönüş hızı motor %
    'timeout_seconds': 300,         # 5 dakita süre limiti (şartname)
    'position_tolerance': 2.0,      # Başlangıç noktası toleransı (m)
    'depth_tolerance': 0.2,         # Derinlik toleransı (m)
    'control_frequency': 10         # Kontrol döngüsü frekansı (Hz)
}

class PIDController:
    """Basit PID kontrolcü"""
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        current_time = time.time()
        dt = max(current_time - self.last_time, 0.01)
        
        error = setpoint - measurement
        
        self.integral += error * dt
        integral_limit = self.max_output / self.ki if self.ki > 0 else float('inf')
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        
        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        self.previous_error = error
        self.last_time = current_time
        
        return max(-self.max_output, min(self.max_output, output))
    
    def reset(self):
        self.integral = self.previous_error = 0.0
        self.last_time = time.time()

# Proje modüllerini import et
sys.path.append(os.path.join(os.path.dirname(__file__), '../..'))

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.pid_controller import SubmarineStabilizer
from common.d300_sensor import D300Sensor
from common.gpio_helper import GPIOController
from x_wing.hardware_pinmap import (
    PixhawkConfig, FinControlConfig, PIDConfig, 
    FinMixingConfig, SensorConfig, RaspberryPiConfig
)

class XWingMission1:
    """X Wing Konfigürasyon - Görev 1 Seyir Yapma & Geri Dönüş"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # Görev parametrelerini yükle
        self.mission_params = self.load_mission_params()
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        
        # Navigation durumu (GPS yok, mesafe bazlı)
        self.start_distance_reading = 0.0  # Başlangıç mesafe ölçümü
        self.current_distance = 0.0        # Pixhawk mesafe sensöründen
        self.current_depth = 0.0           # D300 sensöründen (Pixhawk I2C)
        self.current_heading = 0.0         # IMU'dan
        self.current_pitch = 0.0
        self.current_roll = 0.0
        
        # Görev durumu  
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.straight_course_start_time = None
        self.mission_completion_time = None
        self.running = False
        
        # X Wing mesafe bazlı navigasyon
        self.total_distance_traveled = 0.0
        self.max_distance_from_start = 0.0
        self.distance_at_start = 0.0
        self.heading_at_start = 0.0
        
        # PID kontrol (X Wing için)
        self.depth_pid = PIDController(120.0, 8.0, 35.0, 400)
        self.heading_pid = PIDController(4.0, 0.12, 0.7, 200)
        self.distance_pid = PIDController(80.0, 2.0, 15.0, 150)
        
        # Görev metrikleri (şartname puanlama için)
        self.performance_data = {
            'straight_distance_completed': 0.0,
            'max_offshore_distance': 0.0,
            'final_position_error': float('inf'),
            'mission_duration': 0.0,
            'leak_detected': False
        }
        
        self.logger.info("X Wing Görev 1 başlatıldı - Seyir Yapma & Geri Dönüş")
    
    def load_mission_params(self) -> Dict[str, Any]:
        """Görev parametrelerini JSON'dan yükle"""
        if os.path.exists(MISSION_PARAMS_FILE):
            try:
                with open(MISSION_PARAMS_FILE, 'r') as f:
                    loaded_params = json.load(f)
                    params = DEFAULT_MISSION_PARAMS.copy()
                    params.update(loaded_params)
                    self.logger.info(f"Görev parametreleri yüklendi: {MISSION_PARAMS_FILE}")
                    return params
            except Exception as e:
                self.logger.warning(f"Parametre yükleme hatası: {e}")
        
        # Varsayılan parametreleri kaydet
        self.save_mission_params(DEFAULT_MISSION_PARAMS)
        return DEFAULT_MISSION_PARAMS.copy()
    
    def save_mission_params(self, params: Dict[str, Any]):
        """Görev parametrelerini JSON'a kaydet"""
        try:
            with open(MISSION_PARAMS_FILE, 'w') as f:
                json.dump(params, f, indent=2)
            self.logger.info(f"Görev parametreleri kaydedildi: {MISSION_PARAMS_FILE}")
        except Exception as e:
            self.logger.error(f"Parametre kaydetme hatası: {e}")
    
    def setup_systems(self):
        """X Wing sistemleri ayarla"""
        self.logger.info("X Wing sistemler ayarlanıyor...")
        
        try:
            # MAVLink bağlantısı
            self.mav = MAVLinkController(
                PixhawkConfig.MAVLINK_PORT,
                PixhawkConfig.MAVLINK_BAUD
            )
            if not self.mav.connect():
                raise Exception("MAVLink bağlantısı başarısız")
            
            # Servo kontrolcüsü
            self.servo_controller = ServoController(self.mav, FinControlConfig.FINS)
            self.servo_controller.initialize_servos()
            
            self.logger.info("X Wing sistemler başarıyla ayarlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Sistem ayarlama hatası: {e}")
            return False
    
    def _servo_callback(self, servo_outputs):
        """Servo çıkış callback'i"""
        for fin_name, pwm_value in servo_outputs.items():
            if fin_name in FinControlConfig.FINS:
                aux_port = FinControlConfig.FINS[fin_name]["aux_port"]
                self.mav.set_servo_pwm(aux_port, pwm_value)
    
    def _motor_callback(self, motor_speed):
        """Motor çıkış callback'i"""
        self.mav.set_motor_speed(motor_speed)
    
    def run_mission(self):
        """Ana görev döngüsü"""
        if not self.setup_systems():
            return False
        
        self.logger.info("Otonom görev başlatıyor...")
        self.mission_start_time = time.time()
        self.running = True
        
        try:
            # Görev fazları
            self._phase_initialization()
            self._phase_descent()
            self._phase_stabilized_operation()
            self._phase_ascent()
            self._phase_shutdown()
            
            self.logger.info("Otonom görev başarıyla tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev hatası: {e}")
            self._emergency_shutdown()
            return False
        
        finally:
            self._cleanup_systems()
    
    def _phase_initialization(self):
        """Faz 1: Başlangıç ve sistem kontrolü"""
        self.current_phase = "initialization"
        self.logger.info("Faz 1: Başlangıç - sistem kontrolü")
        
        # LED başlangıç sequence
        if self.gpio:
            self.gpio.startup_sequence()
        
        # Derinlik sensörü kalibrasyonu
        self.depth_sensor.calibrate_surface_level()
        self.depth_sensor.start_continuous_reading()
        
        # Servolar nötr konuma
        self.servo_controller.initialize_servos()
        
        # Stabilizasyonu etkinleştir
        self.stabilizer.enable_stabilization(mode=2)  # DEPTH_HOLD modu
        
        # 10 saniye sistem kontrolü
        for i in range(100):  # 10 saniye, 10Hz
            if not self.running:
                return
            
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            
            # Sistem sağlığı kontrolü
            if not self.mav.is_connected():
                raise Exception("MAVLink bağlantısı koptu")
            
            time.sleep(0.1)
        
        self.logger.info("Başlangıç fazı tamamlandı")
    
    def _phase_descent(self):
        """Faz 2: Hedef derinliğe iniş"""
        self.current_phase = "descent"
        self.logger.info(f"Faz 2: Hedef derinliğe iniş - {self.target_depth}m")
        
        # Derinlik PID'ini ayarla
        depth_setpoint = {"depth": self.target_depth}
        self.stabilizer.pid_controller.set_setpoints(depth_setpoint)
        
        # İniş fazı - maksimum 60 saniye
        descent_start = time.time()
        descent_timeout = 60
        
        while self.running and (time.time() - descent_start) < descent_timeout:
            # Sensör verilerini oku
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            
            # Derinlik kontrolü ekle
            sensor_data = {
                **attitude,
                "depth": depth
            }
            
            # Stabilizasyon güncelle
            self.stabilizer.update_stabilization(sensor_data)
            
            # Hedef derinliğe yakın mı?
            depth_error = abs(self.target_depth - depth)
            if depth_error < 0.1:  # 10cm tolerans
                # 5 saniye sabit kaldıysa tamam
                stable_start = time.time()
                stable_count = 0
                
                while time.time() - stable_start < 5:
                    current_depth = self.depth_sensor.get_depth()
                    if abs(self.target_depth - current_depth) < 0.1:
                        stable_count += 1
                    time.sleep(0.1)
                
                if stable_count > 40:  # 5 saniyenin %80'i stabil
                    break
            
            time.sleep(0.02)  # 50Hz döngü
        
        current_depth = self.depth_sensor.get_depth()
        self.logger.info(f"İniş fazı tamamlandı - mevcut derinlik: {current_depth:.2f}m")
    
    def _phase_stabilized_operation(self):
        """Faz 3: Stabil operasyon"""
        self.current_phase = "stable_operation"
        self.logger.info("Faz 3: Stabilize edilmiş operasyon")
        
        # Ana operasyon süresi - 180 saniye (3 dakika)
        operation_start = time.time()
        operation_duration = 180
        
        # Performans metrikleri
        depth_errors = []
        attitude_errors = []
        
        while self.running and (time.time() - operation_start) < operation_duration:
            # Sensör verilerini oku
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            
            # Derinlik ve attitude kontrolü
            sensor_data = {
                **attitude,
                "depth": depth
            }
            
            # Stabilizasyon güncelle
            self.stabilizer.update_stabilization(sensor_data)
            
            # Performans metrikleri kaydet
            depth_error = abs(self.target_depth - depth)
            attitude_error = (attitude["roll"]**2 + attitude["pitch"]**2)**0.5
            
            depth_errors.append(depth_error)
            attitude_errors.append(attitude_error)
            
            # Her 10 saniyede durum logu
            if len(depth_errors) % 500 == 0:  # 10 saniye @ 50Hz
                avg_depth_error = sum(depth_errors[-500:]) / 500
                avg_attitude_error = sum(attitude_errors[-500:]) / 500
                
                self.logger.info(f"Stabil operasyon - Derinlik hatası: {avg_depth_error:.3f}m, "
                               f"Attitude hatası: {avg_attitude_error:.2f}°")
            
            time.sleep(0.02)  # 50Hz döngü
        
        # Final performans raporu
        if depth_errors and attitude_errors:
            avg_depth_error = sum(depth_errors) / len(depth_errors)
            avg_attitude_error = sum(attitude_errors) / len(attitude_errors)
            
            self.logger.info(f"Stabil operasyon tamamlandı - "
                           f"Ortalama derinlik hatası: {avg_depth_error:.3f}m, "
                           f"Ortalama attitude hatası: {avg_attitude_error:.2f}°")
    
    def _phase_ascent(self):
        """Faz 4: Yüzeye çıkış"""
        self.current_phase = "ascent"
        self.logger.info("Faz 4: Yüzeye çıkış")
        
        # Yüzey hedefi
        surface_target = 0.0
        depth_setpoint = {"depth": surface_target}
        self.stabilizer.pid_controller.set_setpoints(depth_setpoint)
        
        # Çıkış fazı - maksimum 60 saniye
        ascent_start = time.time()
        ascent_timeout = 60
        
        while self.running and (time.time() - ascent_start) < ascent_timeout:
            # Sensör verilerini oku
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            
            sensor_data = {
                **attitude,
                "depth": depth
            }
            
            # Stabilizasyon güncelle
            self.stabilizer.update_stabilization(sensor_data)
            
            # Yüzeye yakın mı?
            if depth < 0.2:  # 20cm'den az derinlik
                break
            
            time.sleep(0.02)  # 50Hz döngü
        
        final_depth = self.depth_sensor.get_depth()
        self.logger.info(f"Çıkış fazı tamamlandı - final derinlik: {final_depth:.2f}m")
    
    def _phase_shutdown(self):
        """Faz 5: Sistem kapatma"""
        self.current_phase = "shutdown"
        self.logger.info("Faz 5: Sistem kapatma")
        
        # Stabilizasyonu devre dışı bırak
        self.stabilizer.disable_stabilization()
        
        # Motor durdur
        self.mav.set_motor_speed(0)
        
        # Servolar nötr
        self.servo_controller.emergency_stop()
        
        # Başarı sequence
        if self.gpio:
            self.gpio.success_sequence()
        
        self.logger.info("Sistem kapatma tamamlandı")
    
    def _emergency_shutdown(self):
        """Acil durum kapatması"""
        self.logger.warning("ACİL DURUM KAPATMASI!")
        self.emergency_stop = True
        
        if self.stabilizer:
            self.stabilizer.disable_stabilization()
        
        if self.mav:
            self.mav.emergency_stop()
        
        if self.servo_controller:
            self.servo_controller.emergency_stop()
        
        if self.gpio:
            self.gpio.error_sequence()
    
    def _cleanup_systems(self):
        """Sistem temizliği"""
        self.logger.info("Sistem temizliği yapılıyor...")
        
        self.running = False
        
        if self.depth_sensor:
            self.depth_sensor.stop_continuous_reading()
            self.depth_sensor.disconnect()
        
        if self.stabilizer:
            self.stabilizer.disable_stabilization()
        
        if self.mav:
            self.mav.disconnect()
        
        if self.gpio:
            self.gpio.cleanup_gpio()
        
        self.logger.info("Sistem temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    mission = AutonomousMission1()
    
    try:
        success = mission.run_mission()
        exit_code = 0 if success else 1
    except KeyboardInterrupt:
        mission.logger.info("Görev kullanıcı tarafından durduruldu")
        mission._emergency_shutdown()
        exit_code = 2
    except Exception as e:
        mission.logger.error(f"Beklenmeyen hata: {e}")
        mission._emergency_shutdown()
        exit_code = 3
    
    sys.exit(exit_code)

if __name__ == "__main__":
    main()
