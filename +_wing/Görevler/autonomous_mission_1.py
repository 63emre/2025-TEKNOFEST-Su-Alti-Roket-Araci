#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - + Wing Konfigürasyon
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
I2C D300 Pixhawk üzerinden bağlı
"""

import os
import sys
import time
import json
import logging
from datetime import datetime
from typing import Dict, Any

# Logging'i dosyaya yönlendir (otonom görev için)
log_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'logs')
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, 'plus_wing_autonomous_mission_1.log')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
    ]
)

# Proje modüllerini import et
project_root = os.path.join(os.path.dirname(__file__), '../..')
sys.path.append(project_root)

from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.pid_controller import SubmarineStabilizer

# + Wing konfigürasyonu
import importlib.util
plus_wing_path = os.path.join(project_root, '+_wing', 'hardware_pinmap.py')
spec = importlib.util.spec_from_file_location("plus_wing_pinmap", plus_wing_path)
plus_wing_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plus_wing_config)

# Görev parametreleri dosyası (JSON ile değiştirilebilir)
MISSION_PARAMS_FILE = os.path.join(os.path.dirname(__file__), "mission_1_params.json")

# Varsayılan görev parametreleri (şartnameye uygun)
DEFAULT_MISSION_PARAMS = {
    'target_depth': 2.0,            # 2m derinlik (şartname)
    'straight_distance': 10.0,      # 10m düz seyir (şartname)
    'min_offshore_distance': 50.0,  # 50m kıyıdan uzaklık (şartname)
    'cruise_speed_pwm': 1620,       # Seyir hızı motor PWM
    'return_speed_pwm': 1650,       # Geri dönüş hızı motor PWM  
    'timeout_seconds': 300,         # 5 dakika süre limiti (şartname)
    'position_tolerance': 2.0,      # Başlangıç noktası toleransı (m)
    'depth_tolerance': 0.2,         # Derinlik toleransı (m)
    'distance_measurement_interval': 0.5,  # Mesafe ölçüm aralığı (s)
    'control_frequency': 10         # Kontrol döngüsü frekansı (Hz)
}

# PID Kontrol parametreleri (+ Wing için optimize edilmiş)
CONTROL_PARAMS = {
    'depth_pid': {'kp': 120.0, 'ki': 8.0, 'kd': 35.0, 'max_output': 400},
    'heading_pid': {'kp': 4.5, 'ki': 0.15, 'kd': 0.8, 'max_output': 200},
    'distance_hold_pid': {'kp': 80.0, 'ki': 2.0, 'kd': 15.0, 'max_output': 150}
}

class PIDController:
    """Basit PID kontrolcü"""
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.max_output = max_output
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01
        
        error = setpoint - measurement
        
        # Integral hesapla ve sınırla
        self.integral += error * dt
        integral_limit = self.max_output / self.ki if self.ki > 0 else float('inf')
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        
        # Derivative hesapla
        derivative = (error - self.previous_error) / dt if dt > 0 else 0
        
        # PID çıkışı
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.max_output, min(self.max_output, output))
        
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

class PlusWingMission1:
    """+ Wing Konfigürasyon - Görev 1 Seyir Yapma & Geri Dönüş"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # Görev parametrelerini yükle
        self.mission_params = self.load_mission_params()
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        
        # Navigation durumu
        self.start_distance_reading = 0.0  # Başlangıç mesafe ölçümü
        self.current_distance = 0.0        # Pixhawk mesafe sensöründen
        self.current_depth = 0.0           # D300 sensöründen
        self.current_heading = 0.0         # IMU'dan
        
        # Görev durumu  
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.straight_course_start_time = None
        self.mission_completion_time = None
        self.running = False
        
        # Mesafe bazlı navigasyon (GPS yok)
        self.total_distance_traveled = 0.0
        self.max_distance_from_start = 0.0
        self.distance_at_start = 0.0
        self.heading_at_start = 0.0
        
        # PID kontrol
        self.depth_pid = PIDController(**CONTROL_PARAMS['depth_pid'])
        self.heading_pid = PIDController(**CONTROL_PARAMS['heading_pid'])
        self.distance_pid = PIDController(**CONTROL_PARAMS['distance_hold_pid'])
        
        # Görev metrikleri
        self.performance_data = {
            'straight_distance_completed': 0.0,
            'max_offshore_distance': 0.0,
            'final_position_error': float('inf'),
            'depth_stability': [],
            'mission_duration': 0.0,
            'leak_detected': False
        }
        
        self.logger.info("+ Wing Görev 1 başlatıldı - Seyir Yapma & Geri Dönüş")
    
    def load_mission_params(self) -> Dict[str, Any]:
        """Görev parametrelerini JSON'dan yükle"""
        if os.path.exists(MISSION_PARAMS_FILE):
            try:
                with open(MISSION_PARAMS_FILE, 'r') as f:
                    loaded_params = json.load(f)
                    # Default parametreleri güncelle
                    params = DEFAULT_MISSION_PARAMS.copy()
                    params.update(loaded_params)
                    self.logger.info(f"Görev parametreleri yüklendi: {MISSION_PARAMS_FILE}")
                    return params
            except Exception as e:
                self.logger.warning(f"Parametre yükleme hatası, varsayılan kullanılıyor: {e}")
        
        # Varsayılan parametreleri JSON olarak kaydet
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
        """Tüm sistemleri ayarla"""
        self.logger.info("+ Wing sistemler ayarlanıyor...")
        
        try:
            # MAVLink bağlantısı
            self.mav = MAVLinkController(
                plus_wing_config.PixhawkConfig.MAVLINK_PORT,
                plus_wing_config.PixhawkConfig.MAVLINK_BAUD
            )
            if not self.mav.connect():
                raise Exception("MAVLink bağlantısı başarısız")
            
            # Servo kontrolcüsü
            self.servo_controller = ServoController(self.mav, plus_wing_config.FinControlConfig.FINS)
            
            # Servolar nötr konuma
            self.servo_controller.initialize_servos()
            
            self.logger.info("+ Wing sistemler başarıyla ayarlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Sistem ayarlama hatası: {e}")
            return False
    
    def read_sensors(self) -> bool:
        """Sensör verilerini oku (GPS yok, mesafe sensörü ve IMU kullan)"""
        try:
            # Attitude (IMU) verisi
            attitude = self.mav.get_attitude()
            self.current_heading = attitude['yaw']  # -180 to 180 derece
            
            # Mesafe sensörü (Pixhawk'tan)
            self.current_distance = self.mav.get_distance()
            
            # Derinlik verisi (D300 Pixhawk I2C'den simule et)
            # Gerçekte Pixhawk'tan SCALED_PRESSURE mesajından gelecek
            battery_status = self.mav.get_battery_status()
            # Simülasyon için battery voltage'u derinlik olarak kullanıyoruz
            # Gerçek uygulamada pressure sensor'dan gelecek
            pressure_raw = battery_status.get('voltage', 22.0)
            self.current_depth = max(0, (pressure_raw - 20.0) * 2.0)  # Mock derinlik
            
            return True
            
        except Exception as e:
            self.logger.error(f"Sensör okuma hatası: {e}")
            return False
    
    def set_motor_speed(self, speed_percent: float):
        """Motor hızını ayarla"""
        self.mav.set_motor_speed(speed_percent)
    
    def set_fins_mixed(self, roll_cmd=0, pitch_cmd=0, yaw_cmd=0):
        """+ Wing mixing matrix ile fin kontrolü"""
        # + Wing mixing matrix kullan
        mixed_outputs = {}
        
        for fin_name in plus_wing_config.FinControlConfig.FINS.keys():
            output_value = plus_wing_config.PixhawkConfig.SERVO_NEUTRAL
            
            # Roll kontrolü
            roll_contribution = plus_wing_config.FinMixingConfig.MIXING_MATRIX["roll"][fin_name] * roll_cmd
            
            # Pitch kontrolü  
            pitch_contribution = plus_wing_config.FinMixingConfig.MIXING_MATRIX["pitch"][fin_name] * pitch_cmd
            
            # Yaw kontrolü
            yaw_contribution = plus_wing_config.FinMixingConfig.MIXING_MATRIX["yaw"][fin_name] * yaw_cmd
            
            # Toplam çıkış
            total_contribution = roll_contribution + pitch_contribution + yaw_contribution
            output_value += int(total_contribution)
            
            # PWM limitlerini uygula
            mixed_outputs[fin_name] = max(1000, min(2000, output_value))
        
        # Servo komutlarını gönder
        self.servo_controller.set_multiple_servos(mixed_outputs)
    
    def run_mission(self):
        """Ana görev döngüsü"""
        if not self.setup_systems():
            return False
        
        self.logger.info("+ Wing Görev 1 başlatıyor...")
        self.mission_start_time = time.time()
        self.running = True
        
        try:
            # Görev fazları
            self._phase_initialization()
            self._phase_descent()
            self._phase_straight_course()
            self._phase_offshore_navigation()
            self._phase_return_navigation()
            self._phase_final_approach()
            self._phase_surface_and_shutdown()
            
            self.logger.info("+ Wing Görev 1 başarıyla tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev hatası: {e}")
            self._emergency_shutdown()
            return False
        
        finally:
            self._cleanup_systems()
    
    def _phase_initialization(self):
        """Faz 1: Başlangıç ve referans alma"""
        self.mission_stage = "INITIALIZATION"
        self.logger.info("Faz 1: Sistem başlatma ve referans alma")
        
        # Referans sensör değerlerini al
        for i in range(50):  # 5 saniye sensör stabilizasyonu
            self.read_sensors()
            if i == 0:
                self.start_distance_reading = self.current_distance
                self.distance_at_start = self.current_distance  
                self.heading_at_start = self.current_heading
                self.logger.info(f"Başlangıç referansları - Mesafe: {self.start_distance_reading:.2f}m, Heading: {self.heading_at_start:.1f}°")
            time.sleep(0.1)
        
        # Servolar test
        self.servo_controller.all_servos_neutral()
        time.sleep(1)
        
        self.logger.info("Başlangıç fazı tamamlandı")
    
    def _phase_descent(self):
        """Faz 2: 2m derinliğe iniş"""
        self.mission_stage = "DESCENT"
        self.logger.info(f"Faz 2: Hedef derinliğe iniş - {self.mission_params['target_depth']}m")
        
        descent_timeout = 60  # 60 saniye timeout
        descent_start = time.time()
        
        while self.running and (time.time() - descent_start) < descent_timeout:
            self.read_sensors()
            
            # Derinlik PID kontrolü
            depth_error = self.mission_params['target_depth'] - self.current_depth
            
            if abs(depth_error) <= self.mission_params['depth_tolerance']:
                # Hedef derinliğe ulaşıldı, 5 saniye stabil kalma kontrolü
                stable_time = 0
                stable_start = time.time()
                
                while stable_time < 5 and self.running:
                    self.read_sensors()
                    current_depth_error = abs(self.mission_params['target_depth'] - self.current_depth)
                    
                    if current_depth_error <= self.mission_params['depth_tolerance']:
                        stable_time = time.time() - stable_start
                    else:
                        stable_start = time.time()  # Reset timer
                        stable_time = 0
                    
                    # Derinlik tutma kontrolü
                    depth_cmd = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
                    self.set_fins_mixed(pitch_cmd=int(depth_cmd))
                    self.set_motor_speed(15)  # Düşük motor hızı
                    
                    time.sleep(0.1)
                
                if stable_time >= 5:
                    self.logger.info("Hedef derinlik stabil şekilde ulaşıldı")
                    break
            
            # Derinlik kontrolü
            depth_output = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
            
            if depth_error > 0:  # Daha derine inmeli
                motor_speed = 25
                pitch_cmd = max(-200, int(depth_output))  # Nose down
            else:  # Yükselmeli  
                motor_speed = 10
                pitch_cmd = min(200, int(-depth_output))  # Nose up
            
            self.set_motor_speed(motor_speed)
            self.set_fins_mixed(pitch_cmd=pitch_cmd)
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        self.logger.info(f"İniş fazı tamamlandı - mevcut derinlik: {self.current_depth:.2f}m")
    
    def _phase_straight_course(self):
        """Faz 3: 10m düz seyir (süre başlatma)"""
        self.mission_stage = "STRAIGHT_COURSE"
        self.logger.info("Faz 3: 10m düz seyir - SÜRE BAŞLADI!")
        
        # Süre resmi olarak başlıyor
        self.straight_course_start_time = time.time()
        straight_start_distance = self.current_distance
        
        while self.running:
            self.read_sensors()
            
            # Mesafe farkından ilerleme hesapla
            distance_change = abs(self.current_distance - straight_start_distance)
            self.performance_data['straight_distance_completed'] = distance_change
            
            # 10m düz seyir tamamlandı mı?
            if distance_change >= self.mission_params['straight_distance']:
                self.logger.info(f"10m düz seyir tamamlandı! Mesafe değişimi: {distance_change:.2f}m")
                break
            
            # Düz ileri hareket
            motor_speed_pwm = 1600  # Orta hız
            
            # Derinlik tutma
            depth_cmd = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
            
            # Yön tutma (başlangıç heading'ini koru)
            heading_cmd = self.heading_pid.update(self.heading_at_start, self.current_heading)
            
            # + Wing kontrol
            self.set_motor_speed((motor_speed_pwm - 1000) / 10)  # PWM'i yüzdeye çevir
            self.set_fins_mixed(pitch_cmd=int(depth_cmd), yaw_cmd=int(heading_cmd))
            
            # İlerleme logu
            if distance_change > 0 and int(distance_change) % 2 == 0:
                self.logger.info(f"Düz seyir ilerlemesi: {distance_change:.1f}m / 10m")
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        self.logger.info("Düz seyir fazı tamamlandı")
    
    def _phase_offshore_navigation(self):
        """Faz 4: Kıyıdan 50m uzaklaşma"""
        self.mission_stage = "OFFSHORE_NAVIGATION"
        self.logger.info("Faz 4: Kıyıdan 50m uzaklaşma")
        
        offshore_start_distance = self.current_distance
        
        while self.running:
            self.read_sensors()
            
            # Başlangıçtan toplam uzaklık
            total_distance_from_start = abs(self.current_distance - self.start_distance_reading)
            self.max_distance_from_start = max(self.max_distance_from_start, total_distance_from_start)
            self.performance_data['max_offshore_distance'] = self.max_distance_from_start
            
            # 50m uzaklaştık mı?
            if self.max_distance_from_start >= self.mission_params['min_offshore_distance']:
                self.logger.info(f"50m kıyı uzaklığı başarıldı! Max uzaklık: {self.max_distance_from_start:.2f}m")
                break
            
            # Hızlı ileri hareket
            motor_speed = 35
            
            # Derinlik tutma
            depth_cmd = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
            
            # Yön tutma  
            heading_cmd = self.heading_pid.update(self.heading_at_start, self.current_heading)
            
            self.set_motor_speed(motor_speed)
            self.set_fins_mixed(pitch_cmd=int(depth_cmd), yaw_cmd=int(heading_cmd))
            
            # İlerleme logu
            self.logger.info(f"Kıyıdan uzaklaşma: {self.max_distance_from_start:.1f}m / 50m")
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        self.logger.info("Kıyıdan uzaklaşma fazı tamamlandı")
    
    def _phase_return_navigation(self):
        """Faz 5: Başlangıç noktasına geri dönüş"""
        self.mission_stage = "RETURN_NAVIGATION" 
        self.logger.info("Faz 5: Başlangıç noktasına geri dönüş")
        
        # Geri dönüş için ters yön (180° döndür)
        return_heading = (self.heading_at_start + 180) % 360
        if return_heading > 180:
            return_heading -= 360
        
        while self.running:
            self.read_sensors()
            
            # Başlangıç mesafesine ne kadar yaklaştık
            distance_to_start = abs(self.current_distance - self.start_distance_reading)
            
            # Başlangıca yaklaştık mı (5m tolerance)
            if distance_to_start <= 5.0:
                self.logger.info(f"Başlangıç bölgesine yaklaşıldı! Mesafe farkı: {distance_to_start:.2f}m")
                break
            
            # Geri dönüş kontrolü
            if distance_to_start > 20:  # 20m'den uzaksa hızlı git
                motor_speed = 40
            else:  # Yakınsa yavaşla
                motor_speed = 25
            
            # Derinlik tutma
            depth_cmd = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
            
            # Geri dönüş yönünde git
            heading_cmd = self.heading_pid.update(return_heading, self.current_heading)
            
            self.set_motor_speed(motor_speed)
            self.set_fins_mixed(pitch_cmd=int(depth_cmd), yaw_cmd=int(heading_cmd))
            
            # Geri dönüş logu
            self.logger.info(f"Geri dönüş mesafesi: {distance_to_start:.1f}m")
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        self.logger.info("Geri dönüş navigasyonu tamamlandı")
    
    def _phase_final_approach(self):
        """Faz 6: Final yaklaşım ve pozisyon tutma"""
        self.mission_stage = "FINAL_APPROACH"
        self.logger.info("Faz 6: Final yaklaşım ve hassas pozisyon tutma")
        
        approach_start_time = time.time()
        
        while self.running and (time.time() - approach_start_time) < 30:  # 30 saniye max
            self.read_sensors()
            
            # Başlangıç pozisyonuna olan mesafe
            distance_error = abs(self.current_distance - self.start_distance_reading)
            self.performance_data['final_position_error'] = distance_error
            
            # Pozisyon toleransı içinde mi?
            if distance_error <= self.mission_params['position_tolerance']:
                # 5 saniye pozisyon tut
                hold_start = time.time()
                hold_success = True
                
                while time.time() - hold_start < 5 and self.running:
                    self.read_sensors()
                    current_error = abs(self.current_distance - self.start_distance_reading)
                    
                    if current_error > self.mission_params['position_tolerance']:
                        hold_success = False
                        break
                    
                    # Pozisyon tutma kontrolü
                    distance_cmd = self.distance_pid.update(self.start_distance_reading, self.current_distance)
                    depth_cmd = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
                    
                    # Düşük güç ile pozisyon tut
                    motor_speed = 10 + int(abs(distance_cmd) * 0.2)
                    motor_speed = min(motor_speed, 25)
                    
                    self.set_motor_speed(motor_speed)
                    self.set_fins_mixed(pitch_cmd=int(depth_cmd))
                    
                    time.sleep(0.1)
                
                if hold_success:
                    self.logger.info("5 saniye başarılı pozisyon tutma!")
                    break
            
            # Hassas yaklaşım kontrolü
            if distance_error > self.mission_params['position_tolerance']:
                # Yavaş yaklaşım
                motor_speed = 15
                
                # Başlangıç pozisyonuna yönlen
                if self.current_distance > self.start_distance_reading:
                    # Geri git
                    target_heading = (self.heading_at_start + 180) % 360
                    if target_heading > 180:
                        target_heading -= 360
                else:
                    # İleri git
                    target_heading = self.heading_at_start
                
                heading_cmd = self.heading_pid.update(target_heading, self.current_heading)
                depth_cmd = self.depth_pid.update(self.mission_params['target_depth'], self.current_depth)
                
                self.set_motor_speed(motor_speed)
                self.set_fins_mixed(pitch_cmd=int(depth_cmd), yaw_cmd=int(heading_cmd))
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        final_error = abs(self.current_distance - self.start_distance_reading)
        self.logger.info(f"Final yaklaşım tamamlandı - pozisyon hatası: {final_error:.2f}m")
    
    def _phase_surface_and_shutdown(self):
        """Faz 7: Pozitif sephiye ile yüzeye çıkış ve enerji kesme"""
        self.mission_stage = "SURFACE_AND_SHUTDOWN"
        self.logger.info("Faz 7: Pozitif sephiye ile yüzeye çıkış")
        
        # Motor durdur (pozitif sephiye)
        self.set_motor_speed(0)
        
        # Nose up pozisyonu (pozitif sephiye için)
        self.set_fins_mixed(pitch_cmd=150)  # + Wing üst fin max, alt fin min
        
        # Yüzeye çıkış simülasyonu
        surface_wait_time = 10  # 10 saniye yüzeye çıkış simülasyonu
        surface_start = time.time()
        
        while self.running and (time.time() - surface_start) < surface_wait_time:
            self.read_sensors()
            
            # Simüle edilmiş yüzeye çıkış (derinlik azalıyor)
            simulated_depth = self.current_depth * (1 - (time.time() - surface_start) / surface_wait_time)
            
            self.logger.info(f"Yüzeye çıkış simülasyonu - derinlik: {simulated_depth:.2f}m")
            
            # Pozitif sephiye korunuyor
            self.set_fins_mixed(pitch_cmd=150)
            
            time.sleep(1)
        
        # Enerji kesme simülasyonu
        self.logger.info("Yüzeye çıkış tamamlandı - ENERJİ KESİLİYOR!")
        self.set_motor_speed(0)
        self.servo_controller.all_servos_neutral()
        
        self.mission_completion_time = time.time()
        self.performance_data['mission_duration'] = self.mission_completion_time - self.mission_start_time
        
        self.logger.info("GÖREV 1 BAŞARIYLA TAMAMLANDI!")
    
    def _emergency_shutdown(self):
        """Acil durum kapatması"""
        self.logger.warning("ACİL DURUM KAPATMASI!")
        
        if self.mav:
            self.mav.emergency_stop()
        
        if self.servo_controller:
            self.servo_controller.emergency_stop()
    
    def _cleanup_systems(self):
        """Sistem temizliği"""
        self.logger.info("+ Wing sistem temizliği yapılıyor...")
        
        self.running = False
        
        if self.mav:
            self.mav.set_motor_speed(0)
            self.mav.disconnect()
        
        self.logger.info("+ Wing sistem temizliği tamamlandı")
    
    def generate_mission_report(self):
        """Görev raporu oluştur (şartname puanlama)"""
        mission_duration = self.performance_data['mission_duration']
        
        # Puanlama hesaplama (şartnameden)
        report = {
            'timestamp': datetime.now().isoformat(),
            'mission_type': 'Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş',
            'wing_configuration': '+ Wing',
            'mission_duration': mission_duration,
            'mission_params': self.mission_params,
            'performance': self.performance_data
        }
        
        # Seyir yapma puanı (hız bazlı - 150 puan)
        time_factor = max(0, (300 - mission_duration) / 300) if mission_duration > 0 else 0
        conditions_met = (
            self.performance_data['straight_distance_completed'] >= self.mission_params['straight_distance'] and
            self.performance_data['max_offshore_distance'] >= self.mission_params['min_offshore_distance']
        )
        cruise_points = int(150 * time_factor) if conditions_met else 0
        
        # Başlangıç noktasında enerji kesme (90 puan)
        position_points = 90 if self.performance_data['final_position_error'] <= self.mission_params['position_tolerance'] else 0
        
        # Sızdırmazlık (60 puan)
        waterproof_points = 60 if not self.performance_data['leak_detected'] else 0
        
        total_points = cruise_points + position_points + waterproof_points
        
        report['scoring'] = {
            'cruise_points': cruise_points,
            'position_points': position_points,
            'waterproof_points': waterproof_points,
            'total_points': total_points,
            'max_points': 300
        }
        
        # Raporu kaydet
        report_file = os.path.join(
            os.path.dirname(__file__), 
            f"plus_wing_mission_1_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        try:
            with open(report_file, 'w') as f:
                json.dump(report, f, indent=2)
            self.logger.info(f"Görev raporu kaydedildi: {report_file}")
        except Exception as e:
            self.logger.error(f"Rapor kaydetme hatası: {e}")
        
        return total_points >= 180  # %60 başarı kriteri

def main():
    """Ana fonksiyon"""
    mission = PlusWingMission1()
    
    try:
        success = mission.run_mission()
        
        # Görev raporu oluştur
        report_success = mission.generate_mission_report()
        
        exit_code = 0 if success and report_success else 1
        
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
