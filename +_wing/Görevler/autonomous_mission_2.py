#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - + Wing Konfigürasyon
Görev 2: Roket Ateşleme

Şartname Görev Tanımı:
- Güvenli atış bölgesine otonom ulaşma
- Uygun yunuslama açısıyla yüzeye çıkış 
- +30° eğim koşulu sağlandığında roket taşıma bölmesinin açılması
- Fiziksel roket fırlatma beklenmez, yalnızca ayrılma mekanizması

Puanlama:
- Güvenli atış bölgesine ulaşma: 100 puan
- Su yüzeyine istenen açıyla güvenli çıkış: 100 puan  
- Model roketin güvenli ayrılması: 150 puan
- Sızdırmazlık: 50 puan
- Süre limiti: 5 dakika
- Toplam: 400 puan

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

# Logging'i dosyaya yönlendir
log_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'logs')
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, 'plus_wing_autonomous_mission_2.log')

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

# + Wing konfigürasyonu
import importlib.util
plus_wing_path = os.path.join(project_root, '+_wing', 'hardware_pinmap.py')
spec = importlib.util.spec_from_file_location("plus_wing_pinmap", plus_wing_path)
plus_wing_config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plus_wing_config)

# Görev parametreleri dosyası
MISSION_PARAMS_FILE = os.path.join(os.path.dirname(__file__), "mission_2_params.json")

# Varsayılan görev parametreleri (şartname: Görev 2)
DEFAULT_MISSION_PARAMS = {
    'safe_launch_zone_distance': 30.0,    # Güvenli atış bölgesi mesafesi (m)
    'launch_depth': 1.5,                  # Roket atış derinliği (m)
    'surface_approach_angle': 30.0,       # +30° yunuslama açısı (derece)
    'required_pitch_angle': 30.0,         # Roket ayrılması için gerekli pitch
    'timeout_seconds': 300,               # 5 dakika süre limiti
    'depth_tolerance': 0.3,               # Derinlik toleransı (m)
    'angle_tolerance': 5.0,               # Açı toleransı (derece)
    'launch_motor_speed': 35,             # Atış bölgesine gitme hızı
    'surface_motor_speed': 40,            # Yüzeye çıkış motor hızı
    'control_frequency': 10               # Kontrol döngü frekansı (Hz)
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
        
        # PID terms
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

class PlusWingMission2:
    """+ Wing Konfigürasyon - Görev 2 Roket Ateşleme"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        
        # Görev parametrelerini yükle
        self.mission_params = self.load_mission_params()
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        
        # Sensör durumu
        self.current_distance = 0.0
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        
        # Görev durumu
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.mission_completion_time = None
        self.running = False
        
        # Navigasyon
        self.start_distance = 0.0
        self.launch_zone_reached = False
        self.surface_angle_achieved = False
        self.rocket_deployed = False
        
        # PID kontrolcüler
        self.depth_pid = PIDController(120.0, 8.0, 35.0, 400)
        self.heading_pid = PIDController(4.0, 0.12, 0.7, 200)
        self.pitch_pid = PIDController(5.0, 0.2, 1.0, 300)
        
        # Performans metrikleri
        self.performance_data = {
            'launch_zone_reached': False,
            'safe_surface_exit': False,
            'rocket_separation': False,
            'max_pitch_angle': 0.0,
            'launch_zone_distance': 0.0,
            'mission_duration': 0.0,
            'leak_detected': False
        }
        
        self.logger.info("+ Wing Görev 2 başlatıldı - Roket Ateşleme")
    
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
        """Sistem kurulumu"""
        self.logger.info("+ Wing Görev 2 sistemler ayarlanıyor...")
        
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
            self.servo_controller.initialize_servos()
            
            self.logger.info("+ Wing Görev 2 sistemler hazır")
            return True
            
        except Exception as e:
            self.logger.error(f"Sistem kurulum hatası: {e}")
            return False
    
    def read_sensors(self) -> bool:
        """Sensör verilerini oku"""
        try:
            attitude = self.mav.get_attitude()
            self.current_heading = attitude['yaw']
            self.current_pitch = attitude['pitch']  
            self.current_roll = attitude['roll']
            
            self.current_distance = self.mav.get_distance()
            
            # Mock depth from battery voltage
            battery = self.mav.get_battery_status()
            self.current_depth = max(0, (battery.get('voltage', 22.0) - 20.0) * 2.0)
            
            return True
            
        except Exception as e:
            self.logger.error(f"Sensör okuma hatası: {e}")
            return False
    
    def set_fins_mixed(self, roll_cmd=0, pitch_cmd=0, yaw_cmd=0):
        """+ Wing mixing ile fin kontrolü"""
        mixed_outputs = {}
        
        for fin_name in plus_wing_config.FinControlConfig.FINS.keys():
            output_value = plus_wing_config.PixhawkConfig.SERVO_NEUTRAL
            
            # Mixing matrix uygula
            roll_contrib = plus_wing_config.FinMixingConfig.MIXING_MATRIX["roll"][fin_name] * roll_cmd
            pitch_contrib = plus_wing_config.FinMixingConfig.MIXING_MATRIX["pitch"][fin_name] * pitch_cmd
            yaw_contrib = plus_wing_config.FinMixingConfig.MIXING_MATRIX["yaw"][fin_name] * yaw_cmd
            
            total_contrib = roll_contrib + pitch_contrib + yaw_contrib
            output_value += int(total_contrib)
            
            mixed_outputs[fin_name] = max(1000, min(2000, output_value))
        
        self.servo_controller.set_multiple_servos(mixed_outputs)
    
    def run_mission(self):
        """Ana görev döngüsü - Görev 2"""
        if not self.setup_systems():
            return False
        
        self.logger.info("+ Wing Görev 2 başlatıyor - ROKET ATEŞLEME")
        self.mission_start_time = time.time()
        self.running = True
        
        try:
            # Görev fazları
            self._phase_initialization()
            self._phase_navigate_to_launch_zone()
            self._phase_prepare_for_launch()
            self._phase_surface_approach()
            self._phase_rocket_deployment()
            self._phase_mission_complete()
            
            self.logger.info("+ Wing Görev 2 başarıyla tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev hatası: {e}")
            self._emergency_shutdown()
            return False
        
        finally:
            self._cleanup_systems()
    
    def _phase_initialization(self):
        """Faz 1: Başlangıç ve hazırlık"""
        self.mission_stage = "INITIALIZATION"
        self.logger.info("Faz 1: Roket görevi başlangıç hazırlığı")
        
        # Sensör kalibrasyonu
        for i in range(30):
            self.read_sensors()
            if i == 0:
                self.start_distance = self.current_distance
                self.logger.info(f"Başlangıç mesafesi: {self.start_distance:.2f}m")
            time.sleep(0.1)
        
        # Servo test
        self.servo_controller.all_servos_neutral()
        time.sleep(1)
        
        self.logger.info("Roket görevi başlangıç hazırlığı tamamlandı")
    
    def _phase_navigate_to_launch_zone(self):
        """Faz 2: Güvenli atış bölgesine navigasyon"""
        self.mission_stage = "NAVIGATE_TO_LAUNCH_ZONE"
        self.logger.info(f"Faz 2: Güvenli atış bölgesine navigasyon - {self.mission_params['safe_launch_zone_distance']}m")
        
        # Hedef derinliğe in
        target_depth = self.mission_params['launch_depth']
        launch_zone_distance = self.mission_params['safe_launch_zone_distance']
        
        while self.running:
            self.read_sensors()
            
            # Mesafe kontrolü
            distance_traveled = abs(self.current_distance - self.start_distance)
            self.performance_data['launch_zone_distance'] = distance_traveled
            
            # Atış bölgesine ulaştık mı?
            if distance_traveled >= launch_zone_distance:
                self.launch_zone_reached = True
                self.performance_data['launch_zone_reached'] = True
                self.logger.info(f"Güvenli atış bölgesine ulaşıldı! Mesafe: {distance_traveled:.2f}m")
                break
            
            # Navigasyon kontrolü
            motor_speed = self.mission_params['launch_motor_speed']
            
            # Derinlik kontrolü
            depth_cmd = self.depth_pid.update(target_depth, self.current_depth)
            
            # İleri hareket
            self.mav.set_motor_speed(motor_speed)
            self.set_fins_mixed(pitch_cmd=int(depth_cmd))
            
            self.logger.info(f"Atış bölgesine ilerleme: {distance_traveled:.1f}m / {launch_zone_distance}m")
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        self.logger.info("Atış bölgesi navigasyonu tamamlandı")
    
    def _phase_prepare_for_launch(self):
        """Faz 3: Atış hazırlığı"""
        self.mission_stage = "PREPARE_FOR_LAUNCH"
        self.logger.info("Faz 3: Roket atış hazırlığı")
        
        # Atış pozisyonunda 10 saniye bekle
        prep_time = 10
        prep_start = time.time()
        
        while self.running and (time.time() - prep_start) < prep_time:
            self.read_sensors()
            
            # Pozisyon ve derinlik tut
            depth_cmd = self.depth_pid.update(self.mission_params['launch_depth'], self.current_depth)
            
            # Hovering
            self.mav.set_motor_speed(15)
            self.set_fins_mixed(pitch_cmd=int(depth_cmd))
            
            remaining = prep_time - (time.time() - prep_start)
            self.logger.info(f"Atış hazırlığı: {remaining:.1f}s kaldı")
            
            time.sleep(0.5)
        
        self.logger.info("Roket atış hazırlığı tamamlandı")
    
    def _phase_surface_approach(self):
        """Faz 4: +30° açıyla yüzeye çıkış"""
        self.mission_stage = "SURFACE_APPROACH"
        self.logger.info("Faz 4: +30° yunuslama açısıyla yüzeye çıkış")
        
        target_pitch = self.mission_params['surface_approach_angle']
        surface_timeout = 60
        surface_start = time.time()
        
        while self.running and (time.time() - surface_start) < surface_timeout:
            self.read_sensors()
            
            # Pitch açısı kontrolü
            pitch_error = target_pitch - self.current_pitch
            self.performance_data['max_pitch_angle'] = max(
                self.performance_data['max_pitch_angle'], 
                abs(self.current_pitch)
            )
            
            # +30° açıya ulaştık mı?
            if abs(pitch_error) <= self.mission_params['angle_tolerance']:
                # 3 saniye açıyı koru
                angle_hold_start = time.time()
                angle_stable = True
                
                while time.time() - angle_hold_start < 3 and self.running:
                    self.read_sensors()
                    current_pitch_error = abs(target_pitch - self.current_pitch)
                    
                    if current_pitch_error > self.mission_params['angle_tolerance']:
                        angle_stable = False
                        break
                    
                    # Açı tutma kontrolü
                    pitch_cmd = self.pitch_pid.update(target_pitch, self.current_pitch)
                    depth_cmd = self.depth_pid.update(0.5, self.current_depth)  # Yüzeye çık
                    
                    self.mav.set_motor_speed(self.mission_params['surface_motor_speed'])
                    self.set_fins_mixed(pitch_cmd=int(pitch_cmd))
                    
                    time.sleep(0.1)
                
                if angle_stable:
                    self.surface_angle_achieved = True
                    self.performance_data['safe_surface_exit'] = True
                    self.logger.info(f"+30° açı başarıyla sağlandı! Pitch: {self.current_pitch:.1f}°")
                    break
            
            # Yüzeye çıkış kontrolü
            pitch_cmd = self.pitch_pid.update(target_pitch, self.current_pitch)
            
            # Yüzeye doğru motor
            motor_speed = self.mission_params['surface_motor_speed']
            if self.current_depth > 1.0:
                motor_speed += 10  # Derinse daha hızlı
            
            self.mav.set_motor_speed(motor_speed)
            self.set_fins_mixed(pitch_cmd=int(pitch_cmd))
            
            self.logger.info(f"Yüzey yaklaşımı - Pitch: {self.current_pitch:.1f}° / {target_pitch}°, Derinlik: {self.current_depth:.2f}m")
            
            time.sleep(1.0 / self.mission_params['control_frequency'])
        
        self.logger.info("Yüzey yaklaşımı fazı tamamlandı")
    
    def _phase_rocket_deployment(self):
        """Faz 5: Roket ayrılma mekanizması"""
        self.mission_stage = "ROCKET_DEPLOYMENT"
        self.logger.info("Faz 5: Roket ayrılma mekanizması")
        
        # Roket ayrılması için pitch kontrolü
        required_pitch = self.mission_params['required_pitch_angle']
        
        if abs(self.current_pitch) >= required_pitch:
            self.logger.info(f"Roket ayrılması için gerekli açı sağlandı: {self.current_pitch:.1f}°")
            
            # Roket ayrılma simülasyonu (servo ile kapak açma)
            self.logger.info("ROKET TAŞIMA BÖLMESİ AÇILIYOR...")
            
            # Kapak açma simülasyonu (örneğin üst fin'i max pozisyona)
            rocket_deploy_commands = {
                "upper": plus_wing_config.PixhawkConfig.SERVO_MAX,  # Kapak aç
                "lower": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                "left": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL,
                "right": plus_wing_config.PixhawkConfig.SERVO_NEUTRAL
            }
            
            self.servo_controller.set_multiple_servos(rocket_deploy_commands)
            
            # 3 saniye kapağı açık tut (roket ayrılma simülasyonu)
            deploy_time = 3
            self.logger.info(f"Roket ayrılması simülasyonu - {deploy_time} saniye")
            
            for i in range(deploy_time):
                self.logger.info(f"Roket ayrılması: {i+1}/{deploy_time} saniye")
                time.sleep(1)
            
            # Kapağı kapat
            self.servo_controller.all_servos_neutral()
            
            self.rocket_deployed = True
            self.performance_data['rocket_separation'] = True
            self.logger.info("✅ ROKET AYRILMASI BAŞARIYLA TAMAMLANDI!")
            
        else:
            self.logger.warning(f"Yetersiz pitch açısı: {self.current_pitch:.1f}° < {required_pitch}°")
            self.logger.info("Roket ayrılması güvenlik nedeniyle iptal edildi")
        
        self.logger.info("Roket ayrılma fazı tamamlandı")
    
    def _phase_mission_complete(self):
        """Faz 6: Görev tamamlama"""
        self.mission_stage = "MISSION_COMPLETE"
        self.logger.info("Faz 6: Görev tamamlama")
        
        # Motor durdur
        self.mav.set_motor_speed(0)
        self.servo_controller.all_servos_neutral()
        
        self.mission_completion_time = time.time()
        self.performance_data['mission_duration'] = self.mission_completion_time - self.mission_start_time
        
        self.logger.info("🚀 GÖREV 2 - ROKET ATEŞLEME TAMAMLANDI!")
        
        # Sonuç özeti
        self.logger.info("GÖREV SONUÇ ÖZETİ:")
        self.logger.info(f"  ✅ Atış Bölgesi: {'BAŞARILI' if self.performance_data['launch_zone_reached'] else 'BAŞARISIZ'}")
        self.logger.info(f"  ✅ Yüzey Çıkış: {'BAŞARILI' if self.performance_data['safe_surface_exit'] else 'BAŞARISIZ'}")
        self.logger.info(f"  ✅ Roket Ayrılma: {'BAŞARILI' if self.performance_data['rocket_separation'] else 'BAŞARISIZ'}")
        self.logger.info(f"  📏 Max Pitch: {self.performance_data['max_pitch_angle']:.1f}°")
        self.logger.info(f"  ⏱️ Süre: {self.performance_data['mission_duration']:.1f}s")
    
    def _emergency_shutdown(self):
        """Acil durum kapatması"""
        self.logger.warning("ACİL DURUM - GÖREv 2!")
        
        if self.mav:
            self.mav.emergency_stop()
        
        if self.servo_controller:
            self.servo_controller.emergency_stop()
    
    def _cleanup_systems(self):
        """Sistem temizliği"""
        self.logger.info("+ Wing Görev 2 sistem temizliği...")
        
        self.running = False
        
        if self.mav:
            self.mav.set_motor_speed(0)
            self.mav.disconnect()
        
        self.logger.info("+ Wing Görev 2 sistem temizliği tamamlandı")
    
    def generate_mission_report(self):
        """Görev raporu oluştur (şartname puanlama)"""
        mission_duration = self.performance_data['mission_duration']
        
        report = {
            'timestamp': datetime.now().isoformat(),
            'mission_type': 'Görev 2: Roket Ateşleme',
            'wing_configuration': '+ Wing',
            'mission_duration': mission_duration,
            'mission_params': self.mission_params,
            'performance': self.performance_data
        }
        
        # Puanlama (şartname: Görev 2 - 400 puan)
        launch_zone_points = 100 if self.performance_data['launch_zone_reached'] else 0
        surface_exit_points = 100 if self.performance_data['safe_surface_exit'] else 0
        rocket_separation_points = 150 if self.performance_data['rocket_separation'] else 0
        waterproof_points = 50 if not self.performance_data['leak_detected'] else 0
        
        total_points = launch_zone_points + surface_exit_points + rocket_separation_points + waterproof_points
        
        report['scoring'] = {
            'launch_zone_points': launch_zone_points,
            'surface_exit_points': surface_exit_points,
            'rocket_separation_points': rocket_separation_points,
            'waterproof_points': waterproof_points,
            'total_points': total_points,
            'max_points': 400
        }
        
        # Raporu kaydet
        report_file = os.path.join(
            os.path.dirname(__file__),
            f"plus_wing_mission_2_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        )
        
        try:
            with open(report_file, 'w') as f:
                json.dump(report, f, indent=2)
            self.logger.info(f"Görev raporu kaydedildi: {report_file}")
        except Exception as e:
            self.logger.error(f"Rapor kaydetme hatası: {e}")
        
        return total_points >= 240  # %60 başarı kriteri

def main():
    """Ana fonksiyon"""
    mission = PlusWingMission2()
    
    try:
        success = mission.run_mission()
        
        # Görev raporu
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
