#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - X Wing Konfigürasyon
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
import math
import logging

# Logging'i dosyaya yönlendir (buton sistemi için)
log_dir = os.path.join(os.path.dirname(__file__), '..', '..', 'logs')
os.makedirs(log_dir, exist_ok=True)
log_file = os.path.join(log_dir, 'x_wing_autonomous_mission_2.log')

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler(log_file),
        # Autonomous modda console output'u kapatıyoruz
    ]
)

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

class AutonomousMission2:
    """X Wing Otonom Görev 2 - Mesafe Sensörü ve Navigasyon"""
    
    def __init__(self):
        self.logger = logging.getLogger(__name__)
        self.running = False
        
        # Sistem bileşenleri
        self.mav = None
        self.servo_controller = None
        self.stabilizer = None
        self.depth_sensor = None
        self.gpio = None
        
        # Görev parametreleri
        self.mission_duration = 240  # 4 dakika
        self.target_depth = 1.0  # 1 metre hedef derinlik
        self.safe_distance = 2.0  # 2 metre güvenli mesafe
        self.min_distance = 0.5   # 0.5 metre minimum mesafe
        
        # Navigasyon parametreleri
        self.waypoints = [
            {"x": 0, "y": 0, "description": "Başlangıç"},
            {"x": 2, "y": 0, "description": "İleri 2m"},
            {"x": 2, "y": 2, "description": "Sağa 2m"},
            {"x": 0, "y": 2, "description": "Geri 2m"},
            {"x": 0, "y": 0, "description": "Sola 2m - başlangıç"}
        ]
        self.current_waypoint = 0
        
        # Görev durumu
        self.mission_start_time = 0
        self.current_phase = "init"
        self.obstacle_detected = False
        self.position_estimate = {"x": 0, "y": 0}
        
        self.logger.info("X Wing Otonom Görev 2 başlatıldı - Navigasyon ve Mesafe Sensörü")
    
    def setup_systems(self):
        """Tüm sistemleri ayarla"""
        self.logger.info("Sistemler ayarlanıyor...")
        
        try:
            # GPIO ayarla
            self.gpio = GPIOController(
                button_pin=RaspberryPiConfig.BUTTON_PIN,
                led_pin=RaspberryPiConfig.LED_PIN,
                buzzer_pin=RaspberryPiConfig.BUZZER_PIN
            )
            self.gpio.setup_gpio()
            
            # MAVLink bağlantısı
            self.mav = MAVLinkController(
                PixhawkConfig.MAVLINK_PORT,
                PixhawkConfig.MAVLINK_BAUD
            )
            if not self.mav.connect():
                raise Exception("MAVLink bağlantısı başarısız")
            
            # Servo kontrolcüsü
            self.servo_controller = ServoController(self.mav, FinControlConfig.FINS)
            
            # Derinlik sensörü
            self.depth_sensor = D300Sensor(
                bus_number=SensorConfig.D300_BUS,
                address=SensorConfig.D300_I2C_ADDRESS
            )
            if not self.depth_sensor.connect():
                raise Exception("D300 sensörü bağlantısı başarısız")
            
            # Stabilizasyon sistemi
            pid_configs = {
                "roll": PIDConfig.ROLL_PID,
                "pitch": PIDConfig.PITCH_PID,
                "yaw": PIDConfig.YAW_PID,
                "depth": PIDConfig.DEPTH_PID,
                "distance": PIDConfig.DISTANCE_PID
            }
            
            self.stabilizer = SubmarineStabilizer(
                pid_configs,
                FinMixingConfig.MIXING_MATRIX,
                FinMixingConfig.FIN_EFFECTIVENESS
            )
            
            # Callback'leri ayarla
            self.stabilizer.set_callbacks(self._servo_callback, self._motor_callback)
            
            self.logger.info("Tüm sistemler başarıyla ayarlandı")
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
        
        self.logger.info("Navigasyon görevi başlatıyor...")
        self.mission_start_time = time.time()
        self.running = True
        
        try:
            # Görev fazları
            self._phase_initialization()
            self._phase_descent_to_target()
            self._phase_navigation_patrol()
            self._phase_return_home()
            self._phase_ascent()
            self._phase_shutdown()
            
            self.logger.info("Navigasyon görevi başarıyla tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev hatası: {e}")
            self._emergency_shutdown()
            return False
        
        finally:
            self._cleanup_systems()
    
    def _phase_initialization(self):
        """Faz 1: Başlangıç ve kalibrasyon"""
        self.current_phase = "initialization"
        self.logger.info("Faz 1: Navigasyon sistemi başlatma")
        
        # LED başlangıç sequence
        if self.gpio:
            self.gpio.startup_sequence()
        
        # Sensör kalibrasyonları
        self.depth_sensor.calibrate_surface_level()
        self.depth_sensor.start_continuous_reading()
        
        # Servolar nötr konuma
        self.servo_controller.initialize_servos()
        
        # Stabilizasyonu etkinleştir
        self.stabilizer.enable_stabilization(mode=3)  # AUTO_PILOT modu
        
        # Mesafe sensörü testi
        for i in range(50):  # 5 saniye mesafe sensörü test
            distance = self.mav.get_distance()
            if i % 10 == 0:
                self.logger.info(f"Mesafe sensörü: {distance:.2f}m")
            time.sleep(0.1)
        
        self.logger.info("Başlangıç fazı tamamlandı")
    
    def _phase_descent_to_target(self):
        """Faz 2: Hedef derinliğe iniş"""
        self.current_phase = "descent"
        self.logger.info(f"Faz 2: Hedef derinliğe iniş - {self.target_depth}m")
        
        # Derinlik PID'ini ayarla
        setpoints = {
            "roll": 0, "pitch": 0, "yaw": 0, 
            "depth": self.target_depth
        }
        self.stabilizer.pid_controller.set_setpoints(setpoints)
        
        # İniş döngüsü
        descent_timeout = 60
        descent_start = time.time()
        
        while self.running and (time.time() - descent_start) < descent_timeout:
            # Sensör verileri
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            distance = self.mav.get_distance()
            
            # Engel kontrolü
            if distance < self.safe_distance:
                self.logger.warning(f"İniş sırasında engel tespit edildi: {distance:.2f}m")
                # İnişi durdur, yavaşla
                self.mav.set_motor_speed(5)  # Çok yavaş
            
            # Sensör verilerini birleştir
            sensor_data = {
                **attitude,
                "depth": depth,
                "distance": distance
            }
            
            # Stabilizasyon güncelle
            self.stabilizer.update_stabilization(sensor_data)
            
            # Hedefe yakın mı?
            if abs(self.target_depth - depth) < 0.1:
                # 3 saniye stabil kal
                stable_count = 0
                for _ in range(30):  # 3 saniye @ 10Hz
                    current_depth = self.depth_sensor.get_depth()
                    if abs(self.target_depth - current_depth) < 0.1:
                        stable_count += 1
                    time.sleep(0.1)
                
                if stable_count > 20:  # %70 stabil
                    break
            
            time.sleep(0.02)  # 50Hz
        
        self.logger.info(f"İniş fazı tamamlandı - derinlik: {self.depth_sensor.get_depth():.2f}m")
    
    def _phase_navigation_patrol(self):
        """Faz 3: Navigasyon ve devriye"""
        self.current_phase = "navigation"
        self.logger.info("Faz 3: Waypoint navigasyonu başlıyor")
        
        for waypoint_idx, waypoint in enumerate(self.waypoints[1:], 1):  # İlk waypoint başlangıç
            if not self.running:
                break
                
            self.logger.info(f"Waypoint {waypoint_idx}: {waypoint['description']}")
            
            # Bu waypoint'e git
            success = self._navigate_to_waypoint(waypoint)
            
            if not success:
                self.logger.warning(f"Waypoint {waypoint_idx} ulaşılamadı!")
                break
            
            # Waypoint'te 10 saniye bekle
            self._hold_position_with_monitoring(10)
    
    def _navigate_to_waypoint(self, waypoint):
        """Belirli bir waypoint'e navigate et"""
        target_x, target_y = waypoint["x"], waypoint["y"]
        self.logger.info(f"Navigasyon hedefi: ({target_x}, {target_y})")
        
        # Basit navigasyon - önce X ekseni, sonra Y ekseni
        navigation_timeout = 45  # 45 saniye timeout
        nav_start = time.time()
        
        # X ekseni hareketi
        if target_x != self.position_estimate["x"]:
            direction = 1 if target_x > self.position_estimate["x"] else -1
            self.logger.info(f"X ekseni hareketi - yön: {direction}")
            
            if not self._move_in_direction("x", direction, abs(target_x - self.position_estimate["x"])):
                return False
        
        # Y ekseni hareketi  
        if target_y != self.position_estimate["y"]:
            direction = 1 if target_y > self.position_estimate["y"] else -1
            self.logger.info(f"Y ekseni hareketi - yön: {direction}")
            
            if not self._move_in_direction("y", direction, abs(target_y - self.position_estimate["y"])):
                return False
        
        # Pozisyon estimasyonunu güncelle
        self.position_estimate["x"] = target_x
        self.position_estimate["y"] = target_y
        
        return True
    
    def _move_in_direction(self, axis, direction, distance):
        """Belirli bir yönde hareket et"""
        move_start = time.time()
        estimated_time = distance / 0.3  # 0.3 m/s hızla tahmin
        timeout = max(estimated_time * 2, 20)  # En az 20 saniye
        
        # Hareket parametreleri
        if axis == "x":  # İleri/geri
            yaw_setpoint = 0 if direction > 0 else 180
            motor_speed = 20 if direction > 0 else -20
        else:  # Sağ/sol (yaw değiştirme + ileri hareket kombinasyonu)
            yaw_setpoint = 90 if direction > 0 else -90
            motor_speed = 15
        
        # PID setpoints ayarla
        setpoints = {
            "roll": 0, "pitch": 0, "yaw": yaw_setpoint,
            "depth": self.target_depth
        }
        self.stabilizer.pid_controller.set_setpoints(setpoints)
        
        while self.running and (time.time() - move_start) < timeout:
            # Sensör verileri
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            distance = self.mav.get_distance()
            
            # Engel kontrolü
            if distance < self.min_distance:
                self.logger.warning(f"Engel çok yakın: {distance:.2f}m - hareket durduruluyor")
                self.mav.set_motor_speed(0)
                self._handle_obstacle()
                return False
            elif distance < self.safe_distance:
                # Yavaşla
                motor_speed = motor_speed * 0.5
            
            # Motor kontrolü
            self.mav.set_motor_speed(motor_speed)
            
            # Sensör verilerini birleştir
            sensor_data = {
                **attitude,
                "depth": depth,
                "distance": distance
            }
            
            # Stabilizasyon güncelle
            self.stabilizer.update_stabilization(sensor_data)
            
            time.sleep(0.02)  # 50Hz
        
        # Hareketi durdur
        self.mav.set_motor_speed(0)
        time.sleep(1)  # 1 saniye dur
        
        return True
    
    def _handle_obstacle(self):
        """Engel ile karşılaştığında yapılacak işlem"""
        self.logger.info("Engel işleme prosedürü başlatılıyor")
        self.obstacle_detected = True
        
        # Motor durdur
        self.mav.set_motor_speed(0)
        
        # 2 saniye geri git
        self.mav.set_motor_speed(-15)
        time.sleep(2)
        self.mav.set_motor_speed(0)
        
        # Engel durumunu işaretle
        if self.gpio:
            self.gpio.buzzer_beep_pattern([(0.2, 0.2), (0.2, 0.2), (0.2, 0)])
        
        self.logger.info("Engel işleme tamamlandı")
    
    def _hold_position_with_monitoring(self, duration):
        """Pozisyonu tutarak çevre monitörleme yap"""
        self.logger.info(f"Pozisyon tutma ve monitörleme - {duration} saniye")
        
        hold_start = time.time()
        
        while self.running and (time.time() - hold_start) < duration:
            # Sensör verileri
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            distance = self.mav.get_distance()
            
            # Pozisyon tutma
            sensor_data = {
                **attitude,
                "depth": depth,
                "distance": distance
            }
            
            # Stabilizasyon güncelle (hover mode)
            self.stabilizer.update_stabilization(sensor_data)
            
            # Çevresel monitörleme
            if distance < self.safe_distance:
                self.logger.info(f"Monitörleme: Yakın nesne tespit edildi - {distance:.2f}m")
            
            time.sleep(0.02)  # 50Hz
        
        self.logger.info("Pozisyon tutma tamamlandı")
    
    def _phase_return_home(self):
        """Faz 4: Başlangıç noktasına dönüş"""
        self.current_phase = "return_home"
        self.logger.info("Faz 4: Başlangıç noktasına dönüş")
        
        home_waypoint = {"x": 0, "y": 0, "description": "Ana üs"}
        success = self._navigate_to_waypoint(home_waypoint)
        
        if success:
            self.logger.info("Başlangıç noktasına başarıyla dönüldü")
        else:
            self.logger.warning("Başlangıç noktasına dönüş tamamlanamadı")
        
        # 5 saniye pozisyon tutma
        self._hold_position_with_monitoring(5)
    
    def _phase_ascent(self):
        """Faz 5: Yüzeye çıkış"""
        self.current_phase = "ascent"
        self.logger.info("Faz 5: Yüzeye çıkış")
        
        # Yüzey hedefi
        setpoints = {
            "roll": 0, "pitch": 0, "yaw": 0,
            "depth": 0.0
        }
        self.stabilizer.pid_controller.set_setpoints(setpoints)
        
        # Çıkış döngüsü
        ascent_timeout = 60
        ascent_start = time.time()
        
        while self.running and (time.time() - ascent_start) < ascent_timeout:
            # Sensör verileri
            attitude = self.mav.get_attitude()
            depth = self.depth_sensor.get_depth()
            distance = self.mav.get_distance()
            
            sensor_data = {
                **attitude,
                "depth": depth,
                "distance": distance
            }
            
            # Stabilizasyon güncelle
            self.stabilizer.update_stabilization(sensor_data)
            
            # Yüzeye yakın mı?
            if depth < 0.2:
                break
            
            time.sleep(0.02)  # 50Hz
        
        self.logger.info(f"Yüzey çıkışı tamamlandı - final derinlik: {self.depth_sensor.get_depth():.2f}m")
    
    def _phase_shutdown(self):
        """Faz 6: Sistem kapatma"""
        self.current_phase = "shutdown"
        self.logger.info("Faz 6: Sistem kapatma")
        
        # Stabilizasyonu devre dışı bırak
        self.stabilizer.disable_stabilization()
        
        # Motor durdur
        self.mav.set_motor_speed(0)
        
        # Servolar nötr
        self.servo_controller.emergency_stop()
        
        # Başarı veya hata sequence
        if self.gpio:
            if self.obstacle_detected:
                self.gpio.buzzer_beep_pattern([(0.3, 0.2), (0.3, 0.2), (1.0, 0)])  # Engel uyarısı
            else:
                self.gpio.success_sequence()
        
        self.logger.info("Navigasyon görevi tamamlandı")
    
    def _emergency_shutdown(self):
        """Acil durum kapatması"""
        self.logger.warning("ACİL DURUM KAPATMASI!")
        
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
    mission = AutonomousMission2()
    
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
