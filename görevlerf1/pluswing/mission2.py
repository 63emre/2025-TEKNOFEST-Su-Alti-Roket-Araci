#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MISSION 2 - Roket Fırlatma Görevi
Su altında hedefe yaklaşma, roket hazırlığı ve fırlatma sistemi
Not: GPIO10'daki selenoid sadece bu görevde kullanılır
"""

import time
import math
from gpio_wrapper import GPIO
from config import *
from utils import Timer, estimate_distance, format_time
from sensors import SensorManager
from control import StabilizationController, MotionController
from gpio_compat import GPIO

class RocketController:
    """Roket fırlatma kontrol sistemi"""
    
    def __init__(self, logger):
        self.logger = logger
        self.solenoid_active = False
        
        # GPIO ayarla
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_SOLENOID, GPIO.OUT)
        GPIO.output(GPIO_SOLENOID, GPIO.LOW)
        
        self.logger.info("Roket kontrolcüsü başlatıldı")
        
    def prepare_rocket(self):
        """Roket hazırlık prosedürü"""
        self.logger.info("Roket hazırlık prosedürü başlatılıyor...")
        
        # Sistem kontrolleri burada yapılabilir
        # Örneğin: basınç kontrol, kapak kontrol, vs.
        
        time.sleep(2)  # Hazırlık süresi
        self.logger.info("✓ Roket hazırlık tamamlandı")
        return True
        
    def launch_rocket(self):
        """Roket fırlatma"""
        self.logger.info("🚀 ROKET FIRLATILIYOR!")
        
        try:
            # Selenoid valfi aktif et (CO2 tüpünü del)
            GPIO.output(GPIO_SOLENOID, GPIO.HIGH)
            self.solenoid_active = True
            
            # Selenoidi 2 saniye açık tut
            time.sleep(2.0)
            
            # Selenoidi kapat
            GPIO.output(GPIO_SOLENOID, GPIO.LOW)
            self.solenoid_active = False
            
            self.logger.info("✓ Roket fırlatma tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Roket fırlatma hatası: {e}")
            self._emergency_shutdown()
            return False
            
    def _emergency_shutdown(self):
        """Roket sistemi acil kapama"""
        try:
            GPIO.output(GPIO_SOLENOID, GPIO.LOW)
            self.solenoid_active = False
            self.logger.info("Roket sistemi acil kapatma tamamlandı")
        except Exception as e:
            self.logger.error(f"Acil kapatma hatası: {e}")
            
    def get_status(self):
        """Roket sistem durumu"""
        return {
            'solenoid_active': self.solenoid_active,
            'ready_for_launch': not self.solenoid_active
        }
        
    def cleanup(self):
        """Roket sistemi temizliği"""
        self._emergency_shutdown()

class Mission2Controller:
    """Görev 2 Ana Kontrol Sınıfı"""
    
    def __init__(self, mavlink_connection, system_status, logger):
        self.mavlink = mavlink_connection
        self.system_status = system_status
        self.logger = logger
        
        # Ana bileşenler
        self.sensors = SensorManager(mavlink_connection, logger)
        self.stabilizer = StabilizationController(mavlink_connection, self.sensors, logger)
        self.motion = MotionController(self.stabilizer, logger)
        self.rocket = RocketController(logger)
        
        # Görev durumu
        self.mission_timer = Timer()
        self.phase_timer = Timer()
        self.current_phase = MissionPhase.WAITING
        self.mission_completed = False
        self.mission_success = False
        
        # Hedef bilgileri
        self.target_distance = 30.0  # Hedefe mesafe (metre)
        self.target_depth = 3.0      # Roket fırlatma derinliği
        self.approach_speed = SPEED_MEDIUM
        
        # Mesafe takibi
        self.total_distance_traveled = 0.0
        self.current_speed_pwm = MOTOR_STOP
        
        self.logger.info("Görev 2 kontrolcüsü başlatıldı")
        
    def initialize_mission(self):
        """Görev başlangıç hazırlıkları"""
        self.logger.info("Görev 2 başlangıç hazırlıkları...")
        
        try:
            # NOT: Sensör kalibrasyonu main.py'de zaten yapıldı, tekrar yapma!
            self.logger.info("Sensörler ana kontrolcüde kalibre edildi")
                
            # Roket sistemi hazırlığı
            if not self.rocket.prepare_rocket():
                self.logger.error("Roket sistemi hazırlık başarısız!")
                return False
                
            # Stabilizasyonu başlat
            self.stabilizer.enable_stabilization()
            
            # Servoları nötrle
            self.stabilizer.servo_controller.neutral_all_servos()
            
            # Görev zamanlayıcısını başlat
            self.mission_timer.start()
            
            self.logger.info("Görev 2 hazırlıkları tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev başlangıç hatası: {e}")
            return False
            
    def start_mission(self):
        """Ana görev döngüsü"""
        self.logger.info("🚀 GÖREV 2 BAŞLIYOR!")
        self.system_status.set_phase(MissionPhase.PHASE_1)
        
        try:
            # Faz 1: Hedefe yaklaşma
            if not self._execute_approach():
                return False
                
            # Faz 2: Roket hazırlık ve pozisyonlama
            if not self._execute_rocket_preparation():
                return False
                
            # Faz 3: Roket fırlatma
            if not self._execute_rocket_launch():
                return False
                
            # Faz 4: Geri çekilme ve yüzeye çıkış
            if not self._execute_withdrawal():
                return False
                
            # Görev başarılı
            self.mission_completed = True
            self.mission_success = True
            self.system_status.set_phase(MissionPhase.COMPLETED)
            
            self.logger.info("🎉 GÖREV 2 BAŞARIYLA TAMAMLANDI!")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev hatası: {e}")
            self._emergency_abort()
            return False
            
    def _execute_approach(self):
        """Faz 1: Hedefe yaklaşma"""
        self.logger.info("📍 FAZ 1: Hedefe yaklaşma")
        self.current_phase = MissionPhase.PHASE_1
        self.phase_timer.start()
        
        # Hedef değerleri ayarla
        target_depth = self.target_depth
        target_distance = self.target_distance
        speed_pwm = self.approach_speed
        
        self.stabilizer.set_target_depth(target_depth)
        self.current_speed_pwm = speed_pwm
        
        # Motoru başlat
        self.motion.forward(speed_pwm)
        
        approach_distance = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Yaklaşma kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Stabilizasyon güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                # Güvenli mesafe hesaplama - None kontrolü
                if speed_pwm is not None and phase_time is not None:
                    estimated_distance = estimate_distance(speed_pwm, phase_time)
                    approach_distance = estimated_distance
                    self.total_distance_traveled = estimated_distance
                else:
                    estimated_distance = 0.0
                    approach_distance = 0.0
                    self.logger.warning("Yaklaşma mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu (D300 sensöründen)
                sensor_data = self.sensors.get_all_sensor_data()
                current_depth = sensor_data['depth']['depth_m'] if sensor_data['depth']['is_valid'] else None
                depth_str = f"{current_depth:.1f}m" if current_depth else "N/A"
                
                remaining_distance = target_distance - approach_distance
                self.logger.info(f"Yaklaşma - Mesafe: {approach_distance:.1f}m, "
                               f"Kalan: {remaining_distance:.1f}m, "
                               f"Derinlik: {depth_str}/{target_depth}m")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if approach_distance >= target_distance:
                self.logger.info(f"✓ Hedefe ulaşıldı: {approach_distance:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 120:  # 2 dakika maksimum
                self.logger.warning("Yaklaşma zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_rocket_preparation(self):
        """Faz 2: Roket hazırlık ve pozisyonlama"""
        self.logger.info("📍 FAZ 2: Roket hazırlık ve pozisyonlama")
        self.current_phase = MissionPhase.ROCKET_PREP
        self.system_status.set_phase(MissionPhase.ROCKET_PREP)
        
        # Motoru durdur
        self.motion.stop()
        
        # Pozisyonlama için stabilizasyonu aktif tut
        # Hedef derinliği koru
        self.stabilizer.set_target_depth(self.target_depth)
        
        # 10 saniye pozisyonlama
        positioning_time = 10.0
        start_time = time.time()
        
        self.logger.info("Roket fırlatma pozisyonuna geçiliyor...")
        
        while time.time() - start_time < positioning_time:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Roket hazırlık kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Pozisyonlama stabilizasyonu güncellenemedi")
                
            # Durum raporu
            remaining_time = positioning_time - (time.time() - start_time)
            if int(remaining_time) % 2 == 0:  # Her 2 saniyede
                sensor_data = self.sensors.get_all_sensor_data()
                current_depth = sensor_data['depth']['depth_m'] if sensor_data['depth']['is_valid'] else None
                depth_str = f"{current_depth:.1f}m" if current_depth else "N/A"
                
                self.logger.info(f"Pozisyonlama - Kalan: {remaining_time:.1f}s, "
                               f"Derinlik: {depth_str}")
                
            time.sleep(0.1)
            
        self.logger.info("✓ Roket pozisyonlama tamamlandı")
        return True
        
    def _execute_rocket_launch(self):
        """Faz 3: Roket fırlatma"""
        self.logger.info("📍 FAZ 3: Roket fırlatma")
        self.current_phase = MissionPhase.ROCKET_LAUNCH
        self.system_status.set_phase(MissionPhase.ROCKET_LAUNCH)
        
        # Son sistem kontrolleri
        rocket_status = self.rocket.get_status()
        if not rocket_status['ready_for_launch']:
            self.logger.error("Roket sistemi fırlatma için hazır değil!")
            return False
            
        # 5 saniye geri sayım
        self.logger.info("Roket fırlatma geri sayımı başlıyor...")
        for countdown in range(5, 0, -1):
            self.logger.info(f"Fırlatma {countdown} saniye...")
            
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Roket fırlatma iptal edildi!")
                return False
                
            # Stabilizasyonu sürdür
            self.stabilizer.update_stabilization()
            
            time.sleep(1.0)
            
        # ROKET FIRLATMA!
        success = self.rocket.launch_rocket()
        
        if success:
            self.logger.info("✓ Roket başarıyla fırlatıldı!")
            
            # Fırlatma sonrası bekleme (5 saniye)
            time.sleep(5.0)
            
            return True
        else:
            self.logger.error("✗ Roket fırlatma başarısız!")
            return False
            
    def _execute_withdrawal(self):
        """Faz 4: Geri çekilme ve yüzeye çıkış"""
        self.logger.info("📍 FAZ 4: Geri çekilme ve yüzeye çıkış")
        self.current_phase = MissionPhase.RETURN
        self.system_status.set_phase(MissionPhase.RETURN)
        self.phase_timer.start()
        
        # Geri çekilme mesafesi (güvenli mesafe)
        withdrawal_distance = 20.0
        speed_pwm = SPEED_MEDIUM
        
        # Hedef derinliği koru
        self.stabilizer.set_target_depth(self.target_depth)
        
        # Geri hareket (180° döndük sayarsak ileri gitmeye devam)
        self.motion.forward(speed_pwm)
        
        withdrawal_traveled = 0.0
        last_distance_check = time.time()
        
        self.logger.info("Güvenli geri çekilme başlatıldı...")
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Geri çekilme kullanıcı tarafından durduruldu")
                break
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Geri çekilme stabilizasyonu güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                # Güvenli mesafe hesaplama - None kontrolü
                if speed_pwm is not None and phase_time is not None:
                    estimated_distance = estimate_distance(speed_pwm, phase_time)
                    withdrawal_traveled = estimated_distance
                else:
                    estimated_distance = 0.0
                    withdrawal_traveled = 0.0
                    self.logger.warning("Geri çekilme mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu
                remaining_distance = withdrawal_distance - withdrawal_traveled
                self.logger.info(f"Geri çekilme - Mesafe: {withdrawal_traveled:.1f}m, "
                               f"Kalan: {remaining_distance:.1f}m")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if withdrawal_traveled >= withdrawal_distance:
                self.logger.info(f"✓ Güvenli geri çekilme tamamlandı: {withdrawal_traveled:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 60:  # 1 dakika maksimum
                self.logger.warning("Geri çekilme zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        # Yüzeye çıkış
        self.logger.info("Yüzeye çıkış başlatılıyor...")
        self.current_phase = MissionPhase.SURFACING
        self.system_status.set_phase(MissionPhase.SURFACING)
        
        # Stabilizasyonu deaktif et
        self.stabilizer.disable_stabilization()
        
        # Yüzeye çıkış kontrolü
        success = self.stabilizer.surface_control(duration=15)
        
        if success:
            self.logger.info("✓ Yüzeye çıkış tamamlandı")
        else:
            self.logger.warning("Yüzeye çıkış problemi")
            
        return True
        
    def _emergency_abort(self):
        """Acil durum iptal prosedürü"""
        self.logger.error("🚨 GÖREV 2 ACİL İPTAL!")
        
        try:
            # Roket sistemini güvenli hale getir
            self.rocket._emergency_shutdown()
            
            # Stabilizasyonu durdur
            self.stabilizer.emergency_stop()
            
            # Sistem durumunu acil duruma geçir
            self.system_status.emergency_stop()
            
            self.mission_completed = True
            self.mission_success = False
            
        except Exception as e:
            self.logger.error(f"Acil iptal prosedürü hatası: {e}")
            
    def get_mission_status(self):
        """Görev durumu raporu"""
        status = {
            'phase': self.current_phase,
            'mission_time': self.mission_timer.elapsed() if self.mission_timer.is_running() else 0,
            'phase_time': self.phase_timer.elapsed() if self.phase_timer.is_running() else 0,
            'total_distance': self.total_distance_traveled,
            'target_distance': self.target_distance,
            'completed': self.mission_completed,
            'success': self.mission_success,
            'current_speed': self.current_speed_pwm,
            'rocket_status': self.rocket.get_status()
        }
        
        # Sensör durumu ekle
        sensor_data = self.sensors.get_all_sensor_data()
        if sensor_data['depth']['is_valid']:
            status['current_depth'] = sensor_data['depth']['depth_m']
        if sensor_data['attitude']:
            status['current_heading'] = sensor_data['attitude']['yaw_relative_deg']
            
        return status
        
    def log_mission_status(self):
        """Görev durumunu logla"""
        status = self.get_mission_status()
        
        mission_time_str = format_time(status['mission_time'])
        
        self.logger.info(f"Görev 2 Durumu - Faz: {status['phase']}, "
                        f"Süre: {mission_time_str}, "
                        f"Mesafe: {status['total_distance']:.1f}m")
                        
        if 'current_depth' in status:
            self.logger.info(f"Derinlik: {status['current_depth']:.2f}m")
            
        if 'current_heading' in status:
            self.logger.info(f"Heading: {status['current_heading']:.1f}°")
            
        # Roket durumu
        rocket_status = status['rocket_status']
        self.logger.info(f"Roket - Hazır: {rocket_status['ready_for_launch']}, "
                        f"Selenoid: {rocket_status['solenoid_active']}")
            
    def cleanup(self):
        """Görev temizliği"""
        self.logger.info("Görev 2 temizleniyor...")
        
        try:
            # Roket sistemini temizle
            self.rocket.cleanup()
            
            # Stabilizasyonu durdur
            self.stabilizer.disable_stabilization()
            
            # Motoru durdur
            self.motion.stop()
            
            # Zamanlayıcıları durdur
            if self.mission_timer.is_running():
                self.mission_timer.pause()
            if self.phase_timer.is_running():
                self.phase_timer.pause()
                
            self.logger.info("Görev 2 temizliği tamamlandı")
            
        except Exception as e:
            self.logger.error(f"Görev 2 temizlik hatası: {e}")

def run_mission_2(mavlink_connection, system_status, logger):
    """Görev 2'yi çalıştır (dış arayüz fonksiyonu)"""
    mission = Mission2Controller(mavlink_connection, system_status, logger)
    
    try:
        # Görevi başlat
        if not mission.initialize_mission():
            logger.error("Görev 2 başlatma başarısız!")
            return False
            
        # Ana görev döngüsü
        success = mission.start_mission()
        
        # Sonuç raporu
        status = mission.get_mission_status()
        mission_time = format_time(status['mission_time'])
        
        if success:
            logger.info(f"✅ GÖREV 2 BAŞARILI! Süre: {mission_time}, Mesafe: {status['total_distance']:.1f}m")
        else:
            logger.error(f"❌ GÖREV 2 BAŞARISIZ! Süre: {mission_time}")
            
        return success
        
    except Exception as e:
        logger.error(f"Görev 2 çalıştırma hatası: {e}")
        return False
        
    finally:
        mission.cleanup()
