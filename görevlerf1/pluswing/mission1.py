#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MISSION 1 - Seyir ve Başlangıç Noktasına Dönüş Görevi
İlk 10m: 2m derinlik, sonrası: 3m derinlik, toplam 50m mesafe
180° dönüş ve geri dönüş, son olarak yüzeye çıkış
"""

import time
import math
from config import *
from utils import Timer, estimate_distance, format_time
from sensors import SensorManager
from control import StabilizationController, MotionController

class Mission1Controller:
    """Görev 1 Ana Kontrol Sınıfı"""
    
    def __init__(self, mavlink_connection, system_status, logger):
        self.mavlink = mavlink_connection
        self.system_status = system_status
        self.logger = logger
        
        # Ana bileşenler
        self.sensors = SensorManager(mavlink_connection, logger)
        self.stabilizer = StabilizationController(mavlink_connection, self.sensors, logger)
        self.motion = MotionController(self.stabilizer, logger)
        
        # Görev durumu
        self.mission_timer = Timer()
        self.phase_timer = Timer()
        self.current_phase = MissionPhase.WAITING
        self.mission_completed = False
        self.mission_success = False
        
        # Mesafe takibi
        self.total_distance_traveled = 0.0
        self.phase_distance = 0.0
        self.current_speed_pwm = MOTOR_STOP
        
        self.logger.info("Görev 1 kontrolcüsü başlatıldı")
        
    def initialize_mission(self):
        """Görev başlangıç hazırlıkları"""
        self.logger.info("Görev 1 başlangıç hazırlıkları...")
        
        try:
            # Sensör kalibrasyonu
            calibration_results = self.sensors.calibrate_all()
            
            if not all(calibration_results.values()):
                self.logger.warning("Bazı sensörler kalibre edilemedi!")
                
            # Stabilizasyonu başlat
            self.stabilizer.enable_stabilization()
            
            # Servoları nötrle
            self.stabilizer.servo_controller.neutral_all_servos()
            
            # Görev zamanlayıcısını başlat
            self.mission_timer.start()
            
            self.logger.info("Görev 1 hazırlıkları tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev başlangıç hatası: {e}")
            return False
            
    def start_mission(self):
        """Ana görev döngüsü"""
        self.logger.info("🚀 GÖREV 1 BAŞLIYOR!")
        self.system_status.set_phase(MissionPhase.PHASE_1)
        
        try:
            # Faz 1: İlk 10 metre (2m derinlik)
            if not self._execute_phase_1():
                return False
                
            # Faz 2: Ana seyir (3m derinlik)  
            if not self._execute_phase_2():
                return False
                
            # Faz 3: 180° dönüş
            if not self._execute_turning():
                return False
                
            # Faz 4: Geri dönüş
            if not self._execute_return():
                return False
                
            # Faz 5: Yüzeye çıkış
            if not self._execute_surfacing():
                return False
                
            # Görev başarılı
            self.mission_completed = True
            self.mission_success = True
            self.system_status.set_phase(MissionPhase.COMPLETED)
            
            self.logger.info("🎉 GÖREV 1 BAŞARIYLA TAMAMLANDI!")
            return True
            
        except Exception as e:
            self.logger.error(f"Görev hatası: {e}")
            self._emergency_abort()
            return False
            
    def _execute_phase_1(self):
        """Faz 1: İlk 10 metre - 2m derinlik"""
        self.logger.info("📍 FAZ 1: İlk 10 metre (2m derinlik)")
        self.current_phase = MissionPhase.PHASE_1
        self.phase_timer.start()
        
        # Hedef değerleri ayarla
        target_depth = TARGET_DEPTH_FIRST_10M
        target_distance = FIRST_PHASE_DISTANCE
        speed_pwm = get_speed_for_phase(MissionPhase.PHASE_1)
        
        self.stabilizer.set_target_depth(target_depth)
        self.current_speed_pwm = speed_pwm
        
        # Motoru başlat
        self.motion.forward(speed_pwm)
        
        self.phase_distance = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Faz 1 kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Stabilizasyon güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                estimated_distance = estimate_distance(speed_pwm, phase_time)
                self.phase_distance = estimated_distance
                self.total_distance_traveled = estimated_distance
                
                # Durum raporu
                sensor_data = self.sensors.get_all_sensor_data()
                current_depth = sensor_data['depth']['depth_m'] if sensor_data['depth']['is_valid'] else None
                depth_str = f"{current_depth:.1f}m" if current_depth else "N/A"
                
                self.logger.info(f"Faz 1 - Mesafe: {estimated_distance:.1f}m/{target_distance}m, "
                               f"Derinlik: {depth_str}/{target_depth}m")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if self.phase_distance >= target_distance:
                self.logger.info(f"✓ Faz 1 tamamlandı: {self.phase_distance:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 120:  # 2 dakika maksimum
                self.logger.warning("Faz 1 zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_phase_2(self):
        """Faz 2: Ana seyir - 3m derinlik"""
        self.logger.info("📍 FAZ 2: Ana seyir (3m derinlik)")
        self.current_phase = MissionPhase.PHASE_2
        self.phase_timer.start()
        
        # Hedef değerleri ayarla
        target_depth = TARGET_DEPTH_MAIN
        remaining_distance = MISSION_DISTANCE - FIRST_PHASE_DISTANCE
        speed_pwm = get_speed_for_phase(MissionPhase.PHASE_2)
        
        self.stabilizer.set_target_depth(target_depth)
        self.current_speed_pwm = speed_pwm
        
        # Hızı artır
        self.motion.forward(speed_pwm)
        
        phase_2_distance = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Faz 2 kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Stabilizasyon güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                estimated_distance = estimate_distance(speed_pwm, phase_time)
                phase_2_distance = estimated_distance
                self.total_distance_traveled = FIRST_PHASE_DISTANCE + estimated_distance
                
                # Durum raporu
                sensor_data = self.sensors.get_all_sensor_data()
                current_depth = sensor_data['depth']['depth_m'] if sensor_data['depth']['is_valid'] else None
                depth_str = f"{current_depth:.1f}m" if current_depth else "N/A"
                
                self.logger.info(f"Faz 2 - Toplam: {self.total_distance_traveled:.1f}m/{MISSION_DISTANCE}m, "
                               f"Derinlik: {depth_str}/{target_depth}m")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if phase_2_distance >= remaining_distance:
                self.logger.info(f"✓ Faz 2 tamamlandı: Toplam {self.total_distance_traveled:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 180:  # 3 dakika maksimum
                self.logger.warning("Faz 2 zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_turning(self):
        """Faz 3: 180° dönüş"""
        self.logger.info("📍 FAZ 3: 180° dönüş")
        self.current_phase = MissionPhase.TURNING
        self.system_status.set_phase(MissionPhase.TURNING)
        
        # Motoru yavaşlat/durdur
        self.motion.stop()
        time.sleep(2)  # Duraklaması için bekle
        
        # 180° dönüş yap
        success = self.stabilizer.turn_180_degrees(timeout=45)
        
        if success:
            self.logger.info("✓ 180° dönüş tamamlandı")
            return True
        else:
            self.logger.error("✗ 180° dönüş başarısız!")
            return False
            
    def _execute_return(self):
        """Faz 4: Geri dönüş"""
        self.logger.info("📍 FAZ 4: Geri dönüş")
        self.current_phase = MissionPhase.RETURN
        self.system_status.set_phase(MissionPhase.RETURN)
        self.phase_timer.start()
        
        # Hedef değerleri ayarla
        target_depth = TARGET_DEPTH_MAIN
        return_distance = MISSION_DISTANCE  # Aynı mesafeyi geri git
        speed_pwm = get_speed_for_phase(MissionPhase.RETURN)
        
        self.stabilizer.set_target_depth(target_depth)
        self.current_speed_pwm = speed_pwm
        
        # İleri hareket başlat (180° döndük, yön ters)
        self.motion.forward(speed_pwm)
        
        return_distance_traveled = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Geri dönüş kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Stabilizasyon güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                estimated_distance = estimate_distance(speed_pwm, phase_time)
                return_distance_traveled = estimated_distance
                
                # Durum raporu
                sensor_data = self.sensors.get_all_sensor_data()
                current_depth = sensor_data['depth']['depth_m'] if sensor_data['depth']['is_valid'] else None
                depth_str = f"{current_depth:.1f}m" if current_depth else "N/A"
                
                self.logger.info(f"Geri dönüş - Mesafe: {return_distance_traveled:.1f}m/{return_distance}m, "
                               f"Derinlik: {depth_str}/{target_depth}m")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if return_distance_traveled >= return_distance:
                self.logger.info(f"✓ Geri dönüş tamamlandı: {return_distance_traveled:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 180:  # 3 dakika maksimum
                self.logger.warning("Geri dönüş zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_surfacing(self):
        """Faz 5: Yüzeye çıkış"""
        self.logger.info("📍 FAZ 5: Yüzeye çıkış")
        self.current_phase = MissionPhase.SURFACING
        self.system_status.set_phase(MissionPhase.SURFACING)
        
        # Stabilizasyonu deaktif et
        self.stabilizer.disable_stabilization()
        
        # Yüzeye çıkış kontrolü
        success = self.stabilizer.surface_control(duration=15)
        
        if success:
            self.logger.info("✓ Yüzeye çıkış tamamlandı")
            return True
        else:
            self.logger.warning("Yüzeye çıkış problemi")
            return True  # Kritik hata değil
            
    def _emergency_abort(self):
        """Acil durum iptal prosedürü"""
        self.logger.error("🚨 GÖREV 1 ACİL İPTAL!")
        
        try:
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
            'phase_distance': self.phase_distance,
            'completed': self.mission_completed,
            'success': self.mission_success,
            'current_speed': self.current_speed_pwm
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
        phase_time_str = format_time(status['phase_time'])
        
        self.logger.info(f"Görev Durumu - Faz: {status['phase']}, "
                        f"Süre: {mission_time_str}, "
                        f"Mesafe: {status['total_distance']:.1f}m")
                        
        if 'current_depth' in status:
            self.logger.info(f"Derinlik: {status['current_depth']:.2f}m")
            
        if 'current_heading' in status:
            self.logger.info(f"Heading: {status['current_heading']:.1f}°")
            
    def cleanup(self):
        """Görev temizliği"""
        self.logger.info("Görev 1 temizleniyor...")
        
        try:
            # Stabilizasyonu durdur
            self.stabilizer.disable_stabilization()
            
            # Motoru durdur
            self.motion.stop()
            
            # Zamanlayıcıları durdur
            if self.mission_timer.is_running():
                self.mission_timer.pause()
            if self.phase_timer.is_running():
                self.phase_timer.pause()
                
            self.logger.info("Görev 1 temizliği tamamlandı")
            
        except Exception as e:
            self.logger.error(f"Görev 1 temizlik hatası: {e}")

def run_mission_1(mavlink_connection, system_status, logger):
    """Görev 1'i çalıştır (dış arayüz fonksiyonu)"""
    mission = Mission1Controller(mavlink_connection, system_status, logger)
    
    try:
        # Görevi başlat
        if not mission.initialize_mission():
            logger.error("Görev 1 başlatma başarısız!")
            return False
            
        # Ana görev döngüsü
        success = mission.start_mission()
        
        # Sonuç raporu
        status = mission.get_mission_status()
        mission_time = format_time(status['mission_time'])
        
        if success:
            logger.info(f"✅ GÖREV 1 BAŞARILI! Süre: {mission_time}, Mesafe: {status['total_distance']:.1f}m")
        else:
            logger.error(f"❌ GÖREV 1 BAŞARISIZ! Süre: {mission_time}")
            
        return success
        
    except Exception as e:
        logger.error(f"Görev 1 çalıştırma hatası: {e}")
        return False
        
    finally:
        mission.cleanup()
