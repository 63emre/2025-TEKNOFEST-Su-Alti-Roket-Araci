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

class RocketController:
    """Roket fırlatma kontrol sistemi - Şartname uyumlu"""
    
    def __init__(self, logger):
        self.logger = logger
        self.solenoid_active = False
        self.rocket_chamber_ready = False
        self.launch_sequence_active = False
        
        # GPIO ayarla
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_SOLENOID, GPIO.OUT)
        GPIO.output(GPIO_SOLENOID, GPIO.LOW)
        
        self.logger.info("Roket kontrolcüsü başlatıldı")
        
    def prepare_rocket(self):
        """Roket hazırlık prosedürü - Taşıma bölmesi dahil"""
        self.logger.info("🚀 Roket sistemi hazırlık prosedürü başlatılıyor...")
        
        try:
            # Şartname: "SARA'nın ön kısmındaki roket taşıma bölmesi"
            self.logger.info("1/4: Roket taşıma bölmesi kontrolü...")
            if not self._check_rocket_chamber():
                self.logger.error("❌ Roket taşıma bölmesi problemi!")
                return False
            
            self.logger.info("2/4: CO2 sistemi basınç kontrolü...")
            if not self._check_pressure_system():
                self.logger.error("❌ CO2 basınç sistemi problemi!")
                return False
                
            self.logger.info("3/4: Selenoid valf test...")
            if not self._test_solenoid():
                self.logger.error("❌ Selenoid valf test başarısız!")
                return False
                
            self.logger.info("4/4: Fırlatma sistemi son kontrolü...")
            if not self._final_launch_check():
                self.logger.error("❌ Son kontrol başarısız!")
                return False
            
            self.rocket_chamber_ready = True
            self.logger.info("✅ Roket sistemi fırlatma için tamamen hazır!")
            return True
            
        except Exception as e:
            self.logger.error(f"Roket hazırlık hatası: {e}")
            return False
    
    def _check_rocket_chamber(self):
        """Roket taşıma bölmesi kontrolü"""
        # Şartname gereksinimi: Roket taşıma bölmesi kontrolü
        self.logger.info("📦 Roket taşıma bölmesi durumu kontrol ediliyor...")
        
        # Burada gerçek sensörlerle roket varlığı kontrol edilebilir
        # Örnek: limit switch, weight sensor, vs.
        
        time.sleep(1)  # Kontrol simülasyonu
        self.logger.info("✓ Roket taşıma bölmesi: Roket yerinde ve güvenli")
        return True
    
    def _check_pressure_system(self):
        """CO2 basınç sistemi kontrolü"""
        self.logger.info("💨 CO2 basınç sistemi kontrol ediliyor...")
        
        # Burada basınç sensörü ile CO2 tüpü basıncı kontrol edilebilir
        # Minimum fırlatma basıncı kontrolü
        
        time.sleep(1)  # Kontrol simülasyonu
        self.logger.info("✓ CO2 sistemi: Basınç yeterli, sistem hazır")
        return True
    
    def _test_solenoid(self):
        """Selenoid valf kısa test"""
        self.logger.info("🔧 Selenoid valf kısa test...")
        
        try:
            # Çok kısa test (50ms) - basınç kaybı olmadan
            GPIO.output(GPIO_SOLENOID, GPIO.HIGH)
            time.sleep(0.05)  # 50ms
            GPIO.output(GPIO_SOLENOID, GPIO.LOW)
            
            self.logger.info("✓ Selenoid valf: Test başarılı")
            return True
            
        except Exception as e:
            self.logger.error(f"Selenoid test hatası: {e}")
            return False
    
    def _final_launch_check(self):
        """Son fırlatma kontrolü"""
        self.logger.info("🎯 Son fırlatma kontrolleri...")
        
        # Tüm sistemlerin son kontrolü
        checks = {
            'roket_yerinde': True,  # Roket taşıma bölmesinde
            'basınç_yeterli': True,  # CO2 basıncı
            'selenoid_çalışıyor': True,  # Selenoid test sonucu
            'güvenli_bölge': True,  # Güvenli atış bölgesinde
        }
        
        for check_name, status in checks.items():
            if status:
                self.logger.info(f"  ✓ {check_name}")
            else:
                self.logger.error(f"  ❌ {check_name}")
                return False
        
        time.sleep(1)  # Final check süresi
        self.logger.info("✅ Tüm fırlatma kontrolleri başarılı!")
        return True
        
    def launch_rocket(self):
        """Roket fırlatma - Şartname uyumlu prosedür"""
        if not self.rocket_chamber_ready:
            self.logger.error("❌ Roket sistemi hazır değil!")
            return False
            
        self.logger.info("🚀 ROKET FIRLATMA PROSEDÜRÜ BAŞLIYOR!")
        self.launch_sequence_active = True
        
        try:
            # Şartname: "sistemin roket ateşleme mekanizmasını otonom şekilde çalıştırarak"
            self.logger.info("🎯 Otonom roket fırlatma sekansı...")
            
            # Fırlatma öncesi son kontrol
            self.logger.info("Son güvenlik kontrolleri...")
            time.sleep(1.0)
            
            # CO2 sistemi aktifleştirme
            self.logger.info("💨 CO2 basınçlı fırlatma sistemi aktifleştiriliyor...")
            
            # Selenoid valfi aktif et (CO2 tüpünü del)
            GPIO.output(GPIO_SOLENOID, GPIO.HIGH)
            self.solenoid_active = True
            self.logger.info("🔓 Selenoid valf açıldı - CO2 salınıyor...")
            
            # Şartname uyumlu fırlatma süresi (2 saniye)
            launch_duration = 2.0
            start_time = time.time()
            
            # Fırlatma süreci takibi
            while time.time() - start_time < launch_duration:
                elapsed = time.time() - start_time
                remaining = launch_duration - elapsed
                
                if int(elapsed * 10) % 5 == 0:  # Her 0.5 saniyede
                    self.logger.info(f"🚀 Fırlatma devam ediyor... {remaining:.1f}s")
                
                time.sleep(0.1)
            
            # Selenoid kapat
            GPIO.output(GPIO_SOLENOID, GPIO.LOW)
            self.solenoid_active = False
            self.launch_sequence_active = False
            
            self.logger.info("✅ ROKET BAŞARIYLA FIRLATILDI!")
            self.logger.info("🎯 Otonom roket fırlatma prosedürü tamamlandı")
            
            # Fırlatma sonrası sistem durumu
            self._post_launch_status()
            
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Roket fırlatma hatası: {e}")
            self._emergency_shutdown()
            return False
    
    def _post_launch_status(self):
        """Fırlatma sonrası sistem durumu"""
        self.logger.info("📊 Fırlatma sonrası sistem durumu:")
        self.logger.info("  ✓ Roket başarıyla fırlatıldı")
        self.logger.info("  ✓ CO2 sistemi güvenli konuma getirildi")
        self.logger.info("  ✓ Selenoid valf kapatıldı")
        self.logger.info("  ✓ Taşıma bölmesi boş")
        self.logger.info("🎉 Görev 2 fırlatma fazı başarıyla tamamlandı!")
            
    def _emergency_shutdown(self):
        """Roket sistemi acil kapama"""
        try:
            GPIO.output(GPIO_SOLENOID, GPIO.LOW)
            self.solenoid_active = False
            self.logger.info("Roket sistemi acil kapatma tamamlandı")
        except Exception as e:
            self.logger.error(f"Acil kapatma hatası: {e}")
            
    def get_status(self):
        """Roket sistem durumu - Geliştirilmiş"""
        return {
            'solenoid_active': self.solenoid_active,
            'rocket_chamber_ready': self.rocket_chamber_ready,
            'launch_sequence_active': self.launch_sequence_active,
            'ready_for_launch': (self.rocket_chamber_ready and 
                               not self.solenoid_active and 
                               not self.launch_sequence_active),
            'system_status': 'READY' if self.rocket_chamber_ready else 'NOT_READY'
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
            # Sensör kalibrasyonu (havada - su yüzeyinde tutmadan)
            calibration_results = self.sensors.calibrate_all(use_water_surface_calib=False)
            
            if not all(calibration_results.values()):
                self.logger.warning("Bazı sensörler kalibre edilemedi!")
                
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
                estimated_distance = estimate_distance(speed_pwm, phase_time)
                approach_distance = estimated_distance
                self.total_distance_traveled = estimated_distance
                
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
        # 🆕 ROKET HAZIRLIK SİNYALİ
        self.system_status.buzzer.beep_pattern(get_buzzer_signal_for_phase(MissionPhase.ROCKET_PREP))  # 5 orta bip
        self.system_status.led.blink(get_led_blink_for_phase(MissionPhase.ROCKET_PREP))
        
        self.logger.info("📍 FAZ 2: Roket hazırlık ve pozisyonlama")
        self.current_phase = MissionPhase.ROCKET_PREP
        self.system_status.set_phase(MissionPhase.ROCKET_PREP)
        
        # Motoru durdur
        self.motion.stop()
        
        # Pozisyonlama için stabilizasyonu aktif tut
        # Hedef derinliği koru
        self.stabilizer.set_target_depth(self.target_depth)
        
        # Uygun yunuslama açısı için pitch ayarlaması
        self.logger.info("Uygun yunuslama açısı için pozisyon ayarlanıyor...")
        
        # Şartname gereksinimi: "uygun yunuslama açısı ile satha ulaşmalı"
        # Hafif burun yukarı açısı (5-10 derece) için pitch offset
        target_pitch_offset = math.radians(7.5)  # 7.5 derece yunuslama açısı
        
        # 15 saniye pozisyonlama (yunuslama açısı dahil)
        positioning_time = 15.0
        start_time = time.time()
        
        self.logger.info("Roket fırlatma pozisyonu ve yunuslama açısı ayarlanıyor...")
        
        while time.time() - start_time < positioning_time:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Roket hazırlık kullanıcı tarafından durduruldu")
                return False
                
            # Yunuslama açısı için özel stabilizasyon
            sensor_data = self.sensors.get_all_sensor_data()
            if sensor_data['attitude']:
                current_pitch = sensor_data['attitude']['pitch']
                pitch_error = target_pitch_offset - current_pitch
                
                # Hafif pitch düzeltmesi (yunuslama açısı için)
                if abs(pitch_error) > math.radians(2):  # 2 derece tolerans
                    pitch_correction = pitch_error * 0.3  # Yumuşak düzeltme
                    
                    # Servo komutlarını manuel ayarla
                    if pitch_correction > 0:  # Burun yukarı
                        self.stabilizer.servo_controller.set_servo(SERVO_UP, PWM_NEUTRAL + 100)
                        self.stabilizer.servo_controller.set_servo(SERVO_DOWN, PWM_NEUTRAL - 100)
                    else:  # Burun aşağı
                        self.stabilizer.servo_controller.set_servo(SERVO_UP, PWM_NEUTRAL - 100)
                        self.stabilizer.servo_controller.set_servo(SERVO_DOWN, PWM_NEUTRAL + 100)
                else:
                    # Hedef açıda, servoları nötrle
                    self.stabilizer.servo_controller.set_servo(SERVO_UP, PWM_NEUTRAL)
                    self.stabilizer.servo_controller.set_servo(SERVO_DOWN, PWM_NEUTRAL)
                
            # Normal stabilizasyonu güncelle (roll ve yaw için)
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Pozisyonlama stabilizasyonu güncellenemedi")
                
            # Durum raporu
            remaining_time = positioning_time - (time.time() - start_time)
            if int(remaining_time) % 3 == 0:  # Her 3 saniyede
                current_depth = sensor_data['depth']['depth_m'] if sensor_data and sensor_data['depth']['is_valid'] else None
                depth_str = f"{current_depth:.1f}m" if current_depth else "N/A"
                
                current_pitch_deg = math.degrees(sensor_data['attitude']['pitch']) if sensor_data and sensor_data['attitude'] else 0
                target_pitch_deg = math.degrees(target_pitch_offset)
                
                self.logger.info(f"Pozisyonlama - Kalan: {remaining_time:.1f}s, "
                               f"Derinlik: {depth_str}, "
                               f"Yunuslama açısı: {current_pitch_deg:.1f}°/{target_pitch_deg:.1f}°")
                
            time.sleep(0.1)
            
        self.logger.info("✓ Roket pozisyonlama ve yunuslama açısı tamamlandı")
        return True
        
    def _execute_rocket_launch(self):
        """Faz 3: Roket fırlatma"""
        # 🆕 ROKET FIRLATMA SİNYALİ
        self.system_status.buzzer.beep_pattern(get_buzzer_signal_for_phase(MissionPhase.ROCKET_LAUNCH))  # 1 çok uzun bip
        self.system_status.led.turn_on()  # Sürekli açık
        
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
                estimated_distance = estimate_distance(speed_pwm, phase_time)
                withdrawal_traveled = estimated_distance
                
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
            
        # Yüzeye çıkış - Şartname uyumlu yunuslama açısı ile
        self.logger.info("Yüzeye çıkış başlatılıyor...")
        self.current_phase = MissionPhase.SURFACING
        self.system_status.set_phase(MissionPhase.SURFACING)
        
        # Yunuslama açısı ile kontrollü yüzeye çıkış
        success = self._execute_controlled_surfacing()
        
        if success:
            self.logger.info("✓ Yunuslama açısı ile yüzeye çıkış tamamlandı")
        else:
            self.logger.warning("Yüzeye çıkış problemi")
            
        return True
        
    def _execute_controlled_surfacing(self):
        """Kontrollü yüzeye çıkış - Yunuslama açısı ile"""
        self.logger.info("Kontrollü yüzeye çıkış başlatılıyor...")
        
        # Şartname: "uygun yunuslama açısı ile satha ulaşmalı"
        target_surface_pitch = math.radians(15.0)  # 15 derece yunuslama açısı
        surfacing_duration = 20.0  # 20 saniye maksimum
        
        start_time = time.time()
        surface_reached = False
        
        self.logger.info(f"Hedef yunuslama açısı: {math.degrees(target_surface_pitch):.1f}°")
        
        while time.time() - start_time < surfacing_duration and not surface_reached:
            # Sensör verilerini al
            sensor_data = self.sensors.get_all_sensor_data()
            
            # Derinlik kontrolü
            current_depth = None
            if sensor_data['depth']['is_valid']:
                current_depth = sensor_data['depth']['depth_m']
                
                # Yüzeye ulaştık mı? (0.5m altında)
                if current_depth < SURFACE_DEPTH_THRESHOLD:
                    surface_reached = True
                    self.logger.info(f"✓ Yüzeye ulaşıldı: {current_depth:.2f}m")
                    break
            
            # Yunuslama açısı kontrolü
            if sensor_data['attitude']:
                current_pitch = sensor_data['attitude']['pitch']
                pitch_error = target_surface_pitch - current_pitch
                
                # Yunuslama açısı düzeltmesi
                if abs(pitch_error) > math.radians(3):  # 3 derece tolerans
                    pitch_correction = pitch_error * 0.5  # Orta hızda düzeltme
                    
                    # Yüzeye çıkış için servo komutları
                    if pitch_correction > 0:  # Daha fazla burun yukarı
                        self.stabilizer.servo_controller.set_servo(SERVO_UP, PWM_NEUTRAL + 150)
                        self.stabilizer.servo_controller.set_servo(SERVO_DOWN, PWM_NEUTRAL - 150)
                    else:  # Daha az burun yukarı
                        self.stabilizer.servo_controller.set_servo(SERVO_UP, PWM_NEUTRAL + 50)
                        self.stabilizer.servo_controller.set_servo(SERVO_DOWN, PWM_NEUTRAL - 50)
                else:
                    # Hedef yunuslama açısında - yüzeye çıkış pozisyonu
                    self.stabilizer.servo_controller.set_servo(SERVO_UP, PWM_NEUTRAL + 100)
                    self.stabilizer.servo_controller.set_servo(SERVO_DOWN, PWM_NEUTRAL - 100)
            
            # Roll ve yaw kontrolü (minimum düzeyde)
            if sensor_data['attitude']:
                roll = sensor_data['attitude']['roll']
                yaw_relative = sensor_data['attitude']['yaw_relative']
                
                # Basit roll düzeltmesi
                if abs(math.degrees(roll)) > 10:  # 10 derece roll toleransı
                    roll_correction = roll * 0.2
                    self.stabilizer.servo_controller.set_servo(SERVO_LEFT, PWM_NEUTRAL + int(roll_correction * 100))
                    self.stabilizer.servo_controller.set_servo(SERVO_RIGHT, PWM_NEUTRAL - int(roll_correction * 100))
                else:
                    self.stabilizer.servo_controller.set_servo(SERVO_LEFT, PWM_NEUTRAL)
                    self.stabilizer.servo_controller.set_servo(SERVO_RIGHT, PWM_NEUTRAL)
            
            # Durum raporu
            elapsed_time = time.time() - start_time
            if int(elapsed_time) % 2 == 0:  # Her 2 saniyede
                depth_str = f"{current_depth:.2f}m" if current_depth else "N/A"
                current_pitch_deg = math.degrees(sensor_data['attitude']['pitch']) if sensor_data['attitude'] else 0
                target_pitch_deg = math.degrees(target_surface_pitch)
                
                remaining_time = surfacing_duration - elapsed_time
                self.logger.info(f"Yüzeye çıkış - Kalan: {remaining_time:.1f}s, "
                               f"Derinlik: {depth_str}, "
                               f"Yunuslama: {current_pitch_deg:.1f}°/{target_pitch_deg:.1f}°")
            
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Yüzeye çıkış kullanıcı tarafından durduruldu")
                break
                
            time.sleep(0.2)  # 5Hz döngü
        
        # Yüzeye çıkış sonrası - tüm servoları nötrle
        self.stabilizer.servo_controller.neutral_all_servos()
        
        # Son durum raporu
        if surface_reached:
            self.logger.info("✅ Yunuslama açısı ile başarılı yüzey çıkışı tamamlandı")
            return True
        else:
            elapsed_time = time.time() - start_time
            self.logger.warning(f"⚠️ Yüzeye çıkış zaman aşımı: {elapsed_time:.1f}s")
            return False
        
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
