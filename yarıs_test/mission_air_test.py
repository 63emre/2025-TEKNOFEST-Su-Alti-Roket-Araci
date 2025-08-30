#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MISSION AIR TEST - Hava Yarışı Test Görevi
D300 derinlik sensörü KALDIRILDI - Havada test için optimize edildi
Pluswing/mission1.py'den uyarlanmıştır
"""

import time
import math
from config_air import *
from utils_air import Timer, estimate_distance, format_time
from sensors_air import SensorManager
from control_air import StabilizationController, MotionController

# GPIO import - Raspberry Pi 5 uyumlu
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("✅ RPi.GPIO sistemi yüklendi")
except ImportError:
    try:
        import lgpio as GPIO
        GPIO_AVAILABLE = True
        print("✅ lgpio sistemi yüklendi")
    except ImportError:
        GPIO_AVAILABLE = False
        print("❌ GPIO sistemi mevcut değil - simülasyon modunda çalışacak")

class AirTestController:
    """Hava Yarişi Test Ana Kontrol Sınıfı"""
    
    def __init__(self, mavlink_connection, system_status, logger, sensor_manager=None):
        self.mavlink = mavlink_connection
        self.system_status = system_status
        self.logger = logger
        
        # Ana bileşenler - kalibre edilmiş sensör manager'ı kullan
        self.sensors = sensor_manager if sensor_manager else SensorManager(mavlink_connection, logger)
        self.stabilizer = StabilizationController(mavlink_connection, self.sensors, logger)
        self.motion = MotionController(self.stabilizer, logger)
        
        # Test durumu
        self.mission_timer = Timer()
        self.phase_timer = Timer()
        self.current_phase = MissionPhase.WAITING
        self.mission_completed = False
        self.mission_success = False
        
        # Mesafe takibi
        self.total_distance_traveled = 0.0
        self.phase_distance = 0.0
        self.current_speed_pwm = MOTOR_STOP
        
        # Faz mesafeleri
        self.phase1_distance = 0.0   # İlk faz
        self.phase2_distance = 0.0   # İkinci faz
        self.return_distance = 0.0   # Geri dönüş
        
        # Hava testi özel parametreler
        self.initial_altitude = None    # Başlangıç altitude
        self.max_test_altitude = MAX_TEST_ALTITUDE  # Maksimum test yüksekliği
        
        self.logger.info("Hava yarışı test kontrolcüsü başlatıldı")
        
    def initialize_mission(self):
        """Test başlangıç hazırlıkları"""
        self.logger.info("Hava yarışı testi başlangıç hazırlıkları...")
        
        try:
            # Sensörler ana kontrolcüde kalibre edildi
            self.logger.info("Sensörler ana kontrolcüde kalibre edildi")
                
            # Stabilizasyonu başlat
            self.stabilizer.enable_stabilization()
            
            # Servoları nötrle
            self.stabilizer.servo_controller.neutral_all_servos()
            
            # Test zamanlayıcısını başlat
            self.mission_timer.start()
            
            self.logger.info("Hava yarışı testi hazırlıkları tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Test başlangıç hatası: {e}")
            return False
            
    def start_mission(self):
        """Ana test döngüsü"""
        self.logger.info("🚁 HAVA YARIŞI TESTİ BAŞLIYOR!")
        
        try:
            # Başlangıç altitude'ını kaydet
            self._record_initial_altitude()
                
            # Faz 1: İlk mesafe testi
            if not self._execute_phase_1():
                return False
                
            # Faz 2: Ana mesafe testi
            if not self._execute_phase_2():
                return False
                
            # Faz 3: U dönüşü (180°)
            if not self._execute_u_turn():
                return False
                
            # Faz 4: Geri dönüş testi
            if not self._execute_return():
                return False
                
            # Faz 5: İniş testi
            if not self._execute_landing():
                return False
                
            # Faz 6: Sistem kapatma
            if not self._execute_system_shutdown():
                return False
                
            # Test başarılı
            self.mission_completed = True
            self.mission_success = True
            self.system_status.set_phase(MissionPhase.COMPLETED)
            
            self.logger.info("🎉 HAVA YARIŞI TESTİ BAŞARIYLA TAMAMLANDI!")
            return True
            
        except Exception as e:
            self.logger.error(f"Test hatası: {e}")
            self._emergency_abort()
            return False
    
    def _record_initial_altitude(self):
        """Başlangıç altitude'ını kaydet"""
        try:
            sensor_data = self.sensors.get_all_sensor_data()
            altitude_data = sensor_data.get('altitude', {})
            
            if altitude_data.get('is_valid'):
                measured_altitude = altitude_data['altitude_m']
                self.initial_altitude = measured_altitude
                self.logger.info(f"🌍 Başlangıç altitude: {self.initial_altitude:.2f}m (ölçülen)")
            else:
                # Altitude okunamıyorsa 0m varsayılan (yer seviyesi)
                self.initial_altitude = 0.0
                self.logger.warning("Altitude sensörü okunamadı, varsayılan 0.0m kullanılıyor")
                
        except Exception as e:
            self.logger.error(f"Başlangıç altitude okuma hatası: {e}")
            self.initial_altitude = 0.0  # Güvenli varsayılan
    
    def _execute_phase_1(self):
        """Faz 1: İlk mesafe testi"""
        self.logger.info("📍 FAZ 1: İlk mesafe testi")
        self.current_phase = MissionPhase.PHASE_1
        self.system_status.set_phase(MissionPhase.PHASE_1)
        self.phase_timer.start()
        
        # Hedef değerler
        target_distance = TARGET_DISTANCE_PHASE1  # 20m
        speed_pwm = SPEED_MEDIUM
        
        # Mevcut altitude'da kal (başlangıç altitude)
        if self.initial_altitude is not None:
            self.stabilizer.set_target_altitude(self.initial_altitude)
        
        self.current_speed_pwm = speed_pwm
        
        # Stabilizasyonu aktif et ve motoru başlat
        self.stabilizer.enable_stabilization()
        self.motion.forward(speed_pwm)
        
        self.phase_distance = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "restart":
                self.logger.info("Faz 1 kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            try:
                if not self.stabilizer.update_stabilization():
                    self.logger.warning("Stabilizasyon güncellenemedi")
            except Exception as stab_error:
                self.logger.error(f"Faz 1 stabilizasyon hatası: {stab_error}")
                return False
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                if speed_pwm is not None and phase_time is not None:
                    estimated_distance = estimate_distance(speed_pwm, phase_time)
                    self.phase_distance = estimated_distance
                    self.total_distance_traveled = estimated_distance
                else:
                    estimated_distance = 0.0
                    self.logger.warning("Mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu
                altitude_result = self.sensors.altitude.get_altitude_safe()
                current_altitude, connection_status, fallback_used = altitude_result
                
                altitude_str = f"{current_altitude:.1f}m" if current_altitude is not None else f"~{self.initial_altitude:.1f}m"
                status_indicator = "⚠️" if fallback_used else "✅"
                
                self.logger.info(f"Faz 1 - Mesafe: {estimated_distance:.1f}m/{target_distance}m, "
                               f"Altitude: {altitude_str} {status_indicator}")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if self.phase_distance >= target_distance:
                self.phase1_distance = self.phase_distance
                self.logger.info(f"✓ Faz 1 tamamlandı: {self.phase1_distance:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 60:  # 1 dakika maksimum
                self.logger.warning("Faz 1 zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_phase_2(self):
        """Faz 2: Ana mesafe testi"""
        self.logger.info("📍 FAZ 2: Ana mesafe testi")
        self.current_phase = MissionPhase.PHASE_2
        self.system_status.set_phase(MissionPhase.PHASE_2)
        self.phase_timer.start()
        
        # Hedef değerler
        target_distance = TARGET_DISTANCE_PHASE2  # 30m
        speed_pwm = SPEED_FAST
        
        # Aynı altitude'da devam et
        if self.initial_altitude is not None:
            self.stabilizer.set_target_altitude(self.initial_altitude)
        
        self.current_speed_pwm = speed_pwm
        
        # Hızı artır
        self.motion.forward(speed_pwm)
        
        phase_2_distance = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "restart":
                self.logger.info("Faz 2 kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Stabilizasyon güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                if speed_pwm is not None and phase_time is not None:
                    estimated_distance = estimate_distance(speed_pwm, phase_time)
                    phase_2_distance = estimated_distance
                    self.total_distance_traveled = self.phase1_distance + phase_2_distance
                else:
                    estimated_distance = 0.0
                    phase_2_distance = 0.0
                    self.logger.warning("Faz 2 mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu
                altitude_result = self.sensors.altitude.get_altitude_safe()
                current_altitude, connection_status, fallback_used = altitude_result
                
                altitude_str = f"{current_altitude:.1f}m" if current_altitude is not None else f"~{self.initial_altitude:.1f}m"
                status_indicator = "⚠️" if fallback_used else "✅"
                
                total_forward = self.phase1_distance + phase_2_distance
                self.logger.info(f"Faz 2 - Mesafe: {phase_2_distance:.1f}m/{target_distance}m, "
                               f"Toplam: {total_forward:.1f}m/{TOTAL_FORWARD_DISTANCE}m, "
                               f"Altitude: {altitude_str} {status_indicator}")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if phase_2_distance >= target_distance:
                self.phase2_distance = phase_2_distance
                total_forward = self.phase1_distance + self.phase2_distance
                self.logger.info(f"✓ Faz 2 tamamlandı: {self.phase2_distance:.1f}m (Toplam ileri: {total_forward:.1f}m)")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 90:  # 1.5 dakika maksimum
                self.logger.warning("Faz 2 zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_u_turn(self):
        """Faz 3: U dönüşü (180°)"""
        self.logger.info("🔄 FAZ 3: U dönüşü (180°)")
        self.current_phase = MissionPhase.TURNING
        self.system_status.set_phase(MissionPhase.TURNING)
        
        # Motoru yavaşlat/durdur
        self.motion.stop()
        time.sleep(2)  # 2 saniye duraklaması
        
        # 180° U dönüşü yap
        success = self.stabilizer.turn_180_degrees(timeout=120)
        
        if success:
            self.logger.info("✓ U dönüşü tamamlandı")
            return True
        else:
            self.logger.error("✗ U dönüşü başarısız!")
            return False
            
    def _execute_return(self):
        """Faz 4: Geri dönüş testi"""
        self.logger.info("🔙 FAZ 4: Geri dönüş testi")
        self.current_phase = MissionPhase.RETURN
        self.system_status.set_phase(MissionPhase.RETURN)
        self.phase_timer.start()
        
        # Hedef değerler - tam olarak 50 metre geri
        return_distance = RETURN_DISTANCE  # 50m
        speed_pwm = SPEED_FAST
        
        # Aynı altitude'da devam et
        if self.initial_altitude is not None:
            self.stabilizer.set_target_altitude(self.initial_altitude)
        
        self.current_speed_pwm = speed_pwm
        
        self.logger.info(f"Geri dönüş hedefi: {return_distance}m")
        
        # İleri hareket başlat (180° döndük, yön ters oldu)
        self.motion.forward(speed_pwm)
        
        return_distance_traveled = 0.0
        last_distance_check = time.time()
        
        while True:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "restart":
                self.logger.info("Geri dönüş kullanıcı tarafından durduruldu")
                return False
                
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                self.logger.warning("Stabilizasyon güncellenemedi")
                
            # Mesafe hesaplama
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                if speed_pwm is not None and phase_time is not None:
                    estimated_distance = estimate_distance(speed_pwm, phase_time)
                    return_distance_traveled = estimated_distance
                else:
                    estimated_distance = 0.0
                    return_distance_traveled = 0.0
                    self.logger.warning("Geri dönüş mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu
                altitude_result = self.sensors.altitude.get_altitude_safe()
                current_altitude, connection_status, fallback_used = altitude_result
                
                altitude_str = f"{current_altitude:.1f}m" if current_altitude is not None else f"~{self.initial_altitude:.1f}m"
                status_indicator = "⚠️" if fallback_used else "✅"
                
                remaining = return_distance - return_distance_traveled
                
                self.logger.info(f"Geri dönüş - Mesafe: {return_distance_traveled:.1f}m/{return_distance}m, "
                               f"Kalan: {remaining:.1f}m, "
                               f"Altitude: {altitude_str} {status_indicator}")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if return_distance_traveled >= return_distance:
                self.return_distance = return_distance_traveled
                self.logger.info(f"✓ Geri dönüş tamamlandı: {self.return_distance:.1f}m")
                break
                
            # Zaman aşımı kontrolü
            if self.phase_timer.elapsed() > 120:  # 2 dakika maksimum
                self.logger.warning("Geri dönüş zaman aşımı!")
                break
                
            time.sleep(0.02)  # 50Hz
            
        return True
    
    def _execute_landing(self):
        """Faz 5: İniş testi"""
        self.logger.info("🛬 FAZ 5: İniş testi")
        self.current_phase = MissionPhase.LANDING
        self.system_status.set_phase(MissionPhase.LANDING)
        self.phase_timer.start()
        
        # Motoru durdur
        self.motion.stop()
        time.sleep(1)
        
        # İniş kontrolü
        success = self.stabilizer.land_control(duration=AUTO_LAND_TIMEOUT)
        
        if success:
            self.logger.info("✅ İniş tamamlandı")
            return True
        else:
            self.logger.warning("⚠️ İniş kısmen başarılı")
            return True  # Kritik hata değil
    
    def _execute_system_shutdown(self):
        """Faz 6: Sistemlerin kapatılması"""
        self.logger.info("🔌 FAZ 6: Sistem kapatma işlemi başlatılıyor...")
        
        try:
            # 1. Tüm motorları durdur
            self.logger.info("1️⃣ Motorlar durduruluyor...")
            self.motion.stop()
            time.sleep(1)
            
            # 2. Stabilizasyonu deaktif et
            self.logger.info("2️⃣ Stabilizasyon deaktif ediliyor...")
            self.stabilizer.disable_stabilization()
            
            # 3. Tüm servoları nötr pozisyona getir
            self.logger.info("3️⃣ Servolar nötr pozisyona getiriliyor...")
            self.stabilizer.servo_controller.neutral_all_servos()
            time.sleep(2)
            
            # 4. GPIO temizliği ve sinyaller
            self.logger.info("4️⃣ GPIO sinyalleri ve temizlik...")
            if GPIO_AVAILABLE:
                try:
                    # Test tamamlandı sinyali
                    for _ in range(3):
                        if hasattr(GPIO, 'output'):
                            GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                            GPIO.output(GPIO_LED_RED, GPIO.HIGH)
                        time.sleep(0.3)
                        if hasattr(GPIO, 'output'):
                            GPIO.output(GPIO_BUZZER, GPIO.LOW)
                            GPIO.output(GPIO_LED_RED, GPIO.LOW)
                        time.sleep(0.3)
                    
                    # GPIO temizliği
                    GPIO.cleanup()
                    self.logger.info("✅ GPIO temizliği tamamlandı")
                except Exception as e:
                    self.logger.warning(f"GPIO temizlik hatası: {e}")
            else:
                self.logger.info("✅ GPIO simülasyonu - temizlik atlandı")
            
            # 5. Sensör bağlantılarını kapat
            self.logger.info("5️⃣ Sensör bağlantıları kapatılıyor...")
            try:
                if hasattr(self.sensors, 'cleanup'):
                    self.sensors.cleanup()
            except Exception as e:
                self.logger.warning(f"Sensör temizlik hatası: {e}")
            
            # 6. Zamanlayıcıları durdur
            self.logger.info("6️⃣ Zamanlayıcılar durduruluyor...")
            try:
                if self.mission_timer.is_running():
                    self.mission_timer.pause()
                if self.phase_timer.is_running():
                    self.phase_timer.pause()
            except Exception as e:
                self.logger.warning(f"Zamanlayıcı durdurma hatası: {e}")
            
            # 7. Son durum raporu
            total_mission_time = self.mission_timer.elapsed() if self.mission_timer else 0
            total_distance = self.phase1_distance + self.phase2_distance + self.return_distance
            
            self.logger.info("📊 HAVA YARIŞI TEST ÖZET RAPORU:")
            self.logger.info(f"   • Toplam süre: {format_time(total_mission_time)}")
            self.logger.info(f"   • Toplam mesafe: {total_distance:.1f}m")
            self.logger.info(f"   • Faz 1: {self.phase1_distance:.1f}m")
            self.logger.info(f"   • Faz 2: {self.phase2_distance:.1f}m") 
            self.logger.info(f"   • Geri dönüş: {self.return_distance:.1f}m")
            self.logger.info(f"   • Başlangıç altitude: {self.initial_altitude:.1f}m")
            
            self.logger.info("✅ Sistem kapatma işlemi tamamlandı")
            self.logger.info("🏁 HAVA YARIŞI TESTİ TAMAMEN BİTİRİLDİ - SİSTEM GÜVENLİ DURUMDA")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Sistem kapatma hatası: {e}")
            # Kritik hata olsa bile devam et
            return True
            
    def _emergency_abort(self):
        """Genel acil durum prosedürü"""
        self.logger.error("🚨 ACİL DURUM - Test iptal ediliyor!")
        
        try:
            # Motoru durdur
            self.motion.stop()
            
            # Stabilizasyonu durdur
            self.stabilizer.disable_stabilization()
            
            # Sistem durumunu ayarla
            self.system_status.emergency_stop()
            
            # Test durumunu ayarla
            self.mission_completed = True
            self.mission_success = False
            self.current_phase = MissionPhase.EMERGENCY
            
        except Exception as e:
            self.logger.error(f"Acil durum prosedürü hatası: {e}")
            
    def get_mission_status(self):
        """Test durumu raporu"""
        status = {
            'phase': self.current_phase,
            'mission_time': self.mission_timer.elapsed() if self.mission_timer.is_running() else 0,
            'phase_time': self.phase_timer.elapsed() if self.phase_timer.is_running() else 0,
            'total_distance': self.total_distance_traveled if self.total_distance_traveled is not None else 0.0,
            'phase_distance': self.phase_distance if self.phase_distance is not None else 0.0,
            'completed': self.mission_completed,
            'success': self.mission_success,
            'current_speed': self.current_speed_pwm if self.current_speed_pwm is not None else MOTOR_STOP
        }
        
        # Sensör durumu ekle
        try:
            sensor_data = self.sensors.get_all_sensor_data()
            if sensor_data and sensor_data.get('altitude') and sensor_data['altitude'].get('is_valid'):
                altitude_val = sensor_data['altitude']['altitude_m']
                if altitude_val is not None:
                    status['current_altitude'] = altitude_val
            if sensor_data and sensor_data['attitude']:
                heading_val = sensor_data['attitude'].get('yaw_relative_deg')
                if heading_val is not None:
                    status['current_heading'] = heading_val
        except Exception as e:
            self.logger.warning(f"Sensör durumu alma hatası: {e}")
            
        return status
        
    def cleanup(self):
        """Test temizliği"""
        self.logger.info("Hava yarışı testi temizleniyor...")
        
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
                
            self.logger.info("Hava yarışı testi temizliği tamamlandı")
            
        except Exception as e:
            self.logger.error(f"Test temizlik hatası: {e}")

def run_air_test_mission(mavlink_connection, system_status, logger, sensor_manager=None):
    """Hava yarışı testini çalıştır (dış arayüz fonksiyonu)"""
    mission = AirTestController(mavlink_connection, system_status, logger, sensor_manager)
    
    try:
        # Testi başlat
        if not mission.initialize_mission():
            logger.error("Hava yarışı testi başlatma başarısız!")
            return False
            
        # Ana test döngüsü
        success = mission.start_mission()
        
        # Sonuç raporu
        status = mission.get_mission_status()
        mission_time = format_time(status['mission_time'])
        
        if success:
            logger.info(f"✅ HAVA YARIŞI TESTİ BAŞARILI! Süre: {mission_time}, Mesafe: {status['total_distance']:.1f}m")
        else:
            logger.error(f"❌ HAVA YARIŞI TESTİ BAŞARISIZ! Süre: {mission_time}")
            
        return success
        
    except Exception as e:
        logger.error(f"Hava yarışı testi çalıştırma hatası: {e}")
        return False
        
    finally:
        mission.cleanup()
