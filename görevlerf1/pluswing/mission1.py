#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MISSION 1 - Seyir ve Başlangıç Noktasına Dönüş Görevi
Yeni görev tanımı:
- Dalgıç aracı belirli derinliğe indirir, butona basar
- 90 saniye bekleme (buzzer ve LED mesajlarıyla)
- Suyun altında herhangi bir derinlikte görev başlar
- İlk 10m ileri gider
- 40m daha ileri gider (toplam 50m)
- U dönüşü yapar
- 50m geri gelir
- Son 2m kala kontrollü yüzeye çıkar
- Sistemler yazılımsal olarak kapatılır
"""

import time
import math
from config import *
from utils import Timer, estimate_distance, format_time, _advanced_calculator
from sensors import SensorManager
from control import StabilizationController, MotionController

# GPIO import - Raspberry Pi 5 uyumlu
try:
    from gpio_compat import GPIO
    GPIO_AVAILABLE = True
    print("✅ Pi 5 uyumlu GPIO sistemi yüklendi")
except ImportError:
    try:
        from gpio_wrapper import GPIO
        GPIO_AVAILABLE = True
        print("✅ GPIO wrapper sistemi yüklendi")
    except ImportError:
        GPIO_AVAILABLE = False
        print("❌ GPIO sistemi mevcut değil - simülasyon modunda çalışacak")

class Mission1Controller:
    """Görev 1 Ana Kontrol Sınıfı"""
    
    def __init__(self, mavlink_connection, system_status, logger, sensor_manager=None):
        self.mavlink = mavlink_connection
        self.system_status = system_status
        self.logger = logger
        
        # Ana bileşenler - kalibre edilmiş sensör manager'ı kullan
        self.sensors = sensor_manager if sensor_manager else SensorManager(mavlink_connection, logger)
        self.stabilizer = StabilizationController(mavlink_connection, self.sensors, logger)
        self.motion = MotionController(self.stabilizer, logger)
        
        # PWM sinyalleri etkinleştir (65 saniye tamamlandı)
        self.stabilizer.servo_controller.enable_pwm_signals()
        self.logger.info("🚀 Mission1: PWM sinyalleri etkinleştirildi!")
        
        # MOTOR TEST - PWM etkinleştirme sonrası
        self.test_motor_after_pwm_enable()
        
        # Görev durumu
        self.mission_timer = Timer()
        self.phase_timer = Timer()
        self.current_phase = MissionPhase.WAITING
        self.mission_completed = False
        self.mission_success = False
        
        # 90 saniye bekleme için
        self.waiting_timer = Timer()
        self.waiting_completed = False
        
        # Mesafe takibi - yeni görev planı
        self.total_distance_traveled = 0.0
        self.phase_distance = 0.0
        self.current_speed_pwm = MOTOR_STOP
        
        # Yeni mesafe planı için değişkenler
        self.phase1_distance = 0.0   # İlk 10m
        self.phase2_distance = 0.0   # Sonraki 40m
        self.return_distance = 0.0   # Geri dönüş 50m
        
        # Yüzeye çıkış kontrolü için
        self.surface_approach_started = False
        self.initial_depth = None    # Başlangıç derinliği
        
        self.logger.info("Görev 1 kontrolcüsü başlatıldı")
    
    def test_motor_after_pwm_enable(self):
        """PWM etkinleştirme sonrası motor testi"""
        self.logger.info("🧪 MOTOR PWM TEST BAŞLIYOR...")
        
        try:
            # PWM durumunu kontrol et
            pwm_status = self.stabilizer.servo_controller.pwm_allowed
            self.logger.info(f"🔍 PWM Durumu: {pwm_status}")
            
            if not pwm_status:
                self.logger.error("❌ PWM henüz etkinleştirilmemiş!")
                return False
            
            # Motor STOP testi
            self.logger.info("🔧 Motor STOP testi...")
            stop_success = self.motion.servo_controller.set_motor(MOTOR_STOP)
            self.logger.info(f"Motor STOP sonucu: {stop_success}")
            
            time.sleep(0.5)
            
            # Motor İLERİ testi (kısa süre)
            self.logger.info("🔧 Motor İLERİ testi (1 saniye)...")
            forward_success = self.motion.servo_controller.set_motor(MOTOR_FORWARD_MIN)
            self.logger.info(f"Motor İLERİ sonucu: {forward_success}")
            
            time.sleep(1.0)  # 1 saniye test
            
            # Tekrar STOP
            self.logger.info("🔧 Motor STOP (test sonu)...")
            stop_success2 = self.motion.servo_controller.set_motor(MOTOR_STOP)
            self.logger.info(f"Motor STOP sonucu: {stop_success2}")
            
            if stop_success and forward_success and stop_success2:
                self.logger.info("✅ MOTOR PWM TESTİ BAŞARILI!")
                return True
            else:
                self.logger.error("❌ MOTOR PWM TESTİ BAŞARISIZ!")
                return False
                
        except Exception as e:
            self.logger.error(f"Motor test hatası: {e}")
            return False
        
    def initialize_mission(self):
        """Görev başlangıç hazırlıkları"""
        self.logger.info("Görev 1 başlangıç hazırlıkları...")
        
        try:
            # NOT: Sensör kalibrasyonu main.py'de zaten yapıldı, tekrar yapma!
            self.logger.info("Sensörler ana kontrolcüde kalibre edildi")
                
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
        """Ana görev döngüsü - yeni görev tanımına göre"""
        self.logger.info("🚀 GÖREV 1 BAŞLIYOR!")
        
        try:
            # Faz 0: 90 saniye bekleme (buzzer ve LED ile)
            if not self._execute_waiting_phase():
                return False
            
            # Başlangıç derinliğini kaydet
            self._record_initial_depth()
                
            # Faz 1: İlk 10 metre ileri
            if not self._execute_phase_1_new():
                return False
                
            # Faz 2: 40 metre daha ileri (toplam 50m)
            if not self._execute_phase_2_new():
                return False
                
            # Faz 3: U dönüşü (180°)
            if not self._execute_u_turn():
                return False
                
            # Faz 4: 50 metre geri dönüş
            if not self._execute_return_new():
                return False
                
            # Faz 5: Son 2m kala kontrollü yüzeye çıkış
            if not self._execute_controlled_surfacing():
                return False
                
            # Faz 6: Sistem kapatma
            if not self._execute_system_shutdown():
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
            
    def _execute_waiting_phase(self):
        """Faz 0: 90 saniye bekleme (buzzer ve LED mesajlarıyla)"""
        self.logger.info("🕰️ FAZ 0: 90 saniye bekleme başlatılıyor...")
        self.current_phase = MissionPhase.WAITING
        self.system_status.set_phase(MissionPhase.WAITING)
        self.waiting_timer.start()
        
        # Buzzer başlangıç sinyali
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(GPIO_BUZZER, GPIO.OUT)
                GPIO.setup(GPIO_LED_RED, GPIO.OUT)
            except:
                self.logger.warning("GPIO kurulumu yapılamadı, buzzer/LED çalışmayacak")
        else:
            self.logger.warning("GPIO mevcut değil, buzzer/LED simüle edilecek")
        
        countdown_seconds = 10
        
        while countdown_seconds > 0:
            # Buton kontrolü - acil durdurma
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Bekleme fazı kullanıcı tarafından durduruldu")
                return False
            
            # Her 10 saniyede bir durum raporu
            if countdown_seconds % 10 == 0:
                self.logger.info(f"⏰ Kalan süre: {countdown_seconds} saniye")
                
                # Buzzer ve LED sinyali
                if GPIO_AVAILABLE:
                    try:
                        # Kısa bip ve LED yanıp sönme
                        GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                        GPIO.output(GPIO_LED_RED, GPIO.HIGH)
                        time.sleep(0.2)
                        GPIO.output(GPIO_BUZZER, GPIO.LOW)
                        GPIO.output(GPIO_LED_RED, GPIO.LOW)
                    except:
                        pass
            
            # Son 10 saniyede hızlı bip
            if countdown_seconds <= 10:
                if GPIO_AVAILABLE:
                    try:
                        GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                        GPIO.output(GPIO_LED_RED, GPIO.HIGH)
                        time.sleep(0.1)
                        GPIO.output(GPIO_BUZZER, GPIO.LOW)
                        GPIO.output(GPIO_LED_RED, GPIO.LOW)
                        time.sleep(0.1)
                    except:
                        time.sleep(0.2)
                else:
                    time.sleep(0.2)
            else:
                time.sleep(1.0)
            
            countdown_seconds -= 1
        
        # Bekleme tamamlandı sinyali
        self.logger.info("✅ 90 saniye bekleme tamamlandı!")
        if GPIO_AVAILABLE:
            try:
                for _ in range(3):
                    GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                    GPIO.output(GPIO_LED_RED, GPIO.HIGH)
                    time.sleep(0.5)
                    GPIO.output(GPIO_BUZZER, GPIO.LOW)
                    GPIO.output(GPIO_LED_RED, GPIO.LOW)
                    time.sleep(0.5)
            except:
                pass
        
        self.waiting_completed = True
        return True
    
    def _record_initial_depth(self):
        """Başlangıç derinliğini kaydet - araç 2-2.5m derinliğe indirileceği için"""
        try:
            sensor_data = self.sensors.get_all_sensor_data()
            depth_data = sensor_data['depth']
            if depth_data['is_valid']:
                measured_depth = depth_data['depth_m']
                
                # 2-2.5m aralığında olmalı
                if 1.8 <= measured_depth <= 2.7:
                    self.initial_depth = measured_depth
                    self.logger.info(f"🌊 Başlangıç derinliği: {self.initial_depth:.2f}m (ölçülen)")
                else:
                    # Aralık dışındaysa 2.25m varsayılan kullan
                    self.initial_depth = 2.25
                    self.logger.warning(f"Ölçülen derinlik ({measured_depth:.2f}m) beklenen aralık dışında, varsayılan 2.25m kullanılıyor")
            else:
                # D300 okunamıyorsa 2.25m varsayılan (2-2.5m ortası)
                self.initial_depth = 2.25
                self.logger.warning("D300 sensörü okunamadı, varsayılan 2.25m kullanılıyor (2-2.5m aralık ortası)")
                
        except Exception as e:
            self.logger.error(f"Başlangıç derinlik okuma hatası: {e}")
            self.initial_depth = 2.25  # Güvenli varsayılan
    
    def _execute_phase_1_new(self):
        """Faz 1: İlk 10 metre ileri hareket"""
        self.logger.info("📍 FAZ 1: İlk 10 metre ileri")
        self.current_phase = MissionPhase.PHASE_1
        self.system_status.set_phase(MissionPhase.PHASE_1)
        self.phase_timer.start()

        # GELİŞMİŞ MESAFE HESAPLAYICIYI SIFIRLA
        _advanced_calculator.reset()
        self.logger.info("🔄 Gelişmiş mesafe hesaplayıcı sıfırlandı - Faz 1")
        self.logger.info(f"🎯 Hedef: 10m, Hız: {SPEED_MEDIUM} PWM, Mod: {DISTANCE_CALC_MODE}")
        
        # Hedef değerler
        target_distance = 10.0  # İlk 10 metre
        speed_pwm = SPEED_MEDIUM
        
        # Mevcut derinlikte kal (başlangıç derinliği)
        if self.initial_depth:
            self.stabilizer.set_target_depth(self.initial_depth)
        
        self.current_speed_pwm = speed_pwm
        
        # Stabilizasyonu aktif et ve motoru başlat
        self.stabilizer.enable_stabilization()
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
            try:
                if not self.stabilizer.update_stabilization():
                    self.logger.warning("Stabilizasyon güncellenemedi")
            except Exception as stab_error:
                self.logger.error(f"Faz 1 stabilizasyon hatası: {stab_error}")
                return False
                
            # GELİŞMİŞ MESAFE HESAPLAMA
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                if speed_pwm is not None and phase_time is not None:
                    # Gelişmiş mesafe hesaplaması
                    estimated_distance = estimate_distance(speed_pwm, phase_time)

                    # Hız bilgilerini al (gelişmiş moddaysa)
                    current_speed = 0.0
                    if DISTANCE_CALC_MODE == "ADVANCED":
                        try:
                            current_speed = _advanced_calculator.last_speed
                        except:
                            current_speed = 0.0

                    self.phase_distance = estimated_distance
                    self.total_distance_traveled = estimated_distance

                    # Detaylı mesafe logu
                    self.logger.info(f"📏 Faz 1 Mesafe: {estimated_distance:.2f}m/{target_distance}m, "
                                   f"Hız: {current_speed:.1f}m/s, Süre: {phase_time:.1f}s, "
                                   f"PWM: {speed_pwm}")
                else:
                    estimated_distance = 0.0
                    self.logger.warning("Mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu - D300 kesilirse sistem durmasın
                depth_result = self.sensors.depth.get_depth_safe("PHASE_1")
                current_depth, connection_status, fallback_used = depth_result
                
                # D300 kesintisi durumunda uyarı ver ama devam et
                if connection_status in ["EMERGENCY_PHASE1", "TEMPORARY_ISSUE"]:
                    self.logger.warning(f"⚠️ D300 bağlantı sorunu ({connection_status}) - tahmine dayalı devam ediliyor")
                    # D300'ü yeniden almaya çalış (arka planda)
                    try:
                        self.sensors.depth.reconnect_attempt()
                    except:
                        pass
                
                depth_str = f"{current_depth:.1f}m" if current_depth else f"~{self.initial_depth:.1f}m"
                status_indicator = "⚠️" if fallback_used else "✅"
                
                self.logger.info(f"Faz 1 - Mesafe: {estimated_distance:.1f}m/{target_distance}m, "
                               f"Derinlik: {depth_str} {status_indicator}")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if self.phase_distance >= target_distance:
                self.phase1_distance = self.phase_distance
                self.logger.info(f"✓ Faz 1 tamamlandı: {self.phase1_distance:.1f}m")
                break
                
            # Global görev timeout kontrolü (sadece bu)
            if self.mission_timer.elapsed() > MISSION_TIMEOUT_SECONDS:
                self.logger.error(f"🚨 GLOBAL TIMEOUT! Sistem 180s sonra otomatik kapanıyor")
                return False
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_phase_2_new(self):
        """Faz 2: 40 metre daha ileri (toplam 50m)"""
        self.logger.info("📍 FAZ 2: 40 metre daha ileri (toplam 50m olacak)")
        self.current_phase = MissionPhase.PHASE_2
        self.system_status.set_phase(MissionPhase.PHASE_2)
        self.phase_timer.start()

        # GELİŞMİŞ MESAFE HESAPLAYICIYI SIFIRLA (Faz 2 için yeni hız profili)
        _advanced_calculator.reset()
        self.logger.info("🔄 Gelişmiş mesafe hesaplayıcı sıfırlandı - Faz 2")
        self.logger.info(f"🎯 Hedef: 40m, Hız: {SPEED_FAST} PWM, Mod: {DISTANCE_CALC_MODE}")
        
        # Hedef değerler
        target_distance = 40.0  # 40 metre daha
        speed_pwm = SPEED_FAST  # Daha hızlı
        
        # Aynı derinlikte devam et
        if self.initial_depth:
            self.stabilizer.set_target_depth(self.initial_depth)
        
        self.current_speed_pwm = speed_pwm
        
        # Stabilizasyonu aktif et (güvenlik kontrolü)
        if not self.stabilizer.stabilization_active:
            self.stabilizer.enable_stabilization()
            self.logger.info("⚠️ Faz 2'de stabilizasyon yeniden aktif edildi")
        
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
                
            # GELİŞMİŞ MESAFE HESAPLAMA
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                if speed_pwm is not None and phase_time is not None:
                    # Gelişmiş mesafe hesaplaması
                    estimated_distance = estimate_distance(speed_pwm, phase_time)

                    # Hız bilgilerini al (gelişmiş moddaysa)
                    current_speed = 0.0
                    if DISTANCE_CALC_MODE == "ADVANCED":
                        try:
                            current_speed = _advanced_calculator.last_speed
                        except:
                            current_speed = 0.0

                    phase_2_distance = estimated_distance
                    self.total_distance_traveled = self.phase1_distance + phase_2_distance

                    # Detaylı mesafe logu
                    total_forward = self.phase1_distance + phase_2_distance
                    self.logger.info(f"📏 Faz 2 Mesafe: {phase_2_distance:.2f}m/{target_distance}m, "
                                   f"Toplam: {total_forward:.2f}m/50m, "
                                   f"Hız: {current_speed:.1f}m/s, Süre: {phase_time:.1f}s, "
                                   f"PWM: {speed_pwm}")
                else:
                    estimated_distance = 0.0
                    phase_2_distance = 0.0
                    self.logger.warning("Faz 2 mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu - D300 kesilirse sistem durmasın
                depth_result = self.sensors.depth.get_depth_safe("PHASE_2")
                current_depth, connection_status, fallback_used = depth_result
                
                # D300 kesintisi durumunda uyarı ver ama devam et
                if connection_status in ["EMERGENCY_PHASE1", "TEMPORARY_ISSUE"]:
                    self.logger.warning(f"⚠️ D300 bağlantı sorunu ({connection_status}) - tahmine dayalı devam ediliyor")
                    # D300'ü yeniden almaya çalış
                    try:
                        self.sensors.depth.reconnect_attempt()
                    except:
                        pass
                
                depth_str = f"{current_depth:.1f}m" if current_depth else f"~{self.initial_depth:.1f}m"
                status_indicator = "⚠️" if fallback_used else "✅"
                
                total_forward = self.phase1_distance + phase_2_distance
                self.logger.info(f"Faz 2 - Mesafe: {phase_2_distance:.1f}m/{target_distance}m, "
                               f"Toplam: {total_forward:.1f}m/50m, "
                               f"Derinlik: {depth_str} {status_indicator}")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if phase_2_distance >= target_distance:
                self.phase2_distance = phase_2_distance
                total_forward = self.phase1_distance + self.phase2_distance
                self.logger.info(f"✓ Faz 2 tamamlandı: {self.phase2_distance:.1f}m (Toplam ileri: {total_forward:.1f}m)")
                break
                
            # Global görev timeout kontrolü (sadece bu)
            if self.mission_timer.elapsed() > MISSION_TIMEOUT_SECONDS:
                self.logger.error(f"🚨 GLOBAL TIMEOUT! Sistem 180s sonra otomatik kapanıyor")
                return False
                
            time.sleep(0.02)  # 50Hz
            
        return True
        
    def _execute_u_turn(self):
        """Faz 3: U dönüşü (180°)"""
        self.logger.info("🔄 FAZ 3: U dönüşü (180°)")
        self.current_phase = MissionPhase.TURNING
        self.system_status.set_phase(MissionPhase.TURNING)
        
        # Motoru yavaşlat/durdur
        self.motion.stop()
        time.sleep(4)  # 4 saniye duraklaması için bekle
        
        # 180° U dönüşü yap
        success = self.stabilizer.turn_180_degrees(timeout=30)
        
        if success:
            self.logger.info("✓ U dönüşü tamamlandı")
            return True
        else:
            self.logger.error("✗ U dönüşü başarısız!")
            return False
            
    def _execute_return_new(self):
        """Faz 4: 50 metre geri dönüş"""
        self.logger.info("🔙 FAZ 4: 50 metre geri dönüş")
        self.current_phase = MissionPhase.RETURN
        self.system_status.set_phase(MissionPhase.RETURN)
        self.phase_timer.start()

        # GELİŞMİŞ MESAFE HESAPLAYICIYI SIFIRLA (Geri dönüş için)
        _advanced_calculator.reset()
        self.logger.info("🔄 Gelişmiş mesafe hesaplayıcı sıfırlandı - Geri Dönüş")
        self.logger.info(f"🎯 Hedef: 50m, Hız: {SPEED_FAST} PWM, Mod: {DISTANCE_CALC_MODE}")
        
        # Hedef değerler - tam olarak 50 metre geri
        return_distance = 50.0  # Sabit 50 metre
        speed_pwm = SPEED_FAST
        
        # Aynı derinlikte devam et
        if self.initial_depth:
            self.stabilizer.set_target_depth(self.initial_depth)
        
        self.current_speed_pwm = speed_pwm
        
        # Stabilizasyonu aktif et (U dönüş sonrası güvenlik kontrolü)
        if not self.stabilizer.stabilization_active:
            self.stabilizer.enable_stabilization()
            self.logger.info("✅ Faz 4'te stabilizasyon yeniden aktif edildi (U dönüş sonrası)")
        
        self.logger.info(f"Geri dönüş hedefi: {return_distance}m")
        
        # İleri hareket başlat (180° döndük, yön ters oldu)
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
                
            # GELİŞMİŞ MESAFE HESAPLAMA
            current_time = time.time()
            if current_time - last_distance_check >= 1.0:  # Her saniye
                phase_time = self.phase_timer.elapsed()
                if speed_pwm is not None and phase_time is not None:
                    # Gelişmiş mesafe hesaplaması
                    estimated_distance = estimate_distance(speed_pwm, phase_time)

                    # Hız bilgilerini al (gelişmiş moddaysa)
                    current_speed = 0.0
                    if DISTANCE_CALC_MODE == "ADVANCED":
                        try:
                            current_speed = _advanced_calculator.last_speed
                        except:
                            current_speed = 0.0

                    return_distance_traveled = estimated_distance

                    # Detaylı mesafe logu
                    remaining = return_distance - return_distance_traveled
                    self.logger.info(f"📏 Geri Dönüş: {return_distance_traveled:.2f}m/{return_distance}m, "
                                   f"Kalan: {remaining:.2f}m, "
                                   f"Hız: {current_speed:.1f}m/s, Süre: {phase_time:.1f}s, "
                                   f"PWM: {speed_pwm}")
                else:
                    estimated_distance = 0.0
                    return_distance_traveled = 0.0
                    self.logger.warning("Geri dönüş mesafe hesaplama için gerekli veriler eksik")
                
                # Durum raporu - D300 kesilirse sistem durmasın
                depth_result = self.sensors.depth.get_depth_safe("RETURN")
                current_depth, connection_status, fallback_used = depth_result
                
                # D300 kesintisi durumunda uyarı ver ama devam et
                if connection_status in ["EMERGENCY_PHASE1", "TEMPORARY_ISSUE"]:
                    self.logger.warning(f"⚠️ D300 bağlantı sorunu ({connection_status}) - tahmine dayalı devam ediliyor")
                    # D300'ü yeniden almaya çalış
                    try:
                        self.sensors.depth.reconnect_attempt()
                    except:
                        pass
                
                depth_str = f"{current_depth:.1f}m" if current_depth else f"~{self.initial_depth:.1f}m"
                status_indicator = "⚠️" if fallback_used else "✅"
                
                # Son 6-7 metre kontrolü - kademeli yüzeye çıkış
                remaining = return_distance - return_distance_traveled
                if remaining <= 7.0 and not self.surface_approach_started:
                    self.logger.info("🌊 Son 7 metre! Kademeli yüzeye çıkış başlıyor...")
                    self.surface_approach_started = True
                    # Kademeli yüzeye çıkış başlat
                    self._start_gradual_surface_approach(remaining)
                
                self.logger.info(f"Geri dönüş - Mesafe: {return_distance_traveled:.1f}m/{return_distance}m, "
                               f"Kalan: {remaining:.1f}m, "
                               f"Derinlik: {depth_str} {status_indicator}")
                
                last_distance_check = current_time
                
            # Hedef mesafeye ulaştık mı?
            if return_distance_traveled >= return_distance:
                self.return_distance = return_distance_traveled
                self.logger.info(f"✓ Geri dönüş tamamlandı: {self.return_distance:.1f}m")
                break
                
            # Global görev timeout kontrolü (sadece bu)
            if self.mission_timer.elapsed() > MISSION_TIMEOUT_SECONDS:
                self.logger.error(f"🚨 GLOBAL TIMEOUT! Sistem 180s sonra otomatik kapanıyor")
                return False
                
            time.sleep(0.02)  # 50Hz
            
        return True
    
    def _start_gradual_surface_approach(self, remaining_distance):
        """Son 6-7m kala kademeli yüzeye çıkış başlat"""
        try:
            self.logger.info(f"🔄 Kademeli yüzeye çıkış başlatılıyor - Kalan mesafe: {remaining_distance:.1f}m")
            
            # Motor gücünü kademeli azalt
            if remaining_distance <= 7.0:
                # 7m kaldığında motor gücünü %80'e düşür
                reduced_power = int(SPEED_FAST * 0.8)
                self.motion.forward(reduced_power)
                self.logger.info(f"Motor gücü %80'e düşürüldü: {reduced_power}")
                
            if remaining_distance <= 5.0:
                # 5m kaldığında motor gücünü %60'a düşür ve pitch ver
                reduced_power = int(SPEED_FAST * 0.6)
                self.motion.forward(reduced_power)
                self._apply_gradual_pitch_up(0.1)  # Hafif pitch yukarı
                self.logger.info(f"Motor gücü %60'a düşürüldü ve hafif pitch verildi")
                
            if remaining_distance <= 3.0:
                # 3m kaldığında motor gücünü %40'a düşür ve daha fazla pitch
                reduced_power = int(SPEED_FAST * 0.4)
                self.motion.forward(reduced_power)
                self._apply_gradual_pitch_up(0.2)  # Daha fazla pitch yukarı
                self.logger.info(f"Motor gücü %40'a düşürüldü ve pitch artırıldı")
                
        except Exception as e:
            self.logger.error(f"Kademeli yüzeye çıkış hatası: {e}")
    
    def _apply_gradual_pitch_up(self, pitch_adjustment):
        """Kademeli pitch yukarı uygula"""
        try:
            # Mevcut hedef derinliği al
            current_target = getattr(self.stabilizer, 'target_depth', self.initial_depth)
            
            # Hedef derinliği kademeli azalt (yüzeye doğru)
            new_target = max(0.5, current_target - pitch_adjustment)
            self.stabilizer.set_target_depth(new_target)
            
            self.logger.info(f"Hedef derinlik: {current_target:.2f}m -> {new_target:.2f}m")
            
        except Exception as e:
            self.logger.warning(f"Pitch ayarlama hatası: {e}")
        
    def _execute_controlled_surfacing(self):
        """Faz 5: Son 2m kala kontrollü yüzeye çıkış"""
        self.logger.info("🌊 FAZ 5: Kontrollü yüzeye çıkış")
        self.current_phase = MissionPhase.SURFACING
        self.system_status.set_phase(MissionPhase.SURFACING)
        self.phase_timer.start()
        
        # Motoru durdur
        self.motion.stop()
        time.sleep(1)
        
        # Mevcut derinliği al - D300 kesilirse tahmine dayalı devam et
        try:
            sensor_data = self.sensors.get_all_sensor_data()
            depth_data = sensor_data['depth']
            if depth_data['is_valid']:
                current_depth = depth_data['depth_m']
                self.logger.info(f"Mevcut derinlik: {current_depth:.2f}m (ölçülen)")
            else:
                current_depth = self.initial_depth if self.initial_depth else 2.25
                self.logger.warning(f"D300 okunamadı, tahmine dayalı derinlik: {current_depth:.2f}m")
        except Exception as e:
            self.logger.error(f"Derinlik okuma hatası: {e}")
            current_depth = self.initial_depth if self.initial_depth else 2.25
        
        # Kontrollü yüzeye çıkış - derinlik azaltma
        target_depth_step = 0.5  # Her seferinde 0.5m yüksel
        surface_threshold = 0.3   # 0.3m'de yüzeye ulaştığını kabul et
        
        while current_depth > surface_threshold:
            # Buton kontrolü
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("Yüzeye çıkış kullanıcı tarafından durduruldu")
                return False
            
            # Yeni hedef derinlik hesapla
            new_target = max(0.0, current_depth - target_depth_step)
            self.logger.info(f"Hedef derinlik: {current_depth:.2f}m -> {new_target:.2f}m")
            
            # Stabilizasyonu aktif et ve hedef derinliği ayarla
            self.stabilizer.enable_stabilization()
            self.stabilizer.set_target_depth(new_target)
            
            # Kontrollü yükselme için süre ver
            step_start = time.time()
            while time.time() - step_start < 5.0:  # 5 saniye bekle
                if not self.stabilizer.update_stabilization():
                    self.logger.warning("Stabilizasyon güncellenemedi")
                    
                # Derinlik kontrolü
                try:
                    sensor_data = self.sensors.get_all_sensor_data()
                    depth_data = sensor_data['depth']
                    if depth_data['is_valid']:
                        current_depth = depth_data['depth_m']
                        depth_error = abs(new_target - current_depth)
                        
                        # Hedefe yaklaştık mı?
                        if depth_error < 0.2:  # 20cm tolerans
                            self.logger.info(f"Hedef derinliğe ulaşıldı: {current_depth:.2f}m")
                            break
                            
                except Exception as e:
                    self.logger.warning(f"Derinlik okuma hatası: {e}")
                    
                time.sleep(0.1)
            
            # Yeni derinliği güncelle
            try:
                sensor_data = self.sensors.get_all_sensor_data()
                depth_data = sensor_data['depth']
                if depth_data['is_valid']:
                    current_depth = depth_data['depth_m']
                    self.logger.info(f"Güncel derinlik: {current_depth:.2f}m")
            except:
                current_depth = new_target  # Tahmin olarak kullan
            
            # Global görev timeout kontrolü (sadece bu)
            if self.mission_timer.elapsed() > MISSION_TIMEOUT_SECONDS:
                self.logger.error(f"🚨 GLOBAL TIMEOUT! Sistem 180s sonra otomatik kapanıyor")
                break  # Yüzeye çıkışta güvenlik için break
        
        # Son kontrol - yüzeye çıkış tamamlandı mı?
        if current_depth <= surface_threshold:
            self.logger.info(f"✅ Yüzeye başarıyla çıkıldı! Derinlik: {current_depth:.2f}m")
            
            # Stabilizasyonu deaktif et
            self.stabilizer.disable_stabilization()
            
            # Tüm servoları nötrle
            self.stabilizer.servo_controller.neutral_all_servos()
            
            return True
        else:
            self.logger.warning(f"⚠️ Yüzeye tam olarak çıkılamadı. Mevcut derinlik: {current_depth:.2f}m")
            return True  # Kritik hata değil, devam et
    
    def _execute_system_shutdown(self):
        """Faz 6: Sistemlerin yazılımsal kapatılması"""
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
            
            # 4. MAVLink bağlantısını kapat
            self.logger.info("4️⃣ MAVLink bağlantısı kapatılıyor...")
            try:
                if hasattr(self.mavlink, 'close'):
                    self.mavlink.close()
            except Exception as e:
                self.logger.warning(f"MAVLink kapatma hatası: {e}")
            
            # 5. GPIO temizliği
            self.logger.info("5️⃣ GPIO temizliği yapılıyor...")
            if GPIO_AVAILABLE:
                try:
                    # Son uyarı sinyali
                    for _ in range(3):
                        GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                        GPIO.output(GPIO_LED_RED, GPIO.HIGH)
                        time.sleep(0.3)
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
            
            # 6. Sensör bağlantılarını kapat
            self.logger.info("6️⃣ Sensör bağlantıları kapatılıyor...")
            try:
                if hasattr(self.sensors, 'cleanup'):
                    self.sensors.cleanup()
            except Exception as e:
                self.logger.warning(f"Sensör temizlik hatası: {e}")
            
            # 7. Zamanlayıcıları durdur
            self.logger.info("7️⃣ Zamanlayıcılar durduruluyor...")
            try:
                if self.mission_timer.is_running():
                    self.mission_timer.pause()
                if self.phase_timer.is_running():
                    self.phase_timer.pause()
                if self.waiting_timer.is_running():
                    self.waiting_timer.pause()
            except Exception as e:
                self.logger.warning(f"Zamanlayıcı durdurma hatası: {e}")
            
            # 8. Son durum raporu
            total_mission_time = self.mission_timer.elapsed() if self.mission_timer else 0
            total_distance = self.phase1_distance + self.phase2_distance + self.return_distance
            
            self.logger.info("📊 GÖREV ÖZET RAPORU:")
            self.logger.info(f"   • Toplam süre: {format_time(total_mission_time)}")
            self.logger.info(f"   • Toplam mesafe: {total_distance:.1f}m")
            self.logger.info(f"   • Faz 1: {self.phase1_distance:.1f}m")
            self.logger.info(f"   • Faz 2: {self.phase2_distance:.1f}m") 
            self.logger.info(f"   • Geri dönüş: {self.return_distance:.1f}m")
            self.logger.info(f"   • Başlangıç derinliği: {self.initial_depth:.1f}m")
            
            self.logger.info("✅ Sistem kapatma işlemi tamamlandı")
            self.logger.info("🏁 GÖREV 1 TAMAMEN BİTİRİLDİ - SİSTEM GÜVENLİ DURUMDA")
            
            return True
            
        except Exception as e:
            self.logger.error(f"Sistem kapatma hatası: {e}")
            # Kritik hata olsa bile devam et
            return True
            

            
    def _emergency_abort(self):
        """Genel acil durum prosedürü"""
        self.logger.error("🚨 ACİL DURUM - Görev iptal ediliyor!")
        
        try:
            # Motoru durdur
            self.motion.stop()
            
            # Stabilizasyonu durdur
            self.stabilizer.disable_stabilization()
            
            # Sistem durumunu ayarla
            self.system_status.emergency_stop()
            
            # Görev durumunu ayarla
            self.mission_completed = True
            self.mission_success = False
            self.current_phase = MissionPhase.EMERGENCY
            
        except Exception as e:
            self.logger.error(f"Acil durum prosedürü hatası: {e}")
            
    def get_mission_status(self):
        """Görev durumu raporu"""
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
            if sensor_data and sensor_data['depth'] and sensor_data['depth']['is_valid']:
                depth_val = sensor_data['depth']['depth_m']
                if depth_val is not None:
                    status['current_depth'] = depth_val
            if sensor_data and sensor_data['attitude']:
                heading_val = sensor_data['attitude'].get('yaw_relative_deg')
                if heading_val is not None:
                    status['current_heading'] = heading_val
        except Exception as e:
            self.logger.warning(f"Sensör durumu alma hatası: {e}")
            
        return status
        
    def log_mission_status(self):
        """Görev durumunu logla"""
        status = self.get_mission_status()
        
        mission_time_str = format_time(status['mission_time'])
        phase_time_str = format_time(status['phase_time'])
        
        self.logger.info(f"Görev Durumu - Faz: {status['phase']}, "
                        f"Süre: {mission_time_str}, "
                        f"Mesafe: {status['total_distance']:.1f}m")
                        
        if 'current_depth' in status and status['current_depth'] is not None:
            self.logger.info(f"Derinlik: {status['current_depth']:.2f}m")
            
        if 'current_heading' in status and status['current_heading'] is not None:
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

def run_mission_1(mavlink_connection, system_status, logger, sensor_manager=None):
    """Görev 1'i çalıştır (dış arayüz fonksiyonu)"""
    mission = Mission1Controller(mavlink_connection, system_status, logger, sensor_manager)
    
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
