#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN - Su Altı Roket Aracı (SARA) Ana Program
Raspberry Pi'de otomatik olarak çalışacak ana dosya
90 saniye güvenlik gecikmesi, görev yönetimi ve kontrol döngüsü
"""

import sys
import time
import signal
import threading
from pymavlink import mavutil

# Kendi modüllerimizi import et
from config import *
from utils import init_system_status, wait_with_button_check, safe_gpio_cleanup
from sensors import SensorManager
from control import StabilizationController
from mission1 import run_mission_1
from mission2 import run_mission_2

class SaraMainController:
    """SARA Ana Kontrol Sistemi"""
    
    def __init__(self):
        # Sistem durumu başlat
        self.system_status = init_system_status()
        self.logger = self.system_status.logger
        
        # MAVLink bağlantısı
        self.mavlink = None
        
        # Görev durumu
        self.current_mission = None
        self.mission_running = False
        self.system_running = True
        
        # Thread kontrolü
        self.main_thread = None
        
        self.logger.info("SARA Ana Kontrolcü başlatıldı")
        
    def setup_mavlink(self, retries=5):
        """MAVLink bağlantısını güvenilir şekilde kur"""
        self.logger.info("MAVLink bağlantısı kuruluyor...")
        
        for attempt in range(retries):
            try:
                self.logger.info(f"Bağlantı denemesi {attempt+1}/{retries}")
                
                # Port seçimi (Linux/Windows uyumlu)
                port = MAVLINK_PORT
                if sys.platform.startswith('win'):
                    port = MAVLINK_PORT_WIN
                    
                self.mavlink = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
                
                # Heartbeat bekle
                self.logger.info("Heartbeat bekleniyor...")
                if not self.mavlink.wait_heartbeat(timeout=15):
                    raise Exception("Pixhawk heartbeat alınamadı!")
                    
                self.logger.info("✓ MAVLink bağlantısı başarılı")
                self.logger.info(f"  Sistem ID: {self.mavlink.target_system}")
                self.logger.info(f"  Bileşen ID: {self.mavlink.target_component}")
                
                # Sensör bağlantısını test et
                if self._test_sensor_connectivity():
                    # Veri akışı istekleri
                    self._request_data_streams()
                    return True
                else:
                    raise Exception("Sensör bağlantı testi başarısız")
                    
            except Exception as e:
                self.logger.warning(f"Bağlantı denemesi {attempt+1} başarısız: {e}")
                if attempt < retries - 1:
                    self.logger.info("2 saniye sonra yeniden denenecek...")
                    time.sleep(2)
                    
        self.logger.error("❌ MAVLink bağlantısı kurulamadı!")
        return False
        
    def _test_sensor_connectivity(self):
        """Sensör bağlantısını test et"""
        try:
            # D300 sensörü için hızlı test
            sensors = SensorManager(self.mavlink, self.logger)
            time.sleep(1)  # Sensörlerin hazırlanması için bekle
            
            # D300 test
            depth_data = sensors.depth.read_raw_data()
            d300_ok = depth_data[0] is not None
            
            # Attitude test
            attitude_data = sensors.attitude.get_attitude(timeout=2.0)
            attitude_ok = attitude_data is not None
            
            self.logger.info(f"Sensör testi: D300={d300_ok}, Attitude={attitude_ok}")
            
            # En azından D300 çalışmalı
            return d300_ok
            
        except Exception as e:
            self.logger.error(f"Sensör bağlantı testi hatası: {e}")
            return False
            
    def _request_data_streams(self):
        """Gerekli veri akışlarını iste"""
        try:
            # ATTITUDE mesajlarını düzenli olarak iste
            self.mavlink.mav.request_data_stream_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
                10,  # 10 Hz
                1    # Başlat
            )
            
            # SYS_STATUS mesajlarını iste
            self.mavlink.mav.request_data_stream_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,  # SYS_STATUS
                2,   # 2 Hz
                1    # Başlat
            )
            
            self.logger.info("Veri akışları istendi")
            
        except Exception as e:
            self.logger.warning(f"Veri akışı istek hatası: {e}")
            
    def wait_for_start_button(self):
        """Başlatma butonu için bekle"""
        self.logger.info("🔘 Başlatma butonu bekleniyor...")
        self.system_status.set_phase(MissionPhase.WAITING)
        
        # LED yanıp sönsün (bekleme modunda)
        self.system_status.led.blink(0.5)
        
        while self.system_running:
            button_action = self.system_status.check_start_button()
            
            if button_action == "start":
                self.logger.info("✅ Başlatma butonu basıldı!")
                self.system_status.led.turn_on()
                self.system_status.buzzer.beep_pattern(BUZZER_STARTUP)
                time.sleep(2)  # Buton bouncing önlemi
                return True
                
            time.sleep(0.1)
            
        return False
        
    def countdown_phase(self):
        """90 saniye güvenlik geri sayımı"""
        self.logger.info(f"⏱️  {ARMING_DELAY_SECONDS} saniye güvenlik geri sayımı başlıyor...")
        self.system_status.set_phase(MissionPhase.CALIBRATION)
        
        # Geri sayım buzzer'ını başlat
        self.system_status.buzzer.countdown_buzzer(ARMING_DELAY_SECONDS)
        
        countdown_start = time.time()
        last_announce = 0
        
        while time.time() - countdown_start < ARMING_DELAY_SECONDS:
            if not self.system_running:
                return False
                
            elapsed = time.time() - countdown_start
            remaining = ARMING_DELAY_SECONDS - elapsed
            
            # Her 10 saniyede durumu bildir
            if int(remaining) % 10 == 0 and int(remaining) != last_announce:
                self.logger.info(f"⏱️  Arming'e {int(remaining)} saniye...")
                last_announce = int(remaining)
                
            # Buton kontrol - eğer tekrar basıldıysa başa dön
            button_action = self.system_status.check_start_button()
            if button_action == "stop":
                self.logger.info("🔘 Buton tekrar basıldı, başa dönülüyor...")
                self.system_status.buzzer.stop_buzzer()
                self.system_status.buzzer.beep(0.5)
                time.sleep(2)
                return False
                
            time.sleep(0.1)
            
        self.logger.info("✅ Güvenlik süresi tamamlandı!")
        self.system_status.buzzer.mission_start_buzzer()
        return True
        
    def _perform_calibrations(self):
        """Kalibrasyon işlemleri (90 saniye içinde)"""
        self.logger.info("🔧 Kalibrasyonlar yapılıyor...")
        
        try:
            # Sensör yöneticisi oluştur
            sensors = SensorManager(self.mavlink, self.logger)
            
            # Sensör kalibrasyonları
            calibration_results = sensors.calibrate_all()
            
            success_count = sum(calibration_results.values())
            total_count = len(calibration_results)
            
            if success_count == total_count:
                self.logger.info("✅ Tüm kalibrasyonlar başarılı")
            else:
                self.logger.warning(f"⚠️  Kalibrasyon: {success_count}/{total_count} başarılı")
                
            # Sistem sağlığı kontrolü
            health = sensors.check_sensor_health()
            if health['overall_healthy']:
                self.logger.info("✅ Sistem sağlık kontrolü: TAMAM")
            else:
                self.logger.warning("⚠️  Sistem sağlık kontrolü: PROBLEM TESPİT EDİLDİ")
                
        except Exception as e:
            self.logger.error(f"Kalibrasyon hatası: {e}")
            
    def _perform_startup_calibrations(self):
        """Sistem başlangıcında yapılacak otomatik kalibrasyonlar (kart kıpırdamaz)"""
        self.logger.info("🔧 Başlangıç kalibrasyonları başlıyor...")
        
        try:
            # Sensör yöneticisini başlat
            sensors = SensorManager(self.mavlink, self.logger)
            
            # 1. Gyroscope kalibrasyonu (kart hareketsiz)
            self.logger.info("📐 Gyroscope kalibrasyonu yapılıyor...")
            self._calibrate_gyroscope()
            
            # 2. Barometer kalibrasyonu (kart hareketsiz)  
            self.logger.info("🌡️ Barometer kalibrasyonu yapılıyor...")
            self._calibrate_barometer()
            
            # 3. D300 derinlik sensörü kalibrasyonu (havada)
            self.logger.info("🌊 D300 derinlik sensörü kalibrasyonu...")
            d300_success = sensors.depth.calibrate_surface(
                duration=D300_CALIB_DURATION_SEAWATER, 
                use_water_surface=False  # Havada kalibrasyon
            )
            
            if d300_success:
                self.logger.info("✅ D300 kalibrasyonu tamamlandı")
            else:
                self.logger.warning("⚠️ D300 kalibrasyonu başarısız")
                
            self.logger.info("✅ Tüm başlangıç kalibrasyonları tamamlandı")
            
        except Exception as e:
            self.logger.error(f"Başlangıç kalibrasyon hatası: {e}")
            
    def _calibrate_gyroscope(self):
        """Gyroscope kalibrasyonu - kart hareketsiz"""
        try:
            self.logger.info("Gyroscope kalibrasyonu: 10 saniye hareketsiz bekleyin...")
            
            # Gyroscope bias hesaplama
            gyro_samples = []
            start_time = time.time()
            
            while time.time() - start_time < 10.0:  # 10 saniye
                # SCALED_IMU mesajından gyroscope verisi al
                msg = self.mavlink.recv_match(type='SCALED_IMU', blocking=False, timeout=0.1)
                if msg:
                    gyro_samples.append({
                        'x': msg.xgyro,  # mrad/s
                        'y': msg.ygyro,
                        'z': msg.zgyro
                    })
                time.sleep(0.05)
                
            if len(gyro_samples) > 50:
                # Ortalama bias hesapla
                gyro_bias_x = sum(s['x'] for s in gyro_samples) / len(gyro_samples)
                gyro_bias_y = sum(s['y'] for s in gyro_samples) / len(gyro_samples)
                gyro_bias_z = sum(s['z'] for s in gyro_samples) / len(gyro_samples)
                
                self.logger.info(f"✅ Gyroscope bias: X={gyro_bias_x:.2f}, Y={gyro_bias_y:.2f}, Z={gyro_bias_z:.2f} mrad/s")
            else:
                self.logger.warning("⚠️ Gyroscope kalibrasyon verisi yetersiz")
                
        except Exception as e:
            self.logger.error(f"Gyroscope kalibrasyon hatası: {e}")
            
    def _calibrate_barometer(self):
        """Barometer kalibrasyonu - kart hareketsiz"""
        try:
            self.logger.info("Barometer kalibrasyonu: 5 saniye bekleyin...")
            
            pressure_samples = []
            start_time = time.time()
            
            while time.time() - start_time < 5.0:  # 5 saniye
                # SCALED_PRESSURE mesajından basınç verisi al
                msg = self.mavlink.recv_match(type='SCALED_PRESSURE', blocking=False, timeout=0.1)
                if msg:
                    pressure_samples.append(msg.press_abs)  # hPa/mbar
                time.sleep(0.1)
                
            if len(pressure_samples) > 20:
                # Ortalama referans basınç
                ref_pressure = sum(pressure_samples) / len(pressure_samples)
                self.logger.info(f"✅ Barometer referans basınç: {ref_pressure:.2f} mbar")
            else:
                self.logger.warning("⚠️ Barometer kalibrasyon verisi yetersiz")
                
        except Exception as e:
            self.logger.error(f"Barometer kalibrasyon hatası: {e}")
            
    def select_mission(self):
        """Görev seçimi (gelecekte genişletilebilir)"""
        # Şu an için sabit olarak Görev 1
        # Gelecekte buton kombinasyonları ile seçim yapılabilir
        return 1
        
    def run_selected_mission(self, mission_number):
        """Seçilen görevi çalıştır"""
        self.mission_running = True
        success = False
        
        try:
            if mission_number == 1:
                self.logger.info("🎯 GÖREV 1 SEÇİLDİ: Seyir ve Başlangıç Noktasına Dönüş")
                self.current_mission = "Görev 1"
                success = run_mission_1(self.mavlink, self.system_status, self.logger)
                
            elif mission_number == 2:
                self.logger.info("🎯 GÖREV 2 SEÇİLDİ: Roket Fırlatma")
                self.current_mission = "Görev 2"
                success = run_mission_2(self.mavlink, self.system_status, self.logger)
                
            else:
                self.logger.error(f"❌ Geçersiz görev numarası: {mission_number}")
                success = False
                
        except Exception as e:
            self.logger.error(f"Görev çalıştırma hatası: {e}")
            success = False
            
        finally:
            self.mission_running = False
            
        return success
        
    def mission_complete_sequence(self, success):
        """Görev tamamlama sekansı"""
        if success:
            self.logger.info("🏆 GÖREV BAŞARIYLA TAMAMLANDI!")
            self.system_status.buzzer.mission_end_buzzer()
            
            # Başarı için LED pattern
            self.system_status.led.blink(0.2, 10)
            
        else:
            self.logger.error("💥 GÖREV BAŞARISIZ!")
            self.system_status.buzzer.emergency_buzzer()
            
            # Başarısızlık için LED pattern
            self.system_status.led.blink(0.1, 20)
            
        # Sonuç bekleme
        time.sleep(5)
        
    def main_loop(self):
        """Ana program döngüsü"""
        self.logger.info("=" * 60)
        self.logger.info("SU ALTI ROKET ARACI (SARA) - ANA PROGRAM")
        self.logger.info("Teknofest 2025 Su Altı Roket Aracı Yarışması")
        self.logger.info("=" * 60)
        
        try:
            # MAVLink bağlantısını kur
            if not self.setup_mavlink():
                self.logger.error("❌ MAVLink bağlantısı kurulamadı!")
                return False
                
            # Sistem başlangıç kalibrasyonlarını yap (OTOMATIK)
            self.logger.info("🔧 Sistem başlangıç kalibrasyonları yapılıyor...")
            self._perform_startup_calibrations()
                
            # Ana döngü
            mission_count = 0
            
            while self.system_running:
                mission_count += 1
                self.logger.info(f"\n🔄 DÖNGÜ {mission_count} BAŞLIYOR...")
                
                # 1. Başlatma butonu bekle
                if not self.wait_for_start_button():
                    break
                    
                # 2. 90 saniye geri sayım
                if not self.countdown_phase():
                    continue  # Buton tekrar basıldı, başa dön
                    
                # 3. Görev seçimi
                mission_number = self.select_mission()
                
                # 4. Seçilen görevi çalıştır
                self.logger.info(f"🚀 Görev {mission_number} başlatılıyor...")
                success = self.run_selected_mission(mission_number)
                
                # 5. Görev tamamlama sekansı
                self.mission_complete_sequence(success)
                
                # 6. Yeni görev için hazırlık
                self.logger.info("🔄 Yeni görev için başlatma butonu bekleniyor...")
                time.sleep(2)
                
            self.logger.info("📴 Ana döngü sonlandırıldı")
            return True
            
        except KeyboardInterrupt:
            self.logger.info("\n⏹️  Kullanıcı tarafından durduruldu (Ctrl+C)")
            return True
            
        except Exception as e:
            self.logger.error(f"💥 Ana döngü hatası: {e}")
            return False
            
    def emergency_shutdown(self):
        """Acil durum kapatma"""
        self.logger.error("🚨 ACİL DURUM KAPATMA!")
        
        try:
            self.system_running = False
            
            if self.mission_running:
                self.logger.info("Çalışan görev durdruluyor...")
                
            # Sistem durumunu acil duruma geçir
            self.system_status.emergency_stop()
            
            # MAVLink üzerinden acil durum komutları
            if self.mavlink:
                try:
                    # Tüm servoları nötrle
                    for channel in [SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT]:
                        self.mavlink.mav.command_long_send(
                            self.mavlink.target_system,
                            self.mavlink.target_component,
                            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                            0, float(channel), float(PWM_NEUTRAL), 0, 0, 0, 0, 0
                        )
                        
                    # Motoru durdur
                    self.mavlink.mav.command_long_send(
                        self.mavlink.target_system,
                        self.mavlink.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, float(MOTOR_MAIN), float(MOTOR_STOP), 0, 0, 0, 0, 0
                    )
                    
                    self.logger.info("Acil durum komutları gönderildi")
                    
                except Exception as e:
                    self.logger.error(f"Acil durum komut hatası: {e}")
                    
        except Exception as e:
            self.logger.error(f"Acil durum kapatma hatası: {e}")
            
    def cleanup(self):
        """Sistem temizliği"""
        self.logger.info("🧹 Sistem temizleniyor...")
        
        try:
            # Sistem durumunu temizle
            if self.system_status:
                self.system_status.cleanup()
                
            # GPIO temizle
            safe_gpio_cleanup()
            
            self.logger.info("✅ Sistem temizliği tamamlandı")
            
        except Exception as e:
            self.logger.error(f"Temizlik hatası: {e}")

# Signal handler'lar
def signal_handler(signum, frame):
    """Signal yakalayıcı (Ctrl+C, SIGTERM vs.)"""
    print("\n🛑 Signal alındı, güvenli kapatma...")
    if 'main_controller' in globals():
        main_controller.emergency_shutdown()
    sys.exit(0)

def main():
    """Ana fonksiyon"""
    global main_controller
    
    # Signal handler'ları ayarla
    signal.signal(signal.SIGINT, signal_handler)   # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # Termination
    
    main_controller = None
    
    try:
        # Ana kontrolcüyü başlat
        main_controller = SaraMainController()
        
        # Ana döngüyü çalıştır
        success = main_controller.main_loop()
        
        if success:
            print("✅ Program normal olarak sonlandı")
        else:
            print("❌ Program hata ile sonlandı")
            sys.exit(1)
            
    except Exception as e:
        print(f"💥 Kritik hata: {e}")
        if main_controller:
            main_controller.emergency_shutdown()
        sys.exit(1)
        
    finally:
        if main_controller:
            main_controller.cleanup()

if __name__ == "__main__":
    main()
