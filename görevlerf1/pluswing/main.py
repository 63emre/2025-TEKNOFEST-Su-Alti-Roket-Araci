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
from mission1 import Mission1Controller
from mission2 import Mission2Controller

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
                ports_to_try = []
                if sys.platform.startswith('win'):
                    ports_to_try = [MAVLINK_PORT_WIN]
                else:
                    ports_to_try = MAVLINK_PORTS  # ["/dev/ttyACM0", "/dev/ttyACM1"]
                
                # Her portu dene
                connection_success = False
                for port in ports_to_try:
                    try:
                        self.logger.info(f"Port deneniyor: {port}")
                        self.mavlink = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
                        
                        # Heartbeat bekle
                        self.logger.info("Heartbeat bekleniyor...")
                        msg = self.mavlink.wait_heartbeat(timeout=3)
                        
                        if msg:
                            self.logger.info(f"✅ MAVLink bağlantısı başarılı! Port: {port}")
                            self.logger.info(f"Sistem ID: {msg.get_srcSystem()}, Tip: {msg.get_type()}")
                            
                            # Target system/component ayarla
                            self.mavlink.target_system = msg.get_srcSystem()
                            self.mavlink.target_component = msg.get_srcComponent()
                            
                            # Veri akışlarını iste
                            self._request_data_streams()
                            connection_success = True
                            break
                        else:
                            self.logger.warning(f"Heartbeat alınamadı: {port}")
                            
                    except Exception as port_error:
                        self.logger.warning(f"Port {port} bağlantı hatası: {port_error}")
                        continue
                        
                if connection_success:
                    return True
                    
            except Exception as e:
                self.logger.error(f"MAVLink kurulum hatası (deneme {attempt+1}): {e}")
                
            if attempt < retries - 1:
                self.logger.info("2 saniye beklenip tekrar denenecek...")
                time.sleep(2)
                
        self.logger.error("❌ MAVLink bağlantısı kurulamadı!")
        return False
        
    def test_sensor_connections(self):
        """Sensör bağlantılarını test et"""
        try:
            # SensorManager oluştur
            sensor_manager = SensorManager(self.mavlink, self.system_status.logger)
            
            # D300 test et
            d300_ok = sensor_manager.test_d300_connection()
            
            # Attitude test et
            attitude_ok = sensor_manager.test_attitude_connection()
            
            self.logger.info(f"Sensör testi: D300={d300_ok}, Attitude={attitude_ok}")
            
            # En azından D300 çalışmalı
            return d300_ok
            
        except Exception as e:
            self.logger.error(f"Sensör bağlantı testi hatası: {e}")
            return False
            
    def auto_calibrate_on_power(self):
        """Güç verildiğinde otomatik kalibrasyon yap - butona basılmadan"""
        if not AUTO_CALIBRATION_ON_POWER:
            self.logger.info("Otomatik kalibrasyon deaktif, manuel kalibrasyon gerekli")
            return self.calibrate_sensors()
            
        try:
            self.logger.info("🔧 Otomatik sensör kalibrasyonu başlatılıyor...")
            
            # Kalibrasyon sinyali başlat
            self.system_status.set_phase(MissionPhase.CALIBRATION)
            self.system_status.buzzer.beep_pattern(BUZZER_CALIBRATION)
            self.system_status.led.blink(LED_CALIBRATION_BLINK)
            
            # SensorManager oluştur
            sensor_manager = SensorManager(self.mavlink, self.system_status.logger)
            
            # Tüm sensörleri kalibre et
            calibration_results = sensor_manager.calibrate_all()
            
            # Sonuçları kontrol et
            depth_ok = calibration_results.get('depth', False)
            attitude_ok = calibration_results.get('attitude', False)
            
            self.logger.info(f"Otomatik kalibrasyon sonuçları: D300={depth_ok}, Attitude={attitude_ok}")
            
            # Sonuç sinyalleri
            if depth_ok:
                self.logger.info("✅ Otomatik sensör kalibrasyonu tamamlandı")
                self.system_status.buzzer.beep_pattern(BUZZER_CALIBRATION_OK)
                self.system_status.led.blink(LED_SUCCESS_SLOW_BLINK, count=2)
                time.sleep(2)  # Sinyal tamamlanması için bekle
                return True
            else:
                self.logger.error("❌ D300 derinlik sensörü otomatik kalibrasyonu başarısız")
                self.system_status.buzzer.beep_pattern(BUZZER_CALIBRATION_FAIL)
                self.system_status.led.blink(LED_EMERGENCY_BLINK, count=10)
                return False
                
        except Exception as e:
            self.logger.error(f"Otomatik kalibrasyon hatası: {e}")
            self.system_status.buzzer.beep_pattern(BUZZER_CALIBRATION_FAIL)
            self.system_status.led.blink(LED_EMERGENCY_BLINK, count=10)
            return False

    def calibrate_sensors(self):
        """Manuel sensör kalibrasyonu yap"""
        try:
            self.logger.info("🔧 Manuel sensör kalibrasyonu başlatılıyor...")
            
            # SensorManager oluştur
            sensor_manager = SensorManager(self.mavlink, self.system_status.logger)
            
            # Tüm sensörleri kalibre et
            calibration_results = sensor_manager.calibrate_all()
            
            # Sonuçları kontrol et
            depth_ok = calibration_results.get('depth', False)
            attitude_ok = calibration_results.get('attitude', False)
            
            self.logger.info(f"Manuel kalibrasyon sonuçları: D300={depth_ok}, Attitude={attitude_ok}")
            
            # En azından D300 kalibrasyonu başarılı olmalı
            if depth_ok:
                self.logger.info("✅ Manuel sensör kalibrasyonu tamamlandı")
                return True
            else:
                self.logger.error("❌ D300 derinlik sensörü manuel kalibrasyonu başarısız")
                return False
                
        except Exception as e:
            self.logger.error(f"Manuel kalibrasyon hatası: {e}")
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
        
        # LED yavaş yanıp sönsün (bekleme modunda)
        self.system_status.led.blink(LED_WAITING_BLINK)
        
        while self.system_running:
            button_action = self.system_status.check_start_button()
            
            if button_action == "start":
                self.logger.info("✅ Başlatma butonu basıldı!")
                self.system_status.led.turn_on()
                self.system_status.buzzer.beep_pattern(BUZZER_MISSION_START)
                time.sleep(2)  # Buton bouncing önlemi
                return True
                
            elif button_action == "stop":
                self.logger.info("🛑 Durdurma butonu basıldı!")
                self.emergency_stop()
                return False
                
            time.sleep(0.1)
            
        return False
        
    def countdown_90_seconds(self):
        """90 saniye güvenlik geri sayımı"""
        self.logger.info("⏱️ 90 saniye güvenlik geri sayımı başlıyor...")
        self.system_status.set_phase(MissionPhase.WAITING)
        
        # LED çok hızlı yanıp sönsün (geri sayım modunda)
        self.system_status.led.blink(LED_COUNTDOWN_BLINK)
        
        # 90 saniye = 10 x (9 kısa bip + 1 uzun bip)
        for group in range(10):
            # 9 kısa bip
            for short_beep in range(9):
                if not self.system_running:
                    return False
                    
                # Buton kontrolü
                button_action = self.system_status.check_start_button()
                if button_action == "stop":
                    self.logger.info("🛑 Geri sayım durduruldu!")
                    self.emergency_stop()
                    return False
                    
                self.system_status.buzzer.beep(BUZZER_COUNTDOWN_SHORT)
                time.sleep(BUZZER_COUNTDOWN_PAUSE)
                
            # 1 uzun bip
            if self.system_running:
                self.system_status.buzzer.beep(BUZZER_COUNTDOWN_LONG)
                remaining_groups = 9 - group
                self.logger.info(f"⏱️ Geri sayım: {remaining_groups * 9} saniye kaldı")
                
        self.logger.info("✅ 90 saniye güvenlik gecikmesi tamamlandı!")
        self.system_status.buzzer.beep_pattern(BUZZER_MISSION_START)
        return True
        
    def run_mission(self, mission_type=1):
        """Görev çalıştır"""
        try:
            self.logger.info(f"🚀 Görev {mission_type} başlıyor...")
            self.mission_running = True
            
            # Sensör manager oluştur
            sensor_manager = SensorManager(self.mavlink, self.logger)
            
            # Stabilizasyon kontrolcüsü oluştur
            stabilization = StabilizationController(self.mavlink, self.logger)
            
            # Görev türüne göre çalıştır
            if mission_type == 1:
                mission_controller = Mission1Controller(self.mavlink, self.system_status, self.logger)
                success = mission_controller.start_mission()
            elif mission_type == 2:
                mission_controller = Mission2Controller(self.mavlink, self.system_status, self.logger)
                success = mission_controller.start_mission()
            else:
                self.logger.error(f"Geçersiz görev türü: {mission_type}")
                return False
                
            if success:
                self.logger.info("🎉 Görev başarıyla tamamlandı!")
                self.system_status.set_phase(MissionPhase.COMPLETED)
            else:
                self.logger.error("❌ Görev başarısız!")
                self.system_status.set_phase(MissionPhase.EMERGENCY)
                
            return success
            
        except Exception as e:
            self.logger.error(f"Görev çalıştırma hatası: {e}")
            self.system_status.set_phase(MissionPhase.EMERGENCY)
            return False
        finally:
            self.mission_running = False
            
    def emergency_stop(self):
        """Acil durdurma prosedürü"""
        self.logger.error("🚨 ACİL DURDURMA AKTİF!")
        self.system_running = False
        self.mission_running = False
        self.system_status.emergency_stop()
        
        # Tüm motorları durdur
        if self.mavlink:
            try:
                # Ana motoru durdur
                self.mavlink.mav.rc_channels_override_send(
                    self.mavlink.target_system,
                    self.mavlink.target_component,
                    *[65535] * 8  # Tüm kanalları serbest bırak
                )
                self.logger.info("Tüm motor kanalları serbest bırakıldı")
            except:
                pass
                
    def signal_handler(self, signum, frame):
        """Signal handler (Ctrl+C vs.)"""
        self.logger.info("Signal yakalandı, güvenli kapatma başlıyor...")
        self.emergency_stop()
        
    def run(self, mission_type=1):
        """Ana çalıştırma döngüsü"""
        try:
            # Signal handler ayarla
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)
            
            self.logger.info("🤖 SARA Su Altı Roket Aracı başlatılıyor...")
            
            # 🆕 GÜÇ VERİLDİĞİNDE OTOMATIK PROSEDÜR
            self.system_status.buzzer.beep_pattern(BUZZER_POWER_ON)
            self.system_status.led.blink(LED_POWER_ON_BLINK, count=3)
            time.sleep(2)  # Başlangıç sinyali için bekle
            
            # 1. MAVLink bağlantısını kur
            if not self.setup_mavlink():
                self.logger.error("MAVLink bağlantısı kurulamadı, çıkılıyor")
                return False
                
            # 2. Sensör bağlantılarını test et
            if not self.test_sensor_connections():
                self.logger.error("Sensör bağlantıları başarısız, çıkılıyor")
                return False
                
            # 🆕 3. OTOMATİK KALIBRASYON (BUTONA BASILMADAN)
            if not self.auto_calibrate_on_power():
                self.logger.error("Otomatik kalibrasyon başarısız, çıkılıyor")
                return False
                
            # 4. Başlatma butonu bekle
            if not self.wait_for_start_button():
                self.logger.info("Başlatma iptal edildi")
                return False
                
            # 5. 90 saniye güvenlik gecikmesi
            if not self.countdown_90_seconds():
                self.logger.info("Geri sayım iptal edildi")
                return False
                
            # 6. Görevi çalıştır
            success = self.run_mission(mission_type)
            
            if success:
                self.logger.info("🏆 SARA görevi başarıyla tamamlandı!")
            else:
                self.logger.error("💥 SARA görevi başarısız!")
                
            return success
            
        except Exception as e:
            self.logger.error(f"Ana program hatası: {e}")
            self.emergency_stop()
            return False
            
        finally:
            # Temizlik
            self.cleanup()
            
    def cleanup(self):
        """Sistem temizliği"""
        self.logger.info("Sistem temizleniyor...")
        
        try:
            # MAVLink'i kapat
            if self.mavlink:
                self.mavlink.close()
                
            # Sistem durumunu temizle
            if self.system_status:
                self.system_status.cleanup()
                
            # GPIO temizliği
            safe_gpio_cleanup()
            
        except Exception as e:
            self.logger.warning(f"Temizlik hatası: {e}")
            
        self.logger.info("✅ Sistem temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    import argparse
    
    parser = argparse.ArgumentParser(description='SARA Su Altı Roket Aracı')
    parser.add_argument('--mission', type=int, choices=[1, 2], default=1,
                       help='Görev numarası (1 veya 2)')
    parser.add_argument('--test-only', action='store_true',
                       help='Sadece bağlantı testi yap')
    
    args = parser.parse_args()
    
    sara = SaraMainController()
    
    if args.test_only:
        # Sadece test modu
        print("🧪 Test modu - sadece bağlantılar kontrol ediliyor...")
        mavlink_ok = sara.setup_mavlink()
        sensor_ok = sara.test_sensor_connections() if mavlink_ok else False
        
        print(f"MAVLink: {'✅' if mavlink_ok else '❌'}")
        print(f"Sensörler: {'✅' if sensor_ok else '❌'}")
        
        sara.cleanup()
        return mavlink_ok and sensor_ok
    else:
        # Normal görev modu
        return sara.run(args.mission)

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Program kullanıcı tarafından durduruldu")
        sys.exit(1)
    except Exception as e:
        print(f"💥 Kritik hata: {e}")
        sys.exit(1)
