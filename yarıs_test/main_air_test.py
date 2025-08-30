#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MAIN AIR TEST - Hava Yarışı Test Sistemi
D300 derinlik sensörü KALDIRILDI - Havada test için optimize edildi
Pluswing/main.py'den uyarlanmıştır
"""

import sys
import time
import signal
import threading
from pymavlink import mavutil

# Kendi modüllerimizi import et
from config_air import *
from utils_air import init_system_status, wait_with_button_check, safe_gpio_cleanup
from sensors_air import SensorManager
from control_air import StabilizationController
from mission_air_test import run_air_test_mission

class AirTestController:
    """Hava Yarışı Test Ana Kontrol Sistemi"""
    
    def __init__(self):
        # Sistem durumu başlat
        self.system_status = init_system_status()
        self.logger = self.system_status.logger
        
        # MAVLink bağlantısı
        self.mavlink = None
        
        # Test durumu
        self.current_mission = None
        self.mission_running = False
        self.system_running = True
        
        # Thread kontrolü
        self.main_thread = None
        
        self.logger.info("Hava Yarışı Test Kontrolcüsü başlatıldı")
        
    def get_available_mavlink_ports(self):
        """Sistemdeki mevcut MAVLink portlarını tara"""
        import os
        import glob
        
        available_ports = []
        
        if sys.platform.startswith('win'):
            # Windows COM portları
            for i in range(1, 20):
                port = f"COM{i}"
                try:
                    # Basit bağlantı testi
                    test_conn = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
                    test_conn.close()
                    available_ports.append(port)
                except:
                    continue
        else:
            # Linux/Unix ttyACM* ve ttyUSB* portları
            acm_ports = glob.glob('/dev/ttyACM*')
            usb_ports = glob.glob('/dev/ttyUSB*')
            
            # ACM portlarını öncelikle ekle (Pixhawk genelde ACM)
            for port in sorted(acm_ports):
                if os.path.exists(port):
                    available_ports.append(port)
                    
            # USB portlarını da ekle
            for port in sorted(usb_ports):
                if os.path.exists(port):
                    available_ports.append(port)
        
        self.logger.info(f"Mevcut portlar: {available_ports}")
        return available_ports

    def setup_mavlink(self, retries=3):
        """MAVLink bağlantısını güçlendirilmiş fallback ile kur"""
        self.logger.info("🔗 MAVLink bağlantısı kuruluyor...")
        
        for attempt in range(retries):
            try:
                self.logger.info(f"Bağlantı denemesi {attempt+1}/{retries}")
                
                # Mevcut portları dinamik olarak tara
                available_ports = self.get_available_mavlink_ports()
                
                if not available_ports:
                    self.logger.warning("Hiç port bulunamadı, varsayılan portlar deneniyor...")
                    if sys.platform.startswith('win'):
                        available_ports = [MAVLINK_PORT_WIN]
                    else:
                        available_ports = MAVLINK_PORTS
                
                # Her portu sırayla dene
                connection_success = False
                successful_port = None
                
                for port in available_ports:
                    try:
                        self.logger.info(f"🔌 Port deneniyor: {port}")
                        
                        # Bağlantıyı kur
                        self.mavlink = mavutil.mavlink_connection(
                            port, 
                            baud=MAVLINK_BAUD,
                            timeout=MAVLINK_CONNECTION_TIMEOUT,
                            retries=1
                        )
                        
                        # Heartbeat bekle
                        self.logger.info("💓 Heartbeat bekleniyor...")
                        msg = self.mavlink.wait_heartbeat(timeout=MAVLINK_HEARTBEAT_TIMEOUT)
                        
                        if msg:
                            self.logger.info(f"✅ MAVLink bağlantısı başarılı!")
                            self.logger.info(f"   📍 Port: {port}")
                            self.logger.info(f"   🆔 Sistem ID: {msg.get_srcSystem()}")
                            self.logger.info(f"   🔧 Tip: {msg.get_type()}")
                            
                            # Target system/component ayarla
                            self.mavlink.target_system = msg.get_srcSystem()
                            self.mavlink.target_component = msg.get_srcComponent()
                            
                            # Veri akışlarını iste
                            self._request_data_streams()
                            
                            # Bağlantıyı test et
                            if self._test_mavlink_connection():
                                connection_success = True
                                successful_port = port
                                break
                            else:
                                self.logger.warning(f"⚠️ Port {port} heartbeat aldı ama veri testi başarısız")
                                continue
                        else:
                            self.logger.warning(f"❌ Port {port}: Heartbeat alınamadı")
                            
                    except Exception as port_error:
                        self.logger.warning(f"❌ Port {port} bağlantı hatası: {port_error}")
                        continue
                        
                if connection_success:
                    self.logger.info(f"🎉 MAVLink bağlantısı kuruldu: {successful_port}")
                    self.successful_port = successful_port
                    return True
                    
            except Exception as e:
                self.logger.error(f"MAVLink kurulum hatası (deneme {attempt+1}): {e}")
                
            if attempt < retries - 1:
                self.logger.info("⏳ 3 saniye beklenip tekrar denenecek...")
                time.sleep(3)
                
        self.logger.error("❌ Tüm MAVLink bağlantı denemeleri başarısız!")
        return False
    
    def _test_mavlink_connection(self):
        """MAVLink bağlantısını test et"""
        try:
            # Birkaç mesaj alabilir miyiz test et
            test_count = 0
            start_time = time.time()
            
            while time.time() - start_time < 3.0 and test_count < 3:
                msg = self.mavlink.recv_match(blocking=False, timeout=0.5)
                if msg:
                    test_count += 1
                    
            return test_count >= 1  # En az 1 mesaj alabildi
            
        except Exception as e:
            self.logger.warning(f"MAVLink bağlantı testi hatası: {e}")
            return False
    
    def test_sensor_connections(self):
        """Sensör bağlantılarını test et - HAVA YARIŞI"""
        try:
            # SensorManager oluştur
            sensor_manager = SensorManager(self.mavlink, self.system_status.logger)
            
            # Altitude test et (D300 yerine)
            altitude_ok = sensor_manager.test_altitude_connection()
            
            # Attitude test et
            attitude_ok = sensor_manager.test_attitude_connection()
            
            self.logger.info(f"Hava yarışı sensör testi: Altitude={altitude_ok}, Attitude={attitude_ok}")
            
            # En azından Attitude çalışmalı (Altitude opsiyonel)
            return attitude_ok
            
        except Exception as e:
            self.logger.error(f"Sensör bağlantı testi hatası: {e}")
            return False
            
    def calibrate_sensors(self):
        """Sensör kalibrasyonu - HAVA YARIŞI (D300 YOK)"""
        try:
            self.logger.info("🔧 HAVA YARIŞI KALIBRASYON BAŞLIYOR...")
            self.logger.info("⚠️ ARAÇ YER SEVİYESİNDE TUTULMALI!")
            
            # SensorManager oluştur ve sınıf değişkeni olarak sakla
            self.sensor_manager = SensorManager(self.mavlink, self.system_status.logger)
            
            # 5 saniye kısa buzzer ile kalibrasyon uyarısı
            self.logger.info("🔊 5 saniye kalibrasyon buzzer başlıyor...")
            self.system_status.buzzer.beep(5.0)  # 5 saniye bip
            
            # Altitude sensörü kalibrasyonu (yer seviyesi)
            self.logger.info("🌍 Altitude yer seviyesi kalibrasyonu başlıyor...")
            altitude_calibration_success = self.sensor_manager.altitude.calibrate_ground_level()
            
            if altitude_calibration_success:
                self.logger.info("✅ Altitude yer seviyesi kalibrasyonu başarılı")
            else:
                self.logger.warning("⚠️ Altitude kalibrasyonu başarısız, varsayılan değerler kullanılacak")
            
            # Attitude sensörü kalibrasyonu
            try:
                self.sensor_manager.attitude.set_yaw_reference()
                attitude_calibration_success = self.sensor_manager.attitude.yaw_offset is not None
                if attitude_calibration_success:
                    self.logger.info("✅ Attitude sensörü kalibrasyonu başarılı")
                else:
                    self.logger.warning("⚠️ Attitude kalibrasyonu başarısız, varsayılan değerler kullanılacak")
            except Exception as att_error:
                self.logger.warning(f"Attitude kalibrasyon hatası: {att_error}")
                attitude_calibration_success = False
            
            # Kalibrasyon tamamlandı sinyali
            self.system_status.buzzer.beep_pattern(BUZZER_CALIBRATION_SUCCESS)
            
            self.logger.info("✅ Hava yarışı sensör kalibrasyonu tamamlandı")
            self.logger.info("🚁 Sistem hava testi için hazır!")
            
            return True
                
        except Exception as e:
            self.logger.error(f"Sensör kalibrasyon hatası: {e}")
            return True  # Devam et, test sırasında fallback kullanılacak
            
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
            
            # SCALED_PRESSURE mesajlarını iste (altitude için)
            self.mavlink.mav.request_data_stream_send(
                self.mavlink.target_system,
                self.mavlink.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # SCALED_PRESSURE
                5,   # 5 Hz
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
        self.logger.info("🔘 Test başlatma butonu bekleniyor...")
        self.system_status.set_phase(MissionPhase.WAITING)
        
        # LED yanıp sönsün (bekleme modunda)
        self.system_status.led.blink(0.5)
        
        while self.system_running:
            button_action = self.system_status.check_start_button()
            
            if button_action == "restart":
                self.logger.info("✅ Test başlatma butonu basıldı!")
                self.system_status.led.turn_on()
                self.system_status.buzzer.beep_pattern(BUZZER_STARTUP)
                time.sleep(2)  # Buton bouncing önlemi
                return True
                
            time.sleep(0.1)
            
        return False
        
    def countdown_delay(self):
        """Kısa güvenlik geri sayımı - Hava testi için"""
        self.logger.info("⏱️ 10 saniye güvenlik geri sayımı başlıyor...")
        self.system_status.set_phase(MissionPhase.WAITING)
        
        for remaining in range(ARMING_DELAY_SECONDS, 0, -1):
            if not self.system_running:
                return False
                
            # Buton kontrolü - iptal için
            button_action = self.system_status.check_start_button()
            if button_action == "restart":
                self.logger.info("🛑 Geri sayım iptal edildi! Yeniden buton bekleniyor...")
                return "restart"  # İptal sinyali
                
            self.logger.info(f"⏱️ Geri sayım: {remaining} saniye kaldı")
            self.system_status.buzzer.beep(BUZZER_COUNTDOWN_SHORT)
            time.sleep(1.0)
                
        self.logger.info("✅ Güvenlik gecikmesi tamamlandı!")
        self.system_status.buzzer.beep_pattern(BUZZER_MISSION_START)
        return True
        
    def run_air_test(self):
        """Hava testi çalıştır"""
        try:
            self.logger.info(f"🚁 HAVA YARIŞI TESTİ BAŞLIYOR!")
            self.mission_running = True
            
            # Kalibre edilmiş sensör manager'ı kullan
            if not hasattr(self, 'sensor_manager'):
                self.logger.warning("Sensör manager bulunamadı, yeni oluşturuluyor")
                self.sensor_manager = SensorManager(self.mavlink, self.logger)
            
            # Hava testi çalıştır
            success = run_air_test_mission(
                mavlink_connection=self.mavlink,
                system_status=self.system_status,
                logger=self.logger,
                sensor_manager=self.sensor_manager
            )
                
            if success:
                self.logger.info("🎉 Hava testi başarıyla tamamlandı!")
                self.system_status.set_phase(MissionPhase.COMPLETED)
            else:
                self.logger.error("❌ Hava testi başarısız!")
                self.system_status.set_phase(MissionPhase.EMERGENCY)
                
            return success
            
        except Exception as e:
            self.logger.error(f"Hava testi çalıştırma hatası: {e}")
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
        
    def run(self):
        """Ana çalıştırma döngüsü - Hava testi"""
        try:
            # Signal handler ayarla
            signal.signal(signal.SIGINT, self.signal_handler)
            signal.signal(signal.SIGTERM, self.signal_handler)
            
            self.logger.info("🚁 Hava Yarışı Test Sistemi başlatılıyor...")
            
            # 1. MAVLink bağlantısını kur
            if not self.setup_mavlink():
                self.logger.error("MAVLink bağlantısı kurulamadı, çıkılıyor")
                return False
                
            # 2. Sensör bağlantılarını test et
            if not self.test_sensor_connections():
                self.logger.error("Sensör bağlantıları başarısız, çıkılıyor")
                return False
                
            # 3. Sensör kalibrasyonu
            if not self.calibrate_sensors():
                self.logger.error("Sensör kalibrasyonu başarısız, çıkılıyor")
                return False
                
            # 4-5. Buton bekle ve kısa geri sayım döngüsü
            while True:
                # 4. Başlatma butonu bekle
                if not self.wait_for_start_button():
                    self.logger.info("Test başlatma iptal edildi")
                    return False
                    
                # 5. Kısa güvenlik gecikmesi
                countdown_result = self.countdown_delay()
                
                if countdown_result == True:
                    # Geri sayım tamamlandı, test başlayabilir
                    break
                elif countdown_result == "restart":
                    # Buton basıldı, yeniden buton bekle
                    self.logger.info("🔄 Geri sayım iptal edildi, yeniden buton bekleniyor...")
                    continue
                else:
                    # Hata durumu
                    self.logger.error("Geri sayım hatası")
                    return False
                
            # 6. Hava testini çalıştır
            success = self.run_air_test()
            
            if success:
                self.logger.info("🏆 Hava yarışı testi başarıyla tamamlandı!")
            else:
                self.logger.error("💥 Hava yarışı testi başarısız!")
                
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
    
    parser = argparse.ArgumentParser(description='Hava Yarışı Test Sistemi')
    parser.add_argument('--test-only', action='store_true',
                       help='Sadece bağlantı testi yap')
    
    args = parser.parse_args()
    
    air_test = AirTestController()
    
    if args.test_only:
        # Sadece test modu
        print("🧪 Test modu - sadece bağlantılar kontrol ediliyor...")
        mavlink_ok = air_test.setup_mavlink()
        sensor_ok = air_test.test_sensor_connections() if mavlink_ok else False
        
        print(f"MAVLink: {'✅' if mavlink_ok else '❌'}")
        print(f"Sensörler: {'✅' if sensor_ok else '❌'}")
        
        air_test.cleanup()
        return mavlink_ok and sensor_ok
    else:
        # Normal test modu
        return air_test.run()

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
