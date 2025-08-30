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
                            timeout=5,
                            retries=1
                        )
                        
                        # Heartbeat bekle
                        self.logger.info("💓 Heartbeat bekleniyor...")
                        msg = self.mavlink.wait_heartbeat(timeout=5)
                        
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
                    # Başarılı portu config'e kaydet (gelecek bağlantılar için)
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
            
    def calibrate_sensors(self):
        """D300 sensör kalibrasyonu - su üstünden 10 saniye veri toplama"""
        try:
            self.logger.info("🔧 D300 KALIBRASYON BAŞLIYOR...")
            self.logger.info("⚠️ ARAÇ SU ÜSTÜNDE TUTULMALI!")
            
            # SensorManager oluştur ve sınıf değişkeni olarak sakla
            self.sensor_manager = SensorManager(self.mavlink, self.system_status.logger)
            
            # 10 saniye uzun buzzer ile kalibrasyon uyarısı
            self.logger.info("🔊 10 saniye kalibrasyon buzzer başlıyor...")
            self.system_status.buzzer.beep(10.0)  # 10 saniye uzun bip
            
            # D300 deniz suyu kalibrasyonu (su üstünde)
            self.logger.info("🌊 D300 deniz suyu kalibrasyonu başlıyor...")
            depth_calibration_success = self.sensor_manager.depth.calibrate_seawater()
            
            if depth_calibration_success:
                self.logger.info("✅ D300 deniz suyu kalibrasyonu başarılı")
            else:
                self.logger.warning("⚠️ D300 kalibrasyonu başarısız, varsayılan değerler kullanılacak")
            
            # Attitude sensörü kalibrasyonu (opsiyonel)
            try:
                attitude_calibration_success = self.sensor_manager.calibrate_attitude()
                if attitude_calibration_success:
                    self.logger.info("✅ Attitude sensörü kalibrasyonu başarılı")
                else:
                    self.logger.warning("⚠️ Attitude kalibrasyonu başarısız, varsayılan değerler kullanılacak")
            except Exception as att_error:
                self.logger.warning(f"Attitude kalibrasyon hatası: {att_error}")
                attitude_calibration_success = False
            
            # Kalibrasyon tamamlandı sinyali
            self.system_status.buzzer.beep_pattern([0.5, 0.2, 0.5, 0.2, 0.5])  # Başarı sinyali
            
            self.logger.info("✅ Sensör kalibrasyonu tamamlandı")
            self.logger.info("🤖 Sistem görev için hazır!")
            
            # D300 kalibrasyonu başarısızsa da devam et (fallback mekanizması var)
            return True
                
        except Exception as e:
            self.logger.error(f"Sensör kalibrasyon hatası: {e}")
            # Hata durumunda da devam et, görev sırasında fallback kullanılacak
            return True
            
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
            
            if button_action == "restart":
                self.logger.info("✅ Başlatma butonu basıldı!")
                self.system_status.led.turn_on()
                self.system_status.buzzer.beep_pattern(BUZZER_STARTUP)
                time.sleep(2)  # Buton bouncing önlemi
                return True
                
            time.sleep(0.1)
            
        return False
        
    def countdown_90_seconds(self):
        """90 saniye güvenlik geri sayımı"""
        self.logger.info("⏱️ 90 saniye güvenlik geri sayımı başlıyor...")
        self.system_status.set_phase(MissionPhase.WAITING)
        
        # 90 saniye = 10 x (9 kısa bip + 1 uzun bip)
        for group in range(10):
            # 9 kısa bip
            for short_beep in range(9):
                if not self.system_running:
                    return False
                    
                # Buton kontrolü - iptal için
                button_action = self.system_status.check_start_button()
                if button_action == "restart":
                    self.logger.info("🛑 Geri sayım iptal edildi! Yeniden buton bekleniyor...")
                    return "restart"  # İptal sinyali
                    
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
            
            # Kalibre edilmiş sensör manager'ı kullan
            if not hasattr(self, 'sensor_manager'):
                self.logger.warning("Sensör manager bulunamadı, yeni oluşturuluyor")
                self.sensor_manager = SensorManager(self.mavlink, self.logger)
            
            # Görev türüne göre çalıştır
            if mission_type == 1:
                success = run_mission_1(
                    mavlink_connection=self.mavlink,
                    system_status=self.system_status,
                    logger=self.logger,
                    sensor_manager=self.sensor_manager
                )
            elif mission_type == 2:
                success = run_mission_2(
                    mavlink_connection=self.mavlink,
                    system_status=self.system_status,
                    logger=self.logger,
                    sensor_manager=self.sensor_manager
                )
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
                
            # 4-5. Buton bekle ve 90 saniye döngüsü
            while True:
                # 4. Başlatma butonu bekle
                if not self.wait_for_start_button():
                    self.logger.info("Başlatma iptal edildi")
                    return False
                    
                # 5. 90 saniye güvenlik gecikmesi
                countdown_result = self.countdown_90_seconds()
                
                if countdown_result == True:
                    # 90 saniye tamamlandı, görev başlayabilir
                    break
                elif countdown_result == "restart":
                    # Buton basıldı, yeniden buton bekle
                    self.logger.info("🔄 Geri sayım iptal edildi, yeniden buton bekleniyor...")
                    continue
                else:
                    # Hata durumu
                    self.logger.error("Geri sayım hatası")
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
