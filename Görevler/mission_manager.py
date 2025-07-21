#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
Mission Manager - Görev Yönetim Sistemi

Bu script, her iki görevi (Görev 1 ve Görev 2) yönetir, görev planlaması yapar,
execution kontrolü sağlar ve performans analizi yapar.

Features:
- Otomatik görev sıralama ve planlama
- Real-time mission monitoring
- Emergency abort procedures
- Comprehensive mission reporting
- Configuration management
- Safety system integration

Hardware: Raspberry Pi 4B + Pixhawk PX4 PIX 2.4.8 + 4x DS3230MG Servo + DEGZ Motor + ESC
Protocol: MAVLink via tcp:127.0.0.1:5777
"""

import sys
import os
import time
import json
import argparse
import threading
from datetime import datetime, timedelta
from pymavlink import mavutil

# Import mission modules
sys.path.append(os.path.dirname(__file__))
from mission_1_navigation import Mission1Navigator
from mission_2_rocket_launch import Mission2RocketLaunch

# Mission Manager Configuration
MANAGER_CONFIG = {
    'mission_timeout_minutes': 10,      # Maximum mission duration
    'inter_mission_delay': 120,         # Delay between missions (seconds)
    'safety_check_interval': 5,         # Safety check frequency (seconds)
    'telemetry_log_interval': 1,        # Telemetry logging frequency (seconds)
    'max_retry_attempts': 3,            # Maximum retry attempts per mission
    'emergency_surface_depth': 0.5      # Emergency surface threshold (m)
}

# Competition Scoring
COMPETITION_SCORING = {
    'mission_1': {
        'max_points': 300,
        'components': {
            'fastest_cruise': 150,
            'power_cut_at_start': 90,
            'waterproofing': 60
        }
    },
    'mission_2': {
        'max_points': 400,
        'components': {
            'reach_launch_zone': 100,
            'safe_surface_angle': 100,
            'rocket_separation': 150,
            'waterproofing': 50
        }
    }
}

class MissionManager:
    """Ana Mission Manager Sınıfı"""
    
    def __init__(self):
        # Connection status
        self.mavlink_connected = False
        self.master = None
        
        # Mission instances
        self.mission_1 = None
        self.mission_2 = None
        
        # Manager state
        self.manager_active = False
        self.current_mission = None
        self.mission_queue = []
        self.emergency_abort = False
        
        # Mission data
        self.mission_results = {
            'mission_1': {'completed': False, 'score': 0, 'data': None},
            'mission_2': {'completed': False, 'score': 0, 'data': None}
        }
        
        # Telemetry and logging
        self.telemetry_thread = None
        self.safety_thread = None
        self.telemetry_data = []
        self.telemetry_logging = False
        
        # Competition configuration
        self.competition_config = {
            'start_position': {'lat': None, 'lon': None},
            'launch_zone': {'lat': None, 'lon': None},
            'competition_day': False,
            'video_recording': False
        }
        
    def initialize_mavlink(self):
        """MAVLink bağlantısını başlat"""
        try:
            print("🔌 Mission Manager - MAVLink başlatılıyor...")
            self.master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=10)
            self.mavlink_connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
        except Exception as e:
            print(f"❌ MAVLink bağlantı hatası: {e}")
            return False
    
    def load_mission_config(self, config_file):
        """Mission konfigürasyon dosyasını yükle"""
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            self.competition_config.update(config)
            
            print("✅ Mission konfigürasyonu yüklendi:")
            print(f"   Başlangıç: {self.competition_config['start_position']}")
            print(f"   Atış bölgesi: {self.competition_config['launch_zone']}")
            print(f"   Yarışma günü: {self.competition_config['competition_day']}")
            
            return True
        except Exception as e:
            print(f"❌ Konfigürasyon yükleme hatası: {e}")
            return False
    
    def save_mission_config(self, config_file):
        """Mission konfigürasyonunu kaydet"""
        try:
            with open(config_file, 'w', encoding='utf-8') as f:
                json.dump(self.competition_config, f, ensure_ascii=False, indent=2)
            print(f"✅ Konfigürasyon kaydedildi: {config_file}")
            return True
        except Exception as e:
            print(f"❌ Konfigürasyon kayıt hatası: {e}")
            return False
    
    def pre_mission_checks(self):
        """Görev öncesi güvenlik kontrolleri"""
        print("🔍 Görev öncesi güvenlik kontrolleri...")
        
        checks = {
            'mavlink_connection': False,
            'gps_signal': False,
            'pressure_sensor': False,
            'attitude_sensor': False,
            'servo_response': False,
            'motor_response': False,
            'emergency_systems': False
        }
        
        try:
            # MAVLink bağlantı kontrolü
            if self.mavlink_connected:
                checks['mavlink_connection'] = True
                print("   ✅ MAVLink bağlantısı OK")
            
            # GPS sinyal kontrolü
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps_msg and gps_msg.satellites_visible >= 4:
                checks['gps_signal'] = True
                print(f"   ✅ GPS sinyali OK ({gps_msg.satellites_visible} uydu)")
            else:
                print(f"   ⚠️ GPS sinyali zayıf ({gps_msg.satellites_visible if gps_msg else 0} uydu)")
            
            # Basınç sensörü kontrolü
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=5)
            if pressure_msg:
                checks['pressure_sensor'] = True
                print("   ✅ Basınç sensörü OK")
            
            # Attitude sensörü kontrolü
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
            if attitude_msg:
                checks['attitude_sensor'] = True
                print("   ✅ Attitude sensörü OK")
            
            # Servo yanıt kontrolü (quick test)
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 1, 1500, 0, 0, 0, 0, 0
            )
            time.sleep(0.5)
            checks['servo_response'] = True
            print("   ✅ Servo yanıtı OK")
            
            # Motor yanıt kontrolü (quick test)
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 8, 1500, 0, 0, 0, 0, 0
            )
            time.sleep(0.5)
            checks['motor_response'] = True
            print("   ✅ Motor yanıtı OK")
            
            checks['emergency_systems'] = True
            print("   ✅ Acil durum sistemleri OK")
            
        except Exception as e:
            print(f"   ❌ Kontrol hatası: {e}")
        
        # Genel durum değerlendirmesi
        passed_checks = sum(checks.values())
        total_checks = len(checks)
        
        print(f"\n📊 Güvenlik kontrolleri: {passed_checks}/{total_checks} başarılı")
        
        if passed_checks >= 5:  # Minimum 5/7 geçmeli
            print("✅ Görev güvenlik kriterleri karşılandı")
            return True
        else:
            print("❌ Güvenlik kriterleri yetersiz!")
            return False
    
    def start_telemetry_logging(self):
        """Telemetri kayıt thread'ini başlat"""
        if self.telemetry_logging:
            return
            
        self.telemetry_logging = True
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()
        print("📡 Telemetri kaydı başlatıldı")
    
    def stop_telemetry_logging(self):
        """Telemetri kaydını durdur"""
        self.telemetry_logging = False
        if self.telemetry_thread:
            self.telemetry_thread.join(timeout=2)
        print("📡 Telemetri kaydı durduruldu")
    
    def _telemetry_worker(self):
        """Telemetri kayıt worker thread"""
        while self.telemetry_logging and self.mavlink_connected:
            try:
                telemetry_point = {
                    'timestamp': time.time(),
                    'datetime': datetime.now().isoformat()
                }
                
                # GPS data
                gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
                if gps_msg:
                    telemetry_point['gps'] = {
                        'lat': gps_msg.lat / 1e7,
                        'lon': gps_msg.lon / 1e7,
                        'alt': gps_msg.alt / 1000.0,
                        'satellites': gps_msg.satellites_visible
                    }
                
                # Attitude data
                attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                if attitude_msg:
                    telemetry_point['attitude'] = {
                        'roll': attitude_msg.roll,
                        'pitch': attitude_msg.pitch,
                        'yaw': attitude_msg.yaw
                    }
                
                # Pressure data
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    telemetry_point['pressure'] = {
                        'abs_pressure': pressure_msg.press_abs,
                        'diff_pressure': pressure_msg.press_diff,
                        'temperature': pressure_msg.temperature
                    }
                
                # VFR data
                vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
                if vfr_msg:
                    telemetry_point['vfr'] = {
                        'airspeed': vfr_msg.airspeed,
                        'groundspeed': vfr_msg.groundspeed,
                        'alt': vfr_msg.alt,
                        'climb': vfr_msg.climb,
                        'heading': vfr_msg.heading,
                        'throttle': vfr_msg.throttle
                    }
                
                self.telemetry_data.append(telemetry_point)
                
                # Keep only last 1000 points to manage memory
                if len(self.telemetry_data) > 1000:
                    self.telemetry_data = self.telemetry_data[-500:]
                
            except Exception as e:
                print(f"⚠️ Telemetri hatası: {e}")
            
            time.sleep(MANAGER_CONFIG['telemetry_log_interval'])
    
    def start_safety_monitoring(self):
        """Güvenlik monitörü thread'ini başlat"""
        if self.safety_thread and self.safety_thread.is_alive():
            return
            
        self.safety_thread = threading.Thread(target=self._safety_worker)
        self.safety_thread.daemon = True
        self.safety_thread.start()
        print("🛡️ Güvenlik monitörü başlatıldı")
    
    def _safety_worker(self):
        """Güvenlik monitörü worker thread"""
        while self.manager_active and self.mavlink_connected:
            try:
                # Depth safety check
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    surface_pressure = 1013.25  # Standard atmospheric pressure
                    current_depth = (pressure_msg.press_abs - surface_pressure) * 0.10197
                    
                    # Emergency surface if too deep (>5m safety limit)
                    if current_depth > 5.0:
                        print("🚨 ACİL! Çok derinlik tespit edildi - Acil yüzeye çıkış")
                        self.emergency_abort = True
                
                # Communication timeout check
                if time.time() - self.master.last_heartbeat > 30:  # 30 seconds timeout
                    print("🚨 ACİL! MAVLink heartbeat kaybı")
                    self.emergency_abort = True
                
                # Check for emergency abort
                if self.emergency_abort:
                    self.abort_all_missions()
                    break
                    
            except Exception as e:
                print(f"⚠️ Güvenlik monitörü hatası: {e}")
            
            time.sleep(MANAGER_CONFIG['safety_check_interval'])
    
    def abort_all_missions(self):
        """Tüm görevleri acil durdur"""
        print("🚨 TÜM GÖREVLERİN ACİL DURDURULMASI!")
        
        self.emergency_abort = True
        self.manager_active = False
        
        # Stop current mission
        if self.current_mission:
            self.current_mission.emergency_stop_procedure()
        
        # Send emergency commands
        try:
            # Stop motor
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 8, 1500, 0, 0, 0, 0, 0
            )
            
            # Neutral all servos
            for channel in range(1, 10):
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, channel, 1500, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
                
        except Exception as e:
            print(f"❌ Acil durdurma komut hatası: {e}")
        
        print("✅ Acil durdurma prosedürü tamamlandı")
    
    def execute_mission_1(self, start_lat, start_lon):
        """Mission 1'i çalıştır"""
        print("\n" + "="*60)
        print("🎯 GÖREV 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş")
        print("="*60)
        
        try:
            # Mission 1 instance oluştur
            self.mission_1 = Mission1Navigator(start_lat=start_lat, start_lon=start_lon)
            self.mission_1.master = self.master
            self.mission_1.connected = True
            self.current_mission = self.mission_1
            
            # Mission'ı çalıştır
            success = self.mission_1.run_mission_1()
            
            if success:
                # Scoring hesapla
                report = self.mission_1.generate_mission_report()
                score, _ = self.mission_1.calculate_mission_score()
                
                self.mission_results['mission_1'] = {
                    'completed': True,
                    'score': score,
                    'data': report
                }
                
                print(f"✅ Görev 1 tamamlandı! Puan: {score}/300")
                return True
            else:
                print("❌ Görev 1 başarısız!")
                return False
                
        except Exception as e:
            print(f"❌ Görev 1 çalıştırma hatası: {e}")
            return False
        finally:
            self.current_mission = None
    
    def execute_mission_2(self, launch_zone_lat, launch_zone_lon):
        """Mission 2'yi çalıştır"""
        print("\n" + "="*60)
        print("🚀 GÖREV 2: Roket Ateşleme")
        print("="*60)
        
        try:
            # Mission 2 instance oluştur
            self.mission_2 = Mission2RocketLaunch(
                launch_zone_lat=launch_zone_lat,
                launch_zone_lon=launch_zone_lon
            )
            self.mission_2.master = self.master
            self.mission_2.connected = True
            self.current_mission = self.mission_2
            
            # Mission'ı çalıştır
            success = self.mission_2.run_mission_2()
            
            if success:
                # Scoring hesapla
                report = self.mission_2.generate_mission_report()
                score, _ = self.mission_2.calculate_mission_score()
                
                self.mission_results['mission_2'] = {
                    'completed': True,
                    'score': score,
                    'data': report
                }
                
                print(f"✅ Görev 2 tamamlandı! Puan: {score}/400")
                return True
            else:
                print("❌ Görev 2 başarısız!")
                return False
                
        except Exception as e:
            print(f"❌ Görev 2 çalıştırma hatası: {e}")
            return False
        finally:
            self.current_mission = None
    
    def execute_mission_sequence(self, missions=['mission_1', 'mission_2']):
        """Görev sırasını çalıştır"""
        print("\n🎮 TEKNOFEST 2025 - Mission Sequence Başlatılıyor!")
        print("="*60)
        
        self.manager_active = True
        
        # Güvenlik ve telemetri başlat
        self.start_safety_monitoring()
        self.start_telemetry_logging()
        
        try:
            for mission_name in missions:
                if self.emergency_abort:
                    print("🚨 Acil durdurma nedeniyle görev sırası durduruldu!")
                    break
                
                if mission_name == 'mission_1':
                    if not self.competition_config['start_position']['lat']:
                        print("❌ Görev 1 için başlangıç koordinatları eksik!")
                        continue
                        
                    success = self.execute_mission_1(
                        self.competition_config['start_position']['lat'],
                        self.competition_config['start_position']['lon']
                    )
                    
                elif mission_name == 'mission_2':
                    if not self.competition_config['launch_zone']['lat']:
                        print("❌ Görev 2 için atış bölgesi koordinatları eksik!")
                        continue
                        
                    success = self.execute_mission_2(
                        self.competition_config['launch_zone']['lat'],
                        self.competition_config['launch_zone']['lon']
                    )
                
                else:
                    print(f"❌ Bilinmeyen görev: {mission_name}")
                    continue
                
                if success:
                    print(f"✅ {mission_name} başarılı!")
                else:
                    print(f"❌ {mission_name} başarısız!")
                    
                    if self.competition_config['competition_day']:
                        # Yarışma günü - sonraki göreve geç
                        print("⚠️ Yarışma günü - sonraki göreve geçiliyor...")
                    else:
                        # Test günü - tekrar dene veya dur
                        response = input("Tekrar denemek istiyor musunuz? (e/h): ")
                        if response.lower() != 'e':
                            break
                
                # Görevler arası bekleme
                if mission_name != missions[-1]:  # Son görev değilse
                    print(f"⏳ Sonraki görev için {MANAGER_CONFIG['inter_mission_delay']} saniye bekleniyor...")
                    time.sleep(MANAGER_CONFIG['inter_mission_delay'])
            
            # Final rapor
            self.generate_final_report()
            
        except KeyboardInterrupt:
            print("\n⚠️ Görev sırası kullanıcı tarafından durduruldu")
            self.abort_all_missions()
        except Exception as e:
            print(f"❌ Görev sırası hatası: {e}")
            self.abort_all_missions()
        finally:
            self.manager_active = False
            self.stop_telemetry_logging()
    
    def generate_final_report(self):
        """Final yarışma raporu oluştur"""
        print("\n" + "="*60)
        print("📊 FINAL YARIŞMA RAPORU")
        print("="*60)
        
        total_score = 0
        max_possible = 700  # 300 + 400
        
        for mission_name, result in self.mission_results.items():
            if result['completed']:
                total_score += result['score']
                print(f"✅ {mission_name.upper()}: {result['score']} puan")
            else:
                print(f"❌ {mission_name.upper()}: 0 puan (tamamlanmadı)")
        
        percentage = (total_score / max_possible) * 100
        
        print(f"\n🏆 TOPLAM PUAN: {total_score}/{max_possible} (%{percentage:.1f})")
        
        if percentage >= 80:
            print("🥇 Mükemmel performans!")
        elif percentage >= 60:
            print("🥈 İyi performans!")
        elif percentage >= 40:
            print("🥉 Orta performans")
        else:
            print("📊 Geliştirilmesi gereken alanlar var")
        
        # Detaylı raporu kaydet
        final_report = {
            'competition_date': datetime.now().isoformat(),
            'total_score': total_score,
            'max_possible_score': max_possible,
            'score_percentage': percentage,
            'mission_results': self.mission_results,
            'telemetry_summary': {
                'total_data_points': len(self.telemetry_data),
                'mission_duration': None
            },
            'vehicle_config': self.competition_config
        }
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"final_competition_report_{timestamp}.json"
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(final_report, f, ensure_ascii=False, indent=2)
            print(f"📄 Final rapor kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Rapor kayıt hatası: {e}")
        
        # Telemetri verilerini kaydet
        telemetry_filename = f"telemetry_data_{timestamp}.json"
        try:
            with open(telemetry_filename, 'w', encoding='utf-8') as f:
                json.dump(self.telemetry_data, f, ensure_ascii=False, indent=2)
            print(f"📡 Telemetri verisi kaydedildi: {telemetry_filename}")
        except Exception as e:
            print(f"❌ Telemetri kayıt hatası: {e}")

def main():
    parser = argparse.ArgumentParser(description='TEKNOFEST 2025 - Mission Manager')
    parser.add_argument('--config', type=str, default='mission_config.json',
                       help='Mission konfigürasyon dosyası')
    parser.add_argument('--missions', nargs='+', default=['mission_1', 'mission_2'],
                       choices=['mission_1', 'mission_2'],
                       help='Çalıştırılacak görevler')
    parser.add_argument('--competition-mode', action='store_true',
                       help='Yarışma modu (otomatik devam)')
    parser.add_argument('--pre-checks-only', action='store_true',
                       help='Sadece görev öncesi kontrolleri yap')
    parser.add_argument('--start-lat', type=float,
                       help='Başlangıç latitude koordinatı')
    parser.add_argument('--start-lon', type=float,
                       help='Başlangıç longitude koordinatı')
    parser.add_argument('--launch-lat', type=float,
                       help='Atış bölgesi latitude koordinatı')
    parser.add_argument('--launch-lon', type=float,
                       help='Atış bölgesi longitude koordinatı')
    
    args = parser.parse_args()
    
    # Mission Manager başlat
    manager = MissionManager()
    
    # Konfigürasyon yükle
    if os.path.exists(args.config):
        manager.load_mission_config(args.config)
    
    # Command line koordinatları varsa güncelle
    if args.start_lat and args.start_lon:
        manager.competition_config['start_position'] = {
            'lat': args.start_lat, 'lon': args.start_lon
        }
    
    if args.launch_lat and args.launch_lon:
        manager.competition_config['launch_zone'] = {
            'lat': args.launch_lat, 'lon': args.launch_lon
        }
    
    if args.competition_mode:
        manager.competition_config['competition_day'] = True
    
    # Konfigürasyonu kaydet
    manager.save_mission_config(args.config)
    
    print("🎮 TEKNOFEST 2025 - Mission Manager v1.0")
    print("="*60)
    
    # MAVLink başlat
    if not manager.initialize_mavlink():
        print("❌ MAVLink başlatılamadı!")
        return 1
    
    try:
        # Görev öncesi kontroller
        if not manager.pre_mission_checks():
            print("❌ Güvenlik kontrolleri başarısız!")
            if not args.competition_mode:
                response = input("Yine de devam etmek istiyor musunuz? (e/h): ")
                if response.lower() != 'e':
                    return 1
            else:
                return 1
        
        if args.pre_checks_only:
            print("✅ Görev öncesi kontroller tamamlandı - sadece kontrol modu")
            return 0
        
        # Mission sequence başlat
        manager.execute_mission_sequence(args.missions)
        
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Program kullanıcı tarafından durduruldu")
        manager.abort_all_missions()
        return 1
    finally:
        if manager.master:
            manager.master.close()

if __name__ == "__main__":
    sys.exit(main()) 