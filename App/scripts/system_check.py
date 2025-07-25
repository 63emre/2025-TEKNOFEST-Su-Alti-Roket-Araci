#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Sistem Kontrolü
Kapsamlı Sistem Durumu Kontrolü
"""

import sys
import os
import time
import json
import math
import psutil
import subprocess

# Parent directory import
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from mavlink_handler import MAVLinkHandler

class SystemCheck:
    def __init__(self):
        """Sistem kontrol sistemi"""
        self.mavlink = MAVLinkHandler()
        self.test_results = {
            "mavlink_connection": False,
            "heartbeat": False,
            "imu_data": False,
            "gps_data": False,
            "servo_channels": {},
            "motor_channel": False,
            "system_resources": {},
            "configuration": False
        }
        
    def print_header(self, title):
        """Test başlığı yazdır"""
        print(f"\n{'='*60}")
        print(f"🔍 {title}")
        print(f"{'='*60}")
    
    def check_system_resources(self):
        """Sistem kaynaklarını kontrol et"""
        self.print_header("SİSTEM KAYNAKLARI KONTROLÜ")
        
        try:
            # CPU kullanımı
            cpu_percent = psutil.cpu_percent(interval=1)
            print(f"💻 CPU Kullanımı: {cpu_percent:.1f}%")
            
            # Memory kullanımı
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_available = memory.available / (1024**3)  # GB
            print(f"🧠 RAM Kullanımı: {memory_percent:.1f}% (Müsait: {memory_available:.1f}GB)")
            
            # Disk kullanımı
            disk = psutil.disk_usage('/')
            disk_percent = (disk.used / disk.total) * 100
            disk_free = disk.free / (1024**3)  # GB
            print(f"💾 Disk Kullanımı: {disk_percent:.1f}% (Boş: {disk_free:.1f}GB)")
            
            # Network interfaces
            print(f"🌐 Network Interfaces:")
            for interface, addrs in psutil.net_if_addrs().items():
                for addr in addrs:
                    if addr.family == 2:  # IPv4
                        print(f"   {interface}: {addr.address}")
            
            # Process count
            process_count = len(psutil.pids())
            print(f"⚙️ Aktif Process Sayısı: {process_count}")
            
            # System uptime
            boot_time = psutil.boot_time()
            uptime_seconds = time.time() - boot_time
            uptime_hours = uptime_seconds / 3600
            print(f"⏰ System Uptime: {uptime_hours:.1f} saat")
            
            # Sonuç değerlendirmesi
            issues = []
            if cpu_percent > 80:
                issues.append("Yüksek CPU kullanımı")
            if memory_percent > 85:
                issues.append("Yüksek RAM kullanımı")
            if disk_percent > 90:
                issues.append("Disk alanı yetersiz")
            if memory_available < 0.5:
                issues.append("Düşük RAM müsaitliği")
            
            if issues:
                print(f"⚠️ UYARILAR: {', '.join(issues)}")
                self.test_results["system_resources"]["status"] = "warning"
            else:
                print("✅ Sistem kaynakları normal")
                self.test_results["system_resources"]["status"] = "ok"
            
            self.test_results["system_resources"]["cpu_percent"] = cpu_percent
            self.test_results["system_resources"]["memory_percent"] = memory_percent
            self.test_results["system_resources"]["disk_percent"] = disk_percent
            
            return len(issues) == 0
            
        except Exception as e:
            print(f"❌ Sistem kaynak kontrolü hatası: {e}")
            self.test_results["system_resources"]["status"] = "error"
            return False
    
    def check_configuration_files(self):
        """Konfigürasyon dosyalarını kontrol et"""
        self.print_header("KONFİGÜRASYON DOSYALARI KONTROLÜ")
        
        config_files = [
            ("../config/hardware_config.json", "Hardware Konfigürasyonu"),
            ("../config/control_settings.json", "Kontrol Ayarları")
        ]
        
        all_configs_ok = True
        
        for config_path, description in config_files:
            print(f"\n📄 {description}:")
            
            if not os.path.exists(config_path):
                print(f"❌ Dosya bulunamadı: {config_path}")
                all_configs_ok = False
                continue
            
            try:
                with open(config_path, 'r') as f:
                    config_data = json.load(f)
                
                file_size = os.path.getsize(config_path)
                print(f"✅ Dosya yüklendi ({file_size} bytes)")
                
                # Hardware config kontrolü
                if "hardware_config" in config_path:
                    required_keys = ["pixhawk", "raspberry_pi", "mavlink"]
                    for key in required_keys:
                        if key in config_data:
                            print(f"   ✅ {key} bölümü var")
                        else:
                            print(f"   ❌ {key} bölümü eksik!")
                            all_configs_ok = False
                    
                    # Servo kanalları kontrol
                    if "pixhawk" in config_data and "servos" in config_data["pixhawk"]:
                        servo_count = len(config_data["pixhawk"]["servos"])
                        print(f"   📍 Servo kanalları: {servo_count} adet")
                    
                # Control settings kontrolü
                elif "control_settings" in config_path:
                    required_keys = ["control_modes", "navigation_modes", "movement_commands"]
                    for key in required_keys:
                        if key in config_data:
                            print(f"   ✅ {key} bölümü var")
                        else:
                            print(f"   ❌ {key} bölümü eksik!")
                            all_configs_ok = False
                
            except json.JSONDecodeError as e:
                print(f"❌ JSON format hatası: {e}")
                all_configs_ok = False
            except Exception as e:
                print(f"❌ Dosya okuma hatası: {e}")
                all_configs_ok = False
        
        self.test_results["configuration"] = all_configs_ok
        
        if all_configs_ok:
            print("\n✅ Tüm konfigürasyon dosyaları OK")
        else:
            print("\n❌ Konfigürasyon dosyalarında sorun var!")
        
        return all_configs_ok
    
    def check_mavlink_connection(self):
        """MAVLink bağlantısını kontrol et"""
        self.print_header("MAVLINK BAĞLANTI KONTROLÜ")
        
        print("🔌 MAVLink bağlantısı test ediliyor...")
        
        try:
            if self.mavlink.connect():
                print("✅ MAVLink bağlantısı başarılı")
                self.test_results["mavlink_connection"] = True
                
                # Heartbeat kontrolü
                print("💓 Heartbeat kontrolü...")
                if self.mavlink.check_system_status():
                    print("✅ Heartbeat alındı")
                    self.test_results["heartbeat"] = True
                    
                    # System info
                    armed_status = "ARMED" if self.mavlink.armed else "DISARMED"
                    print(f"🔒 Sistem durumu: {armed_status}")
                    
                else:
                    print("❌ Heartbeat alınamadı")
                    self.test_results["heartbeat"] = False
                
                return True
            else:
                print("❌ MAVLink bağlantısı başarısız")
                print("   • Pixhawk açık mı?")
                print("   • TCP:127.0.0.1:5777 portu açık mı?")
                print("   • MAVProxy/QGroundControl çalışıyor mu?")
                self.test_results["mavlink_connection"] = False
                return False
                
        except Exception as e:
            print(f"❌ MAVLink test hatası: {e}")
            self.test_results["mavlink_connection"] = False
            return False
    
    def check_imu_sensors(self):
        """IMU sensörlerini kontrol et"""
        self.print_header("IMU SENSÖR KONTROLÜ")
        
        if not self.test_results["mavlink_connection"]:
            print("❌ MAVLink bağlantısı yok - IMU test edilemez")
            return False
        
        print("🧭 IMU sensörleri test ediliyor...")
        
        try:
            imu_data = self.mavlink.get_imu_data()
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                
                print("✅ IMU verisi alınıyor")
                print(f"📊 Accelerometer: X={accel_x:.2f}, Y={accel_y:.2f}, Z={accel_z:.2f} m/s²")
                print(f"📊 Gyroscope: X={math.degrees(gyro_x):.2f}, Y={math.degrees(gyro_y):.2f}, Z={math.degrees(gyro_z):.2f} °/s")
                
                # Gravity check
                gravity = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
                print(f"🌍 Gravity magnitude: {gravity:.2f} m/s² (ideal: 9.81)")
                
                if abs(gravity - 9.81) < 2.0:
                    print("✅ Gravity değeri normal")
                else:
                    print("⚠️ Gravity değeri anormal - IMU kalibrasyonu gerekebilir")
                
                # Gyro stability check
                gyro_magnitude = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
                print(f"⚡ Gyro magnitude: {math.degrees(gyro_magnitude):.2f} °/s")
                
                if gyro_magnitude < 0.1:  # 5.7°/s
                    print("✅ Gyro stabil")
                else:
                    print("⚠️ Gyro yüksek - araç hareket ediyor olabilir")
                
                self.test_results["imu_data"] = True
                return True
            else:
                print("❌ IMU verisi alınamıyor")
                print("   • RAW_IMU mesajı geliyor mu?")
                print("   • IMU sensörü çalışıyor mu?")
                self.test_results["imu_data"] = False
                return False
                
        except Exception as e:
            print(f"❌ IMU test hatası: {e}")
            self.test_results["imu_data"] = False
            return False
    
    def check_gps_system(self):
        """GPS sistemini kontrol et"""
        self.print_header("GPS SİSTEM KONTROLÜ")
        
        if not self.test_results["mavlink_connection"]:
            print("❌ MAVLink bağlantısı yok - GPS test edilemez")
            return False
        
        print("🛰️ GPS sistemi test ediliyor...")
        
        try:
            gps_data = self.mavlink.get_gps_data()
            if gps_data:
                lat, lon, alt, satellites = gps_data
                
                print("✅ GPS verisi alınıyor")
                print(f"📍 Konum: {lat:.6f}, {lon:.6f}")
                print(f"📏 Yükseklik: {alt:.1f} m")
                print(f"🛰️ Uydu sayısı: {satellites}")
                
                if satellites >= 4:
                    print("✅ GPS sinyali iyi (4+ uydu)")
                elif satellites >= 2:
                    print("⚠️ GPS sinyali zayıf (2-3 uydu)")
                else:
                    print("❌ GPS sinyali çok zayıf (<2 uydu)")
                
                # Position sanity check
                if -90 <= lat <= 90 and -180 <= lon <= 180:
                    print("✅ GPS koordinatları makul")
                else:
                    print("❌ GPS koordinatları anormal!")
                
                self.test_results["gps_data"] = True
                return True
            else:
                print("⚠️ GPS verisi alınamıyor")
                print("   • GPS modülü bağlı mı?")
                print("   • GPS anteni dışarıda mı?")
                print("   • GLOBAL_POSITION_INT mesajı geliyor mu?")
                self.test_results["gps_data"] = False
                return False  # GPS opsiyonel, critical değil
                
        except Exception as e:
            print(f"❌ GPS test hatası: {e}")
            self.test_results["gps_data"] = False
            return False
    
    def check_servo_channels(self):
        """Servo kanallarını kontrol et"""
        self.print_header("SERVO KANALLARI KONTROLÜ")
        
        if not self.test_results["mavlink_connection"]:
            print("❌ MAVLink bağlantısı yok - Servo test edilemez")
            return False
        
        if not self.mavlink.armed:
            print("⚠️ Sistem DISARMED - Servo PWM gönderilecek ama hareket olmayacak")
        
        # Config'den servo kanallarını yükle
        try:
            with open('../config/hardware_config.json', 'r') as f:
                config = json.load(f)
            
            servo_channels = config['pixhawk']['servos']
            neutral_pwm = config['pixhawk']['pwm_limits']['servo_neutral']
            
        except:
            print("❌ Servo konfigürasyonu yüklenemedi")
            return False
        
        print(f"🔧 {len(servo_channels)} servo kanalı test ediliyor...")
        
        all_servos_ok = True
        
        for servo_name, channel in servo_channels.items():
            print(f"\n📍 {servo_name} (Kanal {channel}) test ediliyor...")
            
            try:
                # Neutral PWM gönder
                success = self.mavlink.send_raw_servo_pwm(channel, neutral_pwm)
                
                if success:
                    print(f"   ✅ PWM gönderildi: {neutral_pwm}µs")
                    self.test_results["servo_channels"][servo_name] = True
                else:
                    print(f"   ❌ PWM gönderilemedi!")
                    self.test_results["servo_channels"][servo_name] = False
                    all_servos_ok = False
                
                time.sleep(0.5)  # Kısa bekleme
                
            except Exception as e:
                print(f"   ❌ Test hatası: {e}")
                self.test_results["servo_channels"][servo_name] = False
                all_servos_ok = False
        
        if all_servos_ok:
            print("\n✅ Tüm servo kanalları çalışıyor")
        else:
            print("\n❌ Bazı servo kanallarında sorun var!")
        
        return all_servos_ok
    
    def check_motor_channel(self):
        """Motor kanalını kontrol et"""
        self.print_header("MOTOR KANAL KONTROLÜ")
        
        if not self.test_results["mavlink_connection"]:
            print("❌ MAVLink bağlantısı yok - Motor test edilemez")
            return False
        
        # Config'den motor kanalını yükle
        try:
            with open('../config/hardware_config.json', 'r') as f:
                config = json.load(f)
            
            motor_channel = config['pixhawk']['motor']
            stop_pwm = config['pixhawk']['pwm_limits']['motor_stop']
            
        except:
            print("❌ Motor konfigürasyonu yüklenemedi")
            return False
        
        print(f"🚁 Motor (AUX {motor_channel}) test ediliyor...")
        
        if not self.mavlink.armed:
            print("⚠️ Sistem DISARMED - Motor PWM gönderilecek ama motor çalışmayacak")
        
        try:
            # Stop PWM gönder
            success = self.mavlink.send_raw_motor_pwm(stop_pwm)
            
            if success:
                print(f"✅ Motor PWM gönderildi: {stop_pwm}µs (STOP)")
                self.test_results["motor_channel"] = True
                return True
            else:
                print("❌ Motor PWM gönderilemedi!")
                self.test_results["motor_channel"] = False
                return False
                
        except Exception as e:
            print(f"❌ Motor test hatası: {e}")
            self.test_results["motor_channel"] = False
            return False
    
    def run_full_system_check(self):
        """Tam sistem kontrolü"""
        print("🚀 TEKNOFEST KAPSAMLI SİSTEM KONTROLÜ")
        print("=" * 80)
        print("Bu test sistemin tüm bileşenlerini kontrol eder")
        print("=" * 80)
        
        start_time = time.time()
        
        # Test sırası (dependency order)
        tests = [
            ("Sistem Kaynakları", self.check_system_resources),
            ("Konfigürasyon Dosyaları", self.check_configuration_files),
            ("MAVLink Bağlantısı", self.check_mavlink_connection),
            ("IMU Sensörleri", self.check_imu_sensors),
            ("GPS Sistemi", self.check_gps_system),
            ("Servo Kanalları", self.check_servo_channels),
            ("Motor Kanalı", self.check_motor_channel)
        ]
        
        passed_tests = 0
        total_tests = len(tests)
        
        for test_name, test_function in tests:
            print(f"\n⏳ {test_name} test ediliyor...")
            
            try:
                if test_function():
                    passed_tests += 1
                    print(f"✅ {test_name}: BAŞARILI")
                else:
                    print(f"❌ {test_name}: BAŞARISIZ")
            except Exception as e:
                print(f"❌ {test_name}: HATA - {e}")
        
        # Sonuç raporu
        elapsed_time = time.time() - start_time
        
        print("\n" + "="*80)
        print("📊 SİSTEM KONTROL RAPORU")
        print("="*80)
        print(f"⏱️ Test Süresi: {elapsed_time:.1f} saniye")
        print(f"📊 Başarılı Testler: {passed_tests}/{total_tests}")
        print(f"📈 Başarı Oranı: {(passed_tests/total_tests)*100:.1f}%")
        
        if passed_tests == total_tests:
            print("🎉 TÜM SİSTEM TESTLERİ BAŞARILI!")
            print("✅ Sistem tam operasyonel")
        elif passed_tests >= total_tests * 0.8:
            print("⚠️ Sistem çoğunlukla operasyonel")
            print("🔧 Bazı bileşenler düzeltme gerektirebilir")
        else:
            print("❌ SİSTEMDE CİDDİ SORUNLAR VAR!")
            print("🚨 Operasyon öncesi düzeltme gerekli")
        
        # Kritik sorunlar
        critical_issues = []
        if not self.test_results["mavlink_connection"]:
            critical_issues.append("MAVLink bağlantısı yok")
        if not self.test_results["configuration"]:
            critical_issues.append("Konfigürasyon dosyaları hatalı")
        if not self.test_results["imu_data"]:
            critical_issues.append("IMU sensörleri çalışmıyor")
        
        if critical_issues:
            print(f"\n🚨 KRİTİK SORUNLAR:")
            for issue in critical_issues:
                print(f"   • {issue}")
        
        # Uyarılar
        warnings = []
        if not self.test_results["gps_data"]:
            warnings.append("GPS verisi yok (Su altında normal)")
        if not self.test_results["motor_channel"]:
            warnings.append("Motor kanalı sorunlu")
        
        if warnings:
            print(f"\n⚠️ UYARILAR:")
            for warning in warnings:
                print(f"   • {warning}")
        
        print("="*80)
        
        return passed_tests >= total_tests * 0.8
    
    def cleanup(self):
        """Temizlik"""
        if self.mavlink.connected:
            self.mavlink.disconnect()
        print("✅ Sistem kontrol temizliği tamamlandı")

def main():
    """Ana fonksiyon"""
    system_check = SystemCheck()
    
    print("🔍 TEKNOFEST SİSTEM KONTROL ARACI")
    print("=" * 60)
    print("1. Tam sistem kontrolü (Önerilen)")
    print("2. Sadece MAVLink kontrolü")
    print("3. Sadece IMU kontrolü")
    print("4. Sadece konfigürasyon kontrolü")
    print("5. Çıkış")
    
    try:
        choice = input("\nSeçiminiz (1-5): ").strip()
        
        if choice == '1':
            system_check.run_full_system_check()
        elif choice == '2':
            system_check.check_mavlink_connection()
            if system_check.test_results["mavlink_connection"]:
                system_check.check_imu_sensors()
                system_check.check_gps_system()
        elif choice == '3':
            if system_check.check_mavlink_connection():
                system_check.check_imu_sensors()
        elif choice == '4':
            system_check.check_configuration_files()
            system_check.check_system_resources()
        elif choice == '5':
            print("👋 Çıkış yapılıyor...")
        else:
            print("❌ Geçersiz seçim!")
            
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
    except Exception as e:
        print(f"\n❌ Program hatası: {e}")
    finally:
        system_check.cleanup()
        print("Program bitti.")

if __name__ == "__main__":
    main() 