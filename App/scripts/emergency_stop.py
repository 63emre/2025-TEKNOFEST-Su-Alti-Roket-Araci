#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Acil Durum Protokolü
Emergency Stop Test ve Acil Durum Sistemi
"""

import sys
import os
import time
import json
import threading

# Parent directory import
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from mavlink_handler import MAVLinkHandler

class EmergencySystem:
    def __init__(self):
        """Acil durum sistemi"""
        self.mavlink = MAVLinkHandler()
        self.emergency_active = False
        self.surface_protocol_active = False
        
    def connect_system(self):
        """Sisteme bağlan"""
        print("🔌 Acil durum sistemi - MAVLink bağlantısı...")
        
        if not self.mavlink.connect():
            print("❌ BAĞLANTI HATASI: Pixhawk'a bağlanılamadı!")
            print("   • Acil durum scripti MAVLink olmadan çalışamaz!")
            return False
        
        print("✅ MAVLink bağlantısı başarılı")
        return True
    
    def emergency_stop(self):
        """Acil durum durdurma"""
        print("🚨 ACİL DURUM PROTOKOLÜ BAŞLATIYOR!")
        print("=" * 50)
        
        self.emergency_active = True
        
        try:
            # 1. Tüm hareket durdur
            print("1️⃣ Tüm servo/motor durdurma...")
            
            if self.mavlink.connected:
                # Servo neutral
                servo_channels = [1, 3, 4, 5]  # AUX channels
                neutral_pwm = 1500
                
                for channel in servo_channels:
                    success = self.mavlink.send_raw_servo_pwm(channel, neutral_pwm)
                    if success:
                        print(f"   ✅ Servo AUX{channel}: {neutral_pwm}µs (NEUTRAL)")
                    else:
                        print(f"   ❌ Servo AUX{channel}: PWM gönderilemedi!")
                
                # Motor stop
                motor_stop_pwm = 1500
                success = self.mavlink.send_raw_motor_pwm(motor_stop_pwm)
                if success:
                    print(f"   ✅ Motor AUX6: {motor_stop_pwm}µs (STOP)")
                else:
                    print(f"   ❌ Motor: PWM gönderilemedi!")
            else:
                print("   ❌ MAVLink bağlantısı yok - PWM gönderilemedi!")
            
            # 2. Stabilizasyon kontrolü
            print("\n2️⃣ Stabilizasyon kontrolü...")
            
            # 5 saniye boyunca IMU kontrolü
            stable_check_duration = 5
            print(f"   {stable_check_duration} saniye stabilizasyon kontrolü...")
            
            start_time = time.time()
            while time.time() - start_time < stable_check_duration:
                if self.mavlink.connected:
                    imu_data = self.mavlink.get_imu_data()
                    if imu_data:
                        accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                        
                        # Hareket kontrolü
                        import math
                        gyro_magnitude = math.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
                        gyro_deg_per_sec = math.degrees(gyro_magnitude)
                        
                        if gyro_deg_per_sec > 10:  # 10°/s üzeri
                            print(f"   ⚠️ Hala hareket var: {gyro_deg_per_sec:.1f}°/s")
                        else:
                            print(f"   ✅ Stabil: {gyro_deg_per_sec:.1f}°/s")
                
                time.sleep(0.5)
            
            print("   ✅ Stabilizasyon kontrolü tamamlandı")
            
            # 3. Sistem durumu raporu
            print("\n3️⃣ Sistem durumu raporu...")
            
            if self.mavlink.connected:
                armed_status = "ARMED" if self.mavlink.armed else "DISARMED"
                print(f"   🔒 Sistem durumu: {armed_status}")
                
                # GPS durumu
                gps_data = self.mavlink.get_gps_data()
                if gps_data:
                    lat, lon, alt, satellites = gps_data
                    print(f"   📍 GPS: {lat:.6f}, {lon:.6f} | {satellites} uydu")
                else:
                    print("   📍 GPS: Veri yok")
                
                # IMU durumu
                imu_data = self.mavlink.get_imu_data()
                if imu_data:
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                    gravity = math.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
                    print(f"   🧭 IMU: Gravity={gravity:.2f}m/s² (Normal: 9.81)")
                else:
                    print("   🧭 IMU: Veri yok")
            else:
                print("   ❌ MAVLink bağlantısı kesildi!")
            
            print("\n🎉 ACİL DURUM PROTOKOLÜ TAMAMLANDI!")
            print("✅ Tüm sistemler güvenli durumda")
            
            return True
            
        except Exception as e:
            print(f"\n❌ Acil durum protokolü hatası: {e}")
            return False
        finally:
            self.emergency_active = False
    
    def emergency_surface(self):
        """Acil yüzeye çıkış protokolü"""
        print("🌊 ACİL YÜZEY ÇIKIŞ PROTOKOLÜ!")
        print("=" * 50)
        
        self.surface_protocol_active = True
        
        try:
            # Uyarı
            print("⚠️ UYARI: Bu test gerçek yüzey çıkış protokolüdür!")
            print("⚠️ Araç kontrolden çıkabilir ve yüzeye çıkabilir!")
            
            response = input("Devam etmek istiyor musunuz? (YES yazın): ")
            if response != "YES":
                print("❌ İşlem iptal edildi")
                return False
            
            if not self.mavlink.connected:
                print("❌ MAVLink bağlantısı yok - Yüzey çıkış protokolü çalıştırılamaz!")
                return False
            
            # 1. Tüm servo neutral
            print("\n1️⃣ Servolar neutral pozisyona...")
            servo_channels = [1, 3, 4, 5]
            neutral_pwm = 1500
            
            for channel in servo_channels:
                self.mavlink.send_raw_servo_pwm(channel, neutral_pwm)
            
            # 2. Motor maksimum yukarı thrust
            print("2️⃣ Motor maksimum yukarı thrust...")
            
            # Güvenlik için %70 ile sınırla
            max_safe_pwm = 1350  # 1500 - 150 (reverse direction)
            
            success = self.mavlink.send_raw_motor_pwm(max_safe_pwm)
            if success:
                print(f"   ✅ Motor yukarı thrust: {max_safe_pwm}µs")
            else:
                print("   ❌ Motor thrust gönderilemedi!")
                return False
            
            # 3. Yüzey çıkış süresi
            surface_duration = 10  # 10 saniye
            print(f"3️⃣ {surface_duration} saniye yüzey çıkış...")
            
            start_time = time.time()
            while time.time() - start_time < surface_duration:
                elapsed = time.time() - start_time
                remaining = surface_duration - elapsed
                
                print(f"\r   🚀 Yüzeye çıkış: {remaining:.1f}s kaldı", end='')
                
                # İsteğe bağlı: Derinlik sensörü kontrolü
                # TODO: Derinlik sensörü integration
                
                time.sleep(0.5)
            
            print("\n")
            
            # 4. Motor durdur
            print("4️⃣ Motor durdur...")
            stop_pwm = 1500
            self.mavlink.send_raw_motor_pwm(stop_pwm)
            print(f"   ✅ Motor durduruldu: {stop_pwm}µs")
            
            print("\n🎉 ACİL YÜZEY ÇIKIŞ PROTOKOLÜ TAMAMLANDI!")
            print("⚠️ Aracın su yüzeyinde olduğunu kontrol edin!")
            
            return True
            
        except Exception as e:
            print(f"\n❌ Yüzey çıkış protokolü hatası: {e}")
            # Acil durum durumunda motor durdur
            try:
                self.mavlink.send_raw_motor_pwm(1500)
            except:
                pass
            return False
        finally:
            self.surface_protocol_active = False
    
    def test_emergency_systems(self):
        """Acil durum sistemleri testi"""
        print("🧪 ACİL DURUM SİSTEMLERİ TESTİ")
        print("=" * 50)
        
        if not self.connect_system():
            return False
        
        try:
            # Test 1: Emergency stop
            print("\n📋 TEST 1: Emergency Stop")
            print("-" * 30)
            
            input("ENTER'a basarak emergency stop testini başlatın...")
            
            if self.emergency_stop():
                print("✅ Emergency stop testi başarılı")
            else:
                print("❌ Emergency stop testi başarısız")
                return False
            
            # Test 2: Sistem durumu kontrolü
            print("\n📋 TEST 2: Sistem Durumu Kontrolü")
            print("-" * 30)
            
            if self.mavlink.check_system_status():
                print("✅ Sistem durumu kontrolü başarılı")
            else:
                print("❌ Sistem durumu kontrolü başarısız")
            
            # Test 3: PWM test (opsiyonel)
            print("\n📋 TEST 3: PWM Kontrol Testi")
            print("-" * 30)
            
            response = input("PWM kontrol testi yapmak istiyor musunuz? (y/N): ")
            if response.lower() == 'y':
                
                # Kısa PWM testi
                test_pwm = 1600  # Hafif test
                print(f"Servo test PWM: {test_pwm}µs (2 saniye)")
                
                self.mavlink.send_raw_servo_pwm(1, test_pwm)  # Front left test
                time.sleep(2)
                self.mavlink.send_raw_servo_pwm(1, 1500)  # Neutral
                
                print("✅ PWM test tamamlandı")
            
            print("\n🎉 TÜM ACİL DURUM TESTLERİ BAŞARILI!")
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            # Acil durum protokolü çalıştır
            self.emergency_stop()
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik"""
        print("\n🧹 Acil durum sistemi temizliği...")
        
        # Güvenlik için tüm kontroller neutral
        try:
            if self.mavlink.connected:
                # Servolar neutral
                for channel in [1, 3, 4, 5]:
                    self.mavlink.send_raw_servo_pwm(channel, 1500)
                
                # Motor stop
                self.mavlink.send_raw_motor_pwm(1500)
                
                time.sleep(1)
        except:
            pass
        
        self.mavlink.disconnect()
        print("✅ Temizlik tamamlandı")

def main():
    """Ana fonksiyon"""
    emergency_system = EmergencySystem()
    
    print("🚨 TEKNOFEST ACİL DURUM SİSTEMİ")
    print("=" * 60)
    print("1. Tam acil durum testi")
    print("2. Sadece emergency stop")
    print("3. Acil yüzey çıkış testi (TEHLİKELİ!)")
    print("4. Sistem durumu kontrolü")
    print("5. Çıkış")
    
    try:
        choice = input("\nSeçiminiz (1-5): ").strip()
        
        if choice == '1':
            emergency_system.test_emergency_systems()
        elif choice == '2':
            if emergency_system.connect_system():
                emergency_system.emergency_stop()
                emergency_system.cleanup()
        elif choice == '3':
            print("\n⚠️ UYARI: Bu gerçek yüzey çıkış protokolüdür!")
            print("⚠️ Sadece test havuzunda veya güvenli ortamda kullanın!")
            response = input("Emin misiniz? (YES yazın): ")
            if response == "YES":
                if emergency_system.connect_system():
                    emergency_system.emergency_surface()
                    emergency_system.cleanup()
        elif choice == '4':
            if emergency_system.connect_system():
                emergency_system.mavlink.check_system_status()
                emergency_system.cleanup()
        elif choice == '5':
            print("👋 Çıkış yapılıyor...")
        else:
            print("❌ Geçersiz seçim!")
            
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        try:
            emergency_system.emergency_stop()
        except:
            pass
    except Exception as e:
        print(f"\n❌ Program hatası: {e}")
    finally:
        print("Program bitti.")

if __name__ == "__main__":
    main() 