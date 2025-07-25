#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Servo Kalibrasyonu
4 Servo (AUX 1,3,4,5) Otomatik Kalibrasyon Scripti
"""

import sys
import os
import time
import json

# Parent directory import
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from mavlink_handler import MAVLinkHandler

class ServoCalibration:
    def __init__(self):
        """Servo kalibrasyon sistemi"""
        self.mavlink = MAVLinkHandler()
        self.load_config()
        
    def load_config(self):
        """Konfigürasyon yükle"""
        try:
            with open('../config/hardware_config.json', 'r') as f:
                config = json.load(f)
            
            self.servo_channels = config['pixhawk']['servos']
            self.pwm_limits = config['pixhawk']['pwm_limits']
            
            print("✅ Konfigürasyon yüklendi")
            print(f"📍 Servo kanalları: {self.servo_channels}")
            
        except Exception as e:
            print(f"❌ Konfigürasyon hatası: {e}")
            # Varsayılan değerler
            self.servo_channels = {
                "front_left": 1, "rear_left": 3, 
                "rear_right": 4, "front_right": 5
            }
            self.pwm_limits = {
                "servo_min": 1100, "servo_max": 1900, "servo_neutral": 1500
            }
    
    def connect_system(self):
        """Sisteme bağlan"""
        print("🔌 MAVLink bağlantısı kuruluyor...")
        
        if not self.mavlink.connect():
            print("❌ Bağlantı başarısız!")
            return False
        
        print("✅ MAVLink bağlantısı başarılı")
        
        # ARM kontrol et
        if not self.mavlink.armed:
            print("⚠️ Sistem DISARMED - ARM ediliyor...")
            if not self.mavlink.arm_system():
                print("❌ ARM işlemi başarısız!")
                return False
            print("✅ Sistem ARM edildi")
        
        return True
    
    def calibrate_servo(self, servo_name, channel):
        """Tek servo kalibrasyonu"""
        print(f"\n🔧 {servo_name} kalibrasyonu (Kanal {channel})")
        print("-" * 40)
        
        # Test pozisyonları
        positions = [
            (self.pwm_limits["servo_neutral"], "Neutral"),
            (self.pwm_limits["servo_min"], "Minimum"), 
            (self.pwm_limits["servo_max"], "Maximum"),
            (self.pwm_limits["servo_neutral"], "Neutral (Son)")
        ]
        
        for pwm, description in positions:
            print(f"📍 {description}: {pwm}µs")
            
            # PWM gönder
            success = self.mavlink.send_raw_servo_pwm(channel, pwm)
            if not success:
                print(f"❌ {servo_name} PWM gönderimi başarısız!")
                return False
            
            # Pozisyon bekle
            time.sleep(2.0)
        
        print(f"✅ {servo_name} kalibrasyonu tamamlandı")
        return True
    
    def full_calibration(self):
        """Tüm servo kalibrasyonu"""
        print("🚀 SERVO KALİBRASYON SİSTEMİ")
        print("=" * 50)
        
        if not self.connect_system():
            return False
        
        try:
            # Her servo için kalibrasyon
            for servo_name, channel in self.servo_channels.items():
                if not self.calibrate_servo(servo_name, channel):
                    print(f"❌ {servo_name} kalibrasyonu başarısız!")
                    return False
                
                # Servoların arasında kısa bekleme
                time.sleep(1.0)
            
            print("\n🎉 TÜM SERVO KALİBRASYONU BAŞARILI!")
            print("=" * 50)
            
            # Final test - Tüm servolar eşzamanlı neutral
            print("🔄 Final test - Tüm servolar neutral pozisyonda")
            
            for servo_name, channel in self.servo_channels.items():
                self.mavlink.send_raw_servo_pwm(channel, self.pwm_limits["servo_neutral"])
            
            time.sleep(3.0)
            print("✅ Final test tamamlandı")
            
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Kalibrasyon kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Kalibrasyon hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def interactive_calibration(self):
        """İnteraktif kalibrasyon"""
        print("🎮 İNTERAKTİF SERVO KALİBRASYONU")
        print("=" * 50)
        
        if not self.connect_system():
            return False
        
        try:
            while True:
                print("\nMevcut servolar:")
                servo_list = list(self.servo_channels.items())
                for i, (name, channel) in enumerate(servo_list, 1):
                    print(f"  {i}. {name} (Kanal {channel})")
                
                print("  0. Çıkış")
                print("  9. Tüm servolar neutral")
                
                choice = input("\nSeçiminiz: ").strip()
                
                if choice == '0':
                    break
                elif choice == '9':
                    print("🔄 Tüm servolar neutral pozisyona ayarlanıyor...")
                    for name, channel in self.servo_channels.items():
                        self.mavlink.send_raw_servo_pwm(channel, self.pwm_limits["servo_neutral"])
                    time.sleep(2.0)
                    print("✅ Tamamlandı")
                    
                elif choice.isdigit():
                    idx = int(choice) - 1
                    if 0 <= idx < len(servo_list):
                        servo_name, channel = servo_list[idx]
                        
                        # PWM değeri iste
                        try:
                            pwm = int(input(f"{servo_name} için PWM değeri (1100-1900): "))
                            if self.pwm_limits["servo_min"] <= pwm <= self.pwm_limits["servo_max"]:
                                self.mavlink.send_raw_servo_pwm(channel, pwm)
                                print(f"✅ {servo_name}: {pwm}µs gönderildi")
                            else:
                                print("❌ PWM değeri geçersiz!")
                        except ValueError:
                            print("❌ Geçerli bir sayı girin!")
                    else:
                        print("❌ Geçersiz seçim!")
                else:
                    print("❌ Geçersiz seçim!")
                    
        except KeyboardInterrupt:
            print("\n⚠️ İnteraktif kalibrasyon sonlandırıldı")
        finally:
            self.cleanup()
    
    def range_test(self):
        """Servo aralık testi"""
        print("📐 SERVO ARALIK TESTİ")
        print("=" * 50)
        
        if not self.connect_system():
            return False
        
        try:
            for servo_name, channel in self.servo_channels.items():
                print(f"\n🔧 {servo_name} aralık testi")
                
                # Min'den Max'a sweep
                print("🔄 Min→Max sweep...")
                for pwm in range(self.pwm_limits["servo_min"], 
                               self.pwm_limits["servo_max"] + 1, 20):
                    self.mavlink.send_raw_servo_pwm(channel, pwm)
                    time.sleep(0.05)
                
                # Max'tan Min'e sweep
                print("🔄 Max→Min sweep...")
                for pwm in range(self.pwm_limits["servo_max"], 
                               self.pwm_limits["servo_min"] - 1, -20):
                    self.mavlink.send_raw_servo_pwm(channel, pwm)
                    time.sleep(0.05)
                
                # Neutral'a dön
                self.mavlink.send_raw_servo_pwm(channel, self.pwm_limits["servo_neutral"])
                print(f"✅ {servo_name} aralık testi tamamlandı")
                time.sleep(1.0)
            
            print("\n🎉 TÜM ARALIK TESTLERİ BAŞARILI!")
            
        except KeyboardInterrupt:
            print("\n⚠️ Aralık testi durduruldu")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik"""
        print("\n🧹 Temizlik yapılıyor...")
        
        # Tüm servolar neutral'a
        try:
            for servo_name, channel in self.servo_channels.items():
                self.mavlink.send_raw_servo_pwm(channel, self.pwm_limits["servo_neutral"])
            time.sleep(1.0)
        except:
            pass
        
        # Bağlantıyı kapat
        self.mavlink.disconnect()
        print("✅ Temizlik tamamlandı")

def main():
    """Ana fonksiyon"""
    calibration = ServoCalibration()
    
    print("🤖 TEKNOFEST SERVO KALİBRASYON SİSTEMİ")
    print("=" * 60)
    print("1. Otomatik kalibrasyon (Tüm servolar)")
    print("2. İnteraktif kalibrasyon (Manuel)")
    print("3. Aralık testi (Sweep test)")
    print("4. Çıkış")
    
    try:
        choice = input("\nSeçiminiz (1-4): ").strip()
        
        if choice == '1':
            calibration.full_calibration()
        elif choice == '2':
            calibration.interactive_calibration()
        elif choice == '3':
            calibration.range_test()
        elif choice == '4':
            print("👋 Çıkış yapılıyor...")
        else:
            print("❌ Geçersiz seçim!")
            
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
    except Exception as e:
        print(f"\n❌ Program hatası: {e}")
    finally:
        print("Program bitti.")

if __name__ == "__main__":
    main() 