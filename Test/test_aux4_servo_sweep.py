#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - AUX4 Servo Sweep Test
AUX4 servo sürekli sweep hareketi testi
"""

import time
import threading
import math
from pymavlink import mavutil

# MAVLink bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
try:
    from connection_config import get_test_constants
    CONFIG = get_test_constants()
    MAV_ADDRESS = CONFIG['MAV_ADDRESS']
    print(f"📡 Using dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to static config
    MAV_ADDRESS = 'tcp:127.0.0.1:5777'
    print(f"⚠️ Using fallback connection: {MAV_ADDRESS}")

# AUX4 servo kanal (Pixhawk AUX OUT 4 = Servo channel 12)
SERVO_CHANNEL = 12

# Arduino kodundaki ayarlar
PWM_FREQUENCY = 100  # Hz (Arduino kodundan)
PERIOD_US = 1000000 // PWM_FREQUENCY  # 10000 µs

# Açı aralığı ve PWM değerleri
ANGLE_MIN = -90     # Minimum açı (derece)
ANGLE_MAX = 90      # Maksimum açı (derece)
PULSE_MIN = 1000    # Minimum PWM (µs) - Arduino yorumundaki değer
PULSE_MAX = 2000    # Maksimum PWM (µs) - Arduino yorumundaki değer

# PWM değer aralıkları (güvenlik için)
PWM_MIN = 1000    # Minimum PWM (µs)
PWM_MAX = 2000    # Maksimum PWM (µs)

class AUX4ServoSweep:
    def __init__(self):
        self.master = None
        self.connected = False
        self.current_angle = 0
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("⏳ Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Servo frekansını ayarla
            self.set_servo_frequency(PWM_FREQUENCY)
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def set_servo_frequency(self, frequency):
        """Servo frekansını ayarla"""
        try:
            # SERVO_CONFIG mesajı - servo frekansını ayarla
            self.master.mav.servo_output_raw_send(
                time_usec=int(time.time() * 1000000),
                port=0,  # Main port
                servo1_raw=0,
                servo2_raw=0,
                servo3_raw=0,
                servo4_raw=0,
                servo5_raw=0,
                servo6_raw=0,
                servo7_raw=0,
                servo8_raw=0
            )
            
            print(f"🎛️ Servo frekansı {frequency}Hz olarak ayarlandı")
            
        except Exception as e:
            print(f"⚠️ Servo frekans ayarı hatası: {e}")
    
    def angle_to_pwm(self, angle):
        """Açıyı PWM değerine çevir"""
        # Açıyı normalize et (-90 ile +90 arasında)
        angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))
        
        # Linear interpolation
        pwm_value = PULSE_MIN + (angle - ANGLE_MIN) * (PULSE_MAX - PULSE_MIN) / (ANGLE_MAX - ANGLE_MIN)
        
        # PWM limitleri içinde tut
        pwm_value = max(PWM_MIN, min(PWM_MAX, int(pwm_value)))
        
        return pwm_value
    
    def send_servo_command(self, angle):
        """Servo komutunu gönder"""
        if not self.connected:
            return False
        
        try:
            pwm_value = self.angle_to_pwm(angle)
            
            # RC_CHANNELS_OVERRIDE mesajı
            self.master.mav.rc_channels_override_send(
                self.master.target_system,
                self.master.target_component,
                0,  # Kanal 1 (not used)
                0,  # Kanal 2 (not used)
                0,  # Kanal 3 (not used)
                0,  # Kanal 4 (not used)
                0,  # Kanal 5 (not used)
                0,  # Kanal 6 (not used)
                0,  # Kanal 7 (not used)
                0,  # Kanal 8 (not used)
                0,  # Kanal 9 (AUX1) (not used)
                0,  # Kanal 10 (AUX2) (not used)
                0,  # Kanal 11 (AUX3) (not used)
                pwm_value,  # Kanal 12 (AUX4) - TARGET SERVO
                0,  # Kanal 13 (AUX5) (not used)
                0,  # Kanal 14 (AUX6) (not used)
                0,  # Kanal 15 (not used)
                0   # Kanal 16 (not used)
            )
            
            self.current_angle = angle
            return True
            
        except Exception as e:
            print(f"❌ Servo komut hatası: {e}")
            return False
    
    def sweep_test(self, sweep_duration=60, sweep_speed=1.0):
        """Sweep testi - sürekli salınım"""
        print(f"🔄 Sweep testi başlatılıyor: {sweep_duration}s süre, {sweep_speed}x hız")
        
        start_time = time.time()
        
        while time.time() - start_time < sweep_duration:
            try:
                # Sinüs dalgası ile smooth hareket
                elapsed = time.time() - start_time
                angle = ANGLE_MAX * math.sin(2 * math.pi * sweep_speed * elapsed / 10)  # 10 saniye period
                
                # Servo komutunu gönder
                if self.send_servo_command(angle):
                    pwm_value = self.angle_to_pwm(angle)
                    print(f"🎯 Açı: {angle:+6.1f}° → PWM: {pwm_value}µs")
                else:
                    print("❌ Servo komutu gönderilemedi!")
                    break
                
                time.sleep(0.1)  # 10Hz güncelleme
                
            except KeyboardInterrupt:
                print("\n⏹️ Kullanıcı tarafından durduruldu!")
                break
            except Exception as e:
                print(f"❌ Sweep hatası: {e}")
                break
        
        # Test sonunda neutral pozisyona getir
        print("🏁 Test tamamlandı, neutral pozisyona getiriliyor...")
        self.send_servo_command(0)
        time.sleep(1)
    
    def manual_control(self):
        """Manuel açı kontrolü"""
        print("🎮 Manuel kontrol modu - Açı değerleri girin (-90 ile +90 arası)")
        print("'q' ile çıkış yapabilirsiniz")
        
        while True:
            try:
                user_input = input("Açı (derece): ").strip()
                
                if user_input.lower() == 'q':
                    break
                
                angle = float(user_input)
                
                if ANGLE_MIN <= angle <= ANGLE_MAX:
                    if self.send_servo_command(angle):
                        pwm_value = self.angle_to_pwm(angle)
                        print(f"✅ Servo: {angle}° → {pwm_value}µs")
                    else:
                        print("❌ Servo komutu gönderilemedi!")
                else:
                    print(f"⚠️ Açı {ANGLE_MIN}° ile {ANGLE_MAX}° arasında olmalı!")
                    
            except ValueError:
                print("⚠️ Geçerli bir sayı girin!")
            except KeyboardInterrupt:
                print("\n⏹️ Manuel kontrol sonlandırıldı!")
                break
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.connected:
            try:
                # Servo'yu neutral pozisyona getir
                self.send_servo_command(0)
                time.sleep(0.5)
                
                # Bağlantıyı kapat
                if self.master:
                    self.master.close()
                
                self.connected = False
                print("🔌 MAVLink bağlantısı kapatıldı")
                
            except Exception as e:
                print(f"⚠️ Disconnect hatası: {e}")

def main():
    """Ana fonksiyon"""
    print("🚀 TEKNOFEST - AUX4 Servo Sweep Test")
    print("=" * 40)
    
    # Servo controller oluştur
    servo = AUX4ServoSweep()
    
    # Pixhawk'a bağlan
    if not servo.connect_pixhawk():
        print("❌ Bağlantı başarısız!")
        return
    
    try:
        while True:
            print("\n📋 Test Seçenekleri:")
            print("1: Sweep Test (60s)")
            print("2: Hızlı Sweep Test (30s)")
            print("3: Yavaş Sweep Test (120s)")
            print("4: Manuel Kontrol")
            print("Q: Çıkış")
            
            choice = input("Seçiminiz: ").strip().upper()
            
            if choice == '1':
                servo.sweep_test(60, 1.0)
            elif choice == '2':
                servo.sweep_test(30, 2.0)
            elif choice == '3':
                servo.sweep_test(120, 0.5)
            elif choice == '4':
                servo.manual_control()
            elif choice == 'Q':
                break
            else:
                print("⚠️ Geçersiz seçim!")
                
    except KeyboardInterrupt:
        print("\n⏹️ Program sonlandırıldı!")
    finally:
        servo.disconnect()

if __name__ == "__main__":
    main() 