#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - AUX1 Servo Test
Basit servo döndürme testi - 330Hz frekans ayarlı
Pixhawk AUX OUT 1 servo motor kontrolü ve test
"""

import time
from pymavlink import mavutil
import threading
import platform

# MAVLink Serial bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
import os
try:
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Using dynamic serial connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to serial config with environment variables
    # Platform-based default port detection
    if platform.system() == "Windows":
        default_port = "COM17"
    else:
        # Linux/Unix systems
        default_port = "/dev/ttyACM0"
    
    serial_port = os.getenv("MAV_ADDRESS", default_port)
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Using fallback serial connection: {MAV_ADDRESS} (Platform: {platform.system()})")

# AUX1 servo kanal (Pixhawk AUX OUT 1 = Servo channel 7 - Disabled function)
SERVO_CHANNEL = 11

# Servo frekansı (Hz)
SERVO_FREQUENCY = 330

# PWM değer aralıkları
PWM_MIN = 1000    # Minimum PWM (µs)
PWM_MID = 1500    # Orta PWM (µs) 
PWM_MAX = 2000    # Maksimum PWM (µs)

class AUX1ServoTest:
    def __init__(self):
        self.master = None
        self.connected = False
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor...")
            
            # Handle serial vs TCP connection
            if ',' in MAV_ADDRESS:
                # Serial connection: port,baud
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                # TCP or other connection
                print(f"🌐 TCP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Servo frekansını ayarla
            self.set_servo_frequency(SERVO_FREQUENCY)
            
            # Frekans ayarını doğrula
            current_freq = self.get_servo_frequency()
            if current_freq != SERVO_FREQUENCY:
                print(f"⚠️ Frekans doğrulaması başarısız: Hedef {SERVO_FREQUENCY}Hz, Mevcut {current_freq}Hz")
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def set_servo_frequency(self, frequency):
        """Servo frekansını ayarla (Hz)"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
            
        try:
            print(f"🔧 Servo frekansı ayarlanıyor: {frequency}Hz")
            
            # AUX output frekansını ayarla (PWM_AUX_RATE parametresi)
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'PWM_AUX_RATE',  # Parameter name
                frequency,        # Parameter value
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            
            # Parametrenin ayarlandığından emin olmak için kısa süre bekle
            time.sleep(0.5)
            
            print(f"✅ Servo frekansı {frequency}Hz olarak ayarlandı")
            return True
            
        except Exception as e:
            print(f"❌ Frekans ayarlama hatası: {e}")
            return False
    
    def get_servo_frequency(self):
        """Mevcut servo frekansını oku"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return None
            
        try:
            # PWM_AUX_RATE parametresini oku
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                b'PWM_AUX_RATE',
                -1
            )
            
            # Cevap bekle
            msg = self.master.recv_match(type='PARAM_VALUE', timeout=5)
            if msg:
                frequency = int(msg.param_value)
                print(f"📡 Mevcut servo frekansı: {frequency}Hz")
                return frequency
            else:
                print("⚠️ Frekans parametresi okunamadı")
                return None
                
        except Exception as e:
            print(f"❌ Frekans okuma hatası: {e}")
            return None
    
    def set_servo_pwm(self, pwm_value):
        """AUX1 servo PWM ayarı"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
            
        # PWM değer kontrolü
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            # MAVLink servo komut gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                SERVO_CHANNEL,  # AUX1 = Servo channel 9
                pwm_value,      # PWM value
                0, 0, 0, 0, 0
            )
            
            print(f"📍 AUX1 Servo: {pwm_value}µs")
            return True
            
        except Exception as e:
            print(f"❌ Servo kontrol hatası: {e}")
            return False
    
    def test_servo_positions(self):
        """Temel pozisyon testi"""
        print("\n🔧 AUX1 SERVO POZİSYON TESTİ")
        print("-" * 40)
        
        positions = [
            (PWM_MID, "Orta Pozisyon"),
            (PWM_MIN, "Minimum Pozisyon"), 
            (PWM_MAX, "Maksimum Pozisyon"),
            (PWM_MID, "Orta Pozisyon")
        ]
        
        for pwm, description in positions:
            print(f"📍 {description}: {pwm}µs")
            self.set_servo_pwm(pwm)
            time.sleep(3)  # 3 saniye bekle
            
        print("✅ Pozisyon testi tamamlandı!")
    
    def test_servo_sweep(self):
        """Servo tarama testi"""
        print("\n🌊 AUX1 SERVO SWEEP TESTİ")
        print("-" * 40)
        
        print("🔄 5 saniye boyunca min-max tarama...")
        
        start_time = time.time()
        while time.time() - start_time < 5:
            # Min'den Max'a
            for pwm in range(PWM_MIN, PWM_MAX + 1, 20):
                self.set_servo_pwm(pwm)
                time.sleep(0.05)
            
            # Max'tan Min'e
            for pwm in range(PWM_MAX, PWM_MIN - 1, -20):
                self.set_servo_pwm(pwm)
                time.sleep(0.05)
        
        # Orta pozisyona dön
        self.set_servo_pwm(PWM_MID)
        print("✅ Sweep testi tamamlandı!")
    
    def test_step_movement(self):
        """Adım adım hareket testi"""
        print("\n📐 AUX1 SERVO ADIM TESTİ")
        print("-" * 40)
        
        steps = [1000, 1250, 1500, 1750, 2000, 1500]
        
        for i, pwm in enumerate(steps):
            print(f"📍 Adım {i+1}: {pwm}µs")
            self.set_servo_pwm(pwm)
            time.sleep(2)
        
        print("✅ Adım testi tamamlandı!")
    
    def interactive_test(self):
        """Interaktif test"""
        print("\n🎮 AUX1 SERVO İNTERAKTİF TEST")
        print("-" * 40)
        print("PWM değeri girin (1000-2000) veya 'q' ile çıkın:")
        
        while True:
            try:
                user_input = input("PWM > ").strip()
                
                if user_input.lower() == 'q':
                    break
                    
                pwm_value = int(user_input)
                
                if PWM_MIN <= pwm_value <= PWM_MAX:
                    self.set_servo_pwm(pwm_value)
                else:
                    print(f"⚠️ PWM değeri {PWM_MIN}-{PWM_MAX} arasında olmalı!")
                    
            except ValueError:
                print("⚠️ Geçerli bir sayı girin!")
            except KeyboardInterrupt:
                break
        
        # Orta pozisyona dön
        self.set_servo_pwm(PWM_MID)
        print("✅ İnteraktif test tamamlandı!")
    
    def run_all_tests(self):
        """Tüm testleri çalıştır"""
        print("🧪 AUX1 SERVO TAM TEST PAKETİ")
        print("=" * 50)
        print(f"📡 Servo Frekansı: {SERVO_FREQUENCY}Hz")
        print(f"📍 Servo Kanalı: {SERVO_CHANNEL}")
        print("-" * 50)
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        try:
            # 1. Pozisyon testi
            self.test_servo_positions()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 2. Sweep testi
            self.test_servo_sweep()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 3. Adım testi
            self.test_step_movement()
            
            input("\n⏸️ İnteraktif test için ENTER'a basın...")
            
            # 4. İnteraktif test
            self.interactive_test()
            
            print("\n🎉 TÜM AUX1 SERVO TESTLERİ BAŞARILI!")
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik"""
        # Servo orta pozisyon
        if self.connected:
            self.set_servo_pwm(PWM_MID)
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    servo_test = AUX1ServoTest()
    
    print(f"🚀 TEKNOFEST AUX1 Servo Test - Frekans: {SERVO_FREQUENCY}Hz")
    print("=" * 60)
    print("AUX1 Servo Test Menüsü:")
    print("1. Tam test paketi")
    print("2. Sadece pozisyon testi")
    print("3. Sadece sweep testi")
    print("4. İnteraktif test")
    
    try:
        choice = input("Seçiminiz (1-4): ").strip()
        
        if not servo_test.connect_pixhawk():
            return 1
            
        if choice == '1':
            servo_test.run_all_tests()
        elif choice == '2':
            servo_test.test_servo_positions()
        elif choice == '3':
            servo_test.test_servo_sweep()
        elif choice == '4':
            servo_test.interactive_test()
        else:
            print("Geçersiz seçim!")
            return 1
            
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1
    finally:
        servo_test.cleanup()

if __name__ == "__main__":
    import sys
    sys.exit(main()) 