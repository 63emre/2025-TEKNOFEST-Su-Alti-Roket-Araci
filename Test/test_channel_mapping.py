#!/usr/bin/env python3
"""
TEKNOFEST - Servo Channel Mapping Test
Hangi channel AUX1'e bağlı bulalım
"""

import time
from pymavlink import mavutil

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

class ChannelMappingTest:
    def __init__(self):
        self.master = None
        self.connected = False
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM ayarı"""
        if not self.connected:
            return False
            
        pwm_value = max(1000, min(2000, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Channel {channel} hata: {e}")
            return False
    
    def test_all_channels(self):
        """Tüm servo channelları test et"""
        print("\n🔍 TÜM SERVO CHANNEL TEST")
        print("=" * 50)
        print("Her channel için servo hareket edip etmediğini kontrol edin!")
        
        # Test edilecek channellar (ArduSub AUX channelları)
        test_channels = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        
        for channel in test_channels:
            print(f"\n📍 Channel {channel} TEST EDILIYOR:")
            print("   Servo hareket ediyor mu? (Evet için ENTER, Hayır için 'n')")
            
            # Servo sweep yap
            for pwm in [1000, 2000, 1000, 2000, 1500]:
                self.set_servo_pwm(channel, pwm)
                time.sleep(0.5)
            
            user_input = input(f"   Channel {channel} hareket etti mi? (ENTER=Evet, n=Hayır): ")
            
            if user_input.strip().lower() != 'n':
                print(f"✅ Channel {channel} = AUX1 BULUNDU!")
                return channel
                
        print("❌ Hiçbir channel'da hareket tespit edilmedi")
        return None
    
    def test_specific_channel(self, channel):
        """Belirli bir channel'ı detaylı test et"""
        print(f"\n🎯 Channel {channel} DETAYLI TEST")
        print("-" * 40)
        
        positions = [
            (1000, "Minimum"),
            (1250, "25%"),
            (1500, "Orta"), 
            (1750, "75%"),
            (2000, "Maximum"),
            (1500, "Orta")
        ]
        
        for pwm, desc in positions:
            print(f"📍 {desc}: {pwm}µs")
            self.set_servo_pwm(channel, pwm)
            time.sleep(2)
    
    def cleanup(self):
        """Temizlik"""
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    test = ChannelMappingTest()
    
    if not test.connect_pixhawk():
        return 1
    
    print("\nChannel Mapping Test:")
    print("1. Tüm channelları test et (AUX1'i bul)")
    print("2. Belirli channel test et")
    
    try:
        choice = input("Seçiminiz (1-2): ").strip()
        
        if choice == '1':
            found_channel = test.test_all_channels()
            if found_channel:
                print(f"\n🎉 AUX1 = Channel {found_channel}")
                test.test_specific_channel(found_channel)
            
        elif choice == '2':
            channel = int(input("Test edilecek channel (1-14): "))
            test.test_specific_channel(channel)
            
        else:
            print("Geçersiz seçim!")
            
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
    except Exception as e:
        print(f"❌ Hata: {e}")
    finally:
        test.cleanup()

if __name__ == "__main__":
    main() 