#!/usr/bin/env python3
"""
TEKNOFEST - Physical Channel LED Test
Hangi fiziksel port hangi channel bulalım
"""

import time
from pymavlink import mavutil

MAV_ADDRESS = 'tcp:127.0.0.1:5777'

def test_with_led():
    try:
        print("🔌 Pixhawk'a bağlanılıyor...")
        master = mavutil.mavlink_connection(MAV_ADDRESS)
        master.wait_heartbeat(timeout=10)
        print("✅ Bağlantı başarılı!")
        
        print("\n💡 LED TEST - AUX Portlarını Tespit Et")
        print("=" * 50)
        print("LED'i her AUX portuna sırayla tak ve gözlemle!")
        print("LED yanarsa o channel aktif demektir.")
        print()
        print("🔌 LED Bağlantısı:")
        print("   LED Uzun bacak (+) -> Signal pin") 
        print("   LED Kısa bacak (-) -> Ground pin")
        print()
        
        # Test all possible channels
        for channel in range(1, 17):  # 1-16 channels
            print(f"\n📍 CHANNEL {channel} TEST:")
            print("   LED bu channel için yanar mı?")
            
            # Blink pattern for identification
            for i in range(5):  # 5 times blink
                # HIGH
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, channel, 2000, 0, 0, 0, 0, 0
                )
                print(f"   Blink {i+1} - HIGH", end="")
                time.sleep(0.5)
                
                # LOW
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, channel, 1000, 0, 0, 0, 0, 0
                )
                print(" -> LOW")
                time.sleep(0.5)
            
            response = input(f"\n   Channel {channel} LED yanıp söndü mü? (y/n/q): ")
            
            if response.lower() == 'y':
                print(f"✅ AKTİF CHANNEL BULUNDU: {channel}")
                
                # Test servo on this channel
                print(f"   Servo'yu bu channel'a tak ve test et...")
                
                for pwm in [1000, 1500, 2000, 1500]:
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                        0, channel, pwm, 0, 0, 0, 0, 0
                    )
                    print(f"   Servo PWM: {pwm}µs")
                    time.sleep(2)
                
                servo_works = input("   Servo çalıştı mı? (y/n): ")
                if servo_works.lower() == 'y':
                    print(f"🎉 ÇALIŞAN SERVO CHANNEL: {channel}")
                    return channel
                    
            elif response.lower() == 'q':
                break
        
        print("❌ Test tamamlandı")
        master.close()
        
    except Exception as e:
        print(f"❌ Hata: {e}")

if __name__ == "__main__":
    test_with_led() 