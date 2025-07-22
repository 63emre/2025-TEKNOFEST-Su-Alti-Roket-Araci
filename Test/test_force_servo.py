#!/usr/bin/env python3
"""
TEKNOFEST - Force Servo Test
Function atamalarını bypass ederek servo test
"""

import time
from pymavlink import mavutil

MAV_ADDRESS = 'tcp:127.0.0.1:5777'

def force_servo_test():
    try:
        print("🔌 Pixhawk'a bağlanılıyor...")
        master = mavutil.mavlink_connection(MAV_ADDRESS)
        master.wait_heartbeat(timeout=10)
        print("✅ Bağlantı başarılı!")
        
        # RC override ile servo kontrolü
        print("\n🚀 RC OVERRIDE İLE SERVO KONTROLÜ")
        print("Servo hangi AUX port'a takılı? (AUX1=9, AUX2=10, AUX3=11, AUX4=12)")
        
        channel = input("Servo channel (9-16): ")
        channel = int(channel)
        
        print(f"\n📍 Channel {channel} RC Override testi:")
        
        # RC override mesajı gönder
        channels = [65535] * 18  # Initialize all channels
        
        # Test PWM values
        test_values = [1000, 1250, 1500, 1750, 2000, 1500]
        
        for i, pwm in enumerate(test_values):
            print(f"PWM: {pwm}µs")
            
            channels[channel-1] = pwm  # RC channels are 0-indexed
            
            master.mav.rc_channels_override_send(
                master.target_system,
                master.target_component,
                *channels
            )
            
            time.sleep(2)
        
        # Clear override
        channels = [65535] * 18
        master.mav.rc_channels_override_send(
            master.target_system,
            master.target_component,
            *channels
        )
        
        print("✅ RC Override test tamamlandı")
        
        # Alternative: Direct PWM output
        print("\n🎯 DIRECT PWM OUTPUT TEST:")
        
        for pwm in [1000, 1500, 2000, 1500]:
            print(f"Direct PWM: {pwm}µs")
            
            # Set servo direct output
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,    # servo number
                pwm,        # PWM value
                0, 0, 0, 0, 0
            )
            
            time.sleep(2)
        
        print("✅ Direct PWM test tamamlandı")
        
        # Set PWM output frequency (if needed)
        print("\n⚡ PWM FREQUENCY AYARI:")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            50,     # 50Hz PWM frequency
            20000,  # 20ms interval
            0, 0, 0, 0, 0
        )
        
        master.close()
        
    except Exception as e:
        print(f"❌ Hata: {e}")

if __name__ == "__main__":
    force_servo_test() 