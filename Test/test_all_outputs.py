#!/usr/bin/env python3
"""
TEKNOFEST - Tüm Output Test
Hangi fiziksel port çalışıyor bulalım
"""

import time
from pymavlink import mavutil

MAV_ADDRESS = 'tcp:127.0.0.1:5777'

def test_all_outputs():
    """Tüm output'ları test et"""
    try:
        print("🔌 Pixhawk'a bağlanılıyor...")
        master = mavutil.mavlink_connection(MAV_ADDRESS)
        master.wait_heartbeat(timeout=10)
        print("✅ Bağlantı başarılı!")
        
        print("\n🎯 TÜM OUTPUT TEST")
        print("Her output için servo hareket kontrol edin!")
        print("-" * 50)
        
        for channel in range(1, 15):  # Output 1-14
            print(f"\n📍 OUTPUT {channel} TEST:")
            
            # Servo sweep
            for pwm in [1000, 2000, 1500]:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, channel, pwm, 0, 0, 0, 0, 0
                )
                print(f"   PWM: {pwm}µs")
                time.sleep(1)
            
            response = input(f"   Output {channel} hareket etti mi? (y/n/q): ")
            
            if response.lower() == 'y':
                print(f"✅ ÇALIŞAN OUTPUT BULUNDU: {channel}")
                return channel
            elif response.lower() == 'q':
                break
                
        print("❌ Hiçbir output'ta hareket tespit edilmedi")
        master.close()
        
    except Exception as e:
        print(f"❌ Hata: {e}")

if __name__ == "__main__":
    test_all_outputs() 