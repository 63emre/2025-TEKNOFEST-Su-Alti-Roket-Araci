#!/usr/bin/env python3
"""
TEKNOFEST - Tüm Output Test
Hangi fiziksel port çalışıyor bulalım
"""

import time
from pymavlink import mavutil

# MAVLink bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
import os
try:
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Using dynamic serial connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to serial config with environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Using fallback serial connection: {MAV_ADDRESS}")

def test_all_outputs():
    """Tüm output'ları test et"""
    try:
        print("🔌 Pixhawk'a bağlanılıyor...")
        
        # Handle serial vs TCP connection
        if ',' in MAV_ADDRESS:
            # Serial connection: port,baud
            port, baud = MAV_ADDRESS.split(',')
            print(f"📡 Serial: {port} @ {baud} baud")
            master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
        else:
            # TCP or other connection
            print(f"🌐 TCP: {MAV_ADDRESS}")
            master = mavutil.mavlink_connection(MAV_ADDRESS)
        
        print("💓 Heartbeat bekleniyor...")
        master.wait_heartbeat(timeout=15)
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