#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D300 Direct Test - Verilen koda göre basit test
Mevcut sensors.py'deki sorunları tespit etmek için
"""

from pymavlink import mavutil
import time

CONN = "/dev/ttyACM0"
BAUD = 115200

SEA_LEVEL_PRESSURE_PA = 101325.0
FLUID_DENSITY = 1025.0  # Deniz suyu
G = 9.80665

def to_depth_m(press_hpa: float) -> float:
    pa = press_hpa * 100.0
    dp = max(0.0, pa - SEA_LEVEL_PRESSURE_PA)
    print(f"Basınç PA: {pa:.1f}, Fark: {dp:.1f}")
    return dp / (FLUID_DENSITY * G)

def test_d300_direct():
    """Verilen koda göre direkt D300 test"""
    print("🧪 D300 Direkt Test - Verilen Kod")
    print("=" * 50)
    
    try:
        print("MAVLink bağlantısı kuruluyor...")
        m = mavutil.mavlink_connection(CONN, baud=BAUD)
        print("MAVLink connection oluşturuldu")
        
        print("Heartbeat bekleniyor...")
        if not m.wait_heartbeat(timeout=10):
            print("❌ Heartbeat yok — bağlantıyı/portu kontrol edin.")
            return False

        print("✅ Heartbeat alındı")
        print(f"Target system: {m.target_system}, component: {m.target_component}")

        # Verilen kodun exact kopyası
        print("SCALED_PRESSURE2 veri akışı isteniyor...")
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
            int(1e6/5), 0, 0, 0, 0, 0
        )

        print("✅ Veri akışı isteği gönderildi")
        print("D300 verisi bekleniyor...")

        # 10 örnek al
        for i in range(10):
            print(f"Örnek {i+1}/10...")
            msg = m.recv_match(type="SCALED_PRESSURE2", blocking=True, timeout=2)
            if not msg:
                print(f"❌ {i+1}. mesaj boş veya gelmiyor")
                continue
                
            press_hpa = float(getattr(msg, "press_abs", 0.0))
            temp_c = float(getattr(msg, "temperature", 0)) / 100.0
            depth_m = to_depth_m(press_hpa)
            
            print(f"✅ Derinlik: {depth_m:.3f}m | Basınç: {press_hpa:.1f}hPa | Sıcaklık: {temp_c:.1f}°C")
            
            time.sleep(0.5)
        
        print("✅ D300 direkt test başarılı!")
        return True
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        return False
    finally:
        try:
            m.close()
        except:
            pass

if __name__ == "__main__":
    test_d300_direct()
