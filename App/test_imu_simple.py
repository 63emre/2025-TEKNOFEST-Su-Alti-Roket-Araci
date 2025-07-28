#!/usr/bin/env python3
"""
Basit IMU Test Scripti
TCP üzerinden IMU verisi alıp göster
"""

import time
import math
from pymavlink import mavutil

def test_imu_data():
    """IMU verisi test et"""
    print("🔍 IMU Test Başlatılıyor...")
    
    try:
        # Serial bağlantısı with environment variables
        import os
        serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
        baud_rate = int(os.getenv("MAV_BAUD", "115200"))
        
        print("🔌 Serial bağlantısı kuruluyor...")
        print(f"📡 Serial: {serial_port} @ {baud_rate} baud")
        master = mavutil.mavlink_connection(serial_port, baud=baud_rate, autoreconnect=True)
        
        # Heartbeat bekle
        print("💓 Heartbeat bekleniyor...")
        master.wait_heartbeat(timeout=10)
        print("✅ Bağlantı kuruldu!")
        
        # IMU stream'lerini iste - ArduSub için doğru yöntem
        print("📡 IMU stream'leri isteniyor...")
        
        # ArduSub için REQUEST_DATA_STREAM kullan
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
            10,  # 10 Hz
            1    # start
        )
        
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            10,  # 10 Hz
            1    # start
        )
        
        # Ayrıca SET_MESSAGE_INTERVAL da dene
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000, 0, 0, 0, 0, 0
        )
        
        time.sleep(1)  # Stream'lerin başlaması için bekle
        
        print("📊 IMU verisi dinleniyor (10 saniye)...")
        print("-" * 60)
        
        start_time = time.time()
        imu_count = 0
        attitude_count = 0
        raw_imu_count = 0
        scaled_imu_count = 0
        message_types = set()
        
        while time.time() - start_time < 10:
            # Herhangi bir mesaj al
            msg = master.recv_match(blocking=False, timeout=0.1)
            if msg:
                msg_type = msg.get_type()
                message_types.add(msg_type)
                
                # HIGHRES_IMU mesajı
                if msg_type == 'HIGHRES_IMU':
                    imu_count += 1
                    ax, ay, az = msg.xacc, msg.yacc, msg.zacc
                    gx, gy, gz = msg.xgyro, msg.ygyro, msg.zgyro
                    
                    # Roll/Pitch hesapla
                    if abs(az) > 0.001:
                        roll = math.degrees(math.atan2(ay, az))
                        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
                    else:
                        roll = pitch = 0
                    
                    print(f"HIGHRES_IMU #{imu_count}: Acc=({ax:.2f},{ay:.2f},{az:.2f}) | "
                          f"Gyro=({math.degrees(gx):.1f}°,{math.degrees(gy):.1f}°,{math.degrees(gz):.1f}°) | "
                          f"Roll={roll:.1f}° Pitch={pitch:.1f}°")
                
                # ATTITUDE mesajı
                elif msg_type == 'ATTITUDE':
                    attitude_count += 1
                    roll = math.degrees(msg.roll)
                    pitch = math.degrees(msg.pitch)
                    yaw = math.degrees(msg.yaw)
                    print(f"ATTITUDE #{attitude_count}: Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°")
                
                # RAW_IMU mesajı
                elif msg_type == 'RAW_IMU':
                    raw_imu_count += 1
                    if raw_imu_count <= 3:  # İlk 3 mesajı göster
                        print(f"RAW_IMU #{raw_imu_count}: xacc={msg.xacc} yacc={msg.yacc} zacc={msg.zacc}")
                
                # SCALED_IMU mesajı
                elif msg_type == 'SCALED_IMU':
                    scaled_imu_count += 1
                    if scaled_imu_count <= 3:  # İlk 3 mesajı göster
                        print(f"SCALED_IMU #{scaled_imu_count}: xacc={msg.xacc} yacc={msg.yacc} zacc={msg.zacc}")
            
            time.sleep(0.05)  # Daha hızlı polling
        
        print("-" * 60)
        print(f"📈 SONUÇ:")
        print(f"   HIGHRES_IMU: {imu_count} mesaj")
        print(f"   ATTITUDE: {attitude_count} mesaj")
        print(f"   RAW_IMU: {raw_imu_count} mesaj")
        print(f"   SCALED_IMU: {scaled_imu_count} mesaj")
        print(f"   Toplam mesaj türü: {len(message_types)}")
        
        if message_types:
            print(f"📡 Alınan mesaj türleri: {', '.join(sorted(message_types))}")
        
        if imu_count == 0 and attitude_count == 0 and raw_imu_count == 0:
            print("❌ Hiç IMU verisi alınamadı!")
            print("💡 ArduSub parametrelerini kontrol et")
        else:
            print("✅ IMU verisi alınıyor!")
        
        master.close()
        
    except Exception as e:
        print(f"❌ Hata: {e}")

if __name__ == "__main__":
    test_imu_data() 