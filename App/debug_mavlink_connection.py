#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Serial MAVLink Connection Debug
Pixhawk PX4 PIX 2.4.8 Serial Communication Debug
Environment Variable Support: MAV_ADDRESS, MAV_BAUD
"""

import os
import time
import math
from mavlink_handler import MAVLinkHandler

# Environment variables for serial connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

def test_mavlink_connection():
    """Serial MAVLink bağlantısını test et"""
    print("🔧 Serial MAVLink Debug Test Başlatılıyor...")
    print(f"📡 Serial Configuration:")
    print(f"   Port: {MAV_ADDRESS}")
    print(f"   Baud: {MAV_BAUD}")
    
    # MAVLink handler oluştur
    handler = MAVLinkHandler()
    
    # Bağlantıyı test et
    print(f"📡 Serial {MAV_ADDRESS}@{MAV_BAUD} bağlantısı test ediliyor...")
    if handler.connect():
        print("✅ Serial MAVLink bağlantısı başarılı!")
        
        # Sistem durumunu kontrol et
        print("\n🔍 Sistem durumu kontrol ediliyor...")
        handler.check_system_status()
        print(f"   Armed: {handler.armed}")
        print(f"   Connected: {handler.connected}")
        
        # IMU veri testini yap
        print("\n📊 IMU Veri Testi (10 saniye)...")
        imu_count = 0
        test_duration = 10
        start_time = time.time()
        
        while time.time() - start_time < test_duration:
            imu_data = handler.get_imu_data()
            if imu_data:
                imu_count += 1
                if imu_count % 10 == 0:  # Her 10 veriden birini göster
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                    print(f"   IMU #{imu_count}: Accel=({accel_x:.3f}, {accel_y:.3f}, {accel_z:.3f}) Gyro=({math.degrees(gyro_x):.1f}°, {math.degrees(gyro_y):.1f}°, {math.degrees(gyro_z):.1f}°)")
            
            time.sleep(0.1)
        
        print(f"\n📈 IMU Test Sonucu: {test_duration} saniyede {imu_count} veri paketi alındı")
        print(f"   Data Rate: {imu_count/test_duration:.1f} Hz")
        
        if imu_count == 0:
            print("❌ IMU verisi alınamadı!")
            print("💡 Pixhawk'da RAW_IMU/ATTITUDE mesajları aktif mi kontrol edin")
            print("💡 ArduSub firmware çalışıyor mu kontrol edin")
        else:
            print("✅ IMU verileri alınıyor!")
        
        # ARM testi
        print(f"\n🔐 ARM Testi...")
        if not handler.armed:
            print("⚠️ Sistem DISARMED durumda")
            print("💡 Servo test için ARM etmek gerekiyor")
            
            # ARM etmeyi dene
            print("🔴 ARM etmeyi deniyorum...")
            if handler.arm_system():
                print("✅ ARM başarılı!")
                
                # Servo test
                print("\n🎮 Servo Komut Testi...")
                print("   Roll=10°, Pitch=5°, Yaw=15° gönderiliyor...")
                
                success = handler.control_servos_raw(10, 5, 15)
                if success:
                    print("✅ Servo komutları gönderildi!")
                    print("💡 AUX pinlerinde çıkış olmalı")
                else:
                    print("❌ Servo komutları gönderilemedi!")
                
                time.sleep(2)
                
                # Servo'ları sıfırla
                print("   Servo'ları sıfırlıyorum...")
                handler.control_servos_raw(0, 0, 0)
                
                # DISARM et
                print("🟢 DISARM ediyorum...")
                handler.disarm_system()
                
            else:
                print("❌ ARM başarısız!")
                print("💡 Pixhawk pre-arm check'lerini kontrol edin")
        else:
            print("✅ Sistem zaten ARM durumda!")
        
        # Bağlantıyı kapat
        print("\n🔌 Bağlantı kapatılıyor...")
        handler.disconnect()
        print("✅ Test tamamlandı!")
        
    else:
        print("❌ Serial MAVLink bağlantısı başarısız!")
        print(f"💡 {MAV_ADDRESS} portu'nda Pixhawk bağlı mı?")
        print(f"💡 Baud rate {MAV_BAUD} doğru mu?")
        print("💡 ArduSub firmware çalışıyor mu?")

def test_raw_mavlink():
    """Ham MAVLink mesajlarını test et"""
    print("\n🔧 Ham Serial MAVLink Mesaj Testi...")
    
    try:
        from pymavlink import mavutil
        
        # Direkt serial bağlantı
        print(f"📡 Ham serial MAVLink bağlantısı kuruluyor: {MAV_ADDRESS}@{MAV_BAUD}")
        connection = mavutil.mavlink_connection(MAV_ADDRESS, baud=MAV_BAUD)
        
        # Heartbeat bekle
        print("💓 Heartbeat bekleniyor...")
        heartbeat = connection.wait_heartbeat(timeout=15)
        
        if heartbeat:
            print("✅ Heartbeat alındı!")
            print(f"   System ID: {heartbeat.get_srcSystem()}")
            print(f"   Component ID: {heartbeat.get_srcComponent()}")
            print(f"   Vehicle Type: {heartbeat.type}")
            print(f"   Base Mode: {heartbeat.base_mode}")
            
            # 5 saniye boyunca mesaj dinle
            print("\n📨 Gelen mesajları dinliyorum (5 saniye)...")
            message_types = {}
            start_time = time.time()
            
            while time.time() - start_time < 5:
                msg = connection.recv_match(blocking=False)
                if msg:
                    msg_type = msg.get_type()
                    message_types[msg_type] = message_types.get(msg_type, 0) + 1
                
                time.sleep(0.01)
            
            print("\n📊 Alınan Mesaj Türleri:")
            for msg_type, count in sorted(message_types.items()):
                print(f"   {msg_type}: {count} mesaj")
            
            if 'RAW_IMU' in message_types or 'ATTITUDE' in message_types:
                print("✅ IMU mesajları alınıyor!")
            else:
                print("❌ IMU mesajları alınamıyor!")
                print("💡 Pixhawk parameterlerinde IMU mesajlarını aktif edin")
            
        else:
            print("❌ Heartbeat alınamadı!")
            print(f"💡 {MAV_ADDRESS} portu kontrol edin")
            print(f"💡 Baud rate {MAV_BAUD} kontrol edin")
            
        connection.close()
        
    except Exception as e:
        print(f"❌ Ham serial MAVLink test hatası: {e}")
        print("💡 Serial port erişim izinleri kontrol edin")
        print("💡 Pixhawk USB kablosu kontrol edin")

if __name__ == "__main__":
    print("=" * 60)
    print("🚀 TEKNOFEST ROV - Serial MAVLink Debug Test")
    print("=" * 60)
    print(f"🔧 Environment Variables:")
    print(f"   MAV_ADDRESS = {MAV_ADDRESS}")
    print(f"   MAV_BAUD = {MAV_BAUD}")
    
    # Ana test
    test_mavlink_connection()
    
    # Ham MAVLink testi
    test_raw_mavlink()
    
    print("\n" + "=" * 60)
    print("🔧 Debug test tamamlandı!")
    print("💡 Sorunlar devam ediyorsa:")
    print("   • Serial port izinlerini kontrol edin")
    print("   • Pixhawk USB bağlantısını kontrol edin")
    print("   • ArduSub firmware'ini kontrol edin")
    print("=" * 60) 