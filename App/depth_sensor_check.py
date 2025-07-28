#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Depth Sensor Check
ArduSub depth sensor ayarlarını kontrol et
"""

import sys
import time
try:
    from pymavlink import mavutil
except ImportError:
    print("❌ pymavlink yok! Yüklemek için: pip install pymavlink")
    sys.exit(1)

def check_depth_sensor():
    print("🔍 TEKNOFEST Su Altı ROV - Depth Sensor Check")
    print("=" * 50)
    
    try:
        # Serial MAVLink bağlantısı with environment variables
        import os
        serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
        baud_rate = int(os.getenv("MAV_BAUD", "115200"))
        
        print("🔌 MAVLink Serial bağlantısı kuruluyor...")
        print(f"📡 Serial: {serial_port} @ {baud_rate} baud")
        master = mavutil.mavlink_connection(serial_port, baud=baud_rate, autoreconnect=True)
        
        # Heartbeat bekle
        print("💓 Heartbeat bekleniyor...")
        master.wait_heartbeat()
        print(f"✅ ArduSub bağlantısı kuruldu! System ID: {master.target_system}")
        
        # Parametreleri iste
        print("📋 Depth sensor parametrelerini alıyor...")
        
        # İlgili parametreler
        depth_params = [
            'BARO_PRIMARY',      # Birincil barometer
            'INS_GYR_CAL',       # IMU kalibrasyonu
            'AHRS_EKF_TYPE',     # EKF tipi
            'EK2_ENABLE',        # EKF2 etkin mi
            'EK3_ENABLE',        # EKF3 etkin mi
            'BARO_PROBE_EXT',    # Harici barometer tespiti
        ]
        
        # Tüm parametreleri iste
        master.mav.param_request_list_send(
            master.target_system, master.target_component
        )
        
        print("⏳ Parametreler alınıyor...")
        time.sleep(3)
        
        # Mesajları kontrol et
        print("\n📊 SENSÖR MESAJLARI:")
        print("-" * 30)
        
        message_count = 0
        depth_messages = []
        imu_messages = []
        
        # 10 saniye boyunca mesajları dinle
        start_time = time.time()
        while time.time() - start_time < 10 and message_count < 100:
            msg = master.recv_match(blocking=False)
            if msg:
                message_count += 1
                msg_type = msg.get_type()
                
                # Depth/pressure mesajları
                if msg_type in ['SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3']:
                    depth_messages.append(msg)
                    if len(depth_messages) <= 3:  # İlk 3 mesajı göster
                        press = getattr(msg, 'press_abs', 0)
                        temp = getattr(msg, 'temperature', 0) / 100.0
                        print(f"📏 {msg_type}: Basınç={press:.1f}mbar, Sıcaklık={temp:.1f}°C")
                
                # IMU mesajları
                elif msg_type in ['RAW_IMU', 'SCALED_IMU', 'SCALED_IMU2']:
                    imu_messages.append(msg)
                    if len(imu_messages) <= 2:  # İlk 2 mesajı göster
                        print(f"🧭 {msg_type}: IMU verisi alındı")
            
            time.sleep(0.1)
        
        # Sonuçları raporla
        print(f"\n📈 TOPLAM MESAJ: {message_count}")
        print(f"🌊 Depth Mesajları: {len(depth_messages)}")
        print(f"🧭 IMU Mesajları: {len(imu_messages)}")
        
        # Depth sensor durumu
        if depth_messages:
            print("\n✅ DEPTH SENSOR DURUMU: AKTIF")
            latest_depth = depth_messages[-1]
            press = getattr(latest_depth, 'press_abs', 0)
            temp = getattr(latest_depth, 'temperature', 0) / 100.0
            
            # Derinlik hesapla (basit)
            depth_m = max(0.0, (press - 1013.25) / 100.0)
            
            print(f"   📊 Son Okuma:")
            print(f"   • Basınç: {press:.1f} mbar")
            print(f"   • Sıcaklık: {temp:.1f}°C")
            print(f"   • Tahmini Derinlik: {depth_m:.2f}m")
            
        else:
            print("\n❌ DEPTH SENSOR DURUMU: MESAJ YOK")
            print("   💡 Olası nedenler:")
            print("   • D300 sensor bağlı değil")
            print("   • ArduSub parametresi kapalı")
            print("   • Firmware problemi")
        
        # Terminal GUI önerisi
        print(f"\n🎮 TERMINAL GUI TESİ:")
        if depth_messages:
            print("   ✅ Terminal GUI depth verisi alabilir!")
            print("   🚀 Çalıştırmak için: python3 terminal_gui.py")
        else:
            print("   ⚠️  Terminal GUI'de depth verisi olmayabilir")
            print("   🔧 Önce ArduSub ayarlarını kontrol et")
            
    except KeyboardInterrupt:
        print("\n👋 Kontrol durduruldu!")
    except Exception as e:
        print(f"\n❌ Hata: {e}")
        print("💡 MAVLink serial bağlantısını kontrol et")
        print("   • Pixhawk'ın bağlı olduğunu kontrol edin")
        print("   • MAV_ADDRESS ve MAV_BAUD environment variable'larını kontrol edin")

if __name__ == "__main__":
    check_depth_sensor() 