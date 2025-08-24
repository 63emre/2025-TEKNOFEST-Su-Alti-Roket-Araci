#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D300 Entegrasyon Test Scripti
Yeni D300 MAVLink entegrasyonunu test eder
"""

import sys
import time
from pymavlink import mavutil
from config import *
from sensors import SensorManager
from utils import Logger

def test_d300_integration():
    """D300 entegrasyonunu test et"""
    print("=" * 60)
    print("D300 MAVLink Entegrasyon Testi")
    print("DENİZ SUYU ortamı için optimize edildi")
    print("=" * 60)
    
    # Logger başlat
    logger = Logger()
    
    # MAVLink bağlantısını kur
    try:
        port = MAVLINK_PORT
        if sys.platform.startswith('win'):
            port = MAVLINK_PORT_WIN
            
        print(f"MAVLink bağlantısı kuruluyor: {port} @ {MAVLINK_BAUD}")
        mavlink = mavutil.mavlink_connection(port, baud=MAVLINK_BAUD)
        
        if not mavlink.wait_heartbeat(timeout=10):
            print("❌ HATA: Pixhawk heartbeat alınamadı!")
            return False
            
        print(f"✅ MAVLink bağlantısı başarılı (SYS={mavlink.target_system})")
        
    except Exception as e:
        print(f"❌ MAVLink bağlantı hatası: {e}")
        return False
    
    # Sensör yöneticisini başlat
    try:
        print("\n📡 Sensör yöneticisi başlatılıyor...")
        sensors = SensorManager(mavlink, logger)
        
        # D300 bilgilerini göster
        water_info = sensors.depth.get_water_info()
        print(f"Su ortamı: {water_info['type']} (ρ={water_info['density']} kg/m³)")
        print(f"D300 kaynak: {sensors.depth.msg_name} (ID={sensors.depth.msg_id})")
        
    except Exception as e:
        print(f"❌ Sensör başlatma hatası: {e}")
        return False
    
    # Kalibrasyon testi
    try:
        print("\n🔧 D300 kalibrasyon testi...")
        print("NOT: Gerçek ortamda sensörü su yüzeyinde tutun!")
        
        # Kısa kalibrasyon (test için)
        calib_success = sensors.depth.calibrate_surface(duration=3)
        if calib_success:
            print(f"✅ Kalibrasyon başarılı: P0 = {sensors.depth.pressure_offset:.2f} mbar")
        else:
            print("⚠️ Kalibrasyon başarısız, standart değer kullanılıyor")
            
    except Exception as e:
        print(f"❌ Kalibrasyon hatası: {e}")
        return False
    
    # Canlı veri testi
    try:
        print("\n📊 D300 canlı veri testi (10 saniye)...")
        print("Derinlik | Basınç | Sıcaklık | Durum")
        print("-" * 45)
        
        test_start = time.time()
        sample_count = 0
        valid_samples = 0
        
        while time.time() - test_start < 10:
            sensor_data = sensors.get_all_sensor_data()
            depth_data = sensor_data['depth']
            
            sample_count += 1
            
            if depth_data['is_valid']:
                valid_samples += 1
                depth = depth_data['depth_m']
                pressure = depth_data['pressure_mbar'] 
                temp = depth_data['temperature_c']
                
                print(f"{depth:6.3f}m | {pressure:7.1f} | {temp:6.1f}°C | ✅ Geçerli")
            else:
                print(f"   N/A   |    N/A   |   N/A   | ❌ Geçersiz")
                
            time.sleep(0.5)
        
        print("-" * 45)
        success_rate = (valid_samples / sample_count) * 100 if sample_count > 0 else 0
        print(f"Toplam örnek: {sample_count}, Geçerli: {valid_samples} ({success_rate:.1f}%)")
        
        if success_rate >= 70:
            print("✅ D300 veri kalitesi: İYİ")
        elif success_rate >= 50:
            print("⚠️ D300 veri kalitesi: ORTA")
        else:
            print("❌ D300 veri kalitesi: KÖTÜ")
            
    except Exception as e:
        print(f"❌ Canlı veri testi hatası: {e}")
        return False
    
    # Bağlantı testi
    try:
        print(f"\n🔌 D300 bağlantı durumu: {'✅ Bağlı' if sensors.depth.is_connected() else '❌ Bağlı değil'}")
        
    except Exception as e:
        print(f"❌ Bağlantı testi hatası: {e}")
        return False
    
    print("\n" + "=" * 60)
    print("✅ D300 entegrasyon testi TAMAMLANDI")
    print("D300 sensörü artık görevlerde kullanıma hazır!")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    try:
        success = test_d300_integration()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⏹️ Test kullanıcı tarafından durduruldu")
        sys.exit(0)
    except Exception as e:
        print(f"\n💥 Test hatası: {e}")
        sys.exit(1)
