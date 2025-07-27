#!/usr/bin/env python3
"""
Terminal GUI Connection Debug (GUI açmadan)
Sadece bağlantı problemini debug et
"""

import sys
import time
import threading
from datetime import datetime

# Terminal GUI'nin import kısmı
try:
    from mavlink_handler import MAVLinkHandler
    print("✅ MAVLinkHandler import OK")
except ImportError as e:
    print(f"❌ MAVLinkHandler import error: {e}")
    sys.exit(1)

def debug_mavlink_connection():
    """Terminal GUI'deki aynı süreci debug et"""
    print("🔧 DEBUG: Terminal GUI Bağlantı Süreci Simülasyonu")
    print("=" * 60)
    
    # Config yükle
    try:
        import json
        with open("config/hardware_config.json", 'r') as f:
            config = json.load(f)
        
        tcp_config = config.get("mavlink", {})
        tcp_config["connection_string"] = "tcp:127.0.0.1:5777"
        print(f"✅ Config yüklendi: {tcp_config['connection_string']}")
        
    except Exception as e:
        print(f"❌ Config yükleme hatası: {e}")
        return False
    
    # MAVLink handler oluştur
    try:
        print("\n🔧 MAVLinkHandler() oluşturuluyor...")
        mavlink = MAVLinkHandler()
        print("✅ MAVLinkHandler() oluşturuldu")
        
        # Connect çağır
        print("\n⏳ mavlink.connect() çağrılıyor (timeout: 20s)...")
        print("   Bu işlem 20 saniye sürebilir...")
        
        connect_start = time.time()
        connect_result = mavlink.connect()
        connect_time = time.time() - connect_start
        
        print(f"🔍 mavlink.connect() sonucu: {connect_result}")
        print(f"⏱️ Bağlantı süresi: {connect_time:.2f} saniye")
        
        if connect_result:
            print("✅ Connect başarılı!")
            
            # Connected flag kontrol et
            print(f"🔍 mavlink.connected flag: {mavlink.connected}")
            
            # System status kontrol et
            print("\n🔍 check_system_status() çağrılıyor...")
            mavlink.check_system_status()
            print(f"📊 check_system_status() sonrası mavlink.connected: {mavlink.connected}")
            print(f"📊 mavlink.armed: {mavlink.armed}")
            
            # IMU test
            print("\n📊 IMU veri testi...")
            for i in range(3):
                imu_data = mavlink.get_imu_data()
                if imu_data:
                    print(f"   IMU {i+1}: ✅ Veri var: {imu_data[:3]}")
                else:
                    print(f"   IMU {i+1}: ❌ Veri yok")
                time.sleep(0.5)
            
            # Disconnect
            print("\n🔌 Bağlantı kapatılıyor...")
            mavlink.disconnect()
            print("✅ Debug tamamlandı")
            
            return True
            
        else:
            print("❌ Connect başarısız!")
            print("💡 mavlink_handler.py'deki hata mesajlarını kontrol et")
            return False
            
    except Exception as e:
        print(f"❌ MAVLinkHandler exception: {e}")
        print(f"🔧 Exception türü: {type(e).__name__}")
        import traceback
        print(f"📄 Traceback:\n{traceback.format_exc()}")
        return False

def main():
    """Ana debug fonksiyonu"""
    print(f"🧪 Terminal GUI Connection Debug")
    print(f"⏰ Zaman: {datetime.now().strftime('%H:%M:%S')}")
    print(f"🐍 Python version: {sys.version}")
    
    # Ortam kontrol
    print("\n📦 Ortam kontrolü:")
    try:
        import pymavlink
        print(f"✅ pymavlink: {pymavlink.__version__}")
    except:
        print("❌ pymavlink import hatası")
    
    try:
        import json
        print("✅ json: OK")
    except:
        print("❌ json import hatası")
    
    # Ana debug
    success = debug_mavlink_connection()
    
    print("\n" + "=" * 60)
    if success:
        print("🎯 SONUÇ: MAVLink bağlantısı ÇALIŞIYOR!")
        print("💡 Terminal GUI'de problem başka bir yerde")
        print("🔧 Muhtemelen TCP thread veya UI update sorunu")
    else:
        print("❌ SONUÇ: MAVLink bağlantısı ÇALIŞMIYOR!")
        print("💡 Bu yüzden Terminal GUI'de TCP BAĞLI değil")
        print("🔧 MAVLink handler sorununu çöz")
    
    return success

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n👋 Debug kullanıcı tarafından durduruldu")
        sys.exit(1) 