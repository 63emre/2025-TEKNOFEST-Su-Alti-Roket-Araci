#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Pi Terminal GUI Quick Test
Pi'da optimize edilmiş terminal GUI için hızlı test
"""

import sys
import os
import time

def test_pi_mavlink():
    """Pi'da MAVLink bağlantısı test et"""
    print("🔧 Pi MAVLink bağlantı testi...")
    
    try:
        from pymavlink import mavutil
        
        # Pi'da çalışan ayarlar
        connection_string = "/dev/serial0"
        baud_rate = 57600
        
        print(f"🔌 Bağlantı: {connection_string} @ {baud_rate} baud")
        
        # Test bağlantısı (timeout ile)
        print("⏳ TELEM2 bağlantısı test ediliyor...")
        master = mavutil.mavlink_connection(connection_string, baud=baud_rate)
        
        # Heartbeat bekle (5 saniye timeout)
        heartbeat = master.wait_heartbeat(timeout=5)
        
        if heartbeat:
            print(f"✅ MAVLink bağlantısı başarılı!")
            print(f"   SYS={master.target_system} COMP={master.target_component}")
            
            # Kısa mesaj testi
            print("📊 2 saniye veri dinleme testi...")
            start_time = time.time()
            message_count = 0
            
            while time.time() - start_time < 2:
                msg = master.recv_match(blocking=False)
                if msg:
                    message_count += 1
                time.sleep(0.01)
            
            print(f"✅ {message_count} MAVLink mesaj alındı!")
            master.close()
            return True
        else:
            print("❌ Heartbeat alınamadı!")
            return False
            
    except Exception as e:
        print(f"❌ MAVLink test hatası: {e}")
        return False

def test_terminal_gui_imports():
    """Terminal GUI import testi"""
    print("\n🔧 Terminal GUI import testi...")
    
    try:
        from terminal_gui import TerminalROVGUI
        print("✅ TerminalROVGUI import başarılı")
        
        from mavlink_handler import MAVLinkHandler
        print("✅ MAVLinkHandler import başarılı")
        
        return True
        
    except ImportError as e:
        print(f"❌ Import hatası: {e}")
        return False

def test_config():
    """Config testi"""
    print("\n🔧 Config dosyası testi...")
    
    try:
        import json
        
        with open("config/hardware_config.json", 'r') as f:
            config = json.load(f)
        
        mavlink_config = config.get("mavlink", {})
        connection = mavlink_config.get("connection_string")
        baud = mavlink_config.get("baud_rate")
        
        print(f"✅ Connection: {connection}")
        print(f"✅ Baud: {baud}")
        
        if connection == "/dev/serial0" and baud == 57600:
            print("✅ Config Pi için optimize edilmiş!")
            return True
        else:
            print("❌ Config Pi ayarları doğru değil!")
            return False
            
    except Exception as e:
        print(f"❌ Config test hatası: {e}")
        return False

def test_curses():
    """Curses test"""
    print("\n🔧 Curses kütüphanesi testi...")
    
    try:
        import curses
        print("✅ Curses import başarılı")
        
        # Terminal boyutu kontrolü
        import subprocess
        result = subprocess.run(['stty', 'size'], capture_output=True, text=True)
        if result.returncode == 0:
            height, width = map(int, result.stdout.strip().split())
            print(f"✅ Terminal boyutu: {width}x{height}")
            
            if width >= 120 and height >= 30:
                print("✅ Terminal boyutu yeterli!")
                return True
            else:
                print(f"⚠️ Terminal çok küçük! Min: 120x30, Mevcut: {width}x{height}")
                return False
        else:
            print("⚠️ Terminal boyutu tespit edilemedi")
            return True
            
    except ImportError:
        print("❌ Curses import hatası!")
        print("💡 Çözüm: sudo apt-get install python3-dev")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🚀 TEKNOFEST Pi Terminal GUI - Quick Test")
    print("=" * 50)
    
    # Çalışma dizini kontrolü
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı!")
        print("💡 App/ klasörünün içinden çalıştırın")
        sys.exit(1)
    
    # Test sırası
    tests = [
        ("Config Test", test_config),
        ("Import Test", test_terminal_gui_imports),
        ("Curses Test", test_curses),
        ("MAVLink Test", test_pi_mavlink),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\n📋 {test_name}")
        print("-" * 30)
        result = test_func()
        results.append((test_name, result))
        
        if result:
            print(f"✅ {test_name} BAŞARILI")
        else:
            print(f"❌ {test_name} BAŞARISIZ")
    
    # Sonuçlar
    print("\n" + "=" * 50)
    print("📊 TEST SONUÇLARI:")
    
    success_count = 0
    for test_name, result in results:
        status = "✅" if result else "❌"
        print(f"  {status} {test_name}")
        if result:
            success_count += 1
    
    print(f"\n🎯 {success_count}/{len(results)} test başarılı")
    
    if success_count == len(results):
        print("\n🎉 TÜM TESTLER BAŞARILI!")
        print("\n🚀 Terminal GUI'yi başlatmak için:")
        print("   python terminal_gui.py")
        print("\n💡 Optimizasyonlar:")
        print("   ✅ Pi CPU friendly FPS (15 screen, 10 data)")
        print("   ✅ TELEM2 /dev/serial0 @ 57600 baud")
        print("   ✅ D300 depth sensor I2C 0x76 support")
        print("   ✅ IMU: RAW_IMU/SCALED_IMU/ATTITUDE support")
    else:
        print(f"\n⚠️ Bazı testler başarısız!")
        print("💡 Başarısız testleri düzelttikten sonra tekrar deneyin")
    
    print("\n📱 SSH bağlantısı için terminal boyutunu ayarlayın:")
    print("   Minimum: 120x30 karakter")

if __name__ == "__main__":
    main() 