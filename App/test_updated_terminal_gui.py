#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Updated Terminal GUI Test
telem2_con.py ayarları ile test
"""

import sys
import os
import time

def test_imports():
    """Import test"""
    print("🔧 Import testleri yapılıyor...")
    
    try:
        from pymavlink import mavutil
        print("✅ pymavlink import başarılı")
    except ImportError as e:
        print(f"❌ pymavlink import hatası: {e}")
        return False
    
    try:
        from mavlink_handler import MAVLinkHandler
        print("✅ MAVLinkHandler import başarılı")
    except ImportError as e:
        print(f"❌ MAVLinkHandler import hatası: {e}")
        return False
    
    try:
        from terminal_gui import TerminalROVGUI
        print("✅ TerminalROVGUI import başarılı")
    except ImportError as e:
        print(f"❌ TerminalROVGUI import hatası: {e}")
        return False
    
    return True

def test_config():
    """Config test"""
    print("\n🔧 Config dosyası testi...")
    
    try:
        import json
        with open("config/hardware_config.json", 'r') as f:
            config = json.load(f)
        
        mavlink_config = config.get("mavlink", {})
        connection_string = mavlink_config.get("connection_string")
        baud_rate = mavlink_config.get("baud_rate")
        
        print(f"✅ Connection string: {connection_string}")
        print(f"✅ Baud rate: {baud_rate}")
        
        if connection_string == "/dev/serial0" and baud_rate == 57600:
            print("✅ Config ayarları telem2_con.py ile uyumlu!")
            return True
        else:
            print("❌ Config ayarları güncel değil!")
            return False
            
    except Exception as e:
        print(f"❌ Config test hatası: {e}")
        return False

def test_mavlink_handler():
    """MAVLink handler test"""
    print("\n🔧 MAVLink Handler testi...")
    
    try:
        from mavlink_handler import MAVLinkHandler
        
        # Handler oluştur
        handler = MAVLinkHandler()
        print("✅ MAVLinkHandler oluşturuldu")
        
        # Config kontrolü
        connection_string = handler.config["mavlink"]["connection_string"]
        baud_rate = handler.config["mavlink"].get("baud_rate", 57600)
        
        print(f"✅ Handler config - Connection: {connection_string}")
        print(f"✅ Handler config - Baud: {baud_rate}")
        
        print("⚠️ Gerçek bağlantı testi manuel olarak yapılmalı (/dev/serial0 gerekli)")
        
        return True
        
    except Exception as e:
        print(f"❌ MAVLink Handler test hatası: {e}")
        return False

def test_terminal_gui():
    """Terminal GUI test"""
    print("\n🔧 Terminal GUI testi...")
    
    try:
        from terminal_gui import TerminalROVGUI
        
        # GUI oluştur (sadece başlatma testi)
        gui = TerminalROVGUI()
        print("✅ TerminalROVGUI oluşturuldu")
        
        # Config kontrolü
        mavlink_config = gui.config.get("mavlink", {})
        connection_string = mavlink_config.get("connection_string")
        
        print(f"✅ GUI config - Connection: {connection_string}")
        
        print("⚠️ Tam GUI testi için: python terminal_gui.py çalıştırın")
        
        return True
        
    except Exception as e:
        print(f"❌ Terminal GUI test hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🚀 TEKNOFEST Terminal GUI - Updated Test (telem2_con.py uyumlu)")
    print("=" * 60)
    
    # Çalışma dizini kontrolü
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı! App/ klasörünün içinden çalıştırın.")
        sys.exit(1)
    
    test_results = []
    
    # Test sırası
    tests = [
        ("Import Tests", test_imports),
        ("Config Tests", test_config),
        ("MAVLink Handler Tests", test_mavlink_handler),
        ("Terminal GUI Tests", test_terminal_gui)
    ]
    
    for test_name, test_func in tests:
        print(f"\n📋 {test_name}")
        print("-" * 40)
        result = test_func()
        test_results.append((test_name, result))
        
        if result:
            print(f"✅ {test_name} BAŞARILI")
        else:
            print(f"❌ {test_name} BAŞARISIZ")
    
    # Sonuçlar
    print("\n" + "=" * 60)
    print("📊 TEST SONUÇLARI:")
    
    success_count = 0
    for test_name, result in test_results:
        status = "✅ BAŞARILI" if result else "❌ BAŞARISIZ"
        print(f"  {test_name}: {status}")
        if result:
            success_count += 1
    
    print(f"\n🎯 Toplam: {success_count}/{len(test_results)} test başarılı")
    
    if success_count == len(test_results):
        print("\n🎉 TÜM TESTLER BAŞARILI! Terminal GUI güncellemesi tamamlandı.")
        print("\n🚀 Başlatmak için:")
        print("   cd App/")
        print("   python terminal_gui.py")
    else:
        print(f"\n⚠️ {len(test_results) - success_count} test başarısız. Lütfen hataları düzeltin.")
    
    print("\n💡 telem2_con.py ayarları uygulandı:")
    print("   - Connection: /dev/serial0")
    print("   - Baud rate: 57600")
    print("   - ARM/DISARM method düzeltildi")
    print("   - D300 depth sensor (I2C 0x76) optimize edildi")

if __name__ == "__main__":
    main() 