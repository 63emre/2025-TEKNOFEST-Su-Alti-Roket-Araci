#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Pi5 + PiOS System Test
Terminal GUI ve bağlantıları test et
"""

import sys
import time
import os

def test_imports():
    """Import testleri"""
    print("📦 Import testleri...")
    
    tests = [
        ("curses", "Terminal UI"),
        ("pymavlink", "MAVLink"),
        ("json", "JSON Config"),
        ("threading", "Thread Support"),
        ("time", "Time Module"),
        ("math", "Math Module")
    ]
    
    failed = []
    for module, description in tests:
        try:
            __import__(module)
            print(f"✅ {description}: OK")
        except ImportError as e:
            print(f"❌ {description}: {e}")
            failed.append(module)
    
    # Opsiyonel testler
    optional_tests = [
        ("RPi.GPIO", "GPIO Control"),
        ("smbus2", "I2C Communication")
    ]
    
    for module, description in optional_tests:
        try:
            __import__(module)
            print(f"✅ {description}: OK (Opsiyonel)")
        except ImportError:
            print(f"⚠️ {description}: Yok (Normal)")
    
    return len(failed) == 0

def test_config():
    """Config dosyası testi"""
    print("\n⚙️ Config dosyası testi...")
    
    config_path = "config/hardware_config.json"
    if os.path.exists(config_path):
        try:
            import json
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # Gerekli anahtarları kontrol et
            required_keys = ["pixhawk", "mavlink", "raspberry_pi"]
            for key in required_keys:
                if key in config:
                    print(f"✅ Config [{key}]: OK")
                else:
                    print(f"❌ Config [{key}]: Eksik")
                    return False
            
            # TCP bağlantı stringini kontrol et
            tcp_string = config.get("mavlink", {}).get("connection_string", "")
            if "tcp:127.0.0.1:5777" in tcp_string:
                print(f"✅ TCP Config: {tcp_string}")
            else:
                print(f"⚠️ TCP Config: {tcp_string}")
            
            return True
            
        except Exception as e:
            print(f"❌ Config parse hatası: {e}")
            return False
    else:
        print(f"❌ Config dosyası bulunamadı: {config_path}")
        return False

def test_mavlink_import():
    """MAVLink handler import testi"""
    print("\n📡 MAVLink handler testi...")
    
    try:
        from mavlink_handler import MAVLinkHandler
        print("✅ MAVLink handler import: OK")
        
        # Handler oluştur (bağlanmaya çalışma)
        handler = MAVLinkHandler()
        print("✅ MAVLink handler oluşturuldu")
        
        return True
        
    except Exception as e:
        print(f"❌ MAVLink handler hatası: {e}")
        return False

def test_terminal_gui_import():
    """Terminal GUI import testi"""
    print("\n🖥️ Terminal GUI import testi...")
    
    try:
        # Sadece import et, çalıştırma
        sys.path.append('.')
        import terminal_gui
        print("✅ Terminal GUI import: OK")
        
        # Class oluştur (GUI başlatma)
        gui_class = getattr(terminal_gui, 'AdvancedTerminalGUI', None)
        if gui_class:
            print("✅ AdvancedTerminalGUI class bulundu")
            return True
        else:
            print("❌ AdvancedTerminalGUI class bulunamadı")
            return False
        
    except Exception as e:
        print(f"❌ Terminal GUI import hatası: {e}")
        return False

def test_i2c_system():
    """I2C sistem testi (Depth sensor için)"""
    print("\n🔌 I2C sistem testi...")
    
    try:
        # I2C detect komutunu çalıştır
        import subprocess
        result = subprocess.run(['i2cdetect', '-y', '1'], 
                              capture_output=True, text=True, timeout=5)
        
        if result.returncode == 0:
            print("✅ I2C Bus 1 aktif")
            
            # 0x76 adresini kontrol et (D300 depth sensor)
            if '76' in result.stdout:
                print("🎯 D300 Depth Sensor (0x76) tespit edildi!")
            else:
                print("ℹ️ 0x76 adresinde cihaz yok (normal olabilir)")
            
            return True
        else:
            print(f"⚠️ I2C hatası: {result.stderr}")
            return False
            
    except Exception as e:
        print(f"⚠️ I2C test hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🧪 TEKNOFEST Pi5 + PiOS System Test")
    print("=" * 50)
    
    test_results = []
    
    # Testleri çalıştır
    test_results.append(("Import", test_imports()))
    test_results.append(("Config", test_config()))  
    test_results.append(("MAVLink", test_mavlink_import()))
    test_results.append(("Terminal GUI", test_terminal_gui_import()))
    test_results.append(("I2C", test_i2c_system()))
    
    # Sonuçları özetle
    print("\n" + "=" * 50)
    print("📊 TEST SONUÇLARI:")
    
    passed = 0
    total = len(test_results)
    
    for test_name, result in test_results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {test_name:15} : {status}")
        if result:
            passed += 1
    
    print(f"\n🎯 Başarı Oranı: {passed}/{total} ({100*passed//total}%)")
    
    if passed == total:
        print("🚀 Sistem HAZIR! Terminal GUI başlatılabilir:")
        print("   cd App && python3 terminal_gui.py")
    else:
        print("⚠️ Bazı testler başarısız. Eksikleri gider ve tekrar test et:")
        print("   bash pi5_setup.sh")
    
    return passed == total

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 