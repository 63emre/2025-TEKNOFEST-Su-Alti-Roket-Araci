#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Real Terminal Launcher
Network config check + Terminal GUI başlatıcısı
"""

import os
import sys
import subprocess
import time

def check_terminal_requirements():
    """Terminal gereksinimleri kontrolü"""
    print("🔍 Terminal gereksinimlerini kontrol ediliyor...")
    
    # Curses kontrolü
    try:
        import curses
        print("✅ Curses library: OK")
    except ImportError:
        print("❌ Curses library: MISSING")
        print("💡 Çözüm: sudo apt update && sudo apt install python3-dev")
        return False
    
    # MAVLink kontrolü
    try:
        from pymavlink import mavutil
        print("✅ PyMAVLink library: OK")
    except ImportError:
        print("❌ PyMAVLink library: MISSING")
        print("💡 Çözüm: pip install pymavlink")
        return False
    
    # Config dosyası kontrolü
    if os.path.exists("config/hardware_config.json"):
        print("✅ Hardware config: OK")
    else:
        print("❌ Hardware config: MISSING")
        print("💡 Çözüm: App/ klasörünün içinde çalıştırın")
        return False
    
    print("✅ Tüm gereksinimler karşılandı!")
    return True

def run_network_config_check():
    """Network configuration check çalıştır"""
    print("\n📡 Network Configuration Check başlatılıyor...")
    
    try:
        result = subprocess.run([
            sys.executable, "network_config_check.py"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            print("✅ Network config check başarılı!")
            print(result.stdout)
            return True
        else:
            print("⚠️ Network config check uyarıları:")
            print(result.stdout)
            if result.stderr:
                print("Hatalar:")
                print(result.stderr)
            return True  # Uyarı olsa bile devam et
            
    except subprocess.TimeoutExpired:
        print("⚠️ Network config check timeout - devam ediliyor...")
        return True
    except Exception as e:
        print(f"❌ Network config check hatası: {e}")
        print("⚠️ Network check atlanıyor, direkt terminal GUI başlatılıyor...")
        return True

def setup_terminal_environment():
    """Terminal environment'ı hazırla"""
    print("\n🖥️ Terminal environment hazırlanıyor...")
    
    # Terminal boyutu kontrolü
    try:
        terminal_size = os.get_terminal_size()
        width, height = terminal_size.columns, terminal_size.lines
        print(f"📏 Terminal boyutu: {width}x{height}")
        
        if width < 120 or height < 30:
            print("⚠️ Terminal boyutu küçük!")
            print(f"💡 Önerilen minimum: 120x30, mevcut: {width}x{height}")
            print("🔧 Terminal penceresini büyütün veya font boyutunu küçültün")
            
            # Kullanıcıya sorma - direkt devam et
            print("📝 Küçük terminal boyutuna rağmen devam ediliyor...")
        
    except Exception as e:
        print(f"⚠️ Terminal boyutu tespit edilemedi: {e}")
    
    # Terminal encoding kontrolü
    encoding = sys.stdout.encoding or 'utf-8'
    print(f"🔤 Terminal encoding: {encoding}")
    
    # Environment variables
    os.environ['PYTHONIOENCODING'] = 'utf-8'
    os.environ['TERM'] = os.environ.get('TERM', 'xterm-256color')
    
    print("✅ Terminal environment hazır!")

def launch_terminal_gui():
    """Terminal GUI'yi başlat"""
    print("\n🚀 Terminal GUI başlatılıyor...")
    print("=" * 60)
    print("🎮 KONTROLLER:")
    print("   W/S: Pitch ±    A/D: Roll ±     Q/E: Yaw ±")
    print("   O/L: Motor ±    Space: ARM/DISARM")
    print("   0: Mission Plan T: Test Scripts  X: Exit")
    print("=" * 60)
    print("\n⏳ GUI yükleniyor... (3 saniye)")
    time.sleep(3)
    
    try:
        # Terminal GUI'yi çalıştır
        from terminal_gui import AdvancedTerminalGUI
        
        gui = AdvancedTerminalGUI()
        gui.run()
        
    except KeyboardInterrupt:
        print("\n👋 Kullanıcı tarafından durduruldu!")
    except Exception as e:
        print(f"\n❌ Terminal GUI hatası: {e}")
        import traceback
        print("🔍 Hata detayları:")
        traceback.print_exc()
        print("\n💡 Çözüm önerileri:")
        print("1. sudo apt update && sudo apt install python3-dev python3-curses")
        print("2. pip install -r requirements.txt")
        print("3. Terminal boyutunu artırın (min 120x30)")

def main():
    """Ana fonksiyon"""
    print("🚀 TEKNOFEST Su Altı ROV - Real Terminal Launcher")
    print("=" * 60)
    
    # Working directory kontrolü
    if not os.path.basename(os.getcwd()) == "App":
        print("❌ Yanlış dizin! App/ klasörünün içinde çalıştırın.")
        print(f"💡 Mevcut dizin: {os.getcwd()}")
        print("🔧 Çözüm: cd App && python run_real_terminal.py")
        return 1
    
    try:
        # 1. Terminal gereksinimlerini kontrol et
        if not check_terminal_requirements():
            return 1
        
        # 2. Network config check çalıştır
        if not run_network_config_check():
            print("⚠️ Network sorunları var ama devam ediliyor...")
        
        # 3. Terminal environment'ı hazırla
        setup_terminal_environment()
        
        # 4. Terminal GUI'yi başlat
        launch_terminal_gui()
        
        print("\n✅ Terminal GUI sonlandırıldı!")
        return 0
        
    except Exception as e:
        print(f"\n❌ Kritik hata: {e}")
        import traceback
        traceback.print_exc()
        return 1

if __name__ == "__main__":
    try:
        exit_code = main()
        sys.exit(exit_code)
    except KeyboardInterrupt:
        print("\n👋 Program sonlandırıldı!")
        sys.exit(0) 