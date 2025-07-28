#!/usr/bin/env python3
"""
TEKNOFEST Terminal GUI - DEBUG VERSION
Siyah ekran sorununu tespit etmek için basitleştirilmiş versiyon
"""

import sys
import os
import time

def test_basic_imports():
    """Temel import testleri"""
    print("🔍 TERMINAL GUI DEBUG BAŞLIYOR...")
    
    try:
        import curses
        print("✅ curses modülü yüklendi")
    except ImportError as e:
        print(f"❌ curses import hatası: {e}")
        return False
    
    try:
        from mavlink_handler import MAVLinkHandler
        print("✅ mavlink_handler yüklendi")
    except ImportError as e:
        print(f"❌ mavlink_handler import hatası: {e}")
        print("💡 App/ klasörünün içinden çalıştırın!")
        return False
    
    try:
        from gpio_controller import GPIOController
        print("✅ gpio_controller yüklendi")
    except ImportError as e:
        print(f"⚠️ gpio_controller import hatası: {e}")
        print("💡 GPIO devre dışı kalacak")
    
    try:
        from depth_sensor import D300DepthSensor
        print("✅ depth_sensor yüklendi")
    except ImportError as e:
        print(f"⚠️ depth_sensor import hatası: {e}")
        print("💡 Depth sensor devre dışı kalacak")
    
    return True

def test_curses_initialization():
    """Curses initialization testi"""
    print("\n🖥️ CURSES INITIALIZATION TESTI...")
    
    try:
        # Basit curses test
        stdscr = curses.initscr()
        height, width = stdscr.getmaxyx()
        print(f"✅ Terminal boyutu: {width}x{height}")
        
        # Color test
        if curses.has_colors():
            curses.start_color()
            print("✅ Color desteği var")
            print(f"✅ Color pairs: {curses.COLOR_PAIRS}")
        else:
            print("⚠️ Color desteği yok")
        
        # Cleanup
        curses.endwin()
        print("✅ Curses başlatılıp kapatıldı")
        return True
        
    except Exception as e:
        print(f"❌ Curses initialization hatası: {e}")
        try:
            curses.endwin()
        except:
            pass
        return False

def test_mavlink_connection():
    """MAVLink bağlantı testi"""
    print("\n🔌 MAVLINK BAĞLANTI TESTI...")
    
    try:
        from mavlink_handler import MAVLinkHandler
        mavlink = MAVLinkHandler()
        
        print("🔄 TCP bağlantısı deneniyor...")
        success = mavlink.connect()
        
        if success:
            print("✅ MAVLink TCP bağlantısı başarılı!")
            # IMU test
            imu_data = mavlink.get_imu_data()
            if imu_data:
                print(f"✅ IMU verisi alındı: {len(imu_data)} değer")
            else:
                print("⚠️ IMU verisi alınamadı")
            mavlink.disconnect()
        else:
            print("❌ MAVLink bağlantısı başarısız")
            print("💡 ArduSub çalışıyor mu? TCP port 5777 açık mı?")
        
        return success
        
    except Exception as e:
        print(f"❌ MAVLink test hatası: {e}")
        return False

def minimal_curses_demo():
    """Minimal curses demo"""
    print("\n🎮 MİNİMAL CURSES DEMO BAŞLIYOR...")
    print("💡 5 saniye sonra otomatik kapanacak...")
    
    def demo_main(stdscr):
        # Curses ayarları
        curses.curs_set(0)  # Cursor gizle
        stdscr.nodelay(1)   # Non-blocking input
        stdscr.timeout(100) # 100ms timeout
        
        # Color pairs (varsa)
        if curses.has_colors():
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
            curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
            curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
        
        start_time = time.time()
        counter = 0
        
        while time.time() - start_time < 5.0:
            # Ekranı temizle
            stdscr.erase()
            
            # Header
            stdscr.addstr(0, 2, "🚀 TEKNOFEST DEBUG - CURSES TEST", curses.A_BOLD)
            stdscr.addstr(1, 2, "=" * 50)
            
            # Test bilgisi
            elapsed = time.time() - start_time
            remaining = 5.0 - elapsed
            stdscr.addstr(3, 2, f"⏰ Kalan süre: {remaining:.1f}s")
            stdscr.addstr(4, 2, f"🔄 Counter: {counter}")
            
            # Color test (varsa)
            if curses.has_colors():
                stdscr.addstr(6, 2, "✅ GREEN TEXT", curses.color_pair(1))
                stdscr.addstr(7, 2, "❌ RED TEXT", curses.color_pair(2))
                stdscr.addstr(8, 2, "⚠️ YELLOW TEXT", curses.color_pair(3))
            
            # Terminal boyutu
            height, width = stdscr.getmaxyx()
            stdscr.addstr(10, 2, f"📏 Terminal: {width}x{height}")
            
            # Çıkış bilgisi
            stdscr.addstr(12, 2, "💡 Q tuşuna basarak erken çıkabilirsiniz")
            
            # Ekranı yenile
            stdscr.refresh()
            
            # Keyboard input kontrol
            key = stdscr.getch()
            if key == ord('q') or key == ord('Q'):
                break
            
            counter += 1
            time.sleep(0.1)
        
        # Final message
        stdscr.erase()
        stdscr.addstr(10, 2, "✅ CURSES TEST TAMAMLANDI!", curses.A_BOLD)
        stdscr.addstr(11, 2, "ENTER tuşuna basın...")
        stdscr.refresh()
        stdscr.getch()
    
    try:
        curses.wrapper(demo_main)
        print("✅ Minimal curses demo başarılı!")
        return True
    except Exception as e:
        print(f"❌ Curses demo hatası: {e}")
        return False

def main():
    """Ana debug fonksiyonu"""
    print("🚀 TEKNOFEST Su Altı ROV - Terminal GUI Debug")
    print("=" * 60)
    
    # Test 1: Import testleri
    if not test_basic_imports():
        print("\n❌ Import testleri başarısız!")
        return 1
    
    # Test 2: Curses initialization
    if not test_curses_initialization():
        print("\n❌ Curses initialization başarısız!")
        print("💡 Terminal uyumluluğu sorunu olabilir")
        return 1
    
    # Test 3: MAVLink bağlantı (opsiyonel)
    print("\n🤔 MAVLink bağlantısı test edilsin mi? (y/n): ", end="")
    test_mavlink = input().strip().lower() == 'y'
    
    if test_mavlink:
        test_mavlink_connection()
    
    # Test 4: Minimal curses demo
    print("\n🎮 Minimal curses demo çalıştırılsın mı? (y/n): ", end="")
    run_demo = input().strip().lower() == 'y'
    
    if run_demo:
        if minimal_curses_demo():
            print("\n🎉 Tüm testler başarılı!")
            print("💡 Ana terminal_gui.py çalışması gerekiyor")
        else:
            print("\n❌ Curses demo başarısız!")
            print("💡 Terminal/SSH ayarlarını kontrol edin")
    
    print("\n📋 DEBUG RAPORU:")
    print("✅ Python imports: OK")
    print("✅ Curses initialization: OK" if test_curses_initialization() else "❌ Curses: FAILED")
    print("💡 Eğer tüm testler OK ise, ana GUI çalışmalı")
    
    return 0

if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n⚠️ Debug iptal edildi")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Debug hatası: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1) 