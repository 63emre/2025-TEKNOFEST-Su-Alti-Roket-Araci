#!/usr/bin/env python3
"""
Mini Terminal GUI - Küçük Ekranlar İçin
Sadece temel kontrol ve debug
"""

import sys
import time
import threading
from datetime import datetime

# Pi5 + PiOS curses desteği
try:
    import curses
    print("✅ Terminal UI hazır (Pi5 + PiOS)")
except ImportError as e:
    print(f"❌ Terminal UI hatası: {e}")
    sys.exit(1)

# Local imports
try:
    from mavlink_handler import MAVLinkHandler
    print("✅ MAVLinkHandler import OK")
except ImportError as e:
    print(f"❌ MAVLinkHandler import error: {e}")
    sys.exit(1)

class MiniTerminalGUI:
    def __init__(self):
        """Mini GUI başlatıcı"""
        self.mavlink = None
        self.connected = False
        self.armed = False
        self.running = True
        
        # IMU data
        self.imu_data = {'roll': 0, 'pitch': 0, 'yaw': 0}
        
        # Debug log
        self.debug_log = []
        
    def connect_mavlink(self):
        """MAVLink bağlantısı kur"""
        try:
            self.debug_log.append("📡 MAVLink bağlantısı kuruluyor...")
            self.mavlink = MAVLinkHandler()
            
            if self.mavlink.connect():
                self.connected = True
                self.debug_log.append("✅ MAVLink bağlandı!")
                
                # ARM durumu
                self.mavlink.check_system_status()
                self.armed = self.mavlink.armed
                
                return True
            else:
                self.debug_log.append("❌ MAVLink bağlantısı başarısız!")
                return False
                
        except Exception as e:
            self.debug_log.append(f"❌ MAVLink hatası: {e}")
            return False
    
    def update_imu(self):
        """IMU verilerini güncelle"""
        if not self.mavlink or not self.connected:
            return
        
        try:
            imu = self.mavlink.get_imu_data()
            if imu and len(imu) >= 6:
                # Basit orientation hesaplama
                import math
                accel_x, accel_y, accel_z = imu[0], imu[1], imu[2]
                
                if abs(accel_z) > 0.001:
                    roll = math.degrees(math.atan2(accel_y, accel_z))
                    pitch = math.degrees(math.atan2(-accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z)))
                    
                    self.imu_data['roll'] = roll
                    self.imu_data['pitch'] = pitch
                    # YAW ayrı hesaplanır, şimdilik 0
                    
        except:
            pass
    
    def draw_screen(self, stdscr):
        """Ekranı çiz - KOMPAKT"""
        height, width = stdscr.getmaxyx()
        
        # En az 80x20 gerekli
        if width < 80 or height < 20:
            stdscr.addstr(0, 0, f"❌ Terminal çok küçük: {width}x{height}")
            stdscr.addstr(1, 0, "💡 En az 80x20 gerekli")
            stdscr.refresh()
            return
        
        stdscr.clear()
        
        # Başlık
        title = "🚀 TEKNOFEST ROV - Mini GUI"
        stdscr.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)
        
        # Durum satırı
        conn_status = "✅ BAĞLI" if self.connected else "❌ BAĞLI DEĞİL"
        arm_status = "🔴 ARMED" if self.armed else "🟢 DISARMED"
        stdscr.addstr(1, 2, f"TCP: {conn_status}")
        stdscr.addstr(1, 20, f"Durum: {arm_status}")
        
        # IMU verileri
        stdscr.addstr(3, 2, "📊 IMU DATA:", curses.A_BOLD)
        stdscr.addstr(4, 4, f"Roll:  {self.imu_data['roll']:+6.1f}°")
        stdscr.addstr(5, 4, f"Pitch: {self.imu_data['pitch']:+6.1f}°")
        stdscr.addstr(6, 4, f"Yaw:   {self.imu_data['yaw']:+6.1f}°")
        
        # Kontroller
        stdscr.addstr(8, 2, "⌨️ KONTROL:", curses.A_BOLD)
        stdscr.addstr(9, 4, "Space: ARM/DISARM")
        stdscr.addstr(10, 4, "R: Yeniden Bağlan")
        stdscr.addstr(11, 4, "Q: Çıkış")
        
        # Debug log (son 5 mesaj)
        stdscr.addstr(13, 2, "📝 DEBUG:", curses.A_BOLD)
        recent_logs = self.debug_log[-5:] if len(self.debug_log) > 5 else self.debug_log
        
        for i, log in enumerate(recent_logs):
            if 14 + i < height - 1:
                # Log mesajını ekrana sığacak şekilde kısalt
                display_log = log[:width - 6]
                stdscr.addstr(14 + i, 4, display_log)
        
        stdscr.refresh()
    
    def handle_input(self, stdscr):
        """Klavye girişi"""
        key = stdscr.getch()
        
        if key == ord('q') or key == ord('Q'):
            self.running = False
        elif key == ord(' '):  # Space - ARM/DISARM
            self.toggle_arm()
        elif key == ord('r') or key == ord('R'):
            self.reconnect()
    
    def toggle_arm(self):
        """ARM/DISARM toggle"""
        if not self.connected:
            self.debug_log.append("⚠️ Bağlantı yok - ARM edilemez")
            return
        
        try:
            if self.armed:
                self.mavlink.disarm_system()
                self.armed = False
                self.debug_log.append("🟢 DISARM edildi")
            else:
                if self.mavlink.arm_system():
                    self.armed = True
                    self.debug_log.append("🔴 ARM edildi")
                else:
                    self.debug_log.append("❌ ARM başarısız")
        except Exception as e:
            self.debug_log.append(f"❌ ARM hatası: {e}")
    
    def reconnect(self):
        """Yeniden bağlan"""
        self.debug_log.append("🔄 Yeniden bağlanılıyor...")
        self.connected = False
        self.connect_mavlink()
    
    def run(self, stdscr):
        """Ana döngü"""
        # Curses setup
        curses.curs_set(0)
        curses.noecho()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.timeout(100)  # 10Hz update
        
        # İlk bağlantı
        self.connect_mavlink()
        
        # Ana döngü
        while self.running:
            try:
                # IMU güncelle
                self.update_imu()
                
                # Ekranı çiz
                self.draw_screen(stdscr)
                
                # Klavye girişi
                self.handle_input(stdscr)
                
                time.sleep(0.1)  # 10Hz
                
            except KeyboardInterrupt:
                self.running = False
            except Exception as e:
                self.debug_log.append(f"❌ Ana döngü hatası: {e}")
    
    def cleanup(self):
        """Temizlik"""
        if self.mavlink and self.connected:
            try:
                self.mavlink.disconnect()
            except:
                pass

def main():
    """Ana fonksiyon"""
    print("🚀 Mini Terminal GUI başlatılıyor...")
    
    gui = MiniTerminalGUI()
    
    try:
        curses.wrapper(gui.run)
    except Exception as e:
        print(f"❌ GUI hatası: {e}")
    finally:
        gui.cleanup()

if __name__ == "__main__":
    main() 