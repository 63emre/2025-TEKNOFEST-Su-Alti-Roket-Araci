#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Simple Terminal GUI
Siyah ekran sorununu gidermek için basitleştirilmiş versiyon
"""

import sys
import os
import curses
import time
import threading
from collections import deque

# Ana klasörden modülleri import et
try:
    from mavlink_handler import MAVLinkHandler
    HAS_MAVLINK = True
except ImportError:
    HAS_MAVLINK = False

class SimpleTerminalGUI:
    def __init__(self):
        """Basit terminal GUI"""
        self.stdscr = None
        self.running = True
        self.width = 80
        self.height = 24
        
        # Sistem durumu
        self.connected = False
        self.armed = False
        
        # Kontrol değerleri
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0
        
        # MAVLink
        self.mavlink = None
        if HAS_MAVLINK:
            self.mavlink = MAVLinkHandler()
        
        # Log sistemi
        self.logs = deque(maxlen=100)
        self.logs.append("🚀 Simple Terminal GUI başlatıldı")
        
        # IMU verileri (basit)
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0, 
            'yaw': 0.0,
            'connected': False
        }
        
    def log(self, message):
        """Log mesajı ekle"""
        timestamp = time.strftime("%H:%M:%S")
        self.logs.append(f"[{timestamp}] {message}")
    
    def connect_mavlink(self):
        """MAVLink bağlantısı kur"""
        if not HAS_MAVLINK:
            self.log("❌ MAVLink handler yok")
            return False
        
        try:
            self.log("🔌 MAVLink bağlantısı kuruluyor...")
            success = self.mavlink.connect()
            if success:
                self.connected = True
                self.log("✅ MAVLink TCP bağlantısı başarılı!")
                return True
            else:
                self.log("❌ MAVLink bağlantı başarısız")
                return False
        except Exception as e:
            self.log(f"❌ MAVLink bağlantı hatası: {e}")
            return False
    
    def init_curses(self, stdscr):
        """Curses başlat"""
        self.stdscr = stdscr
        self.height, self.width = stdscr.getmaxyx()
        
        # Curses ayarları
        curses.curs_set(0)
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.timeout(100)  # 100ms timeout
        
        # Renkler
        if curses.has_colors():
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
            curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)
            curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)
            curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)
            curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK)
            curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_BLACK)
        
        self.log(f"🖥️ Terminal boyutu: {self.width}x{self.height}")
        self.log("✅ Curses initialization tamamlandı")
    
    def draw_header(self):
        """Başlık çiz"""
        if self.height < 5 or self.width < 50:
            return
        
        title = "🚀 TEKNOFEST ROV - Simple Terminal GUI 🚀"
        start_col = max(0, (self.width - len(title)) // 2)
        self.stdscr.addstr(0, start_col, title, curses.A_BOLD)
        
        # Durum satırı
        tcp_status = "TCP:✅" if self.connected else "TCP:❌"
        arm_status = "ARMED" if self.armed else "DISARMED"
        
        self.stdscr.addstr(1, 2, f"{tcp_status} | {arm_status}")
        
        # Çizgi
        self.stdscr.addstr(2, 0, "=" * min(self.width, 80))
    
    def draw_controls(self):
        """Kontrol paneli"""
        start_row = 4
        
        if start_row + 10 > self.height:
            return
        
        # Servo değerleri
        self.stdscr.addstr(start_row, 2, "⌨️ KONTROLLER:")
        self.stdscr.addstr(start_row + 1, 4, f"Roll:  {self.servo_values['roll']:+4.0f}° [A/D]")
        self.stdscr.addstr(start_row + 2, 4, f"Pitch: {self.servo_values['pitch']:+4.0f}° [W/S]")
        self.stdscr.addstr(start_row + 3, 4, f"Yaw:   {self.servo_values['yaw']:+4.0f}° [Q/E]")
        self.stdscr.addstr(start_row + 4, 4, f"Motor: {self.motor_value:+4.0f}% [J/K]")
        
        # IMU verileri
        self.stdscr.addstr(start_row + 6, 2, "📊 IMU:")
        imu_status = "✅" if self.imu_data['connected'] else "❌"
        self.stdscr.addstr(start_row + 7, 4, f"Status: {imu_status}")
        self.stdscr.addstr(start_row + 8, 4, f"Roll:  {self.imu_data['roll']:+6.1f}°")
        self.stdscr.addstr(start_row + 9, 4, f"Pitch: {self.imu_data['pitch']:+6.1f}°")
        self.stdscr.addstr(start_row + 10, 4, f"Yaw:   {self.imu_data['yaw']:+6.1f}°")
        
        # Komutlar
        self.stdscr.addstr(start_row + 12, 2, "📋 KOMUTLAR:")
        commands = [
            "Space: ARM/DISARM", "X: Çıkış", "C: MAVLink Bağlan"
        ]
        
        for i, cmd in enumerate(commands):
            if start_row + 13 + i < self.height - 2:
                self.stdscr.addstr(start_row + 13 + i, 4, cmd)
    
    def draw_logs(self):
        """Log mesajları"""
        log_start_row = max(18, self.height - 8)
        
        if log_start_row >= self.height - 2:
            return
        
        self.stdscr.addstr(log_start_row - 1, 2, "📋 LOGLAR:")
        
        # Son 5 log mesajını göster
        recent_logs = list(self.logs)[-5:]
        for i, log_msg in enumerate(recent_logs):
            row = log_start_row + i
            if row < self.height - 1:
                # Log mesajını ekran genişliğine sığdır
                display_log = log_msg[:self.width - 4] if len(log_msg) > self.width - 4 else log_msg
                self.stdscr.addstr(row, 4, display_log)
    
    def handle_keyboard(self):
        """Klavye girişi"""
        try:
            key = self.stdscr.getch()
            if key == -1:  # No key pressed
                return
            
            # ESC veya X - Çıkış
            if key == 27 or key == ord('x') or key == ord('X'):
                self.log("👋 Çıkış komutu alındı")
                self.running = False
                return
            
            # Space - ARM/DISARM
            elif key == ord(' '):
                self.armed = not self.armed
                status = "ARM" if self.armed else "DISARM"
                self.log(f"🔄 {status} edildi")
            
            # MAVLink bağlantı
            elif key == ord('c') or key == ord('C'):
                self.connect_mavlink()
            
            # Servo kontrolleri
            elif key == ord('w') or key == ord('W'):
                self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + 5)
                self.log(f"Pitch: {self.servo_values['pitch']}°")
            elif key == ord('s') or key == ord('S'):
                self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - 5)
                self.log(f"Pitch: {self.servo_values['pitch']}°")
            elif key == ord('a') or key == ord('A'):
                self.servo_values['roll'] = max(-45, self.servo_values['roll'] - 5)
                self.log(f"Roll: {self.servo_values['roll']}°")
            elif key == ord('d') or key == ord('D'):
                self.servo_values['roll'] = min(45, self.servo_values['roll'] + 5)
                self.log(f"Roll: {self.servo_values['roll']}°")
            elif key == ord('q') or key == ord('Q'):
                self.servo_values['yaw'] = max(-45, self.servo_values['yaw'] - 5)
                self.log(f"Yaw: {self.servo_values['yaw']}°")
            elif key == ord('e') or key == ord('E'):
                self.servo_values['yaw'] = min(45, self.servo_values['yaw'] + 5)
                self.log(f"Yaw: {self.servo_values['yaw']}°")
            elif key == ord('j') or key == ord('J'):
                self.motor_value = max(-100, self.motor_value - 5)
                self.log(f"Motor: {self.motor_value}%")
            elif key == ord('k') or key == ord('K'):
                self.motor_value = min(100, self.motor_value + 5)
                self.log(f"Motor: {self.motor_value}%")
            
        except Exception as e:
            self.log(f"❌ Klavye hatası: {e}")
    
    def update_imu_data(self):
        """IMU verilerini güncelle"""
        if not self.mavlink or not self.connected:
            self.imu_data['connected'] = False
            return
        
        try:
            # IMU data alma
            imu_raw = self.mavlink.get_imu_data()
            if imu_raw and len(imu_raw) >= 6:
                # IMU verisi var
                ax, ay, az = imu_raw[0], imu_raw[1], imu_raw[2]
                
                # Roll ve pitch hesaplama
                import math
                if abs(az) > 0.001:
                    roll = math.degrees(math.atan2(ay, az))
                    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
                    
                    self.imu_data['roll'] = roll
                    self.imu_data['pitch'] = pitch
                    self.imu_data['connected'] = True
                    
                    # İlk başarılı IMU verisi alındığında log
                    if not hasattr(self, 'imu_success_logged'):
                        self.log(f"✅ IMU verisi alındı: Roll={roll:.1f}° Pitch={pitch:.1f}°")
                        self.imu_success_logged = True
                else:
                    self.imu_data['connected'] = False
            else:
                self.imu_data['connected'] = False
                # Sadece ilk başta hata logla
                if not hasattr(self, 'imu_no_data_logged'):
                    self.log("⚠️ IMU verisi alınamıyor")
                    self.imu_no_data_logged = True
                
        except Exception as e:
            self.imu_data['connected'] = False
            # Hata logunu sınırla
            if not hasattr(self, 'last_imu_error') or time.time() - self.last_imu_error > 10:
                self.log(f"⚠️ IMU güncelleme hatası: {e}")
                self.last_imu_error = time.time()
    
    def main_loop(self):
        """Ana döngü"""
        self.log("🔄 Ana döngü başlatıldı")
        
        last_imu_update = time.time()
        frame_count = 0
        
        while self.running:
            try:
                # Ekranı temizle
                self.stdscr.erase()
                
                # UI bileşenlerini çiz
                self.draw_header()
                self.draw_controls()
                self.draw_logs()
                
                # Debug bilgisi (daha az sıklıkta)
                frame_count += 1
                if frame_count % 100 == 0:  # Her 10 saniyede bir
                    self.log(f"🔄 Frame #{frame_count} - GUI çalışıyor")
                
                # Ekranı yenile
                self.stdscr.refresh()
                
                # Klavye girişi (non-blocking)
                try:
                    self.handle_keyboard()
                except:
                    pass  # Klavye hatası terminal'i bozmasın
                
                # IMU verilerini güncelle (2 saniyede bir - daha az sıklıkta)
                if time.time() - last_imu_update > 2.0:
                    try:
                        self.update_imu_data()
                    except:
                        pass  # IMU hatası terminal'i bozmasın
                    last_imu_update = time.time()
                
                # CPU efficiency
                time.sleep(0.1)  # 10 FPS
                
            except KeyboardInterrupt:
                self.log("⚠️ Ctrl+C ile durduruldu")
                self.running = False
                break
            except Exception as e:
                self.log(f"❌ Ana döngü hatası: {e}")
                time.sleep(1.0)  # Hata durumunda daha uzun bekle
                # Terminal'i yeniden başlatmayı dene
                try:
                    self.stdscr.clear()
                    self.stdscr.refresh()
                except:
                    pass
        
        self.log("🔄 Ana döngü durdu")
    
    def cleanup(self):
        """Temizlik"""
        self.log("🧹 Sistem temizleniyor...")
        
        if self.mavlink and self.connected:
            try:
                self.mavlink.disconnect()
            except:
                pass
    
    def run(self):
        """Ana çalıştırma"""
        def curses_main(stdscr):
            self.init_curses(stdscr)
            self.main_loop()
        
        try:
            # Başlangıç mesajı
            print("🚀 Simple Terminal GUI başlatılıyor...")
            print("💡 MAVLink bağlantısı için C tuşuna basın")
            print("💡 Çıkış için X tuşuna basın")
            time.sleep(1)  # Mesajı okuma zamanı
            
            # Curses başlat
            curses.wrapper(curses_main)
            
        except KeyboardInterrupt:
            print("\n⚠️ Ctrl+C ile durduruldu")
        except Exception as e:
            print(f"❌ Terminal GUI hatası: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.running = False
            self.cleanup()
            print("👋 Simple Terminal GUI kapatıldı")

def main():
    """Ana fonksiyon"""
    # Çalışma dizini kontrolü
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı!")
        print("💡 App/ klasörünün içinden çalıştırın")
        return 1
    
    try:
        gui = SimpleTerminalGUI()
        gui.run()
        return 0
    except KeyboardInterrupt:
        print("\n👋 Kullanıcı tarafından durduruldu!")
        return 0
    except Exception as e:
        print(f"❌ Program hatası: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main()) 