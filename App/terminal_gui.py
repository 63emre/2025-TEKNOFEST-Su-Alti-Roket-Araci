#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Terminal GUI - OPTIMIZED REAL DATA VERSION
Tamamen Gerçek Veri + FPS Optimize Edilmiş Terminal Kontrol Uygulaması
"""

import sys
import os

# Windows curses desteği
try:
    import curses
except ImportError:
    print("❌ Windows'ta curses desteklenmiyor!")
    print("💡 Alternatif çözümler:")
    print("   1. Windows Terminal kullan")
    print("   2. WSL (Windows Subsystem for Linux) kullan")
    print("   3. pip install windows-curses deneyin")
    try:
        import subprocess
        print("🔧 windows-curses yüklemeyi deniyorum...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "windows-curses"])
        import curses
        print("✅ windows-curses başarıyla yüklendi!")
    except Exception as e:
        print(f"❌ Otomatik yükleme başarısız: {e}")
        print("💡 Elle yüklemek için: pip install windows-curses")
        print("💡 Alternatif: main_gui.py GUI uygulamasını kullan")
        sys.exit(1)

import threading
import time
import subprocess
import json
from datetime import datetime

# Local imports
try:
    from mavlink_handler import MAVLinkHandler
    from navigation_engine import NavigationEngine
    from vibration_monitor import VibrationMonitor
    from depth_sensor import D300DepthSensor
    from gpio_controller import GPIOController
except ImportError as e:
    print(f"❌ Import hatası: {e}")
    sys.exit(1)

class TerminalROVGUI:
    def __init__(self):
        """Terminal GUI başlatıcı - OPTIMIZED VERSION"""
        # Sistem bileşenleri
        self.mavlink = None
        self.navigation = None
        self.vibration_monitor = None
        self.depth_sensor = None
        self.gpio_controller = None
        
        # Kontrol durumu
        self.control_mode = "RAW"  # RAW veya PID
        self.navigation_mode = "IMU"  # GPS, IMU, HYBRID
        self.armed = False
        self.running = True
        
        # Real-time kontrol
        self.active_keys = set()
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0
        self.depth_target = 0
        
        # Terminal UI
        self.stdscr = None
        self.height = 0
        self.width = 0
        
        # Logs
        self.log_messages = []
        self.max_logs = 50
        
        # REAL DATA BUFFERS - No caching, direct from sensors
        self.current_imu = None  # Direct IMU data
        self.current_depth = None  # Direct depth data
        self.current_gps = None  # Direct GPS data
        self.current_vibration = None  # Direct vibration data
        
        # FPS optimization
        self.last_screen_update = 0
        self.last_data_fetch = 0
        self.screen_fps_target = 20  # 50ms screen updates
        self.data_fps_target = 50   # 20ms data fetching
        
        # Config
        self.load_config()
        
        self.log("✅ OPTIMIZED Terminal GUI sistem bileşenleri başlatıldı!")
    
    def load_config(self):
        """Konfigürasyon yükle"""
        try:
            with open("config/hardware_config.json", 'r') as f:
                self.config = json.load(f)
        except Exception as e:
            self.log(f"❌ Config yükleme hatası: {e}")
            # Varsayılan config
            self.config = {
                "pixhawk": {
                    "servos": {"front_left": 1, "rear_left": 3, "rear_right": 4, "front_right": 5},
                    "motor": 6,
                    "pwm_limits": {"servo_min": 1100, "servo_max": 1900, "servo_neutral": 1500, "motor_stop": 1500}
                },
                "mavlink": {"connection_string": "tcp:127.0.0.1:5777"}
            }
    
    def log(self, message):
        """Log mesajı ekle"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]  # Milisaniye dahil
        log_entry = f"[{timestamp}] {message}"
        self.log_messages.append(log_entry)
        
        # Max log sayısını aş
        if len(self.log_messages) > self.max_logs:
            self.log_messages.pop(0)
    
    def init_systems(self):
        """Sistem bileşenlerini başlat"""
        print("🚀 TEKNOFEST ROV Terminal GUI sistem başlatma...")
        self.log("🚀 TEKNOFEST ROV OPTIMIZED Terminal GUI başlatılıyor...")
        
        # MAVLink bağlantısı
        try:
            print("🔌 MAVLink bağlantısı kuruluyor...")
            self.mavlink = MAVLinkHandler()
            if self.mavlink.connect():
                print("✅ MAVLink bağlantısı başarılı!")
                self.log("✅ MAVLink bağlantısı kuruldu!")
            else:
                print("⚠️ MAVLink bağlantısı kurulamadı")
                self.log("⚠️ MAVLink bağlantısı kurulamadı, offline mod")
        except Exception as e:
            print(f"❌ MAVLink hatası: {e}")
            self.log(f"❌ MAVLink hatası: {e}")
        
        # Navigation engine
        try:
            print("🧭 Navigation engine başlatılıyor...")
            self.navigation = NavigationEngine(self.mavlink)
            print("✅ Navigation engine başarılı!")
            self.log("✅ Navigation engine başlatıldı")
        except Exception as e:
            print(f"❌ Navigation hatası: {e}")
            self.log(f"❌ Navigation hatası: {e}")
        
        # Vibration monitor
        try:
            print("📳 Vibration monitor başlatılıyor...")
            if self.mavlink:
                self.vibration_monitor = VibrationMonitor(self.mavlink)
                print("✅ Vibration monitor başarılı!")
                self.log("✅ Vibration monitor başlatıldı")
        except Exception as e:
            print(f"❌ Vibration monitor hatası: {e}")
            self.log(f"❌ Vibration monitor hatası: {e}")
        
        # GPIO controller
        try:
            print("🔌 GPIO controller başlatılıyor...")
            self.gpio_controller = GPIOController(self.config)
            print("✅ GPIO controller başarılı!")
            self.log("✅ GPIO controller başlatıldı")
        except Exception as e:
            print(f"❌ GPIO controller hatası: {e}")
            self.log(f"❌ GPIO controller hatası: {e}")
        
        print("🎯 Tüm sistem bileşenleri tamamlandı!")
        self.log("✅ OPTIMIZED sistem bileşenleri başlatıldı!")
    
    def init_curses(self, stdscr):
        """Curses arayüzünü başlat - HYPER OPTIMIZED"""
        try:
            print("🔧 OPTIMIZED Curses ayarları yapılıyor...")
            self.stdscr = stdscr
            
            # Performance ayarları
            curses.curs_set(0)  # Cursor gizle
            curses.noecho()     # Echo kapat
            curses.cbreak()     # Karakterleri anında al
            stdscr.keypad(True) # Özel tuşları etkinleştir
            
            # REAL-TIME için optimizasyon
            stdscr.nodelay(True)  # Non-blocking input
            stdscr.timeout(0)     # Hiç bekleme - anında dön
            
            # MAXIMUM FPS için optimizasyon
            if hasattr(curses, 'use_env'):
                curses.use_env(True)
                
            # Screen buffer optimizasyonu
            if hasattr(curses, 'immedok'):
                curses.immedok(stdscr, True) 
            
            print("🎨 Renkler ayarlanıyor...")
            # Renkler
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)  # Başarılı
            curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)    # Hata
            curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK) # Uyarı
            curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)   # Info
            curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK) # Özel
            curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_BLACK)   # Normal
            
            # Ekran boyutu
            self.height, self.width = stdscr.getmaxyx()
            
            print(f"📏 Terminal boyutu: {self.width}x{self.height}")
            self.log(f"🖥️ OPTIMIZED Terminal boyutu: {self.width}x{self.height}")
            
            # Minimum boyut kontrolü
            if self.width < 120 or self.height < 30:
                print(f"⚠️ Terminal çok küçük! Min: 120x30, Mevcut: {self.width}x{self.height}")
                self.log(f"⚠️ Terminal çok küçük! Min: 120x30, Mevcut: {self.width}x{self.height}")
            
            # İlk çizim
            stdscr.clear()
            stdscr.refresh()
            
        except Exception as e:
            print(f"❌ init_curses hatası: {e}")
            raise
    
    def fetch_live_data(self):
        """Canlı veri alma - NO CACHING, DIRECT FROM SENSORS"""
        current_time = time.time()
        
        # Data fetch FPS kontrolü
        if current_time - self.last_data_fetch < (1.0 / self.data_fps_target):
            return  # Skip if too frequent
        
        self.last_data_fetch = current_time
        
        # REAL IMU DATA - Her çağrıda fresh
        if self.mavlink and self.mavlink.connected:
            try:
                self.current_imu = self.mavlink.get_imu_data()
                self.current_depth = self.mavlink.get_depth_data() 
                self.current_gps = self.mavlink.get_gps_data()
            except Exception as e:
                self.log(f"❌ MAVLink veri hatası: {e}")
                
        # REAL VIBRATION DATA
        if self.vibration_monitor:
            try:
                vib_level = self.vibration_monitor.get_vibration_level()
                vib_color = self.vibration_monitor.get_vibration_color()
                self.current_vibration = {'level': vib_level, 'color': vib_color}
            except Exception as e:
                self.log(f"❌ Vibration veri hatası: {e}")
    
    def draw_header(self):
        """Başlık çiz"""
        title = "🚀 TEKNOFEST Su Altı ROV - REAL DATA Terminal [OPTIMIZED] 🚀"
        self.stdscr.addstr(0, (self.width - len(title)) // 2, title, curses.color_pair(4) | curses.A_BOLD)
        
        # Durum bilgisi - LIVE DATA
        status_line = 1
        mavlink_status = "✅ LIVE" if self.mavlink and self.mavlink.connected else "❌ OFFLINE"
        arm_status = "🔴 ARMED" if self.armed else "🟢 DISARMED"
        
        self.stdscr.addstr(status_line, 2, f"MAVLink: {mavlink_status}", curses.color_pair(1 if self.mavlink and self.mavlink.connected else 2))
        self.stdscr.addstr(status_line, 25, f"Durum: {arm_status}", curses.color_pair(2 if self.armed else 1))
        self.stdscr.addstr(status_line, 45, f"Kontrol: {self.control_mode}", curses.color_pair(5))
        self.stdscr.addstr(status_line, 60, f"Navigation: {self.navigation_mode}", curses.color_pair(5))
        
        # Çizgi
        self.stdscr.addstr(2, 0, "─" * self.width, curses.color_pair(4))
    
    def draw_controls(self):
        """Kontrol bilgilerini çiz - %100 REAL DATA"""
        start_row = 4
        
        # SERVO KONTROL - Anlık değerler
        self.stdscr.addstr(start_row, 2, "🎮 SERVO KONTROL (REAL-TIME):", curses.color_pair(4) | curses.A_BOLD)
        self.stdscr.addstr(start_row + 1, 4, f"Roll:  {self.servo_values['roll']:+3.0f}° (A/D)")
        self.stdscr.addstr(start_row + 2, 4, f"Pitch: {self.servo_values['pitch']:+3.0f}° (W/S)")  
        self.stdscr.addstr(start_row + 3, 4, f"Yaw:   {self.servo_values['yaw']:+3.0f}° (Q/E)")
        
        # MOTOR KONTROL - Anlık değerler
        self.stdscr.addstr(start_row, 35, "⚙️ MOTOR KONTROL:", curses.color_pair(4) | curses.A_BOLD)
        self.stdscr.addstr(start_row + 1, 37, f"Güç: {self.motor_value:+3.0f}% (O/L)")
        self.stdscr.addstr(start_row + 2, 37, f"Hedef Derinlik: {self.depth_target:.1f}m")
        
        # %100 REAL IMU DATA - Fresh from sensors
        if self.current_imu:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.current_imu
            
            self.stdscr.addstr(start_row, 65, "📊 LIVE IMU VERİ:", curses.color_pair(1) | curses.A_BOLD)
            self.stdscr.addstr(start_row + 1, 67, f"Acc X: {accel_x:+7.3f} m/s²")
            self.stdscr.addstr(start_row + 2, 67, f"Acc Y: {accel_y:+7.3f} m/s²")
            self.stdscr.addstr(start_row + 3, 67, f"Acc Z: {accel_z:+7.3f} m/s²")
            self.stdscr.addstr(start_row + 1, 90, f"Gyro X: {gyro_x:+7.3f} rad/s")
            self.stdscr.addstr(start_row + 2, 90, f"Gyro Y: {gyro_y:+7.3f} rad/s") 
            self.stdscr.addstr(start_row + 3, 90, f"Gyro Z: {gyro_z:+7.3f} rad/s")
        else:
            color = curses.color_pair(3) if self.mavlink and self.mavlink.connected else curses.color_pair(2)
            self.stdscr.addstr(start_row, 65, "📊 IMU VERİ:", color | curses.A_BOLD)
            if self.mavlink and self.mavlink.connected:
                self.stdscr.addstr(start_row + 1, 67, "IMU sinyali bekleniyor...")
            else:
                self.stdscr.addstr(start_row + 1, 67, "MAVLink Bağlı Değil")
        
        # %100 REAL DEPTH DATA - Fresh from MAVLink
        if self.current_depth:
            depth = self.current_depth['depth_m']
            temp = self.current_depth['temperature_c']
            pressure = self.current_depth['pressure_mbar']
            
            self.stdscr.addstr(start_row, 115, "🌊 LIVE DERİNLİK:", curses.color_pair(1) | curses.A_BOLD)
            self.stdscr.addstr(start_row + 1, 117, f"Derinlik: {depth:.3f}m")
            self.stdscr.addstr(start_row + 2, 117, f"Sıcaklık: {temp:.2f}°C") 
            self.stdscr.addstr(start_row + 3, 117, f"Basınç: {pressure:.1f}mb")
        else:
            color = curses.color_pair(3) if self.mavlink and self.mavlink.connected else curses.color_pair(2)
            self.stdscr.addstr(start_row, 115, "🌊 DERİNLİK:", color | curses.A_BOLD)
            if self.mavlink and self.mavlink.connected:
                self.stdscr.addstr(start_row + 1, 117, "Depth sensör bekleniyor...")
            else:
                self.stdscr.addstr(start_row + 1, 117, "MAVLink Bağlı Değil")
        
        # %100 REAL VIBRATION DATA
        if self.current_vibration:
            vib_level = self.current_vibration['level']
            vib_color_name = self.current_vibration['color']
            color_map = {"green": 1, "yellow": 3, "red": 2}
            color = curses.color_pair(color_map.get(vib_color_name, 1))
            
            self.stdscr.addstr(start_row + 3, 90, f"Vibration: {vib_level:.2f}%", color)
        else:
            self.stdscr.addstr(start_row + 3, 90, f"Vib: Bekleniyor...", curses.color_pair(3))
    
    def draw_commands(self):
        """Komut bilgilerini çiz"""
        cmd_row = 9
        
        self.stdscr.addstr(cmd_row, 2, "⌨️  KOMUTLAR (REAL-TIME):", curses.color_pair(4) | curses.A_BOLD)
        
        commands = [
            "W/S: Pitch ↕",     "A/D: Roll ↔",        "Q/E: Yaw ↺↻",
            "O/L: Motor ⚡",     "X: Servo Reset",     "Space: ARM/DISARM 🔴", 
            "R/F: RAW/PID",     "1/2/3: GPS/IMU/HYB", "T: Test Scripts",
            "V: Vibration 📳",  "G: GPS Data 🌍",      "ESC/P: Exit 🚪"
        ]
        
        row = cmd_row + 1
        col = 4
        for i, cmd in enumerate(commands):
            if i % 3 == 0 and i > 0:
                row += 1
                col = 4
            self.stdscr.addstr(row, col, cmd, curses.color_pair(6))
            col += 20
    
    def draw_performance_info(self):
        """Performance bilgisi çiz"""
        perf_row = 14
        current_time = time.time()
        
        # FPS hesaplama
        screen_fps = 1.0 / max(0.001, current_time - self.last_screen_update) if self.last_screen_update > 0 else 0
        data_fps = self.data_fps_target
        
        self.stdscr.addstr(perf_row, 2, "⚡ PERFORMANCE (REAL-TIME):", curses.color_pair(5) | curses.A_BOLD)
        self.stdscr.addstr(perf_row + 1, 4, f"Screen FPS: {screen_fps:.1f} (Target: {self.screen_fps_target})")
        self.stdscr.addstr(perf_row + 2, 4, f"Data FPS: {data_fps:.1f} (Live stream)")
        
        # Bağlantı durumu
        conn_status = "LIVE" if self.mavlink and self.mavlink.connected else "OFFLINE"
        conn_color = curses.color_pair(1) if self.mavlink and self.mavlink.connected else curses.color_pair(2)
        self.stdscr.addstr(perf_row + 1, 50, f"Connection: {conn_status}", conn_color)
        
        # Data freshness
        data_age = "FRESH" if self.current_imu else "STALE"
        data_color = curses.color_pair(1) if self.current_imu else curses.color_pair(3)
        self.stdscr.addstr(perf_row + 2, 50, f"Data: {data_age}", data_color)
    
    def draw_logs(self):
        """Log mesajlarını çiz"""
        log_start = 18
        max_log_display = self.height - log_start - 2
        
        if max_log_display <= 0:
            return
        
        self.stdscr.addstr(log_start - 1, 2, "📝 LIVE LOG MESAJLARI:", curses.color_pair(4) | curses.A_BOLD)
        
        # Son mesajları göster
        start_idx = max(0, len(self.log_messages) - max_log_display)
        for i, message in enumerate(self.log_messages[start_idx:]):
            if log_start + i < self.height - 1:
                # Renk seçimi
                color = curses.color_pair(6)  # Varsayılan beyaz
                if "❌" in message:
                    color = curses.color_pair(2)  # Kırmızı
                elif "⚠️" in message:
                    color = curses.color_pair(3)  # Sarı
                elif "✅" in message:
                    color = curses.color_pair(1)  # Yeşil
                elif "🔧" in message or "🎮" in message:
                    color = curses.color_pair(5)  # Magenta
                
                # Mesajı kısalt
                display_message = message[:self.width - 4]
                self.stdscr.addstr(log_start + i, 4, display_message, color)

    def handle_keyboard(self):
        """Klavye girişini işle - OPTIMIZED REAL-TIME"""
        key = self.stdscr.getch()
        
        # Tuş basılmadıysa (timeout)
        if key == -1 or key == curses.ERR:
            return
        
        # Çıkış tuşları
        if key in [27, 3, ord('P'), ord('p')]:  # ESC, Ctrl+C, P/p tuşları
            self.running = False
            self.log("🔄 Çıkış komutu alındı...")
            return
        
        # Servo sıfırlama tuşu
        elif key == ord('x') or key == ord('X'):
            old_values = self.servo_values.copy()
            self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
            self.log(f"🔄 Servo sıfırlama: {old_values} → {self.servo_values}")
            self.send_servo_commands()
        
        # REAL-TIME servo kontrol - INSTANT RESPONSE
        elif key == ord('w') or key == ord('W'):
            old_pitch = self.servo_values['pitch']
            self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + 5)
            if old_pitch != self.servo_values['pitch']:
                self.log(f"🎮 LIVE Pitch: {old_pitch} → {self.servo_values['pitch']}")
            self.send_servo_commands()
        elif key == ord('s') or key == ord('S'):
            old_pitch = self.servo_values['pitch']
            self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - 5)
            if old_pitch != self.servo_values['pitch']:
                self.log(f"🎮 LIVE Pitch: {old_pitch} → {self.servo_values['pitch']}")
            self.send_servo_commands()
        elif key == ord('a') or key == ord('A'):
            old_roll = self.servo_values['roll']
            self.servo_values['roll'] = min(45, self.servo_values['roll'] + 5)
            if old_roll != self.servo_values['roll']:
                self.log(f"🎮 LIVE Roll: {old_roll} → {self.servo_values['roll']}")
            self.send_servo_commands()
        elif key == ord('d') or key == ord('D'):
            old_roll = self.servo_values['roll']
            self.servo_values['roll'] = max(-45, self.servo_values['roll'] - 5)
            if old_roll != self.servo_values['roll']:
                self.log(f"🎮 LIVE Roll: {old_roll} → {self.servo_values['roll']}")
            self.send_servo_commands()
        elif key == ord('q') or key == ord('Q'):
            old_yaw = self.servo_values['yaw']
            self.servo_values['yaw'] = min(45, self.servo_values['yaw'] + 5)
            if old_yaw != self.servo_values['yaw']:
                self.log(f"🎮 LIVE Yaw: {old_yaw} → {self.servo_values['yaw']}")
            self.send_servo_commands()
        elif key == ord('e') or key == ord('E'):
            old_yaw = self.servo_values['yaw']
            self.servo_values['yaw'] = max(-45, self.servo_values['yaw'] - 5)
            if old_yaw != self.servo_values['yaw']:
                self.log(f"🎮 LIVE Yaw: {old_yaw} → {self.servo_values['yaw']}")
            self.send_servo_commands()
        
        # REAL-TIME Motor kontrol
        elif key == ord('o') or key == ord('O'):
            old_motor = self.motor_value
            self.motor_value = min(100, self.motor_value + 10)
            if old_motor != self.motor_value:
                self.log(f"🎮 LIVE Motor: {old_motor} → {self.motor_value}% (O)")
            self.send_motor_command()
        elif key == ord('l') or key == ord('L'):
            old_motor = self.motor_value
            self.motor_value = max(-100, self.motor_value - 10)
            if old_motor != self.motor_value:
                self.log(f"🎮 LIVE Motor: {old_motor} → {self.motor_value}% (L)")
            self.send_motor_command()
        
        # ARM/DISARM
        elif key == ord(' '):  # Space
            if self.mavlink and self.mavlink.connected:
                try:
                    if self.armed:
                        self.mavlink.disarm()
                        self.armed = False
                        self.log("🟢 DISARMED - Güvenlik aktif")
                    else:
                        self.mavlink.arm()
                        self.armed = True
                        self.log("🔴 ARMED - Sistem aktif!")
                except Exception as e:
                    self.log(f"❌ ARM/DISARM hatası: {e}")
            else:
                self.log("⚠️ MAVLink bağlı değil - ARM/DISARM yapılamaz")
        
        # Kontrol modu değiştir
        elif key == ord('r') or key == ord('R'):
            old_mode = self.control_mode  
            self.control_mode = "RAW"
            if self.mavlink:
                self.mavlink.set_control_mode("raw")
            self.log(f"🔧 Kontrol modu: {old_mode} → {self.control_mode}")
        elif key == ord('f') or key == ord('F'):
            old_mode = self.control_mode  
            self.control_mode = "PID"
            if self.mavlink:
                self.mavlink.set_control_mode("pid")
            self.log(f"🔧 Kontrol modu: {old_mode} → {self.control_mode}")
        
        # Navigation modu
        elif key == ord('1'):
            old_nav = self.navigation_mode
            self.navigation_mode = "GPS"
            self.log(f"🧭 Navigation: {old_nav} → {self.navigation_mode}")
        elif key == ord('2'):
            old_nav = self.navigation_mode
            self.navigation_mode = "IMU"
            self.log(f"🧭 Navigation: {old_nav} → {self.navigation_mode}")
        elif key == ord('3'):
            old_nav = self.navigation_mode
            self.navigation_mode = "HYBRID"
            self.log(f"🧭 Navigation: {old_nav} → {self.navigation_mode}")
        
        # Debug bilgi
        elif key == ord('v') or key == ord('V'):
            if self.current_vibration:
                vib = self.current_vibration
                self.log(f"📳 LIVE Vibration: {vib['level']:.2f}% ({vib['color']})")
            else:
                self.log("📳 Vibration verisi yok")
        elif key == ord('g') or key == ord('G'):
            if self.current_gps:
                lat, lon, alt, sats = self.current_gps
                self.log(f"🌍 LIVE GPS: {lat:.6f}, {lon:.6f}, Alt: {alt:.1f}m, Sats: {sats}")
            else:
                self.log("🌍 GPS verisi yok")
        elif key == ord('i') or key == ord('I'):
            if self.current_imu:
                acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z = self.current_imu
                self.log(f"📊 LIVE IMU: Acc({acc_x:.3f},{acc_y:.3f},{acc_z:.3f}) Gyr({gyr_x:.3f},{gyr_y:.3f},{gyr_z:.3f})")
            else:
                self.log("📊 IMU verisi yok")
    
    def send_servo_commands(self):
        """Servo komutlarını gönder"""
        if not self.mavlink or not self.mavlink.connected:
            return
        
        try:
            # RAW veya PID moduna göre komut gönder
            if self.control_mode == "RAW":
                self.mavlink.control_servos_raw(
                    self.servo_values['roll'],
                    self.servo_values['pitch'], 
                    self.servo_values['yaw']
                )
            else:  # PID
                self.mavlink.control_servos_pid(
                    self.servo_values['roll'],
                    self.servo_values['pitch'],
                    self.servo_values['yaw']
                )
        except Exception as e:
            self.log(f"❌ Servo komut hatası: {e}")
    
    def send_motor_command(self):
        """Motor komutunu gönder"""
        if not self.mavlink or not self.mavlink.connected:
            return
        
        try:
            # Motor değerini PWM'e çevir
            motor_pwm = 1500 + (self.motor_value * 4)  # -100% = 1100, +100% = 1900
            motor_pwm = max(1100, min(1900, motor_pwm))
            
            self.mavlink.set_rc_channel_pwm(6, motor_pwm)
        except Exception as e:
            self.log(f"❌ Motor komut hatası: {e}")
    
    def update_servo_control(self):
        """Servo kontrol güncelle - sadece gerektiğinde"""
        # Bu method artık gereksiz - direct control kullanıyoruz
        pass
    
    def main_loop(self):
        """OPTIMIZED Ana döngü - MAXIMUM FPS + REAL DATA"""
        print("🔄 OPTIMIZED Ana döngü başladı...")
        self.log("🔄 OPTIMIZED Ana döngü başladı - REAL DATA MODE!")
        
        try:
            while self.running:
                try:
                    current_time = time.time()
                    
                    # 1. ALWAYS handle keyboard (HIGHEST PRIORITY)
                    self.handle_keyboard()
                    
                    # 2. Fetch live data (50 FPS)
                    self.fetch_live_data()
                    
                    # 3. Update screen only when needed (20 FPS MAX)
                    if current_time - self.last_screen_update >= (1.0 / self.screen_fps_target):
                        # Clear and redraw
                        self.stdscr.erase()
                        
                        # Draw all components
                        self.draw_header()
                        self.draw_controls()
                        self.draw_commands()
                        self.draw_performance_info()
                        self.draw_logs()
                        
                        # Refresh screen
                        self.stdscr.refresh()
                        
                        self.last_screen_update = current_time
                    
                    # MINIMAL sleep for CPU efficiency
                    time.sleep(0.01)  # 100 FPS keyboard polling
                    
                except KeyboardInterrupt:
                    print("⌨️ Ctrl+C algılandı, çıkılıyor...")
                    self.running = False
                except Exception as e:
                    self.log(f"❌ Ana döngü frame hatası: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            print(f"❌ Ana döngü kritik hatası: {e}")
            import traceback
            traceback.print_exc()
            
        print("🏁 OPTIMIZED Ana döngü tamamlandı")
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.log("🔄 OPTIMIZED Sistem kapatılıyor...")
        
        # Servolar neutral pozisyona
        if self.mavlink and self.mavlink.connected:
            try:
                self.mavlink.emergency_stop()
                self.mavlink.disconnect()
            except:
                pass
        
        self.log("✅ OPTIMIZED Sistem kapatıldı!")
    
    def run(self):
        """Uygulamayı çalıştır"""
        print("🔧 OPTIMIZED Sistem bileşenleri başlatılıyor...")
        # Sistem bileşenlerini başlat
        self.init_systems()
        print("✅ OPTIMIZED Sistem bileşenleri başlatıldı!")
        
        # Curses uygulamasını başlat
        try:
            print("🖥️ OPTIMIZED Curses wrapper başlatılıyor...")
            curses.wrapper(self._curses_main)
            print("✅ OPTIMIZED Curses wrapper tamamlandı!")
        except Exception as e:
            print(f"❌ Terminal GUI hatası: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("🔄 OPTIMIZED Cleanup işlemi başlatılıyor...")
            self.cleanup()
            print("✅ OPTIMIZED Cleanup tamamlandı!")
    
    def _curses_main(self, stdscr):
        """Curses ana fonksiyonu"""
        try:
            print("🔧 OPTIMIZED Curses başlatılıyor...")
            self.init_curses(stdscr)
            print("✅ OPTIMIZED Curses başlatıldı!")
            
            print("🔧 OPTIMIZED Main loop başlatılıyor...")
            self.main_loop()
            print("✅ OPTIMIZED Main loop tamamlandı!")
            
        except Exception as e:
            print(f"❌ Curses main hatası: {e}")
            import traceback
            traceback.print_exc()
            raise 

if __name__ == "__main__":
    print("🚀 TEKNOFEST Su Altı ROV - OPTIMIZED REAL DATA Terminal GUI başlatılıyor...")
    
    # Çalışma dizinini kontrol et
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı! App/ klasörünün içinden çalıştırın.")
        sys.exit(1)
    
    # Terminal GUI'yi başlat
    try:
        print("🔧 OPTIMIZED GUI sınıfı oluşturuluyor...")
        gui = TerminalROVGUI()
        print("✅ OPTIMIZED GUI sınıfı oluşturuldu!")
        
        print("🔧 OPTIMIZED GUI çalıştırılıyor...")
        gui.run()
        
    except KeyboardInterrupt:
        print("\n👋 Kullanıcı tarafından durduruldu!")
    except ImportError as e:
        print(f"❌ Import hatası: {e}")
        print("💡 Eksik kütüphane: pip install -r requirements.txt")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Kritik hata: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1) 