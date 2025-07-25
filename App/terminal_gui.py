#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Terminal GUI
Real-time Terminal Kontrol Uygulaması
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
        """Terminal GUI başlatıcı"""
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
        
        # IMU data buffers for graphs
        self.imu_history = {
            'roll': [0] * 100,
            'pitch': [0] * 100, 
            'yaw': [0] * 100,
            'accel_x': [0] * 50,
            'accel_y': [0] * 50,
            'accel_z': [0] * 50
        }
        
        # IMU veri durumu için buffer
        self.last_valid_imu = None
        self.imu_timeout_counter = 0
        
        # IMU veri stabilizasyonu için
        self.last_imu_display = None
        self.imu_update_counter = 0
        
        # Servo kontrol stabilizasyonu için
        self.last_servo_display = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.last_motor_display = 0
        self.servo_display_counter = 0
        
        # Depth verisi stabilizasyonu için
        self.last_depth_display = {'depth_m': 0.0, 'temperature_c': 0.0, 'pressure_mbar': 0.0}
        self.depth_update_counter = 0
        
        # Config
        self.load_config()
        
        # Depth sensor - MAVLink üzerinden al (Pixhawk'a bağlı)
        # I2C depth sensor'ü başlatma, MAVLink'den alacağız
        self.depth_sensor = None  # I2C kullanmıyoruz
        self.depth_data = {'depth_m': 0.0, 'temperature_c': 0.0, 'connected': False}
        self.log("💡 Depth sensörü MAVLink üzerinden alınacak")
        
        # GPIO controller
        try:
            self.gpio_controller = GPIOController(self.config)
            self.log("✅ GPIO controller başlatıldı")
        except Exception as e:
            self.log(f"❌ GPIO controller hatası: {e}")
        
        self.log("✅ Sistem bileşenleri başlatıldı!")
    
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
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.log_messages.append(log_entry)
        
        # Max log sayısını aş
        if len(self.log_messages) > self.max_logs:
            self.log_messages.pop(0)
    
    def init_systems(self):
        """Sistem bileşenlerini başlat"""
        print("🚀 TEKNOFEST ROV Terminal GUI sistem başlatma...")
        self.log("🚀 TEKNOFEST ROV Terminal GUI başlatılıyor...")
        
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
        
        # Depth sensor - MAVLink üzerinden al (Pixhawk'a bağlı)
        # I2C depth sensor'ü başlatma, MAVLink'den alacağız
        self.depth_sensor = None  # I2C kullanmıyoruz
        self.depth_data = {'depth_m': 0.0, 'temperature_c': 0.0, 'connected': False}
        print("💡 Depth sensörü MAVLink üzerinden alınacak")
        self.log("💡 Depth sensörü MAVLink üzerinden alınacak")
        
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
        self.log("✅ Sistem bileşenleri başlatıldı!")
    
    def init_curses(self, stdscr):
        """Curses arayüzünü başlat - optimized"""
        try:
            print("🔧 Curses ayarları yapılıyor...")
            self.stdscr = stdscr
            
            # Temel ayarlar
            curses.curs_set(0)  # Cursor gizle
            curses.noecho()     # Echo kapat
            curses.cbreak()     # Karakterleri anında al
            stdscr.keypad(True) # Özel tuşları etkinleştir
            
            # REAL-TIME için optimizasyon
            stdscr.nodelay(True)  # Non-blocking input
            stdscr.timeout(0)     # Hiç bekleme - anında dön
            
            # Refresh optimizasyonu
            if hasattr(curses, 'use_env'):
                curses.use_env(True)
            
            print("🎨 Renkler ayarlanıyor...")
            # Renkler
            curses.start_color()
            curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)  # Başarılı
            curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)    # Hata
            curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK) # Uyarı
            curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)   # Info
            curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK) # Özel
            
            # Ekran boyutu
            self.height, self.width = stdscr.getmaxyx()
            
            print(f"📏 Terminal boyutu: {self.width}x{self.height}")
            self.log(f"🖥️ Terminal boyutu: {self.width}x{self.height}")
            
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
    
    def draw_header(self):
        """Başlık çiz"""
        title = "🚀 TEKNOFEST Su Altı ROV - Terminal Kontrol 🚀"
        self.stdscr.addstr(0, (self.width - len(title)) // 2, title, curses.color_pair(4) | curses.A_BOLD)
        
        # Durum bilgisi
        status_line = 1
        mavlink_status = "✅ Bağlı" if self.mavlink and self.mavlink.connected else "❌ Bağlı Değil"
        arm_status = "🔴 ARMED" if self.armed else "🟢 DISARMED"
        
        self.stdscr.addstr(status_line, 2, f"MAVLink: {mavlink_status}", curses.color_pair(1 if self.mavlink and self.mavlink.connected else 2))
        self.stdscr.addstr(status_line, 25, f"Durum: {arm_status}", curses.color_pair(2 if self.armed else 1))
        self.stdscr.addstr(status_line, 45, f"Kontrol: {self.control_mode}", curses.color_pair(5))
        self.stdscr.addstr(status_line, 60, f"Navigation: {self.navigation_mode}", curses.color_pair(5))
        
        # Çizgi
        self.stdscr.addstr(2, 0, "─" * self.width, curses.color_pair(4))
    
    def draw_controls(self):
        """Kontrol bilgilerini çiz"""
        start_row = 4
        
        # Servo kontrol bilgisi (stabilized)
        # Sadece değerler değiştiyse güncelle
        servo_changed = (
            self.servo_values['roll'] != self.last_servo_display['roll'] or
            self.servo_values['pitch'] != self.last_servo_display['pitch'] or
            self.servo_values['yaw'] != self.last_servo_display['yaw'] or
            self.motor_value != self.last_motor_display or
            self.servo_display_counter % 50 == 0  # Her 5 saniyede bir zorla güncelle
        )
        
        if servo_changed:
            # Değerleri güncelle
            self.last_servo_display = self.servo_values.copy()
            self.last_motor_display = self.motor_value
        
        # Her zaman son stabilize değerleri göster
        self.stdscr.addstr(start_row, 2, "🎮 SERVO KONTROL:", curses.color_pair(4) | curses.A_BOLD)
        self.stdscr.addstr(start_row + 1, 4, f"Roll:  {self.last_servo_display['roll']:+3.0f}° (A/D)")
        self.stdscr.addstr(start_row + 2, 4, f"Pitch: {self.last_servo_display['pitch']:+3.0f}° (W/S)")
        self.stdscr.addstr(start_row + 3, 4, f"Yaw:   {self.last_servo_display['yaw']:+3.0f}° (Q/E)")
        
        # Motor kontrol (stabilized)
        self.stdscr.addstr(start_row, 35, "⚙️ MOTOR KONTROL:", curses.color_pair(4) | curses.A_BOLD)
        self.stdscr.addstr(start_row + 1, 37, f"Güç: {self.last_motor_display:+3.0f}% (O/L)")
        
        # Derinlik hedefi
        self.stdscr.addstr(start_row + 2, 37, f"Hedef Derinlik: {self.depth_target:.1f}m")
        
        self.servo_display_counter += 1
        
        # Gerçek zamanlı veriler (stabilized)
        if self.mavlink and self.mavlink.connected:
            try:
                imu_data = self.mavlink.get_imu_data()
                if imu_data and len(imu_data) >= 6:
                    # IMU history'yi güncelle
                    self.update_imu_history(imu_data)
                    
                    # Son geçerli veriyi kaydet
                    self.last_valid_imu = imu_data
                    self.imu_timeout_counter = 0
                    
                    # IMU display stabilizasyonu - her 3. frame'de bir güncelle
                    if self.imu_update_counter % 3 == 0 or self.last_imu_display is None:
                        self.last_imu_display = imu_data
                    
                    # Gösterim için stabil veriyi kullan
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.last_imu_display
                    
                    self.stdscr.addstr(start_row, 65, "📊 SENSÖR VERİ:", curses.color_pair(4) | curses.A_BOLD)
                    self.stdscr.addstr(start_row + 1, 67, f"Acc X: {accel_x:+6.2f}")
                    self.stdscr.addstr(start_row + 2, 67, f"Acc Y: {accel_y:+6.2f}")
                    self.stdscr.addstr(start_row + 3, 67, f"Acc Z: {accel_z:+6.2f}")
                    # Gyro verileri de ekleyelim
                    self.stdscr.addstr(start_row + 1, 85, f"Gyro X: {gyro_x:+6.2f}")
                    self.stdscr.addstr(start_row + 2, 85, f"Gyro Y: {gyro_y:+6.2f}")
                    self.stdscr.addstr(start_row + 3, 85, f"Gyro Z: {gyro_z:+6.2f}")
                    
                    self.imu_update_counter += 1
                    
                else:
                    # Veri yoksa ama bağlantı varsa - timeout sayacı
                    self.imu_timeout_counter += 1
                    self.imu_update_counter += 1
                    
                    if self.last_valid_imu and self.imu_timeout_counter < 100:  # 5 saniye timeout
                        # Son geçerli veriyi göster (stale data) - ama sık güncelleme
                        if self.imu_update_counter % 10 == 0:
                            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = self.last_valid_imu
                            self.stdscr.addstr(start_row, 65, "📊 SENSÖR VERİ:", curses.color_pair(3) | curses.A_BOLD)
                            self.stdscr.addstr(start_row + 1, 67, f"Acc X: {accel_x:+6.2f} [STALE]")
                            self.stdscr.addstr(start_row + 2, 67, f"Acc Y: {accel_y:+6.2f} [STALE]")
                            self.stdscr.addstr(start_row + 3, 67, f"Acc Z: {accel_z:+6.2f} [STALE]")
                            self.stdscr.addstr(start_row + 1, 85, f"Gyro X: {gyro_x:+6.2f}")
                            self.stdscr.addstr(start_row + 2, 85, f"Gyro Y: {gyro_y:+6.2f}")
                            self.stdscr.addstr(start_row + 3, 85, f"Gyro Z: {gyro_z:+6.2f}")
                    else:
                        # Gerçekten veri yok - daha az sıklıkta güncelle
                        if self.imu_update_counter % 20 == 0:
                            self.stdscr.addstr(start_row, 65, "📊 SENSÖR VERİ:", curses.color_pair(3) | curses.A_BOLD)
                            self.stdscr.addstr(start_row + 1, 67, f"IMU sinyali bekleniyor... ({self.imu_timeout_counter//20}s)")
                        
            except Exception as e:
                if self.imu_update_counter % 20 == 0:  # Hata mesajını sık gösterme
                    self.stdscr.addstr(start_row, 65, "📊 SENSÖR VERİ:", curses.color_pair(2) | curses.A_BOLD)
                    self.stdscr.addstr(start_row + 1, 67, f"IMU Hatası: {str(e)[:20]}...")
                    self.log(f"❌ IMU veri hatası: {e}")
                self.imu_update_counter += 1
        else:
            if self.imu_update_counter % 20 == 0:  # Bağlantı mesajını sık gösterme
                self.stdscr.addstr(start_row, 65, "📊 SENSÖR VERİ:", curses.color_pair(2) | curses.A_BOLD)
                self.stdscr.addstr(start_row + 1, 67, "MAVLink Bağlı Değil")
            self.imu_update_counter += 1
        
        # Depth sensor verileri - MAVLink üzerinden (stabilized)
        if self.mavlink and self.mavlink.connected:
            try:
                # MAVLink'den depth verisi al
                depth_data = self.mavlink.get_depth_data()
                if depth_data:
                    depth = depth_data['depth_m']
                    temp = depth_data['temperature_c']
                    pressure = depth_data.get('pressure_mbar', 0)
                    
                    # Sadece anlamlı değişiklik varsa güncelle (anti-flicker)
                    depth_changed = abs(depth - self.last_depth_display['depth_m']) > 0.01
                    temp_changed = abs(temp - self.last_depth_display['temperature_c']) > 0.1
                    pressure_changed = abs(pressure - self.last_depth_display['pressure_mbar']) > 0.5
                    
                    if depth_changed or temp_changed or pressure_changed or self.depth_update_counter % 20 == 0:
                        # Değerleri güncelle
                        self.last_depth_display = {
                            'depth_m': depth,
                            'temperature_c': temp, 
                            'pressure_mbar': pressure
                        }
                        self.depth_data = depth_data
                        self.depth_data['connected'] = True
                    
                    # Her zaman son değerleri göster (flicker önleme)
                    depth = self.last_depth_display['depth_m']
                    temp = self.last_depth_display['temperature_c']
                    pressure = self.last_depth_display['pressure_mbar']
                    
                    self.stdscr.addstr(start_row, 110, "🌊 DERİNLİK:", curses.color_pair(4) | curses.A_BOLD)
                    self.stdscr.addstr(start_row + 1, 112, f"Derinlik: {depth:.2f}m")
                    self.stdscr.addstr(start_row + 2, 112, f"Sıcaklık: {temp:.1f}°C")
                    self.stdscr.addstr(start_row + 3, 112, f"Basınç: {pressure:.1f}mb")
                    
                    self.depth_update_counter += 1
                    
                else:
                    # Veri yoksa ama bağlantı varsa - sayaç arttır
                    self.depth_update_counter += 1
                    if self.depth_update_counter % 10 == 0:  # Her 0.5 saniyede bir güncelle
                        self.stdscr.addstr(start_row, 110, "🌊 DERİNLİK:", curses.color_pair(3) | curses.A_BOLD)
                        self.stdscr.addstr(start_row + 1, 112, "MAVLink'den bekleniyor...")
                        self.depth_data['connected'] = False
                        
            except Exception as e:
                if self.depth_update_counter % 20 == 0:  # Hata mesajını sık gösterme
                    self.stdscr.addstr(start_row, 110, "🌊 DERİNLİK:", curses.color_pair(2) | curses.A_BOLD)
                    self.stdscr.addstr(start_row + 1, 112, f"MAVLink Hatası: {str(e)[:15]}")
                    self.depth_data['connected'] = False
                self.depth_update_counter += 1
        else:
            if self.depth_update_counter % 20 == 0:  # Bağlantı mesajını sık gösterme
                self.stdscr.addstr(start_row, 110, "🌊 DERİNLİK:", curses.color_pair(2) | curses.A_BOLD)  
                self.stdscr.addstr(start_row + 1, 112, "MAVLink Bağlı Değil")
                self.depth_data['connected'] = False
            self.depth_update_counter += 1
        
        # Vibration durumu (stabilized)
        if self.vibration_monitor:
            try:
                # Vibration verilerini sadece belirli aralıklarla güncelle
                if self.servo_display_counter % 10 == 0:  # Her 1 saniyede bir
                    vib_level = self.vibration_monitor.get_vibration_level()
                    vib_color = self.vibration_monitor.get_vibration_color()
                    color_map = {"green": 1, "yellow": 3, "red": 2}
                    color = curses.color_pair(color_map.get(vib_color, 1))
                    
                    self.stdscr.addstr(start_row + 3, 67, f"Vibration: {vib_level:.1f}%", color)
            except Exception as e:
                if self.servo_display_counter % 20 == 0:  # Hata mesajını sık gösterme
                    self.stdscr.addstr(start_row + 3, 67, f"Vib: Hata", curses.color_pair(2))
    
    def draw_commands(self):
        """Komut bilgilerini çiz"""
        cmd_row = 9
        
        self.stdscr.addstr(cmd_row, 2, "⌨️  KOMUTLAR:", curses.color_pair(4) | curses.A_BOLD)
        
        commands = [
            "W/S: Pitch",     "A/D: Roll",        "Q/E: Yaw",
            "O/L: Motor",     "X: Servo Sıfırla", "Space: ARM/DISARM", 
            "R/F: RAW/PID",   "1/2/3: GPS/IMU/HYB", "T: Test Scripts",
            "V: Vibration",   "G: GPS Data",      "ESC/P: Çıkış"
        ]
        
        row = cmd_row + 1
        col = 4
        for i, cmd in enumerate(commands):
            if i % 3 == 0 and i > 0:
                row += 1
                col = 4
            self.stdscr.addstr(row, col, cmd, curses.color_pair(3))
            col += 20
    
    def draw_logs(self):
        """Log mesajlarını çiz"""
        # Grafik alanını hesaba katarak log alanını ayarla
        if self.height >= 30:
            log_start = max(35, self.height - 15)  # Grafik varsa alt tarafa
        else:
            log_start = 15  # Grafik yoksa eski pozisyon
            
        max_log_display = self.height - log_start - 2
        
        if max_log_display <= 0:
            return
        
        self.stdscr.addstr(log_start - 1, 2, "📝 LOG MESAJLARI:", curses.color_pair(4) | curses.A_BOLD)
        
        # Son mesajları göster
        start_idx = max(0, len(self.log_messages) - max_log_display)
        for i, message in enumerate(self.log_messages[start_idx:]):
            if log_start + i < self.height - 1:
                # Renk seçimi
                color = curses.color_pair(1)  # Varsayılan yeşil
                if "❌" in message:
                    color = curses.color_pair(2)  # Kırmızı
                elif "⚠️" in message:
                    color = curses.color_pair(3)  # Sarı
                elif "🔧" in message or "🎮" in message:
                    color = curses.color_pair(5)  # Magenta
                
                # Mesajı kısalt
                display_message = message[:self.width - 4]
                self.stdscr.addstr(log_start + i, 4, display_message, color)
    
    def handle_keyboard(self):
        """Klavye girişini işle"""
        key = self.stdscr.getch()
        
        # Tuş basılmadıysa (timeout)
        if key == -1 or key == curses.ERR:
            return
        
        # Debug: Hangi tuş basıldığını göster (sadece önemli değişiklikler)
        important_keys = [ord('w'), ord('W'), ord('s'), ord('S'), ord('a'), ord('A'), 
                         ord('d'), ord('D'), ord('q'), ord('Q'), ord('e'), ord('E'),
                         ord('o'), ord('O'), ord('l'), ord('L'), ord('v'), ord('V'), ord('x'), ord('X')]
        
        if key in important_keys:
            self.log(f"🔤 {chr(key).upper()} tuşu basıldı")
        
        # Çıkış tuşları - ESC, Ctrl+C, P tuşu (çoklu seçenek)
        if key in [27, 3, ord('P'), ord('p'), curses.KEY_EXIT, curses.KEY_BREAK]:  # ESC, Ctrl+C, P/p tuşları
            self.running = False
            self.log("🔄 Çıkış komutu alındı...")
            return
        
        # Servo sıfırlama tuşu
        elif key == ord('x') or key == ord('X'):
            old_values = self.servo_values.copy()
            self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
            self.log(f"🔄 Servo sıfırlama: {old_values} → {self.servo_values}")
            self.send_servo_commands()
        
        # Real-time servo kontrol - ANLIK HAREKET
        elif key == ord('w') or key == ord('W'):
            old_pitch = self.servo_values['pitch']
            self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + 5)
            if old_pitch != self.servo_values['pitch']:
                self.log(f"🎮 Pitch: {old_pitch} → {self.servo_values['pitch']}")
            self.send_servo_commands()
        elif key == ord('s') or key == ord('S'):
            old_pitch = self.servo_values['pitch']
            self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - 5)
            if old_pitch != self.servo_values['pitch']:
                self.log(f"🎮 Pitch: {old_pitch} → {self.servo_values['pitch']}")
            self.send_servo_commands()
        elif key == ord('a') or key == ord('A'):
            old_roll = self.servo_values['roll']
            self.servo_values['roll'] = min(45, self.servo_values['roll'] + 5)
            if old_roll != self.servo_values['roll']:
                self.log(f"🎮 Roll: {old_roll} → {self.servo_values['roll']}")
            self.send_servo_commands()
        elif key == ord('d') or key == ord('D'):
            old_roll = self.servo_values['roll']
            self.servo_values['roll'] = max(-45, self.servo_values['roll'] - 5)
            if old_roll != self.servo_values['roll']:
                self.log(f"🎮 Roll: {old_roll} → {self.servo_values['roll']}")
            self.send_servo_commands()
        elif key == ord('q') or key == ord('Q'):
            old_yaw = self.servo_values['yaw']
            self.servo_values['yaw'] = min(45, self.servo_values['yaw'] + 5)
            if old_yaw != self.servo_values['yaw']:
                self.log(f"🎮 Yaw: {old_yaw} → {self.servo_values['yaw']}")
            self.send_servo_commands()
        elif key == ord('e') or key == ord('E'):
            old_yaw = self.servo_values['yaw']
            self.servo_values['yaw'] = max(-45, self.servo_values['yaw'] - 5)
            if old_yaw != self.servo_values['yaw']:
                self.log(f"🎮 Yaw: {old_yaw} → {self.servo_values['yaw']}")
            self.send_servo_commands()
        
        # Motor kontrol - ANLIK HAREKET
        elif key == curses.KEY_PPAGE:  # Page Up
            old_motor = self.motor_value
            self.motor_value = min(100, self.motor_value + 10)
            if old_motor != self.motor_value:
                self.log(f"🎮 Motor: {old_motor} → {self.motor_value}% (PageUp)")
            self.send_motor_command()
        elif key == curses.KEY_NPAGE:  # Page Down
            old_motor = self.motor_value
            self.motor_value = max(-100, self.motor_value - 10)
            if old_motor != self.motor_value:
                self.log(f"🎮 Motor: {old_motor} → {self.motor_value}% (PageDown)")
            self.send_motor_command()
        # Motor kontrol - O/L tuşları (ana kontrol)
        elif key == ord('o') or key == ord('O'):
            old_motor = self.motor_value
            self.motor_value = min(100, self.motor_value + 10)
            if old_motor != self.motor_value:
                self.log(f"🎮 Motor: {old_motor} → {self.motor_value}% (O)")
            self.send_motor_command()
        elif key == ord('l') or key == ord('L'):
            old_motor = self.motor_value
            self.motor_value = max(-100, self.motor_value - 10)
            if old_motor != self.motor_value:
                self.log(f"🎮 Motor: {old_motor} → {self.motor_value}% (L)")
            self.send_motor_command()
        # İnce ayar motor kontrol
        elif key == ord('+') or key == ord('='):
            old_motor = self.motor_value
            self.motor_value = min(100, self.motor_value + 5)
            if old_motor != self.motor_value:
                self.log(f"🎮 Motor: {old_motor} → {self.motor_value}% (+)")
            self.send_motor_command()
        elif key == ord('-') or key == ord('_'):
            old_motor = self.motor_value
            self.motor_value = max(-100, self.motor_value - 5)
            if old_motor != self.motor_value:
                self.log(f"🎮 Motor: {old_motor} → {self.motor_value}% (-)")
            self.send_motor_command()
        
        # ARM/DISARM
        elif key == ord(' '):  # Space
            self.toggle_arm()
        
        # Kontrol modu değiştir
        elif key == ord('r') or key == ord('R'):
            self.control_mode = "RAW"
            self.log("🎛️ Kontrol modu: RAW PWM")
        elif key == ord('f') or key == ord('F'):  # F tuşu ile PID (Filter) modu
            self.control_mode = "PID"
            self.log("🎛️ Kontrol modu: PID")
        
        # Navigation modu
        elif key == ord('1'):
            self.navigation_mode = "GPS"
            self.log("🧭 Navigation modu: GPS")
        elif key == ord('2'):
            self.navigation_mode = "IMU"
            self.log("🧭 Navigation modu: IMU")
        elif key == ord('3'):
            self.navigation_mode = "HYBRID"
            self.log("🧭 Navigation modu: HYBRID")
        
        # Test scriptleri
        elif key == ord('t') or key == ord('T'):
            self.show_test_menu()
            
        # Pin konfigürasyonu
        elif key == ord('c') or key == ord('C'):
            self.show_pin_config()
            
        # Vibration monitor
        elif key == ord('v') or key == ord('V'):
            self.log("🔤 Vibration menüsü açılıyor...")
            try:
                self.show_vibration_window()
                # Menü kapandıktan sonra ekranı zorla yenile
                self.stdscr.clear()
                self.stdscr.refresh()
                self.log("✅ Vibration menüsü kapatıldı")
            except Exception as e:
                self.log(f"❌ Vibration menü hatası: {e}")
        
        # GPS data
        elif key == ord('g') or key == ord('G'):
            self.log("🔤 GPS menüsü açılıyor...")
            try:
                self.show_gps_window()
                # Menü kapandıktan sonra ekranı zorla yenile
                self.stdscr.clear()
                self.stdscr.refresh()
                self.log("✅ GPS menüsü kapatıldı")
            except Exception as e:
                self.log(f"❌ GPS menü hatası: {e}")
        
        # Bilinmeyen tuş
        else:
            if key < 256:
                self.log(f"🔤 Bilinmeyen tuş: '{chr(key)}' ({key})")
            else:
                self.log(f"🔤 Bilinmeyen özel tuş: {key}")
    
    def update_servo_control(self):
        """Servo değerlerini otomatik sıfırla"""
        # Otomatik sıfırlama - yumuşak geçiş
        changed = False
        
        # Pitch otomatik sıfırlama
        if self.servo_values['pitch'] > 0:
            self.servo_values['pitch'] = max(0, self.servo_values['pitch'] - 0.5)
            changed = True
        elif self.servo_values['pitch'] < 0:
            self.servo_values['pitch'] = min(0, self.servo_values['pitch'] + 0.5)
            changed = True
        
        # Roll otomatik sıfırlama
        if self.servo_values['roll'] > 0:
            self.servo_values['roll'] = max(0, self.servo_values['roll'] - 0.5)
            changed = True
        elif self.servo_values['roll'] < 0:
            self.servo_values['roll'] = min(0, self.servo_values['roll'] + 0.5)
            changed = True
        
        # Yaw otomatik sıfırlama
        if self.servo_values['yaw'] > 0:
            self.servo_values['yaw'] = max(0, self.servo_values['yaw'] - 0.5)
            changed = True
        elif self.servo_values['yaw'] < 0:
            self.servo_values['yaw'] = min(0, self.servo_values['yaw'] + 0.5)
            changed = True
        
        # Değişiklik varsa servo komutlarını gönder
        if changed:
            self.send_servo_commands()
    
    def send_servo_commands(self):
        """Servo komutlarını MAVLink'e gönder"""
        if not self.mavlink or not self.mavlink.connected or not self.armed:
            return
        
        try:
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
        if not self.mavlink or not self.mavlink.connected or not self.armed:
            return
        
        try:
            # Motor PWM değerini hesapla (1500 ± %motor_value)
            pwm_value = 1500 + (self.motor_value * 4)  # ±400 PWM range
            self.mavlink.send_raw_motor_pwm(int(pwm_value))
            self.log(f"🎮 Motor: {self.motor_value}% (PWM: {pwm_value})")
        except Exception as e:
            self.log(f"❌ Motor komut hatası: {e}")
    
    def toggle_arm(self):
        """ARM/DISARM durumunu değiştir"""
        if not self.mavlink or not self.mavlink.connected:
            self.log("❌ MAVLink bağlantısı yok!")
            return
        
        try:
            if self.armed:
                if self.mavlink.disarm_system():
                    self.armed = False
                    self.log("🟢 Sistem DISARM edildi")
                else:
                    self.log("❌ DISARM başarısız!")
            else:
                if self.mavlink.arm_system():
                    self.armed = True
                    self.log("🔴 Sistem ARM edildi - DİKKAT!")
                else:
                    self.log("❌ ARM başarısız!")
        except Exception as e:
            self.log(f"❌ ARM/DISARM hatası: {e}")
    
    def show_test_menu(self):
        """Test menüsünü göster"""
        test_window = curses.newwin(15, 60, 5, 10)
        test_window.box()
        test_window.addstr(1, 2, "🔧 TEST SCRİPTLERİ", curses.color_pair(4) | curses.A_BOLD)
        
        scripts = [
            ("1", "Motor Testi", "motor_test.py"),
            ("2", "Servo Kalibrasyonu", "servo_calibration.py"),
            ("3", "IMU Kalibrasyonu", "imu_calibration.py"),
            ("4", "Sistem Kontrolü", "system_check.py"),
            ("5", "Acil Durum Testi", "emergency_stop.py"),
            ("0", "Geri", None)
        ]
        
        for i, (key, name, script) in enumerate(scripts):
            test_window.addstr(3 + i, 4, f"{key}: {name}")
        
        test_window.refresh()
        
        # Test seçimi bekle
        while True:
            key = test_window.getch()
            if key == ord('0'):  # 0 tuşu ile geri
                break
            elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                script_idx = int(chr(key)) - 1
                script_name = scripts[script_idx][2]
                if script_name:
                    self.run_test_script(script_name)
                break
        
        test_window.clear()
        test_window.refresh()
        del test_window
    
    def run_test_script(self, script_name):
        """Test scriptini çalıştır"""
        try:
            script_path = f"scripts/{script_name}"
            self.log(f"🔧 Script çalıştırılıyor: {script_name}")
            
            # Script'i arka planda çalıştır
            def run_script():
                try:
                    result = subprocess.run(
                        [sys.executable, script_path],
                        capture_output=True,
                        text=True,
                        timeout=30
                    )
                    
                    if result.returncode == 0:
                        self.log(f"✅ {script_name} başarılı!")
                        for line in result.stdout.strip().split('\n')[-3:]:  # Son 3 satır
                            if line.strip():
                                self.log(f"   {line}")
                    else:
                        self.log(f"❌ {script_name} başarısız!")
                        for line in result.stderr.strip().split('\n')[-2:]:  # Son 2 hata satırı
                            if line.strip():
                                self.log(f"   {line}")
                
                except subprocess.TimeoutExpired:
                    self.log(f"⏰ {script_name} timeout!")
                except Exception as e:
                    self.log(f"❌ {script_name} hatası: {e}")
            
            # Thread'de çalıştır
            script_thread = threading.Thread(target=run_script)
            script_thread.daemon = True
            script_thread.start()
            
        except Exception as e:
            self.log(f"❌ Script çalıştırma hatası: {e}")
    
    def show_pin_config(self):
        """Pin konfigürasyon menüsünü göster"""
        config_window = curses.newwin(20, 70, 3, 5)
        config_window.box()
        config_window.addstr(1, 2, "🔧 PIN KONFİGÜRASYONU", curses.color_pair(4) | curses.A_BOLD)
        
        # Mevcut I2C ayarları
        i2c_config = self.config.get("raspberry_pi", {}).get("i2c", {})
        current_address = i2c_config.get("depth_sensor_address", "0x76")
        current_bus = i2c_config.get("bus_number", 1)
        
        config_window.addstr(3, 4, f"Mevcut I2C Ayarları:")
        config_window.addstr(4, 6, f"Bus: {current_bus}")
        config_window.addstr(5, 6, f"D300 Adres: {current_address}")
        
        # GPIO ayarları
        gpio_config = self.config.get("raspberry_pi", {}).get("gpio", {})
        config_window.addstr(7, 4, f"GPIO Pinleri:")
        row = 8
        for pin_name, pin_num in gpio_config.items():
            config_window.addstr(row, 6, f"{pin_name}: Pin {pin_num}")
            row += 1
        
        # Seçenekler
        config_window.addstr(15, 4, "Seçenekler:")
        config_window.addstr(16, 6, "1: I2C Adresini Değiştir")
        config_window.addstr(17, 6, "2: I2C Bus'u Değiştir")
        config_window.addstr(18, 6, "9: Geri")
        
        config_window.refresh()
        
        # Pin ayarları seçimi bekle
        while True:
            key = config_window.getch()
            if key == ord('9'):  # 9 tuşu ile geri
                break
            elif key == ord('1'):
                self.change_i2c_address(config_window)
                break
            elif key == ord('2'):
                self.change_i2c_bus(config_window)
                break
        
        config_window.clear()
        config_window.refresh()
        del config_window
    
    def change_i2c_address(self, parent_window):
        """I2C adresini değiştir"""
        # Input penceresi
        input_window = curses.newwin(8, 50, 10, 15)
        input_window.box()
        input_window.addstr(1, 2, "I2C Adres Değiştir", curses.color_pair(4))
        input_window.addstr(3, 2, "Yeni I2C adresi (hex): 0x")
        input_window.addstr(5, 2, "Örnekler: 76, 77, 40, 48")
        input_window.addstr(6, 2, "Enter: Kaydet, ESC: İptal")
        
        curses.echo()
        curses.curs_set(1)
        input_window.refresh()
        
        try:
            # Hex değer gir
            hex_input = input_window.getstr(3, 25, 2).decode('utf-8')
            
            if hex_input:
                # Yeni adresi valide et
                new_address = int(hex_input, 16)
                new_address_str = f"0x{new_address:02x}"
                
                # Config'i güncelle
                if "raspberry_pi" not in self.config:
                    self.config["raspberry_pi"] = {}
                if "i2c" not in self.config["raspberry_pi"]:
                    self.config["raspberry_pi"]["i2c"] = {}
                
                self.config["raspberry_pi"]["i2c"]["depth_sensor_address"] = new_address_str
                
                # Config dosyasını kaydet
                with open("config/hardware_config.json", 'w') as f:
                    json.dump(self.config, f, indent=2)
                
                self.log(f"✅ I2C adresi {new_address_str} olarak güncellendi!")
                
                # Depth sensörü yeniden başlat
                if self.depth_sensor:
                    self.depth_sensor.disconnect()
                    self.depth_sensor = D300DepthSensor()
                    if self.depth_sensor.connect():
                        self.log("✅ Depth sensörü yeni adresle bağlandı!")
                    else:
                        self.log("⚠️ Depth sensörü yeni adresle bağlanamadı")
            
        except ValueError:
            self.log(f"❌ Geçersiz hex değer: {hex_input}")
        except Exception as e:
            self.log(f"❌ I2C adres değiştirme hatası: {e}")
        finally:
            curses.noecho()
            curses.curs_set(0)
            input_window.clear()
            input_window.refresh()
            del input_window
    
    def change_i2c_bus(self, parent_window):
        """I2C bus numarasını değiştir"""
        input_window = curses.newwin(7, 40, 10, 20)
        input_window.box()
        input_window.addstr(1, 2, "I2C Bus Değiştir", curses.color_pair(4))
        input_window.addstr(3, 2, "Yeni bus numarası (0-9): ")
        input_window.addstr(5, 2, "Enter: Kaydet, ESC: İptal")
        
        curses.echo()
        curses.curs_set(1)
        input_window.refresh()
        
        try:
            # Bus numarası gir
            bus_input = input_window.getstr(3, 25, 1).decode('utf-8')
            
            if bus_input and bus_input.isdigit():
                new_bus = int(bus_input)
                
                # Config'i güncelle
                if "raspberry_pi" not in self.config:
                    self.config["raspberry_pi"] = {}
                if "i2c" not in self.config["raspberry_pi"]:
                    self.config["raspberry_pi"]["i2c"] = {}
                
                self.config["raspberry_pi"]["i2c"]["bus_number"] = new_bus
                
                # Config dosyasını kaydet
                with open("config/hardware_config.json", 'w') as f:
                    json.dump(self.config, f, indent=2)
                
                self.log(f"✅ I2C bus {new_bus} olarak güncellendi!")
                
        except Exception as e:
            self.log(f"❌ I2C bus değiştirme hatası: {e}")
        finally:
            curses.noecho()
            curses.curs_set(0)
            input_window.clear()
            input_window.refresh()
            del input_window
    
    def show_vibration_window(self):
        """Vibration monitoring window"""
        vib_window = curses.newwin(18, 70, 2, 10)
        vib_window.box()
        vib_window.addstr(1, 2, "📈 TİTREŞİM MONİTÖRÜ", curses.color_pair(4) | curses.A_BOLD)
        
        # Vibration bilgilerini göster
        if self.vibration_monitor:
            try:
                vib_data = self.vibration_monitor.get_vibration_data()
                
                vib_window.addstr(3, 4, f"Titreşim Seviyesi: {vib_data.get('level', 0):.1f}%")
                vib_window.addstr(4, 4, f"Titreşim Kategorisi: {vib_data.get('category', 'unknown')}")
                vib_window.addstr(5, 4, f"Renk: {vib_data.get('color', 'green')}")
                
                # Frequency bands
                freq_bands = vib_data.get('frequency_bands', {})
                vib_window.addstr(7, 4, "Frekans Bandları:")
                vib_window.addstr(8, 6, f"Düşük (0-5Hz):  {freq_bands.get('low', 0):.2f}")
                vib_window.addstr(9, 6, f"Orta (5-15Hz):  {freq_bands.get('medium', 0):.2f}")
                vib_window.addstr(10, 6, f"Yüksek (15-25Hz): {freq_bands.get('high', 0):.2f}")
                
                # Dominant frequency
                vib_window.addstr(12, 4, f"Baskın Frekans: {vib_data.get('dominant_frequency', 0):.1f} Hz")
                
                # Buffer durumu
                vib_window.addstr(14, 4, f"Buffer Boyutu: {vib_data.get('buffer_size', 0)}")
                
            except Exception as e:
                vib_window.addstr(3, 4, f"❌ Vibration veri hatası: {e}")
        else:
            vib_window.addstr(3, 4, "❌ Vibration monitor başlatılmamış")
        
        vib_window.addstr(16, 4, "Herhangi bir tuş: Geri")
        vib_window.refresh()
        
        # Herhangi tuş bekle
        vib_window.getch()
        
        vib_window.clear()
        vib_window.refresh()
        del vib_window
    
    def show_gps_window(self):
        """GPS data window"""
        gps_window = curses.newwin(16, 70, 3, 10)
        gps_window.box()
        gps_window.addstr(1, 2, "🗺️  GPS VERİLERİ", curses.color_pair(4) | curses.A_BOLD)
        
        # GPS bilgilerini göster
        if self.mavlink and self.mavlink.connected:
            try:
                gps_data = self.mavlink.get_gps_data()
                
                if gps_data:
                    lat, lon, alt, satellites = gps_data
                    
                    gps_window.addstr(3, 4, f"Enlem (Latitude):  {lat:.7f}°")
                    gps_window.addstr(4, 4, f"Boylam (Longitude): {lon:.7f}°")
                    gps_window.addstr(5, 4, f"Yükseklik:         {alt:.1f} m")
                    gps_window.addstr(6, 4, f"Uydu Sayısı:       {satellites}")
                    
                    # GPS kalitesi
                    if satellites >= 6:
                        gps_status = "✅ İyi"
                        color = curses.color_pair(1)
                    elif satellites >= 4:
                        gps_status = "⚠️ Orta"
                        color = curses.color_pair(3)
                    else:
                        gps_status = "❌ Zayıf"
                        color = curses.color_pair(2)
                    
                    gps_window.addstr(8, 4, f"GPS Kalitesi:      {gps_status}", color)
                    
                    # Coordinate format
                    gps_window.addstr(10, 4, "Coordinate Format:")
                    gps_window.addstr(11, 6, f"DD: {lat:.6f}, {lon:.6f}")
                    
                    # Convert to degrees, minutes, seconds
                    lat_deg = int(abs(lat))
                    lat_min = int((abs(lat) - lat_deg) * 60)
                    lat_sec = ((abs(lat) - lat_deg) * 60 - lat_min) * 60
                    lat_dir = "N" if lat >= 0 else "S"
                    
                    lon_deg = int(abs(lon))
                    lon_min = int((abs(lon) - lon_deg) * 60)
                    lon_sec = ((abs(lon) - lon_deg) * 60 - lon_min) * 60
                    lon_dir = "E" if lon >= 0 else "W"
                    
                    gps_window.addstr(12, 6, f"DMS: {lat_deg}°{lat_min}'{lat_sec:.1f}\"{lat_dir}, {lon_deg}°{lon_min}'{lon_sec:.1f}\"{lon_dir}")
                    
                else:
                    gps_window.addstr(3, 4, "❌ GPS verisi alınamıyor")
                    gps_window.addstr(4, 4, "💡 GPS anteni bağlı mı?")
                    gps_window.addstr(5, 4, "💡 Açık havada mısınız?")
                    
            except Exception as e:
                gps_window.addstr(3, 4, f"❌ GPS veri hatası: {e}")
        else:
            gps_window.addstr(3, 4, "❌ MAVLink bağlantısı yok")
        
        gps_window.addstr(14, 4, "Herhangi bir tuş: Geri")
        gps_window.refresh()
        
        # Herhangi tuş bekle
        gps_window.getch()
        
        gps_window.clear()
        gps_window.refresh()
        del gps_window
    
    def update_imu_history(self, imu_data):
        """IMU verilerini history'ye ekle"""
        if not imu_data or len(imu_data) < 6:
            return
        
        try:
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
            
            # Roll, Pitch, Yaw hesapla (basit yaklaşım)
            roll = gyro_x * 57.3  # rad to deg approximation
            pitch = gyro_y * 57.3
            yaw = gyro_z * 57.3
            
            # History'yi güncelle
            self.imu_history['roll'].pop(0)
            self.imu_history['roll'].append(roll)
            
            self.imu_history['pitch'].pop(0)
            self.imu_history['pitch'].append(pitch)
            
            self.imu_history['yaw'].pop(0)
            self.imu_history['yaw'].append(yaw)
            
            self.imu_history['accel_x'].pop(0)
            self.imu_history['accel_x'].append(accel_x)
            
            self.imu_history['accel_y'].pop(0)
            self.imu_history['accel_y'].append(accel_y)
            
            self.imu_history['accel_z'].pop(0)
            self.imu_history['accel_z'].append(accel_z)
            
        except Exception as e:
            pass
    
    def draw_ascii_graph(self, x, y, width, height, data, title, color_pair=1):
        """ASCII grafik çiz"""
        try:
            if not data or len(data) == 0:
                return
            
            # Başlık
            self.stdscr.addstr(y, x, title, curses.color_pair(color_pair) | curses.A_BOLD)
            
            # Veri normalizasyonu
            min_val = min(data)
            max_val = max(data)
            
            if max_val == min_val:
                range_val = 1
            else:
                range_val = max_val - min_val
            
            # Grafik çerçevesi
            for i in range(height):
                self.stdscr.addstr(y + 1 + i, x, "│", curses.color_pair(4))
                self.stdscr.addstr(y + 1 + i, x + width - 1, "│", curses.color_pair(4))
            
            # Alt ve üst çizgi
            self.stdscr.addstr(y + 1, x, "┌" + "─" * (width - 2) + "┐", curses.color_pair(4))
            self.stdscr.addstr(y + height, x, "└" + "─" * (width - 2) + "┘", curses.color_pair(4))
            
            # Veri noktalarını çiz
            for i, value in enumerate(data[-width+2:]):  # Son width-2 değeri al
                if i >= width - 2:
                    break
                
                # Y pozisyonu hesapla
                normalized = (value - min_val) / range_val if range_val > 0 else 0.5
                graph_y = int((1 - normalized) * (height - 2))  # Ters çevir
                graph_y = max(0, min(height - 3, graph_y))
                
                # Grafik karakteri
                char = "█" if normalized > 0.7 else "▓" if normalized > 0.3 else "░"
                
                try:
                    self.stdscr.addstr(y + 2 + graph_y, x + 1 + i, char, curses.color_pair(color_pair))
                except:
                    pass
            
            # Min/Max değerlerini göster
            if height > 4:
                self.stdscr.addstr(y + 1, x + width + 1, f"Max: {max_val:+.2f}", curses.color_pair(color_pair))
                self.stdscr.addstr(y + height, x + width + 1, f"Min: {min_val:+.2f}", curses.color_pair(color_pair))
                
        except Exception as e:
            pass
    
    def draw_progress_bar(self, x, y, width, value, max_value, title, color_pair=1):
        """İlerleme çubuğu çiz"""
        try:
            # Başlık
            self.stdscr.addstr(y, x, title, curses.color_pair(4) | curses.A_BOLD)
            
            # Değer yüzdesi
            percentage = abs(value) / max_value if max_value > 0 else 0
            percentage = min(1.0, max(0.0, percentage))
            
            # Bar uzunluğu
            filled_length = int(width * percentage)
            
            # Bar çiz
            bar_str = ""
            for i in range(width):
                if i < filled_length:
                    if value >= 0:
                        bar_str += "█"
                    else:
                        bar_str += "▓"
                else:
                    bar_str += "░"
            
            # Renk seçimi - pozitif/negatif
            if value >= 0:
                bar_color = color_pair
            else:
                bar_color = curses.color_pair(2)  # Kırmızı
            
            self.stdscr.addstr(y + 1, x, f"[{bar_str}]", bar_color)
            self.stdscr.addstr(y + 1, x + width + 3, f"{value:+6.1f}", curses.color_pair(4))
            
        except Exception as e:
            pass
    
    def draw_graphs(self):
        """IMU grafikleri ve progress barları çiz"""
        if self.height < 30 or self.width < 120:  # Yeterli yer yoksa çizme
            return
        
        graph_start_row = 20
        
        try:
            # Motor durumu progress barları
            self.draw_progress_bar(2, graph_start_row, 20, self.motor_value, 100, "🎮 MOTOR GÜÇ", 1)
            
            # Servo durumu progress barları
            self.draw_progress_bar(2, graph_start_row + 3, 20, self.servo_values['roll'], 45, "📐 ROLL", 1)
            self.draw_progress_bar(2, graph_start_row + 6, 20, self.servo_values['pitch'], 45, "📐 PITCH", 1)
            self.draw_progress_bar(2, graph_start_row + 9, 20, self.servo_values['yaw'], 45, "📐 YAW", 1)
            
            # IMU grafikleri (sadece MAVLink bağlıysa)
            if self.mavlink and self.mavlink.connected:
                # YAW grafiği
                self.draw_ascii_graph(30, graph_start_row, 30, 6, 
                                    self.imu_history['yaw'], "YAW (°)", 4)
                
                # PITCH grafiği  
                self.draw_ascii_graph(30, graph_start_row + 7, 30, 6,
                                    self.imu_history['pitch'], "PITCH (°)", 4)
                
                # ROLL grafiği
                self.draw_ascii_graph(70, graph_start_row, 30, 6,
                                    self.imu_history['roll'], "ROLL (°)", 4)
                
                # Acceleration X grafiği
                self.draw_ascii_graph(70, graph_start_row + 7, 30, 6,
                                    self.imu_history['accel_x'], "ACCEL X", 3)
                
        except Exception as e:
            # Hata durumunda basit mesaj göster
            try:
                self.stdscr.addstr(graph_start_row, 2, f"📊 Grafik Hatası: {str(e)[:40]}", curses.color_pair(2))
            except:
                pass
    
    def main_loop(self):
        """Ana döngü - optimized anti-flicker"""
        last_update = time.time()
        last_servo_values = self.servo_values.copy()
        last_motor_value = self.motor_value
        force_update_counter = 0
        
        print("🔄 Ana döngü başladı...")
        self.log("🔄 Ana döngü başladı...")
        
        try:
            while self.running:
                try:
                    current_time = time.time()
                    
                    # Klavye girişini kontrol et (YÜKSEK FREKANSTA)
                    self.handle_keyboard()
                    
                    # Servo otomatik sıfırlama (10Hz)
                    if current_time - last_update > 0.1:
                        self.update_servo_control()
                        last_update = current_time
                    
                    # EKRANI SADECE DEĞİŞİKLİK OLDUĞUNDA YENİLE!
                    servo_changed = (
                        self.servo_values != last_servo_values or 
                        self.motor_value != last_motor_value
                    )
                    
                    force_update = force_update_counter % 100 == 0  # Her 10 saniyede zorla
                    
                    if servo_changed or force_update or force_update_counter < 3:
                        # Ekranı temizle ve yenile
                        self.stdscr.erase()
                        
                        # UI bileşenlerini çiz
                        self.draw_header() 
                        self.draw_controls()
                        self.draw_commands()
                        self.draw_logs()
                        self.draw_graphs()
                        
                        # Ekranı yenile
                        self.stdscr.refresh()
                        
                        # Son değerleri kaydet
                        last_servo_values = self.servo_values.copy()
                        last_motor_value = self.motor_value
                        
                        if servo_changed:
                            self.log(f"🔄 Ekran güncellendi - Servo: {self.servo_values}")
                    
                    force_update_counter += 1
                    
                    # YÜKSEK FREKANSTA KLAVYE OKUMA - düşük CPU
                    time.sleep(0.02)  # 50Hz klavye okuma
                    
                except KeyboardInterrupt:
                    print("⌨️ Ctrl+C algılandı, çıkılıyor...")
                    self.running = False
                except Exception as e:
                    print(f"❌ Ana döngü frame hatası: {e}")
                    self.log(f"❌ Ana döngü frame hatası: {e}")
                    time.sleep(0.1)
                    
        except Exception as e:
            print(f"❌ Ana döngü kritik hatası: {e}")
            import traceback
            traceback.print_exc()
            
        print("🏁 Ana döngü tamamlandı")
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.log("🔄 Sistem kapatılıyor...")
        
        # Servolar neutral pozisyona
        if self.mavlink and self.mavlink.connected:
            try:
                self.mavlink.emergency_stop()
                self.mavlink.disconnect()
            except:
                pass
        
        # Depth sensor
        if self.depth_sensor:
            try:
                self.depth_sensor.disconnect()
            except:
                pass
        
        self.log("✅ Sistem kapatıldı!")
    
    def run(self):
        """Uygulamayı çalıştır"""
        print("🔧 Sistem bileşenleri başlatılıyor...")
        # Sistem bileşenlerini başlat
        self.init_systems()
        print("✅ Sistem bileşenleri başlatıldı!")
        
        # Curses uygulamasını başlat
        try:
            print("🖥️ Curses wrapper başlatılıyor...")
            curses.wrapper(self._curses_main)
            print("✅ Curses wrapper tamamlandı!")
        except Exception as e:
            print(f"❌ Terminal GUI hatası: {e}")
            import traceback
            traceback.print_exc()
        finally:
            print("🔄 Cleanup işlemi başlatılıyor...")
            self.cleanup()
            print("✅ Cleanup tamamlandı!")
    
    def _curses_main(self, stdscr):
        """Curses ana fonksiyonu"""
        try:
            print("🔧 Curses başlatılıyor...")
            self.init_curses(stdscr)
            print("✅ Curses başlatıldı!")
            
            print("🔧 Main loop başlatılıyor...")
            self.main_loop()
            print("✅ Main loop tamamlandı!")
            
        except Exception as e:
            print(f"❌ Curses main hatası: {e}")
            import traceback
            traceback.print_exc()
            raise 

if __name__ == "__main__":
    print("🚀 TEKNOFEST Su Altı ROV - Terminal GUI başlatılıyor...")
    
    # Çalışma dizinini kontrol et
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı! App/ klasörünün içinden çalıştırın.")
        sys.exit(1)
    
    # Terminal GUI'yi başlat
    try:
        print("🔧 GUI sınıfı oluşturuluyor...")
        gui = TerminalROVGUI()
        print("✅ GUI sınıfı oluşturuldu!")
        
        print("🔧 GUI çalıştırılıyor...")
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