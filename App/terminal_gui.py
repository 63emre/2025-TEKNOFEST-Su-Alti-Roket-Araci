#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Advanced Terminal GUI
TCP-Based Real-time Control & Mission Planning System
"""

import sys
import os

# Windows curses desteği
try:
    import curses
except ImportError:
    print("❌ Windows'ta curses desteklenmiyor!")
    print("💡 Çözüm: pip install windows-curses")
    try:
        import subprocess
        subprocess.check_call([sys.executable, "-m", "pip", "install", "windows-curses"])
        import curses
        print("✅ windows-curses başarıyla yüklendi!")
    except Exception as e:
        print(f"❌ Otomatik yükleme başarısız: {e}")
        sys.exit(1)

import threading
import time
import subprocess
import json
import math
from datetime import datetime
from collections import deque

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

class MissionPlanner:
    """Görev planlama sistemi"""
    
    def __init__(self):
        self.mission_queue = []
        self.current_mission = None
        self.mission_running = False
        self.mission_commands = {
            '1': {'name': 'SAĞA', 'action': 'strafe_right', 'param': 'distance'},
            '2': {'name': 'SOLA', 'action': 'strafe_left', 'param': 'distance'},
            '3': {'name': 'İLERİ', 'action': 'forward', 'param': 'distance'},
            '4': {'name': 'GERİ', 'action': 'backward', 'param': 'distance'},
            '5': {'name': 'YUKARI', 'action': 'ascend', 'param': 'distance'},
            '6': {'name': 'AŞAĞI', 'action': 'descend', 'param': 'distance'},
            '7': {'name': 'SAG DÖN', 'action': 'yaw_right', 'param': 'angle'},
            '8': {'name': 'SOL DÖN', 'action': 'yaw_left', 'param': 'angle'},
            '9': {'name': 'DUR', 'action': 'stop', 'param': 'time'}
        }
    
    def add_mission(self, command, value):
        """Görev ekle"""
        if command in self.mission_commands:
            mission = {
                'command': command,
                'name': self.mission_commands[command]['name'],
                'action': self.mission_commands[command]['action'],
                'value': value,
                'status': 'pending'
            }
            self.mission_queue.append(mission)
            return True
        return False
    
    def clear_missions(self):
        """Görevleri temizle"""
        self.mission_queue.clear()
        self.mission_running = False
    
    def get_mission_summary(self):
        """Görev özetini döndür"""
        return [f"{i+1}. {m['name']} ({m['value']})" for i, m in enumerate(self.mission_queue)]
    
    def start_mission(self):
        """Görev akışını başlat"""
        if self.mission_queue and not self.mission_running:
            self.mission_running = True
            return True
        return False

class TestScriptManager:
    """Test script yönetim sistemi"""
    
    def __init__(self):
        self.available_scripts = {
            '1': {'name': 'Motor Testi', 'file': 'scripts/motor_test.py', 'desc': 'Tüm motorları test et'},
            '2': {'name': 'Servo Kalibrasyon', 'file': 'scripts/servo_calibration.py', 'desc': 'Servo kalibrasyonu'},
            '3': {'name': 'IMU Kalibrasyon', 'file': 'scripts/imu_calibration.py', 'desc': 'IMU kalibrasyonu'},
            '4': {'name': 'Sistem Kontrolü', 'file': 'scripts/system_check.py', 'desc': 'Tam sistem kontrolü'},
            '5': {'name': 'Depth Sensör Test', 'file': 'scripts/test_d300_depth_sensor.py', 'desc': 'D300 depth sensörü test'},
            '6': {'name': 'GPIO Test', 'file': 'scripts/test_gpio_button.py', 'desc': 'GPIO buton/LED test'},
            '7': {'name': 'Acil Durum Test', 'file': 'scripts/emergency_stop.py', 'desc': 'Acil durum protokolü'},
            '8': {'name': 'Stabilizasyon Test', 'file': 'scripts/test_stabilization.py', 'desc': 'Stabilizasyon test'},
            '9': {'name': 'Full System Test', 'file': 'scripts/test_full_system.py', 'desc': 'Tam sistem testi'}
        }
        self.running_script = None
    
    def get_script_list(self):
        """Test script listesini döndür"""
        return [(k, v['name'], v['desc']) for k, v in self.available_scripts.items()]
    
    def run_script(self, script_id, callback=None):
        """Test scriptini çalıştır"""
        if script_id in self.available_scripts:
            script_info = self.available_scripts[script_id]
            self.running_script = script_id
            
            def execute_script():
                try:
                    result = subprocess.run(
                        [sys.executable, script_info['file']],
                        capture_output=True,
                        text=True,
                        timeout=60
                    )
                    
                    if callback:
                        callback(script_id, result.returncode == 0, result.stdout, result.stderr)
                        
                except subprocess.TimeoutExpired:
                    if callback:
                        callback(script_id, False, "", "Timeout")
                except Exception as e:
                    if callback:
                        callback(script_id, False, "", str(e))
                finally:
                    self.running_script = None
            
            thread = threading.Thread(target=execute_script, daemon=True)
            thread.start()
            return True
        return False

class AdvancedTerminalGUI:
    def __init__(self):
        """Advanced Terminal GUI başlatıcı"""
        # Sistem bileşenleri
        self.mavlink = None
        self.navigation = None
        self.vibration_monitor = None
        self.depth_sensor = None
        self.gpio_controller = None
        
        # Yeni sistem bileşenleri
        self.mission_planner = MissionPlanner()
        self.test_manager = TestScriptManager()
        
        # Kontrol durumu
        self.control_mode = "RAW"  # RAW veya PID
        self.navigation_mode = "IMU"  # GPS, IMU, HYBRID
        self.armed = False
        self.running = True
        
        # Real-time kontrol - düzeltme
        self.active_keys = set()
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0
        self.depth_target = 0
        
        # Terminal UI
        self.stdscr = None
        self.height = 0
        self.width = 0
        self.current_menu = "main"  # main, mission_plan, test_scripts
        
        # Logs - optimize edilmiş
        self.log_messages = deque(maxlen=50)
        
        # Live IMU data - sadece roll/pitch/yaw
        self.live_imu = {
            'roll': 0.0,
            'pitch': 0.0, 
            'yaw': 0.0,
            'update_rate': 0,
            'connected': False,
            'last_update': 0
        }
        
        # TCP Data - direkt TCP'den
        self.tcp_data = {
            'imu_raw': [0, 0, 0, 0, 0, 0],  # ax, ay, az, gx, gy, gz
            'connected': False,
            'data_rate': 0,
            'last_packet': 0
        }
        
        # Thread kontrolü
        self.data_lock = threading.Lock()
        self.data_thread = None
        self.data_thread_running = False
        
        # Config
        self.load_config()
    
    def load_config(self):
        """Konfigürasyon yükle - optimize edilmiş"""
        try:
            with open("config/hardware_config.json", 'r') as f:
                self.config = json.load(f)
            
            # TCP bağlantı ayarları
            tcp_config = self.config.get("mavlink", {})
            tcp_config["connection_string"] = "tcp:127.0.0.1:5777"
            self.config["mavlink"] = tcp_config
            
        except Exception as e:
            self.log(f"❌ Config yükleme hatası: {e}")
            # Minimal config
            self.config = {
                "pixhawk": {
                    "servos": {"front_left": 1, "rear_left": 3, "rear_right": 4, "front_right": 5},
                    "motor": 6,
                    "pwm_limits": {"servo_min": 1100, "servo_max": 1900, "servo_neutral": 1500, "motor_min": 1000, "motor_max": 2000, "motor_stop": 1500}
                },
                "mavlink": {"connection_string": "tcp:127.0.0.1:5777"},
                "raspberry_pi": {"i2c": {"depth_sensor_address": "0x76", "bus_number": 1}}
            }
    
    def log(self, message):
        """Thread-safe log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        with self.data_lock:
            self.log_messages.append(log_entry)
    
    def init_systems(self):
        """Sistem bileşenlerini başlat - TCP odaklı"""
        self.log("🚀 TEKNOFEST ROV Advanced Terminal GUI başlatılıyor...")
        
        # TCP MAVLink bağlantısı
        try:
            self.mavlink = MAVLinkHandler()
            if self.mavlink.connect():
                self.log("✅ TCP MAVLink bağlantısı kuruldu (127.0.0.1:5777)!")
                self.tcp_data['connected'] = True
            else:
                self.log("⚠️ TCP MAVLink bağlantısı kurulamadı, offline mod")
        except Exception as e:
            self.log(f"❌ TCP MAVLink hatası: {e}")
        
        # Navigation engine
        try:
            self.navigation = NavigationEngine(self.mavlink)
            self.log("✅ Navigation engine başlatıldı")
        except Exception as e:
            self.log(f"❌ Navigation hatası: {e}")
        
        # I2C Depth sensor (0x76 adresinde)
        try:
            self.depth_sensor = D300DepthSensor()
            if self.depth_sensor.connect():
                self.log("✅ I2C Depth sensörü (0x76) bağlandı!")
            else:
                self.log("⚠️ I2C Depth sensörü bağlanamadı")
                self.depth_sensor = None
        except Exception as e:
            self.log(f"❌ I2C Depth sensörü hatası: {e}")
            self.depth_sensor = None
        
        # GPIO controller
        try:
            self.gpio_controller = GPIOController(self.config)
            self.log("✅ GPIO controller başlatıldı")
        except Exception as e:
            self.log(f"❌ GPIO controller hatası: {e}")
        
        # TCP veri thread başlat
        self.start_tcp_data_thread()
        
        self.log("✅ Tüm sistem bileşenleri başlatıldı!")
    
    def start_tcp_data_thread(self):
        """TCP veri thread'ini başlat - yüksek frekanslı"""
        self.data_thread_running = True
        self.data_thread = threading.Thread(target=self.tcp_data_loop, daemon=True)
        self.data_thread.start()
        self.log("🔄 TCP veri thread'i başlatıldı (50Hz)")
    
    def tcp_data_loop(self):
        """TCP veri döngüsü - 50Hz live data"""
        last_update = time.time()
        update_counter = 0
        
        while self.data_thread_running and self.running:
            try:
                current_time = time.time()
                dt = current_time - last_update
                
                # 50Hz veri güncelleme
                if dt >= 0.02:  # 20ms = 50Hz
                    self.update_tcp_data()
                    update_counter += 1
                    
                    # Her saniye rate hesapla
                    if update_counter % 50 == 0:
                        with self.data_lock:
                            self.live_imu['update_rate'] = int(50 / max(dt * 50, 1))
                    
                    last_update = current_time
                
                time.sleep(0.005)  # 5ms CPU efficiency
                
            except Exception as e:
                self.log(f"❌ TCP veri döngüsü hatası: {e}")
                time.sleep(0.1)
    
    def update_tcp_data(self):
        """TCP'den live veri güncelle"""
        if not self.mavlink or not self.mavlink.connected:
            with self.data_lock:
                self.tcp_data['connected'] = False
                self.live_imu['connected'] = False
            return
        
        try:
            # IMU verilerini TCP'den direkt al
            raw_imu = self.mavlink.get_imu_data()
            if raw_imu and len(raw_imu) >= 6:
                with self.data_lock:
                    # Raw veriyi sakla
                    self.tcp_data['imu_raw'] = raw_imu
                    self.tcp_data['connected'] = True
                    self.tcp_data['last_packet'] = time.time()
                    
                    # Live IMU hesapla - sadece roll/pitch/yaw
                    self.calculate_live_orientation(raw_imu)
                    
                    # Bağlantı durumu
                    self.live_imu['connected'] = True
                    self.live_imu['last_update'] = time.time()
                    
        except Exception as e:
            with self.data_lock:
                self.tcp_data['connected'] = False
                self.live_imu['connected'] = False
    
    def calculate_live_orientation(self, raw_imu):
        """Live roll/pitch/yaw hesapla - mission planner tarzı"""
        try:
            # Raw IMU verilerini al (SI units)
            accel_x, accel_y, accel_z = raw_imu[0], raw_imu[1], raw_imu[2]
            gyro_x, gyro_y, gyro_z = raw_imu[3], raw_imu[4], raw_imu[5]
            
            # Accelerometer'dan roll/pitch hesapla
            if abs(accel_z) > 0.001:
                roll_rad = math.atan2(accel_y, accel_z)
                pitch_rad = math.atan2(-accel_x, math.sqrt(accel_y*accel_y + accel_z*accel_z))
                
                # Radyan'dan dereceye
                roll_deg = math.degrees(roll_rad)
                pitch_deg = math.degrees(pitch_rad)
            else:
                roll_deg = self.live_imu['roll']
                pitch_deg = self.live_imu['pitch']
            
            # Gyroscope integration for yaw
            dt = 0.02  # 50Hz
            gyro_z_deg = math.degrees(gyro_z)
            yaw_deg = self.live_imu['yaw'] + (gyro_z_deg * dt)
            
            # Yaw normalize (-180 to +180)
            while yaw_deg > 180:
                yaw_deg -= 360
            while yaw_deg < -180:
                yaw_deg += 360
            
            # Live IMU güncelle
            self.live_imu['roll'] = roll_deg
            self.live_imu['pitch'] = pitch_deg
            self.live_imu['yaw'] = yaw_deg
            
        except Exception as e:
            # Hata durumunda mevcut değerleri koru
            pass
    
    def init_curses(self, stdscr):
        """Curses arayüzünü başlat"""
        self.stdscr = stdscr
        curses.curs_set(0)  # Cursor gizle
        curses.noecho()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.nodelay(True)
        stdscr.timeout(50)  # 20Hz UI update
        
        # Renkler
        curses.start_color()
        curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)   # Başarılı
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)     # Hata
        curses.init_pair(3, curses.COLOR_YELLOW, curses.COLOR_BLACK)  # Uyarı
        curses.init_pair(4, curses.COLOR_CYAN, curses.COLOR_BLACK)    # Info
        curses.init_pair(5, curses.COLOR_MAGENTA, curses.COLOR_BLACK) # Özel
        curses.init_pair(6, curses.COLOR_WHITE, curses.COLOR_BLACK)   # Normal
        
        # Ekran boyutu
        self.height, self.width = stdscr.getmaxyx()
        self.log(f"🖥️ Terminal boyutu: {self.width}x{self.height}")
    
    def draw_header(self):
        """Başlık çiz"""
        title = "🚀 TEKNOFEST ROV - Advanced Terminal [TCP:127.0.0.1:5777] 🚀"
        self.stdscr.addstr(0, max(0, (self.width - len(title)) // 2), title, curses.color_pair(4) | curses.A_BOLD)
        
        # Durum satırı
        status_line = 1
        tcp_status = "✅ TCP BAĞLI" if self.tcp_data['connected'] else "❌ TCP BAĞLI DEĞİL"
        arm_status = "🔴 ARMED" if self.armed else "🟢 DISARMED"
        menu_status = f"📋 {self.current_menu.upper()}"
        
        self.stdscr.addstr(status_line, 2, f"TCP: {tcp_status}", curses.color_pair(1 if self.tcp_data['connected'] else 2))
        self.stdscr.addstr(status_line, 25, f"Durum: {arm_status}", curses.color_pair(2 if self.armed else 1))
        self.stdscr.addstr(status_line, 45, f"Kontrol: {self.control_mode}", curses.color_pair(5))
        self.stdscr.addstr(status_line, 65, f"Menü: {menu_status}", curses.color_pair(4))
        
        # Çizgi
        self.stdscr.addstr(2, 0, "─" * min(self.width, 120), curses.color_pair(4))
    
    def draw_live_imu_display(self):
        """Live IMU görüntüleme - Mission Planner tarzı"""
        start_row = 4
        
        with self.data_lock:
            # Başlık
            self.stdscr.addstr(start_row, 2, "📊 LIVE IMU DATA - MISSION PLANNER STYLE", curses.color_pair(4) | curses.A_BOLD)
            
            # Bağlantı durumu
            if self.live_imu['connected']:
                conn_status = "✅ TCP LIVE"
                conn_color = curses.color_pair(1)
                data_age = time.time() - self.live_imu['last_update']
                freshness = f"({data_age:.2f}s)" if data_age < 1 else f"({data_age:.1f}s OLD)"
            else:
                conn_status = "❌ NO DATA"
                conn_color = curses.color_pair(2)
                freshness = ""
            
            self.stdscr.addstr(start_row, 45, f"{conn_status} {freshness}", conn_color)
            
            # Update rate
            rate_text = f"Rate: {self.live_imu['update_rate']}Hz"
            self.stdscr.addstr(start_row, 70, rate_text, curses.color_pair(3))
            
            # Live orientation values - büyük ve net
            if self.live_imu['connected']:
                # Roll - yeşil/kırmızı renk kodlu
                roll_val = self.live_imu['roll']
                roll_color = curses.color_pair(1) if abs(roll_val) < 30 else curses.color_pair(3) if abs(roll_val) < 60 else curses.color_pair(2)
                self.stdscr.addstr(start_row + 2, 4, f"ROLL:  {roll_val:+7.1f}°", roll_color | curses.A_BOLD)
                
                # Pitch - yeşil/kırmızı renk kodlu
                pitch_val = self.live_imu['pitch']
                pitch_color = curses.color_pair(1) if abs(pitch_val) < 30 else curses.color_pair(3) if abs(pitch_val) < 60 else curses.color_pair(2)
                self.stdscr.addstr(start_row + 3, 4, f"PITCH: {pitch_val:+7.1f}°", pitch_color | curses.A_BOLD)
                
                # Yaw - her zaman mavi
                yaw_val = self.live_imu['yaw']
                self.stdscr.addstr(start_row + 4, 4, f"YAW:   {yaw_val:+7.1f}°", curses.color_pair(4) | curses.A_BOLD)
                
                # Görsel çubuklar (0-30-60-90 derece)
                self.draw_angle_bar(start_row + 2, 25, roll_val, "Roll")
                self.draw_angle_bar(start_row + 3, 25, pitch_val, "Pitch")
                self.draw_angle_bar(start_row + 4, 25, yaw_val, "Yaw")
                
            else:
                self.stdscr.addstr(start_row + 2, 4, "ROLL:  ---- °", curses.color_pair(2))
                self.stdscr.addstr(start_row + 3, 4, "PITCH: ---- °", curses.color_pair(2))
                self.stdscr.addstr(start_row + 4, 4, "YAW:   ---- °", curses.color_pair(2))
                self.stdscr.addstr(start_row + 6, 4, "❌ TCP bağlantısı yok - IMU verisi alınamıyor", curses.color_pair(2))
    
    def draw_angle_bar(self, row, col, angle, label):
        """Açı için görsel çubuk çiz"""
        try:
            # -90 ile +90 derece arası normalize et
            normalized = max(-90, min(90, angle))
            bar_length = 20
            center = bar_length // 2
            
            # Çubuk karakteri hesapla
            pos = int((normalized / 90.0) * center) + center
            pos = max(0, min(bar_length - 1, pos))
            
            # Çubuk çiz
            bar = ['─'] * bar_length
            bar[center] = '│'  # Merkez
            bar[pos] = '█'     # Mevcut pozisyon
            
            bar_str = ''.join(bar)
            self.stdscr.addstr(row, col, f"{bar_str} {normalized:+5.1f}°", curses.color_pair(6))
            
        except:
            pass
    
    def draw_controls_and_menu(self):
        """Kontrol ve menü bilgileri"""
        start_row = 9
        
        if self.current_menu == "main":
            self.draw_main_controls(start_row)
        elif self.current_menu == "mission_plan":
            self.draw_mission_planning(start_row)
        elif self.current_menu == "test_scripts":
            self.draw_test_script_menu(start_row)
    
    def draw_main_controls(self, start_row):
        """Ana kontrol menüsü"""
        self.stdscr.addstr(start_row, 2, "⌨️  MANUEL KONTROL:", curses.color_pair(4) | curses.A_BOLD)
        
        # Real-time kontrol durumu
        self.stdscr.addstr(start_row + 1, 4, f"Roll:  {self.servo_values['roll']:+4.0f}° [A/D]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 2, 4, f"Pitch: {self.servo_values['pitch']:+4.0f}° [W/S]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 3, 4, f"Yaw:   {self.servo_values['yaw']:+4.0f}° [Q/E]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 4, 4, f"Motor: {self.motor_value:+4.0f}% [O/L]", curses.color_pair(1 if abs(self.motor_value) < 50 else 3))
        
        # Komut menüsü
        self.stdscr.addstr(start_row + 6, 2, "📋 MENÜ KOMUTLARI:", curses.color_pair(4) | curses.A_BOLD)
        
        commands = [
            "W/S: Pitch ±",       "A/D: Roll ±",        "Q/E: Yaw ±",
            "O/L: Motor ±",       "PgUp/PgDn: Güçlü Motor", "Space: ARM/DISARM",
            "R/F: RAW/PID",       "0: Mission Plan",    "T: Test Scripts",
            "C: Config",          "V: Vibration",       "G: GPS Data",
            "X: Exit (Pencere)",  "",                   ""
        ]
        
        row = start_row + 7
        col = 4
        for i, cmd in enumerate(commands):
            if i % 3 == 0 and i > 0:
                row += 1
                col = 4
            if cmd:
                self.stdscr.addstr(row, col, cmd[:18], curses.color_pair(6))
            col += 22
    
    def draw_mission_planning(self, start_row):
        """Görev planlama menüsü"""
        self.stdscr.addstr(start_row, 2, "🎯 GÖREV PLANLAMA SİSTEMİ:", curses.color_pair(4) | curses.A_BOLD)
        
        # Görev komutları
        self.stdscr.addstr(start_row + 1, 4, "GÖREV KOMUTLARI:", curses.color_pair(3))
        
        mission_cmds = [
            "1: SAĞA (metre)", "2: SOLA (metre)", "3: İLERİ (metre)",
            "4: GERİ (metre)", "5: YUKARI (metre)", "6: AŞAĞI (metre)", 
            "7: SAĞ DÖN (°)", "8: SOL DÖN (°)", "9: DUR (saniye)",
        ]
        
        row = start_row + 2
        for i, cmd in enumerate(mission_cmds):
            if i % 3 == 0 and i > 0:
                row += 1
            col = 4 + (i % 3) * 25
            self.stdscr.addstr(row, col, cmd, curses.color_pair(6))
        
        # Görev akışı kontrolleri
        self.stdscr.addstr(start_row + 6, 4, "AKIŞ KONTROL:", curses.color_pair(3))
        self.stdscr.addstr(start_row + 7, 6, "ENTER: Görev Akışını Başlat", curses.color_pair(1))
        self.stdscr.addstr(start_row + 8, 6, "C: Görevleri Temizle", curses.color_pair(2))
        self.stdscr.addstr(start_row + 9, 6, "B: Ana Menüye Dön", curses.color_pair(4))
        
        # Mevcut görev listesi
        mission_summary = self.mission_planner.get_mission_summary()
        if mission_summary:
            self.stdscr.addstr(start_row + 11, 4, "PLANLANAN GÖREVLER:", curses.color_pair(3))
            for i, mission in enumerate(mission_summary[:6]):  # Max 6 görev göster
                self.stdscr.addstr(start_row + 12 + i, 6, mission, curses.color_pair(5))
        else:
            self.stdscr.addstr(start_row + 11, 4, "Henüz görev eklenmemiş. Yukarıdaki komutları kullanın.", curses.color_pair(6))
    
    def draw_test_script_menu(self, start_row):
        """Test script menüsü"""
        self.stdscr.addstr(start_row, 2, "🔧 TEST SCRIPT YÖNETİCİSİ:", curses.color_pair(4) | curses.A_BOLD)
        
        # Test scriptleri listesi
        script_list = self.test_manager.get_script_list()
        
        self.stdscr.addstr(start_row + 1, 4, "MEVCUT TEST SCRİPTLERİ:", curses.color_pair(3))
        
        for i, (script_id, name, desc) in enumerate(script_list):
            row = start_row + 2 + i
            if row < self.height - 5:  # Ekran sınırı kontrolü
                status_text = "ÇALIŞIYOR..." if self.test_manager.running_script == script_id else ""
                self.stdscr.addstr(row, 6, f"{script_id}: {name}", curses.color_pair(1 if not status_text else 3))
                self.stdscr.addstr(row, 35, f"- {desc}", curses.color_pair(6))
                if status_text:
                    self.stdscr.addstr(row, 70, status_text, curses.color_pair(3) | curses.A_BOLD)
        
        # Kontrol komutları
        self.stdscr.addstr(start_row + 12, 4, "KONTROL:", curses.color_pair(3))
        self.stdscr.addstr(start_row + 13, 6, "1-9: İlgili scripti çalıştır", curses.color_pair(6))
        self.stdscr.addstr(start_row + 14, 6, "S: Çalışan scripti durdur", curses.color_pair(2))
        self.stdscr.addstr(start_row + 15, 6, "B: Ana menüye dön", curses.color_pair(4))
    
    def draw_logs(self):
        """Log mesajları - optimize"""
        if self.height < 20:
            return
            
        log_start = self.height - 6
        max_logs = 4
        
        self.stdscr.addstr(log_start - 1, 2, "📝 SYSTEM LOGS:", curses.color_pair(4) | curses.A_BOLD)
        
        with self.data_lock:
            recent_logs = list(self.log_messages)[-max_logs:]
        
        for i, message in enumerate(recent_logs):
            if log_start + i < self.height - 1:
                # Log rengini belirle
                color = curses.color_pair(6)
                if "❌" in message:
                    color = curses.color_pair(2)
                elif "⚠️" in message:
                    color = curses.color_pair(3)
                elif "✅" in message:
                    color = curses.color_pair(1)
                
                # Mesajı ekrana sığacak şekilde kısalt
                display_message = message[:self.width - 4]
                try:
                    self.stdscr.addstr(log_start + i, 4, display_message, color)
                except:
                    pass
    
    def handle_keyboard(self):
        """Klavye girişi - optimize edilmiş"""
        key = self.stdscr.getch()
        
        if key == -1 or key == curses.ERR:
            return
        
        # Menu-specific handling
        if self.current_menu == "main":
            self.handle_main_controls(key)
        elif self.current_menu == "mission_plan":
            self.handle_mission_planning(key)
        elif self.current_menu == "test_scripts":
            self.handle_test_scripts(key)
    
    def handle_main_controls(self, key):
        """Ana kontrol tuşları"""
        # Real-time servo kontrol
        if key == ord('w'):
            self.active_keys.add('w')
        elif key == ord('s'):
            self.active_keys.add('s')
        elif key == ord('a'):
            self.active_keys.add('a')
        elif key == ord('d'):
            self.active_keys.add('d')
        elif key == ord('q'):
            self.active_keys.add('q')
        elif key == ord('e'):
            self.active_keys.add('e')
        
        # Motor kontrol - DÜZELTİLDİ
        elif key == ord('o') or key == ord('O'):
            self.motor_value = min(100, self.motor_value + 5)
            self.send_motor_command()
            self.log(f"🎮 Motor: {self.motor_value}%")
        elif key == ord('l') or key == ord('L'):
            self.motor_value = max(-100, self.motor_value - 5)
            self.send_motor_command()
            self.log(f"🎮 Motor: {self.motor_value}%")
        elif key == curses.KEY_PPAGE:  # Page Up
            self.motor_value = min(100, self.motor_value + 15)
            self.send_motor_command()
            self.log(f"🎮 Motor BOOST: {self.motor_value}%")
        elif key == curses.KEY_NPAGE:  # Page Down
            self.motor_value = max(-100, self.motor_value - 15)
            self.send_motor_command()
            self.log(f"🎮 Motor BOOST: {self.motor_value}%")
        
        # ARM/DISARM
        elif key == ord(' '):  # Space
            self.toggle_arm()
        
        # Kontrol modu
        elif key == ord('r'):
            self.control_mode = "RAW"
            self.log("🎛️ Kontrol modu: RAW PWM")
        elif key == ord('f'):
            self.control_mode = "PID"
            self.log("🎛️ Kontrol modu: PID")
        
        # Menü geçişleri
        elif key == ord('0'):
            self.current_menu = "mission_plan"
            self.log("🎯 Görev planlama menüsüne geçildi")
        elif key == ord('t') or key == ord('T'):
            self.current_menu = "test_scripts"
            self.log("🔧 Test script menüsüne geçildi")
        
        # Diğer özellikler
        elif key == ord('c'):
            self.show_config_menu()
        elif key == ord('v'):
            self.show_vibration_data()
        elif key == ord('g'):
            self.show_gps_data()
    
    def handle_mission_planning(self, key):
        """Görev planlama tuşları"""
        # Görev ekleme
        if key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6'), ord('7'), ord('8'), ord('9')]:
            command = chr(key)
            value = self.get_mission_value_input(command)
            if value is not None:
                if self.mission_planner.add_mission(command, value):
                    cmd_name = self.mission_planner.mission_commands[command]['name']
                    self.log(f"✅ Görev eklendi: {cmd_name} ({value})")
        
        # Görev kontrolü
        elif key == ord('\n') or key == ord('\r'):  # Enter
            if self.mission_planner.start_mission():
                self.log("🚀 Görev akışı başlatıldı!")
                self.execute_mission_flow()
            else:
                self.log("❌ Başlatılacak görev yok!")
        
        elif key == ord('c') or key == ord('C'):
            self.mission_planner.clear_missions()
            self.log("🗑️ Tüm görevler temizlendi")
        
        elif key == ord('b') or key == ord('B'):
            self.current_menu = "main"
            self.log("🔙 Ana menüye dönüldü")
    
    def handle_test_scripts(self, key):
        """Test script tuşları"""
        # Script çalıştırma
        if key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6'), ord('7'), ord('8'), ord('9')]:
            script_id = chr(key)
            if self.test_manager.run_script(script_id, self.test_script_callback):
                script_info = self.test_manager.available_scripts.get(script_id, {})
                self.log(f"🔧 Test başlatıldı: {script_info.get('name', 'Unknown')}")
        
        # Script durdurma
        elif key == ord('s') or key == ord('S'):
            if self.test_manager.running_script:
                self.log("⏹️ Çalışan script durduruluyor...")
                # Script durdurma işlemi burada implement edilecek
            else:
                self.log("⚠️ Çalışan script yok")
        
        elif key == ord('b') or key == ord('B'):
            self.current_menu = "main"
            self.log("🔙 Ana menüye dönüldü")
    
    def get_mission_value_input(self, command):
        """Görev değeri girişi"""
        cmd_info = self.mission_planner.mission_commands.get(command, {})
        param_type = cmd_info.get('param', 'distance')
        
        if param_type == 'distance':
            prompt = "Mesafe (metre, 0.5-20): "
            default = 2.0
        elif param_type == 'angle':
            prompt = "Açı (derece, -180 to 180): "
            default = 90.0
        elif param_type == 'time':
            prompt = "Süre (saniye, 1-30): "
            default = 3.0
        else:
            return default
        
        # Basit input simülasyonu - gerçek implementasyonda input penceresi açılır
        return default  # Şimdilik varsayılan değer döndür
    
    def execute_mission_flow(self):
        """Görev akışını çalıştır"""
        def mission_executor():
            try:
                for mission in self.mission_planner.mission_queue:
                    if not self.mission_planner.mission_running:
                        break
                    
                    mission['status'] = 'executing'
                    self.log(f"🎯 Görev çalıştırılıyor: {mission['name']}")
                    
                    # Görev tipine göre işlem yap
                    success = self.execute_single_mission(mission)
                    
                    mission['status'] = 'completed' if success else 'failed'
                    
                    if not success:
                        self.log(f"❌ Görev başarısız: {mission['name']}")
                        break
                    
                    time.sleep(0.5)  # Görevler arası bekleme
                
                self.mission_planner.mission_running = False
                self.log("✅ Görev akışı tamamlandı!")
                
            except Exception as e:
                self.log(f"❌ Görev akışı hatası: {e}")
                self.mission_planner.mission_running = False
        
        # Mission executor thread
        mission_thread = threading.Thread(target=mission_executor, daemon=True)
        mission_thread.start()
    
    def execute_single_mission(self, mission):
        """Tek görev çalıştır"""
        try:
            action = mission['action']
            value = mission['value']
            
            if not self.mavlink or not self.mavlink.connected or not self.armed:
                self.log("❌ Görev için ARM gerekli!")
                return False
            
            # Görev tipine göre işlem
            if action == 'strafe_right':
                return self.execute_strafe(1, value)
            elif action == 'strafe_left':
                return self.execute_strafe(-1, value)
            elif action == 'forward':
                return self.execute_forward(value)
            elif action == 'backward':
                return self.execute_forward(-value)
            elif action == 'ascend':
                return self.execute_vertical(1, value)
            elif action == 'descend':
                return self.execute_vertical(-1, value)
            elif action == 'yaw_right':
                return self.execute_yaw(1, value)
            elif action == 'yaw_left':
                return self.execute_yaw(-1, value)
            elif action == 'stop':
                time.sleep(value)
                return True
            
            return False
            
        except Exception as e:
            self.log(f"❌ Görev çalıştırma hatası: {e}")
            return False
    
    def execute_strafe(self, direction, distance):
        """Yatay hareket (strafe)"""
        # Basit implementasyon - gerçekte navigation engine kullanılacak
        duration = distance * 2  # 2 saniye/metre
        roll_value = direction * 25  # ±25° roll
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_running:
                return False
            
            self.servo_values['roll'] = roll_value
            self.send_servo_commands()
            time.sleep(0.1)
        
        # Neutral'a getir
        self.servo_values['roll'] = 0
        self.send_servo_commands()
        return True
    
    def execute_forward(self, distance):
        """İleri/geri hareket"""
        duration = abs(distance) * 2  # 2 saniye/metre
        motor_value = 30 if distance > 0 else -30
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_running:
                return False
            
            self.motor_value = motor_value
            self.send_motor_command()
            time.sleep(0.1)
        
        # Motor'u durdur
        self.motor_value = 0
        self.send_motor_command()
        return True
    
    def execute_vertical(self, direction, distance):
        """Dikey hareket"""
        duration = distance * 3  # 3 saniye/metre
        pitch_value = direction * 20  # ±20° pitch
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_running:
                return False
            
            self.servo_values['pitch'] = pitch_value
            self.send_servo_commands()
            time.sleep(0.1)
        
        # Neutral'a getir
        self.servo_values['pitch'] = 0
        self.send_servo_commands()
        return True
    
    def execute_yaw(self, direction, angle):
        """Yaw dönüşü"""
        duration = abs(angle) / 45  # 45°/saniye
        yaw_value = direction * 30  # ±30° yaw servo
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_running:
                return False
            
            self.servo_values['yaw'] = yaw_value
            self.send_servo_commands()
            time.sleep(0.1)
        
        # Neutral'a getir
        self.servo_values['yaw'] = 0
        self.send_servo_commands()
        return True
    
    def test_script_callback(self, script_id, success, stdout, stderr):
        """Test script callback"""
        script_info = self.test_manager.available_scripts.get(script_id, {})
        script_name = script_info.get('name', 'Unknown')
        
        if success:
            self.log(f"✅ Test tamamlandı: {script_name}")
            # Stdout'un son birkaç satırını göster
            for line in stdout.strip().split('\n')[-2:]:
                if line.strip():
                    self.log(f"   {line}")
        else:
            self.log(f"❌ Test başarısız: {script_name}")
            # Stderr'in son birkaç satırını göster
            for line in stderr.strip().split('\n')[-2:]:
                if line.strip():
                    self.log(f"   {line}")
    
    def update_servo_control(self):
        """Real-time servo kontrolü"""
        # Pitch kontrol
        if 'w' in self.active_keys:
            self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + 3)
        elif 's' in self.active_keys:
            self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - 3)
        else:
            # Otomatik sıfırlama
            if self.servo_values['pitch'] > 0:
                self.servo_values['pitch'] = max(0, self.servo_values['pitch'] - 2)
            elif self.servo_values['pitch'] < 0:
                self.servo_values['pitch'] = min(0, self.servo_values['pitch'] + 2)
        
        # Roll kontrol
        if 'a' in self.active_keys:
            self.servo_values['roll'] = min(45, self.servo_values['roll'] + 3)
        elif 'd' in self.active_keys:
            self.servo_values['roll'] = max(-45, self.servo_values['roll'] - 3)
        else:
            if self.servo_values['roll'] > 0:
                self.servo_values['roll'] = max(0, self.servo_values['roll'] - 2)
            elif self.servo_values['roll'] < 0:
                self.servo_values['roll'] = min(0, self.servo_values['roll'] + 2)
        
        # Yaw kontrol
        if 'q' in self.active_keys:
            self.servo_values['yaw'] = min(45, self.servo_values['yaw'] + 3)
        elif 'e' in self.active_keys:
            self.servo_values['yaw'] = max(-45, self.servo_values['yaw'] - 3)
        else:
            if self.servo_values['yaw'] > 0:
                self.servo_values['yaw'] = max(0, self.servo_values['yaw'] - 2)
            elif self.servo_values['yaw'] < 0:
                self.servo_values['yaw'] = min(0, self.servo_values['yaw'] + 2)
        
        # Servo komutlarını gönder
        self.send_servo_commands()
        
        # Tuş durumunu temizle
        self.active_keys.clear()
    
    def send_servo_commands(self):
        """Servo komutlarını TCP üzerinden gönder"""
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
        """Motor komutunu TCP üzerinden gönder"""
        if not self.mavlink or not self.mavlink.connected or not self.armed:
            return
        
        try:
            # Motor PWM hesapla
            pwm_limits = self.config["pixhawk"]["pwm_limits"]
            neutral = pwm_limits["motor_stop"]
            pwm_range = (pwm_limits["motor_max"] - pwm_limits["motor_min"]) // 2
            
            pwm_value = neutral + (self.motor_value * pwm_range // 100)
            pwm_value = max(pwm_limits["motor_min"], min(pwm_limits["motor_max"], pwm_value))
            
            self.mavlink.send_raw_motor_pwm(int(pwm_value))
        except Exception as e:
            self.log(f"❌ Motor komut hatası: {e}")
    
    def toggle_arm(self):
        """ARM/DISARM toggle"""
        if not self.mavlink or not self.mavlink.connected:
            self.log("❌ TCP MAVLink bağlantısı yok!")
            return
        
        try:
            if self.armed:
                if self.mavlink.disarm_system():
                    self.armed = False
                    # Tüm kontrolleri sıfırla
                    self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
                    self.motor_value = 0
                    self.mission_planner.mission_running = False
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
    
    def show_config_menu(self):
        """Konfigürasyon menüsü"""
        self.log("🔧 Konfigürasyon menüsü açılıyor...")
        # Basit implementasyon - detayları daha sonra
    
    def show_vibration_data(self):
        """Vibration verilerini göster"""
        self.log("📈 Vibration verileri gösteriliyor...")
        # Vibration monitor'den veri al ve göster
    
    def show_gps_data(self):
        """GPS verilerini göster"""
        self.log("🗺️ GPS verileri gösteriliyor...")
        # GPS verilerini al ve göster
    
    def main_loop(self):
        """Ana döngü - optimize edilmiş"""
        last_ui_update = time.time()
        last_servo_update = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # UI güncelleme (20Hz)
                if current_time - last_ui_update > 0.05:
                    self.stdscr.erase()
                    
                    # UI bileşenlerini çiz
                    self.draw_header()
                    self.draw_live_imu_display()
                    self.draw_controls_and_menu()
                    self.draw_logs()
                    
                    # Ekranı yenile
                    self.stdscr.refresh()
                    last_ui_update = current_time
                
                # Klavye girişi
                self.handle_keyboard()
                
                # Real-time servo kontrolü (50Hz)
                if current_time - last_servo_update > 0.02:
                    self.update_servo_control()
                    last_servo_update = current_time
                
                # CPU efficiency
                time.sleep(0.001)
                
            except KeyboardInterrupt:
                self.running = False
            except Exception as e:
                self.log(f"❌ Ana döngü hatası: {e}")
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.log("🔄 Sistem kapatılıyor...")
        
        # Thread'leri durdur
        self.data_thread_running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        
        # Görevleri durdur
        self.mission_planner.mission_running = False
        
        # Servolar neutral'a
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
        
        self.log("✅ Sistem temizlendi!")
    
    def run(self):
        """Uygulamayı çalıştır"""
        # Sistem bileşenlerini başlat
        self.init_systems()
        
        # Curses uygulamasını başlat
        try:
            curses.wrapper(self._curses_main)
        except Exception as e:
            print(f"❌ Terminal GUI hatası: {e}")
        finally:
            self.cleanup()
    
    def _curses_main(self, stdscr):
        """Curses ana fonksiyonu"""
        self.init_curses(stdscr)
        self.main_loop()

if __name__ == "__main__":
    print("🚀 TEKNOFEST Su Altı ROV - Advanced Terminal GUI başlatılıyor...")
    
    # Çalışma dizinini kontrol et
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı! App/ klasörünün içinden çalıştırın.")
        sys.exit(1)
    
    # Terminal GUI'yi başlat
    try:
        gui = AdvancedTerminalGUI()
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