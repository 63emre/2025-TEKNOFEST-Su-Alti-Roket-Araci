#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Terminal GUI
Real-time Terminal Kontrol Uygulaması - Optimized Version
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
        
        # Movement targets
        self.movement_active = False
        self.movement_target = {'x': 0, 'y': 0, 'z': 0}
        self.current_position = {'x': 0, 'y': 0, 'z': 0}
        
        # Terminal UI
        self.stdscr = None
        self.height = 0
        self.width = 0
        
        # Logs
        self.log_messages = deque(maxlen=100)
        
        # Real IMU data - direct from MAVLink TCP
        self.imu_data = {
            'accel_x': 0, 'accel_y': 0, 'accel_z': 0,
            'gyro_x': 0, 'gyro_y': 0, 'gyro_z': 0,
            'roll': 0, 'pitch': 0, 'yaw': 0,
            'timestamp': 0,
            'connected': False
        }
        
        # Simple data storage - no complex history
        self.sensor_readings = {
            'roll_deg': 0.0,
            'pitch_deg': 0.0, 
            'yaw_deg': 0.0,
            'accel_x_mg': 0.0,  # milli-g values (larger numbers)
            'accel_y_mg': 0.0,
            'accel_z_mg': 0.0,
            'gyro_x_dps': 0.0,  # degrees per second (larger numbers)
            'gyro_y_dps': 0.0,
            'gyro_z_dps': 0.0,
            'update_count': 0
        }
        
        # Real depth data
        self.depth_data = {
            'depth_m': 0.0, 
            'temperature_c': 0.0, 
            'pressure_mbar': 0.0,
            'connected': False,
            'timestamp': 0
        }
        
        # IMU integration for real roll/pitch/yaw
        self.integrated_angles = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.last_imu_time = 0
        
        # Config
        self.load_config()
        
        # Data update thread control
        self.data_lock = threading.Lock()
        self.data_thread = None
        self.data_thread_running = False
    
    def load_config(self):
        """Konfigürasyon yükle"""
        try:
            with open("config/hardware_config.json", 'r') as f:
                self.config = json.load(f)
                
            # I2C depth sensor adresini 0x76 olarak ayarla
            if "raspberry_pi" not in self.config:
                self.config["raspberry_pi"] = {}
            if "i2c" not in self.config["raspberry_pi"]:
                self.config["raspberry_pi"]["i2c"] = {}
            
            self.config["raspberry_pi"]["i2c"]["depth_sensor_address"] = "0x76"
            self.config["raspberry_pi"]["i2c"]["bus_number"] = 1
            
        except Exception as e:
            self.log(f"❌ Config yükleme hatası: {e}")
            # Varsayılan config
            self.config = {
                "pixhawk": {
                    "servos": {"front_left": 1, "rear_left": 3, "rear_right": 4, "front_right": 5},
                    "motor": 6,
                    "pwm_limits": {"servo_min": 1100, "servo_max": 1900, "servo_neutral": 1500, "motor_stop": 1500}
                },
                "mavlink": {"connection_string": "tcp:127.0.0.1:5777"},
                "raspberry_pi": {
                    "i2c": {
                        "depth_sensor_address": "0x76",
                        "bus_number": 1
                    }
                }
            }
    
    def log(self, message):
        """Log mesajı ekle - thread-safe"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        log_entry = f"[{timestamp}] {message}"
        
        with self.data_lock:
            self.log_messages.append(log_entry)
    
    def init_systems(self):
        """Sistem bileşenlerini başlat"""
        self.log("🚀 TEKNOFEST ROV Terminal GUI başlatılıyor...")
        
        # MAVLink bağlantısı
        try:
            self.mavlink = MAVLinkHandler()
            if self.mavlink.connect():
                self.log("✅ MAVLink bağlantısı kuruldu!")
            else:
                self.log("⚠️ MAVLink bağlantısı kurulamadı, offline mod")
        except Exception as e:
            self.log(f"❌ MAVLink hatası: {e}")
        
        # Navigation engine
        try:
            self.navigation = NavigationEngine(self.mavlink)
            self.log("✅ Navigation engine başlatıldı")
        except Exception as e:
            self.log(f"❌ Navigation hatası: {e}")
        
        # Vibration monitor
        try:
            if self.mavlink:
                self.vibration_monitor = VibrationMonitor(self.mavlink)
                self.log("✅ Vibration monitor başlatıldı")
        except Exception as e:
            self.log(f"❌ Vibration monitor hatası: {e}")
        
        # I2C Depth sensor - 0x76 adresinde aktif et
        try:
            self.depth_sensor = D300DepthSensor()
            if self.depth_sensor.connect():
                self.log("✅ I2C Depth sensörü (0x76) bağlandı!")
                self.depth_data['connected'] = True
            else:
                self.log("⚠️ I2C Depth sensörü bağlanamadı, MAVLink'den denenecek")
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
        
        # Start data update thread
        self.start_data_thread()
        
        self.log("✅ Sistem bileşenleri başlatıldı!")
    
    def start_data_thread(self):
        """Veri güncelleme thread'ini başlat"""
        self.data_thread_running = True
        self.data_thread = threading.Thread(target=self.data_update_loop, daemon=True)
        self.data_thread.start()
        self.log("🔄 Veri güncelleme thread'i başlatıldı")
    
    def data_update_loop(self):
        """Arkaplanda veri güncelleme döngüsü - 50Hz"""
        last_update = time.time()
        
        while self.data_thread_running and self.running:
            try:
                current_time = time.time()
                dt = current_time - last_update
                
                # 50Hz veri güncelleme
                if dt >= 0.02:  # 50Hz = 20ms
                    self.update_sensor_data()
                    last_update = current_time
                
                time.sleep(0.005)  # 5ms sleep for CPU efficiency
                
            except Exception as e:
                self.log(f"❌ Veri güncelleme hatası: {e}")
                time.sleep(0.1)
    
    def update_sensor_data(self):
        """Tüm sensör verilerini güncelle - Direkt TCP MAVLink"""
        current_time = time.time()
        
        # MAVLink TCP'den direkt IMU verilerini al
        if self.mavlink and self.mavlink.connected:
            try:
                # Raw IMU data from MAVLink TCP
                raw_imu = self.mavlink.get_imu_data()
                if raw_imu and len(raw_imu) >= 6:
                    with self.data_lock:
                        # Convert to meaningful large values
                        # Acceleration: g to milli-g (1g = 1000mg)
                        self.imu_data['accel_x'] = raw_imu[0] * 1000  # mg
                        self.imu_data['accel_y'] = raw_imu[1] * 1000  # mg
                        self.imu_data['accel_z'] = raw_imu[2] * 1000  # mg
                        
                        # Gyroscope: rad/s to degrees/s
                        self.imu_data['gyro_x'] = math.degrees(raw_imu[3])  # deg/s
                        self.imu_data['gyro_y'] = math.degrees(raw_imu[4])  # deg/s
                        self.imu_data['gyro_z'] = math.degrees(raw_imu[5])  # deg/s
                        
                        self.imu_data['timestamp'] = current_time
                        self.imu_data['connected'] = True
                        
                        # Calculate real orientation from TCP data
                        self.calculate_real_orientation_tcp()
                        
                        # Update sensor readings with large values
                        self.update_sensor_readings()
                        
                else:
                    with self.data_lock:
                        self.imu_data['connected'] = False
                        
            except Exception as e:
                with self.data_lock:
                    self.imu_data['connected'] = False
                    
        # Depth sensor verilerini güncelle
        if self.depth_sensor:
            try:
                depth_reading = self.depth_sensor.read_data()
                if depth_reading:
                    with self.data_lock:
                        self.depth_data.update(depth_reading)
                        self.depth_data['timestamp'] = current_time
                        self.depth_data['connected'] = True
            except Exception as e:
                with self.data_lock:
                    self.depth_data['connected'] = False
        
        # MAVLink'den depth verisi (backup)
        elif self.mavlink and self.mavlink.connected:
            try:
                mavlink_depth = self.mavlink.get_depth_data()
                if mavlink_depth:
                    with self.data_lock:
                        self.depth_data.update(mavlink_depth)
                        self.depth_data['timestamp'] = current_time
                        self.depth_data['connected'] = True
            except Exception as e:
                pass
    
    def calculate_real_orientation_tcp(self):
        """TCP MAVLink verilerinden gerçek roll/pitch/yaw hesapla - Büyük değerler"""
        try:
            dt = time.time() - self.last_imu_time if self.last_imu_time > 0 else 0.02
            self.last_imu_time = time.time()
            
            # Accelerometer'dan roll/pitch hesapla (milli-g values)
            accel_x_g = self.imu_data['accel_x'] / 1000.0  # mg to g
            accel_y_g = self.imu_data['accel_y'] / 1000.0  # mg to g
            accel_z_g = self.imu_data['accel_z'] / 1000.0  # mg to g
            
            # Roll ve Pitch hesaplama (degrees)
            if abs(accel_z_g) > 0.001:  # Prevent division by zero
                roll_accel = math.atan2(accel_y_g, accel_z_g)
                pitch_accel = math.atan2(-accel_x_g, math.sqrt(accel_y_g*accel_y_g + accel_z_g*accel_z_g))
                
                roll_accel = math.degrees(roll_accel)
                pitch_accel = math.degrees(pitch_accel)
            else:
                roll_accel = 0
                pitch_accel = 0
            
            # Gyroscope integration (already in deg/s)
            gyro_x_dps = self.imu_data['gyro_x']  # deg/s
            gyro_y_dps = self.imu_data['gyro_y']  # deg/s
            gyro_z_dps = self.imu_data['gyro_z']  # deg/s
            
            # Simple complementary filter (95% gyro, 5% accel for more stability)
            alpha = 0.95
            
            self.integrated_angles['roll'] = alpha * (self.integrated_angles['roll'] + gyro_x_dps * dt) + (1 - alpha) * roll_accel
            self.integrated_angles['pitch'] = alpha * (self.integrated_angles['pitch'] + gyro_y_dps * dt) + (1 - alpha) * pitch_accel
            self.integrated_angles['yaw'] += gyro_z_dps * dt  # Pure integration for yaw
            
            # Yaw normalization (-180 to +180)
            while self.integrated_angles['yaw'] > 180:
                self.integrated_angles['yaw'] -= 360
            while self.integrated_angles['yaw'] < -180:
                self.integrated_angles['yaw'] += 360
            
            # Update IMU data with calculated values
            self.imu_data['roll'] = self.integrated_angles['roll']
            self.imu_data['pitch'] = self.integrated_angles['pitch'] 
            self.imu_data['yaw'] = self.integrated_angles['yaw']
            
        except Exception as e:
            # Use raw gyro integration if accelerometer fails
            dt = time.time() - self.last_imu_time if self.last_imu_time > 0 else 0.02
            self.integrated_angles['roll'] += self.imu_data['gyro_x'] * dt
            self.integrated_angles['pitch'] += self.imu_data['gyro_y'] * dt
            self.integrated_angles['yaw'] += self.imu_data['gyro_z'] * dt
            
            self.imu_data['roll'] = self.integrated_angles['roll']
            self.imu_data['pitch'] = self.integrated_angles['pitch']
            self.imu_data['yaw'] = self.integrated_angles['yaw']
    
    def update_sensor_readings(self):
        """Sensor readings'i büyük değerlerle güncelle"""
        try:
            # Copy current values (already in large format)
            self.sensor_readings['roll_deg'] = self.imu_data['roll']
            self.sensor_readings['pitch_deg'] = self.imu_data['pitch']
            self.sensor_readings['yaw_deg'] = self.imu_data['yaw']
            
            # Acceleration in milli-g (1000mg = 1g)
            self.sensor_readings['accel_x_mg'] = self.imu_data['accel_x']
            self.sensor_readings['accel_y_mg'] = self.imu_data['accel_y']
            self.sensor_readings['accel_z_mg'] = self.imu_data['accel_z']
            
            # Gyroscope in degrees per second
            self.sensor_readings['gyro_x_dps'] = self.imu_data['gyro_x']
            self.sensor_readings['gyro_y_dps'] = self.imu_data['gyro_y']
            self.sensor_readings['gyro_z_dps'] = self.imu_data['gyro_z']
            
            # Update counter
            self.sensor_readings['update_count'] += 1
            
        except Exception as e:
            pass
    
    def init_curses(self, stdscr):
        """Curses arayüzünü başlat"""
        self.stdscr = stdscr
        curses.curs_set(0)  # Cursor gizle
        curses.noecho()     # Echo kapat
        curses.cbreak()     # Karakterleri anında al
        stdscr.keypad(True) # Özel tuşları etkinleştir
        stdscr.nodelay(True) # Non-blocking input
        
        # Timeout optimize - 30Hz UI update
        stdscr.timeout(33)   # 33ms = ~30Hz
        
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
        
        self.log(f"🖥️ Terminal boyutu: {self.width}x{self.height}")
    
    def draw_header(self):
        """Başlık çiz"""
        title = "🚀 TEKNOFEST Su Altı ROV - Real-Time Terminal 🚀"
        self.stdscr.addstr(0, (self.width - len(title)) // 2, title, curses.color_pair(4) | curses.A_BOLD)
        
        # Durum bilgisi
        status_line = 1
        mavlink_status = "✅ Bağlı" if self.mavlink and self.mavlink.connected else "❌ Bağlı Değil"
        arm_status = "🔴 ARMED" if self.armed else "🟢 DISARMED"
        depth_status = "✅ I2C" if self.depth_data['connected'] else "❌ Bağlı Değil"
        
        self.stdscr.addstr(status_line, 2, f"MAVLink: {mavlink_status}", curses.color_pair(1 if self.mavlink and self.mavlink.connected else 2))
        self.stdscr.addstr(status_line, 25, f"Durum: {arm_status}", curses.color_pair(2 if self.armed else 1))
        self.stdscr.addstr(status_line, 45, f"Kontrol: {self.control_mode}", curses.color_pair(5))
        self.stdscr.addstr(status_line, 60, f"Navigation: {self.navigation_mode}", curses.color_pair(5))
        self.stdscr.addstr(status_line, 80, f"Depth: {depth_status}", curses.color_pair(1 if self.depth_data['connected'] else 2))
        
        # Çizgi
        self.stdscr.addstr(2, 0, "─" * self.width, curses.color_pair(4))
    
    def draw_real_sensor_data(self):
        """Gerçek sensör verilerini çiz - TCP MAVLink büyük değerler"""
        start_row = 4
        
        with self.data_lock:
            # TCP MAVLink IMU verileri - Büyük değerler
            self.stdscr.addstr(start_row, 2, "📊 TCP MAVLINK IMU VERİLERİ:", curses.color_pair(4) | curses.A_BOLD)
            
            # Connection status
            if self.imu_data.get('connected', False):
                conn_status = "✅ TCP BAĞLI"
                conn_color = curses.color_pair(1)
            else:
                conn_status = "❌ TCP BAĞLI DEĞİL"
                conn_color = curses.color_pair(2)
            
            self.stdscr.addstr(start_row, 35, conn_status, conn_color)
            
            # Orientation - Large values in degrees
            self.stdscr.addstr(start_row + 1, 4, f"Roll:  {self.sensor_readings['roll_deg']:+8.1f}°", curses.color_pair(1))
            self.stdscr.addstr(start_row + 2, 4, f"Pitch: {self.sensor_readings['pitch_deg']:+8.1f}°", curses.color_pair(1))
            self.stdscr.addstr(start_row + 3, 4, f"Yaw:   {self.sensor_readings['yaw_deg']:+8.1f}°", curses.color_pair(1))
            
            # Acceleration - Large values in milli-g (1000mg = 1g)
            self.stdscr.addstr(start_row + 1, 25, f"AccX: {self.sensor_readings['accel_x_mg']:+7.0f}mg", curses.color_pair(6))
            self.stdscr.addstr(start_row + 2, 25, f"AccY: {self.sensor_readings['accel_y_mg']:+7.0f}mg", curses.color_pair(6))
            self.stdscr.addstr(start_row + 3, 25, f"AccZ: {self.sensor_readings['accel_z_mg']:+7.0f}mg", curses.color_pair(6))
            
            # Gyroscope - Large values in degrees per second
            self.stdscr.addstr(start_row + 1, 45, f"GyroX: {self.sensor_readings['gyro_x_dps']:+6.1f}°/s", curses.color_pair(5))
            self.stdscr.addstr(start_row + 2, 45, f"GyroY: {self.sensor_readings['gyro_y_dps']:+6.1f}°/s", curses.color_pair(5))
            self.stdscr.addstr(start_row + 3, 45, f"GyroZ: {self.sensor_readings['gyro_z_dps']:+6.1f}°/s", curses.color_pair(5))
            
            # Update counter
            self.stdscr.addstr(start_row + 4, 4, f"Updates: {self.sensor_readings['update_count']:>6}", curses.color_pair(3))
            
            # Data freshness
            if self.imu_data.get('timestamp', 0) > 0:
                data_age = time.time() - self.imu_data['timestamp']
                freshness_color = curses.color_pair(1) if data_age < 0.1 else curses.color_pair(3) if data_age < 1 else curses.color_pair(2)
                self.stdscr.addstr(start_row + 4, 20, f"Age: {data_age:.2f}s", freshness_color)
        
        # I2C Depth verilerini göster
        self.stdscr.addstr(start_row, 70, "🌊 I2C DERİNLİK (0x76):", curses.color_pair(4) | curses.A_BOLD)
        if self.depth_data['connected']:
            self.stdscr.addstr(start_row + 1, 72, f"Derinlik:  {self.depth_data['depth_m']:6.2f}m", curses.color_pair(1))
            self.stdscr.addstr(start_row + 2, 72, f"Sıcaklık:  {self.depth_data['temperature_c']:6.1f}°C", curses.color_pair(1))
            self.stdscr.addstr(start_row + 3, 72, f"Basınç:    {self.depth_data['pressure_mbar']:6.1f}mb", curses.color_pair(1))
            
            if self.depth_data.get('timestamp', 0) > 0:
                data_age = time.time() - self.depth_data['timestamp']
                age_color = curses.color_pair(1) if data_age < 1 else curses.color_pair(3)
                self.stdscr.addstr(start_row + 4, 72, f"Age: {data_age:.1f}s", age_color)
        else:
            self.stdscr.addstr(start_row + 1, 72, "❌ I2C Bağlantı Yok", curses.color_pair(2))
            self.stdscr.addstr(start_row + 2, 72, "💡 0x76 adres kontrol", curses.color_pair(3))
        
        # Servo kontrol durumu - Sağ alt
        self.stdscr.addstr(start_row + 6, 4, "🎮 SERVO KONTROL DURUMU:", curses.color_pair(4) | curses.A_BOLD)
        self.stdscr.addstr(start_row + 7, 6, f"Roll:  {self.servo_values['roll']:+4.0f}° [A/D]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 8, 6, f"Pitch: {self.servo_values['pitch']:+4.0f}° [W/S]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 9, 6, f"Yaw:   {self.servo_values['yaw']:+4.0f}° [Q/E]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 10, 6, f"Motor: {self.motor_value:+4.0f}% [O/L]", curses.color_pair(5))
        
        # Movement status
        if self.movement_active:
            self.stdscr.addstr(start_row + 6, 45, "🎯 HAREKET AKTİF", curses.color_pair(3) | curses.A_BOLD)
            self.stdscr.addstr(start_row + 7, 45, f"Target X: {self.movement_target['x']:+5.1f}m", curses.color_pair(3))
            self.stdscr.addstr(start_row + 8, 45, f"Target Y: {self.movement_target['y']:+5.1f}m", curses.color_pair(3))
            self.stdscr.addstr(start_row + 9, 45, f"Target Z: {self.movement_target['z']:+5.1f}m", curses.color_pair(3))
        else:
            self.stdscr.addstr(start_row + 6, 45, "⭕ HAREKET BEKLEMİDE", curses.color_pair(6))
            self.stdscr.addstr(start_row + 7, 45, "M tuşu: Hareket menü", curses.color_pair(6))
    
    def draw_movement_menu(self):
        """Hareket menüsünü çiz"""
        menu_row = 17  # Updated position
        
        self.stdscr.addstr(menu_row, 2, "🎯 HAREKET KOMUTLARI:", curses.color_pair(4) | curses.A_BOLD)
        
        menu_items = [
            "1: Sağa Git (metre gir)",    "2: Sola Git (metre gir)",    "3: Yukarı Git (metre gir)",
            "4: Aşağı Git (metre gir)",   "5: İleri Git (metre gir)",   "6: Geri Git (metre gir)",
            "7: Saat Yönü Dön (derece)", "8: Ters Saat Dön (derece)", "9: Pozisyon Sıfırla",
            "0: Hareketi Durdur",         "",                           ""
        ]
        
        row = menu_row + 1
        col = 4
        for i, item in enumerate(menu_items):
            if i % 3 == 0 and i > 0:
                row += 1
                col = 4
            if item:  # Boş string değilse
                self.stdscr.addstr(row, col, item[:25], curses.color_pair(3))
            col += 30
    
    def draw_controls(self):
        """Kontrol bilgilerini çiz"""
        cmd_row = 22  # Updated position
        
        self.stdscr.addstr(cmd_row, 2, "⌨️  MANUEL KONTROL:", curses.color_pair(4) | curses.A_BOLD)
        
        commands = [
            "W/S: Pitch ±",       "A/D: Roll ±",        "Q/E: Yaw ±",
            "O/L: Motor ±",       "PgUp/PgDn: Güçlü Motor", "Space: ARM/DISARM",
            "R/F: RAW/PID",       "M: Hareket Menü",    "T: Test Scripts",
            "C: Pin Config",      "V: Vibration",       "G: GPS Data",
            "ESC/P: Çıkış",       "",                   ""
        ]
        
        row = cmd_row + 1
        col = 4
        for i, cmd in enumerate(commands):
            if i % 3 == 0 and i > 0:
                row += 1
                col = 4
            if cmd:
                self.stdscr.addstr(row, col, cmd[:18], curses.color_pair(6))
            col += 22
    
    def draw_logs(self):
        """Log mesajlarını çiz - optimize edilmiş"""
        if self.height < 25:
            return
            
        log_start = self.height - 8
        max_log_display = 6
        
        self.stdscr.addstr(log_start - 1, 2, "📝 SON LOG MESAJLARI:", curses.color_pair(4) | curses.A_BOLD)
        
        with self.data_lock:
            # Son mesajları göster
            recent_logs = list(self.log_messages)[-max_log_display:]
        
        for i, message in enumerate(recent_logs):
            if log_start + i < self.height - 1:
                # Renk seçimi
                color = curses.color_pair(6)  # Varsayılan beyaz
                if "❌" in message:
                    color = curses.color_pair(2)  # Kırmızı
                elif "⚠️" in message:
                    color = curses.color_pair(3)  # Sarı
                elif "✅" in message:
                    color = curses.color_pair(1)  # Yeşil
                elif "🎮" in message or "🎯" in message:
                    color = curses.color_pair(5)  # Magenta
                
                # Mesajı kısalt ve göster
                display_message = message[:self.width - 4]
                try:
                    self.stdscr.addstr(log_start + i, 4, display_message, color)
                except:
                    pass
    

    
    def handle_keyboard(self):
        """Klavye girişini işle - optimize edilmiş"""
        key = self.stdscr.getch()
        
        # Tuş basılmadıysa
        if key == -1 or key == curses.ERR:
            return
        
        # Çıkış tuşları
        if key in [27, 3, ord('P'), ord('p')]:  # ESC, Ctrl+C, P/p
            self.running = False
            self.log("🔄 Çıkış komutu alındı...")
            return
        
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
        
        # Motor kontrol - optimize edilmiş
        elif key == curses.KEY_PPAGE:  # Page Up
            self.motor_value = min(100, self.motor_value + 10)
            self.send_motor_command()
        elif key == curses.KEY_NPAGE:  # Page Down
            self.motor_value = max(-100, self.motor_value - 10)
            self.send_motor_command()
        elif key == ord('o') or key == ord('O'):
            self.motor_value = min(100, self.motor_value + 5)
            self.send_motor_command()
        elif key == ord('l') or key == ord('L'):
            self.motor_value = max(-100, self.motor_value - 5)
            self.send_motor_command()
        
        # Movement menu
        elif key == ord('m') or key == ord('M'):
            self.show_movement_menu()
        
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
        
        # Test ve konfigürasyon
        elif key == ord('t'):
            self.show_test_menu()
        elif key == ord('c'):
            self.show_pin_config()
        elif key == ord('v'):
            self.show_vibration_window()
        elif key == ord('g'):
            self.show_gps_window()
    
    def show_movement_menu(self):
        """Hareket menüsünü göster ve işle"""
        menu_window = curses.newwin(20, 80, 5, 10)
        menu_window.box()
        menu_window.addstr(1, 2, "🎯 HAREKET KOMUTLARI - METRE/DERECE GİRİN", curses.color_pair(4) | curses.A_BOLD)
        
        menu_items = [
            ("1", "Sağa Git (X+)", "metre"),
            ("2", "Sola Git (X-)", "metre"), 
            ("3", "Yukarı Git (Z+)", "metre"),
            ("4", "Aşağı Git (Z-)", "metre"),
            ("5", "İleri Git (Y+)", "metre"),
            ("6", "Geri Git (Y-)", "metre"),
            ("7", "Saat Yönü Dön", "derece"),
            ("8", "Ters Saat Dön", "derece"),
            ("9", "Pozisyon Sıfırla", ""),
            ("0", "Hareketi Durdur", ""),
            ("ESC", "Geri", "")
        ]
        
        for i, (key, desc, unit) in enumerate(menu_items):
            menu_window.addstr(3 + i, 4, f"{key}: {desc} {unit}")
        
        menu_window.refresh()
        
        while True:
            key = menu_window.getch()
            
            if key == 27:  # ESC
                break
            elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6')]:
                distance = self.get_distance_input(menu_window, "Metre girin (0.1-10.0): ")
                if distance is not None:
                    self.execute_movement(chr(key), distance)
                break
            elif key in [ord('7'), ord('8')]:
                angle = self.get_distance_input(menu_window, "Derece girin (1-360): ")
                if angle is not None:
                    self.execute_movement(chr(key), angle)
                break
            elif key == ord('9'):
                self.reset_position()
                break
            elif key == ord('0'):
                self.stop_movement()
                break
        
        menu_window.clear()
        menu_window.refresh()
        del menu_window
    
    def get_distance_input(self, parent_window, prompt):
        """Mesafe/açı girişi al"""
        input_window = curses.newwin(5, 50, 12, 20)
        input_window.box()
        input_window.addstr(1, 2, prompt, curses.color_pair(4))
        input_window.addstr(3, 2, "Enter: Uygula, ESC: İptal")
        
        curses.echo()
        curses.curs_set(1)
        input_window.refresh()
        
        try:
            user_input = input_window.getstr(1, len(prompt) + 2, 10).decode('utf-8')
            
            if user_input:
                value = float(user_input)
                if 0 < value <= 100:  # Maksimum limit
                    return value
                else:
                    self.log(f"❌ Geçersiz değer: {value} (0-100 arası olmalı)")
                    return None
            return None
            
        except ValueError:
            self.log(f"❌ Geçersiz sayı: {user_input}")
            return None
        except Exception as e:
            self.log(f"❌ Giriş hatası: {e}")
            return None
        finally:
            curses.noecho()
            curses.curs_set(0)
            input_window.clear()
            input_window.refresh()
            del input_window
    
    def execute_movement(self, command, value):
        """Hareket komutunu çalıştır"""
        movement_map = {
            '1': ('x', value, "Sağa"),
            '2': ('x', -value, "Sola"),
            '3': ('z', value, "Yukarı"),
            '4': ('z', -value, "Aşağı"),
            '5': ('y', value, "İleri"),
            '6': ('y', -value, "Geri"),
            '7': ('yaw', value, "Saat Yönü"),
            '8': ('yaw', -value, "Ters Saat")
        }
        
        if command in movement_map:
            axis, target_value, direction = movement_map[command]
            
            if axis in ['x', 'y', 'z']:
                self.movement_target[axis] = target_value
                self.log(f"🎯 {direction} hareket: {abs(target_value):.1f}m")
            else:  # yaw
                self.log(f"🎯 {direction} dönüş: {abs(target_value):.0f}°")
                self.rotate_yaw(target_value)
            
            self.movement_active = True
            
            # Hareket thread'ini başlat
            movement_thread = threading.Thread(target=self.movement_control_loop, daemon=True)
            movement_thread.start()
    
    def rotate_yaw(self, angle):
        """Yaw dönüşü gerçekleştir"""
        if not self.mavlink or not self.mavlink.connected or not self.armed:
            self.log("❌ Dönüş için ARM gerekli!")
            return
            
        try:
            # Basit yaw kontrolü
            direction = 1 if angle > 0 else -1
            duration = abs(angle) / 90  # 90°/saniye hızında
            
            # Yaw servo kontrolü
            for i in range(int(duration * 10)):  # 10Hz
                self.servo_values['yaw'] = direction * 30  # ±30° servo
                self.send_servo_commands()
                time.sleep(0.1)
            
            # Servo'yu nötr pozisyona getir
            self.servo_values['yaw'] = 0
            self.send_servo_commands()
            
            self.log(f"✅ Yaw dönüşü tamamlandı: {angle:.0f}°")
            
        except Exception as e:
            self.log(f"❌ Yaw dönüş hatası: {e}")
    
    def movement_control_loop(self):
        """Hareket kontrol döngüsü"""
        try:
            while self.movement_active and self.running:
                # Basit hareket kontrolü - bu kısım navigation engine ile entegre edilmeli
                # Şimdilik motor ve servo kontrolü ile basit hareket
                
                target_reached = True
                
                # X ekseni kontrolü (sağa/sola)
                if abs(self.movement_target['x']) > 0.1:
                    direction = 1 if self.movement_target['x'] > 0 else -1
                    self.servo_values['roll'] = direction * 20  # Roll servo
                    target_reached = False
                
                # Y ekseni kontrolü (ileri/geri)
                if abs(self.movement_target['y']) > 0.1:
                    direction = 1 if self.movement_target['y'] > 0 else -1
                    self.motor_value = direction * 30  # Motor kontrolü
                    target_reached = False
                
                # Z ekseni kontrolü (yukarı/aşağı)
                if abs(self.movement_target['z']) > 0.1:
                    direction = 1 if self.movement_target['z'] > 0 else -1
                    self.servo_values['pitch'] = direction * 20  # Pitch servo
                    target_reached = False
                
                # Komutları gönder
                if self.armed:
                    self.send_servo_commands()
                    self.send_motor_command()
                
                # Hedef mesafeye ulaştıysa dur
                if target_reached:
                    self.stop_movement()
                    break
                
                # Güncelleme döngüsü
                time.sleep(0.1)  # 10Hz
                
                # Hareket süresini sınırla (güvenlik)
                if hasattr(self, 'movement_start_time'):
                    if time.time() - self.movement_start_time > 30:  # 30 saniye max
                        self.log("⏰ Hareket zaman aşımı!")
                        self.stop_movement()
                        break
                else:
                    self.movement_start_time = time.time()
                    
        except Exception as e:
            self.log(f"❌ Hareket kontrol hatası: {e}")
            self.stop_movement()
    
    def stop_movement(self):
        """Hareketi durdur"""
        self.movement_active = False
        self.movement_target = {'x': 0, 'y': 0, 'z': 0}
        
        # Servolar nötr pozisyona
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0
        
        if self.armed:
            self.send_servo_commands()
            self.send_motor_command()
        
        self.log("🛑 Hareket durduruldu!")
    
    def reset_position(self):
        """Pozisyonu sıfırla"""
        self.current_position = {'x': 0, 'y': 0, 'z': 0}
        self.movement_target = {'x': 0, 'y': 0, 'z': 0}
        self.integrated_angles = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.log("🔄 Pozisyon sıfırlandı!")
    
    def update_servo_control(self):
        """Real-time servo kontrolünü güncelle - optimize edilmiş"""
        # Pitch kontrol
        if 'w' in self.active_keys:
            self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + 3)
        elif 's' in self.active_keys:
            self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - 3)
        else:
            # Otomatik sıfırlama - hızlı
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
            pwm_value = 1500 + (self.motor_value * 4)
            self.mavlink.send_raw_motor_pwm(int(pwm_value))
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
                    self.stop_movement()  # Hareketi durdur
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
        
        config_window.addstr(3, 4, f"Mevut I2C Ayarları:")
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
    
    def update_imu_history(self):
        """IMU verilerini history'ye ekle"""
        try:
            self.sensor_readings['roll_deg'] = self.imu_data['roll']
            self.sensor_readings['pitch_deg'] = self.imu_data['pitch']
            self.sensor_readings['yaw_deg'] = self.imu_data['yaw']
            self.sensor_readings['accel_x_mg'] = self.imu_data['accel_x']
            self.sensor_readings['accel_y_mg'] = self.imu_data['accel_y']
            self.sensor_readings['accel_z_mg'] = self.imu_data['accel_z']
            self.sensor_readings['gyro_x_dps'] = self.imu_data['gyro_x']
            self.sensor_readings['gyro_y_dps'] = self.imu_data['gyro_y']
            self.sensor_readings['gyro_z_dps'] = self.imu_data['gyro_z']
            self.sensor_readings['update_count'] = time.time()
        except Exception as e:
            pass
    

    
    def draw_tcp_data_status(self):
        """TCP veri durumu özeti - Basit ve etkili"""
        if self.height < 30:
            return
            
        status_row = 27
        
        try:
            with self.data_lock:
                self.stdscr.addstr(status_row, 2, "📡 TCP MAVLink Veri Durumu:", curses.color_pair(4) | curses.A_BOLD)
                
                # IMU data status
                if self.imu_data.get('connected', False):
                    imu_status = f"✅ IMU: {self.sensor_readings['update_count']:>4} updates"
                    imu_color = curses.color_pair(1)
                else:
                    imu_status = "❌ IMU: Bağlantı yok"
                    imu_color = curses.color_pair(2)
                
                self.stdscr.addstr(status_row + 1, 4, imu_status, imu_color)
                
                # Show current significant values
                if self.imu_data.get('connected', False):
                    # Show largest current values for visibility
                    max_accel = max(abs(self.sensor_readings['accel_x_mg']), 
                                  abs(self.sensor_readings['accel_y_mg']), 
                                  abs(self.sensor_readings['accel_z_mg']))
                    max_gyro = max(abs(self.sensor_readings['gyro_x_dps']), 
                                 abs(self.sensor_readings['gyro_y_dps']), 
                                 abs(self.sensor_readings['gyro_z_dps']))
                    
                    self.stdscr.addstr(status_row + 1, 30, f"Max Accel: {max_accel:>5.0f}mg", curses.color_pair(6))
                    self.stdscr.addstr(status_row + 1, 50, f"Max Gyro: {max_gyro:>5.1f}°/s", curses.color_pair(5))
                    
                    # TCP connection quality
                    data_age = time.time() - self.imu_data.get('timestamp', 0)
                    if data_age < 0.1:
                        quality = "🟢 EXCELLENT"
                        quality_color = curses.color_pair(1)
                    elif data_age < 0.5:
                        quality = "🟡 GOOD"
                        quality_color = curses.color_pair(3)
                    else:
                        quality = "🔴 POOR"
                        quality_color = curses.color_pair(2)
                    
                    self.stdscr.addstr(status_row + 1, 70, f"Quality: {quality}", quality_color)
                
        except Exception as e:
            pass
    
    def main_loop(self):
        """Ana döngü - optimize edilmiş"""
        last_update = time.time()
        
        while self.running:
            try:
                current_time = time.time()
                
                # UI güncelleme 30Hz
                if current_time - last_update > 0.033:  # 30Hz = 33ms
                    self.stdscr.erase()
                    
                    # UI bileşenlerini çiz
                    self.draw_header() 
                    self.draw_real_sensor_data() # Gerçek sensör verilerini çiz
                    self.draw_movement_menu() # Hareket menüsünü çiz
                    self.draw_controls()
                    self.draw_logs()
                    self.draw_tcp_data_status()
                    
                    # Ekranı yenile
                    self.stdscr.refresh()
                    last_update = current_time
                
                # Klavye girişini kontrol et
                self.handle_keyboard()
                
                # Real-time servo kontrolü (20Hz)
                if current_time - getattr(self, 'last_servo_update', 0) > 0.05:
                    self.update_servo_control()
                    self.last_servo_update = current_time
                
                # CPU eficiancy için minimal sleep
                time.sleep(0.001)  # 1ms - daha responsive
                
            except KeyboardInterrupt:
                self.running = False
            except Exception as e:
                self.log(f"❌ Ana döngü hatası: {e}")
    
    def cleanup(self):
        """Temizlik işlemleri - optimize edilmiş"""
        self.log("🔄 Sistem kapatılıyor...")
        
        # Data thread'ini durdur
        self.data_thread_running = False
        if self.data_thread and self.data_thread.is_alive():
            self.data_thread.join(timeout=2)
        
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
    print("🚀 TEKNOFEST Su Altı ROV - Real-Time Terminal GUI başlatılıyor...")
    
    # Çalışma dizinini kontrol et
    if not os.path.exists("config"):
        print("❌ config/ klasörü bulunamadı! App/ klasörünün içinden çalıştırın.")
        sys.exit(1)
    
    # Terminal GUI'yi başlat
    try:
        gui = TerminalROVGUI()
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