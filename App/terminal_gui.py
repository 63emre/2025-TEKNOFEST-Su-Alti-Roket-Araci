#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Advanced Terminal GUI
Pixhawk PX4 PIX 2.4.8 Serial MAVLink Control System
Real-time IMU, Depth Sensor, GPIO Integration
"""

import os
import sys
import curses
import time
import json
import math
import threading
import signal
import traceback
from concurrent.futures import ThreadPoolExecutor, TimeoutError

# Import attempt for MAVLink - debug mode for import issues
try:
    from mavlink_handler import MAVLinkHandler
    MAVLINK_AVAILABLE = True
    print("✅ MAVLink handler loaded successfully")
except ImportError as e:
    print(f"❌ MAVLink handler import error: {e}")
    MAVLINK_AVAILABLE = False

# GPIO Controller import
try:
    from gpio_controller import GPIOController
    GPIO_AVAILABLE = True
    print("✅ GPIO controller loaded successfully")
except ImportError as e:
    print(f"⚠️ GPIO controller import error: {e}")
    print("💡 GPIO functions will be disabled in simulation mode")
    GPIO_AVAILABLE = False

# Depth sensor import
try:
    from depth_sensor import D300DepthSensor
    DEPTH_SENSOR_AVAILABLE = True
    print("✅ Depth sensor loaded successfully")
except ImportError as e:
    print(f"⚠️ Depth sensor import error: {e}")
    print("💡 Depth sensor will use MAVLink data")
    DEPTH_SENSOR_AVAILABLE = False

# Navigation engine import
try:
    from navigation_engine import NavigationEngine
    NAVIGATION_AVAILABLE = True
    print("✅ Navigation engine loaded successfully")
except ImportError as e:
    print(f"⚠️ Navigation engine import error: {e}")
    NAVIGATION_AVAILABLE = False

class GPIOIntegration:
    """GPIO ve harici sensör entegrasyonu"""
    
    def __init__(self, config):
        """GPIO integration başlat"""
        self.config = config
        
        if GPIO_AVAILABLE:
            self.gpio = GPIOController(config)
            self.gpio_initialized = self.gpio.initialize()
        else:
            self.gpio = None
            self.gpio_initialized = False
        
        # Depth sensor
        if DEPTH_SENSOR_AVAILABLE:
            try:
                # Önce I2C dene, başarısız olursa simulation
                self.depth_sensor = D300DepthSensor(simulation_mode=False)
                if not self.depth_sensor.connect():
                    # I2C başarısız, simulation'a geç
                    self.depth_sensor = D300DepthSensor(simulation_mode=True)
                    self.depth_sensor.connect()
                    print("🎮 D300 depth sensor simülasyon modunda")
                self.depth_sensor_available = True
            except Exception as e:
                print(f"⚠️ Depth sensor başlatma hatası: {e}")
                self.depth_sensor = None
                self.depth_sensor_available = False
        else:
            self.depth_sensor = None
            self.depth_sensor_available = False
        
        # Emergency callback
        self.emergency_callback = None
        
        if self.gpio_initialized:
            print("✅ GPIO integration başarıyla başlatıldı")
        else:
            print("⚠️ GPIO simülasyon modunda çalışıyor")
    
    def set_emergency_callback(self, callback):
        """Acil durum callback ayarla"""
        self.emergency_callback = callback
    
    def emergency_button_pressed(self):
        """Acil durum butonu basıldığında çağrılan fonksiyon"""
        if self.emergency_callback:
            try:
                self.emergency_callback()
            except Exception as e:
                print(f"❌ Emergency callback error: {e}")
        
        # LED ve buzzer ile acil durum sinyali
        if self.gpio_initialized:
            self.gpio.emergency_led_pattern()
            self.gpio.buzzer_beep(2000, 1.0, 80)
    
    def set_system_status_led(self, status):
        """Sistem durumu LED'i"""
        if self.gpio_initialized:
            self.gpio.status_led_pattern(status)
    
    def set_connection_status_led(self, connected, armed=False):
        """Bağlantı durumu LED göstergesi"""
        if not self.gpio_initialized:
            return
        
        if connected:
            if armed:
                self.gpio.set_rgb_led(0, 100, 0)  # Yeşil - Armed
            else:
                self.gpio.set_rgb_led(0, 0, 100)  # Mavi - Connected
        else:
            self.gpio.set_rgb_led(100, 0, 0)  # Kırmızı - Disconnected
    
    def beep_success(self):
        """Başarı sesi"""
        if self.gpio_initialized:
            self.gpio.buzzer_beep(1500, 0.2, 50)
    
    def beep_error(self):
        """Hata sesi"""
        if self.gpio_initialized:
            self.gpio.buzzer_beep(800, 0.5, 70)
    
    def beep_warning(self):
        """Uyarı sesi"""
        if self.gpio_initialized:
            self.gpio.buzzer_beep(1200, 0.3, 60)
    
    def read_button(self):
        """Kontrol butonunu oku"""
        if self.gpio_initialized:
            return self.gpio.read_button()
        return False
    
    def get_depth_data(self):
        """Depth sensor verilerini al"""
        if self.depth_sensor_available and self.depth_sensor:
            return self.depth_sensor.get_sensor_data()
        return None
    
    def cleanup(self):
        """GPIO temizliği"""
        if self.gpio_initialized:
            self.gpio.cleanup()
        if self.depth_sensor_available and self.depth_sensor:
            self.depth_sensor.disconnect()

class MissionPlanner:
    """Basit mission planning sistemi"""
    
    def __init__(self):
        """Mission planner başlat"""
        self.missions = []
        self.current_mission = None
        self.mission_thread = None
        self.mission_active = False
        
        # Mission türleri
        self.mission_types = {
            'F': ('İleri Git', 'distance'),
            'B': ('Geri Git', 'distance'),
            'L': ('Sol Git', 'distance'),
            'R': ('Sağ Git', 'distance'),
            'U': ('Yukarı', 'distance'),
            'D': ('Aşağı', 'distance'),
            'Y': ('Yaw Dön', 'angle'),
            'W': ('Bekle', 'time'),
            'S': ('Yüzeye Çık', 'none')
        }
    
    def add_mission(self, command, value):
        """Mission ekle"""
        if command in self.mission_types:
            mission_name, param_type = self.mission_types[command]
            self.missions.append({
                'command': command,
                'name': mission_name,
                'value': value,
                'param_type': param_type
            })
            return True
        return False
    
    def clear_missions(self):
        """Tüm mission'ları temizle"""
        self.missions = []
        self.mission_active = False
    
    def get_mission_summary(self):
        """Mission özetini döndür"""
        return [f"{m['name']}: {m['value']}" for m in self.missions]
    
    def start_mission(self):
        """Mission'ları başlat"""
        if self.missions and not self.mission_active:
            self.mission_active = True
            return True
        return False

class TestScriptManager:
    """Test script yönetim sistemi"""
    
    def __init__(self):
        """Test script manager başlat"""
        self.scripts = {
            '1': 'Test/test_servo_control.py',
            '2': 'Test/test_motor_control.py',
            '3': 'Test/test_aux1_servo.py',
            '4': 'Test/test_aux3_servo.py',
            '5': 'Test/test_aux4_servo.py',
            '6': 'Test/test_aux5_servo.py',
            '7': 'Test/test_all_aux_servos.py',
            '8': 'scripts/motor_test.py',
            '9': 'scripts/emergency_stop.py'
        }
        
        self.running_script = None
        self.script_thread = None
    
    def get_script_list(self):
        """Script listesini döndür"""
        return [(k, v) for k, v in self.scripts.items()]
    
    def run_script(self, script_id, callback=None):
        """Script'i çalıştır"""
        if script_id in self.scripts and not self.running_script:
            script_path = self.scripts[script_id]
            self.running_script = script_id
            
            def execute_script():
                try:
                    import subprocess
                    result = subprocess.run([sys.executable, script_path], 
                                          capture_output=True, text=True, timeout=30)
                    success = result.returncode == 0
                    if callback:
                        callback(script_id, success, result.stdout, result.stderr)
                except Exception as e:
                    if callback:
                        callback(script_id, False, "", str(e))
                finally:
                    self.running_script = None
            
            self.script_thread = threading.Thread(target=execute_script, daemon=True)
            self.script_thread.start()
            return True
        return False

class AdvancedTerminalGUI:
    """Terminal GUI ana sınıfı"""
    def __init__(self):
        """Terminal GUI başlat"""
        # Environment variable support for serial
        self.serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
        self.baud_rate = int(os.getenv("MAV_BAUD", "115200"))
        
        print(f"🔧 Terminal GUI Serial Configuration:")
        print(f"   Port: {self.serial_port}")
        print(f"   Baud: {self.baud_rate}")
        
        # Config yükleme
        self.config = self.load_config()
        
        # MAVLink handler
        if MAVLINK_AVAILABLE:
            self.mavlink = MAVLinkHandler()
            self.mavlink_available = True
        else:
            self.mavlink = None
            self.mavlink_available = False
        
        # GPIO integration
        self.gpio_integration = GPIOIntegration(self.config)
        
        # Navigation engine
        if NAVIGATION_AVAILABLE and self.mavlink:
            self.navigation = NavigationEngine(self.mavlink)
            self.navigation_available = True
        else:
            self.navigation = None
            self.navigation_available = False
        
        # Mission planner
        self.mission_planner = MissionPlanner()
        
        # Test script manager
        self.test_scripts = TestScriptManager()
        
        # GUI state
        self.current_menu = "main"  # main, gpio_test, mission_plan, test_scripts
        self.menu_selection = 0
        
        # Control values
        self.motor_power = 0.0
        self.servo_roll = 0.0
        self.servo_pitch = 0.0
        self.servo_yaw = 0.0
        
        # System data
        self.imu_data = None
        self.depth_data = None
        self.tcp_data = {}
        self.gpio_data = {}
        
        # Logging
        self.log_messages = []
        self.max_logs = 100
        
        # Threading
        self.tcp_thread = None
        self.tcp_running = False
        self.data_lock = threading.Lock()
        
        # GUI settings
        self.show_imu_details = True
        self.show_depth_details = True
        self.show_raw_data = False
        
        # Emergency callback setup
        self.gpio_integration.set_emergency_callback(self.emergency_stop_callback)
        
        print("✅ Terminal GUI initialization complete")
    
    def load_config(self):
        """Config dosyasını yükle"""
        config_path = "config/hardware_config.json"
        
        # Default config with serial connection
        default_config = {
            "pixhawk": {
                "servo_channels": {
                    "fin_front_left": 1,
                    "fin_front_right": 3,
                    "fin_rear_left": 4,
                    "fin_rear_right": 5,
                    "main_motor": 6
                },
                "pwm_limits": {"min": 1000, "neutral": 1500, "max": 2000}
            },
            "raspberry_pi": {
                "gpio": {
                    "buzzer": 7, "control_button": 13,
                    "led_red": 4, "led_green": 5, "led_blue": 6,
                    "warning_led": 8, "system_status_led": 10
                },
                "i2c": {"bus_number": 1, "depth_sensor_address": "0x76"}
            },
            "mavlink": {"connection_string": self.serial_port, "baudrate": self.baud_rate},
            "network": {"web_gui_port": 5000, "log_level": "INFO"}
        }
        
        try:
            with open(config_path, 'r') as f:
                config = json.load(f)
            
            # Environment variables'dan serial ayarlarını güncelle
            if "mavlink" in config:
                config["mavlink"]["connection_string"] = self.serial_port
                config["mavlink"]["baudrate"] = self.baud_rate
            
            print("✅ Config başarıyla yüklendi")
            return config
            
        except FileNotFoundError:
            print("⚠️ Config dosyası bulunamadı, varsayılan config kullanılıyor")
            return default_config
        except json.JSONDecodeError as e:
            print(f"❌ Config dosyası parse hatası: {e}")
            return default_config
        except Exception as e:
            print(f"❌ Config yükleme hatası: {e}")
            return default_config
    
    def log(self, message):
        """Thread-safe log"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        with self.data_lock:
            self.log_messages.append(log_entry)
            # Max log limit kontrolü
            if len(self.log_messages) > self.max_logs:
                self.log_messages = self.log_messages[-self.max_logs:]
    
    def emergency_stop_callback(self):
        """Acil durum butonu callback"""
        self.log("🚨 ACİL DURUM BUTONU BASILDI - TÜM SİSTEMLER DURDURULUYOR!")
        
        # Tüm kontrolleri sıfırla
        self.servo_roll = 0.0
        self.servo_pitch = 0.0
        self.servo_yaw = 0.0
        self.motor_power = 0.0
        
        # MAVLink emergency stop
        if self.mavlink_available and self.mavlink and self.mavlink.connected:
            try:
                self.mavlink.emergency_stop()
                self.log("✅ MAVLink emergency stop gönderildi!")
            except Exception as e:
                self.log(f"❌ Emergency stop hatası: {e}")
        
        # Görevleri durdur
        self.mission_planner.mission_active = False
        
        # Buzzer pattern çal
        self.gpio_integration.beep_error()
        
        self.log("✅ Acil durum prosedürü tamamlandı!")
    
    def init_systems(self):
        """Sistem bileşenlerini başlat - Serial MAVLink"""
        self.log("🚀 TEKNOFEST ROV Terminal GUI başlatılıyor...")
        self.log(f"📡 Serial bağlantı: {self.serial_port} @ {self.baud_rate} baud")
        
        # GPIO sistem durumunu belirt
        if self.gpio_integration.gpio_initialized:
            self.log("✅ GPIO sistemleri hazır - Buzzer/LED/Button aktif!")
            self.gpio_integration.set_system_status_led('connecting')
        else:
            self.log("⚠️ GPIO sistemler devre dışı - sadece Serial kontrol")
        
        # D300 derinlik sensörü durumu
        if self.gpio_integration.depth_sensor_available:
            self.log("✅ D300 derinlik sensörü hazır - I2C/MAVLink data aktif!")
        else:
            self.log("⚠️ D300 derinlik sensörü - sadece MAVLink data")
        
        # Serial MAVLink bağlantısı
        if self.mavlink_available:
            try:
                self.log(f"📡 Serial MAVLink bağlantısı başlatılıyor...")
                self.log(f"🔧 Port: {self.serial_port}, Baud: {self.baud_rate}")
                
                connect_result = self.mavlink.connect()
                self.log(f"🔍 Serial bağlantı sonucu: {connect_result}")
                
                if connect_result:
                    self.log("✅ Serial MAVLink bağlantısı kuruldu!")
                    self.log(f"📊 Pixhawk System ID: {self.mavlink.master.target_system}")
                    
                    # System check
                    if self.mavlink.check_system_status():
                        arm_status = "ARMED" if self.mavlink.armed else "DISARMED"
                        self.log(f"🎯 Pixhawk durumu: {arm_status}")
                    
                    # Data stream başlat
                    self.start_serial_data_thread()
                    
                    # GPIO status update
                    if self.gpio_integration.gpio_initialized:
                        self.gpio_integration.set_connection_status_led(True, self.mavlink.armed)
                    
                else:
                    self.log("❌ Serial MAVLink bağlantısı başarısız!")
                    self.log(f"💡 Kontrol: {self.serial_port} portu, {self.baud_rate} baud rate")
                    self.log("💡 Pixhawk bağlı ve ArduSub çalışıyor mu?")
                    
            except Exception as e:
                self.log(f"❌ Serial bağlantı hatası: {e}")
                self.log("💡 Pixhawk serial bağlantısını kontrol edin")
        else:
            self.log("❌ MAVLink handler yüklenemedi - sadece GPIO aktif")
        
        # Final system check
        self.log("🔍 Sistem kontrolü...")
        if self.mavlink_available and self.mavlink and self.mavlink.connected:
            self.log("🎯 HAZIR: Serial bağlı, Pixhawk aktif, kontroller hazır!")
            if self.gpio_integration.gpio_initialized:
                self.log("🎯 GPIO: Buzzer/LED/Button aktif, acil durum butonu hazır!")
            if self.gpio_integration.depth_sensor_available:
                self.log("🎯 D300: I2C derinlik sensörü aktif!")
        else:
            self.log("⚠️ DİKKAT: Serial bağlantı yok - sadece GPIO/I2C test modu")
            
        self.log("✅ Sistem başlatma tamamlandı!")
    
    def start_serial_data_thread(self):
        """Serial veri thread'ini başlat"""
        self.tcp_running = True  # Variable adını koruyorum uyumluluk için
        self.tcp_thread = threading.Thread(target=self.serial_data_loop, daemon=True)
        self.tcp_thread.start()
        self.log("🔄 Serial veri thread'i başlatıldı (10Hz)")
    
    def serial_data_loop(self):
        """Serial veri döngüsü - IMU, GPS, Depth data"""
        self.log("📡 Serial data loop başlatıldı...")
        last_log_time = 0
        data_counter = 0
        
        while self.tcp_running and hasattr(self, 'running') and self.running:
            try:
                current_time = time.time()
                
                # MAVLink'den veri al
                if self.mavlink_available and self.mavlink and self.mavlink.connected:
                    # IMU data
                    imu_data = self.mavlink.get_imu_data()
                    if imu_data:
                        with self.data_lock:
                            self.imu_data = imu_data
                            self.tcp_data['connected'] = True
                            self.tcp_data['last_update'] = current_time
                        data_counter += 1
                    
                    # Depth data (MAVLink'den)
                    depth_data = self.mavlink.get_depth_data()
                    if depth_data:
                        with self.data_lock:
                            self.depth_data = depth_data
                            self.depth_data['connected'] = True
                    
                    # GPS data
                    gps_data = self.mavlink.get_gps_data()
                    if gps_data:
                        with self.data_lock:
                            self.tcp_data['gps'] = gps_data
                
                # GPIO/I2C verilerini güncelle
                self.update_gpio_data()
                
                # Calculate live orientation
                if self.imu_data:
                    self.calculate_live_orientation(self.imu_data)
                
                # Periodic logging (5 saniyede bir)
                if current_time - last_log_time > 5.0:
                    rate = data_counter / 5.0 if data_counter > 0 else 0
                    self.log(f"📊 Serial data rate: {rate:.1f} Hz, IMU packets: {data_counter}")
                    last_log_time = current_time
                    data_counter = 0
                
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                self.log(f"❌ Serial data loop error: {e}")
                time.sleep(0.5)
        
        self.log("📡 Serial data loop durduruldu")
    
    def update_gpio_data(self):
        """GPIO ve I2C veri güncelleme"""
        # D300 derinlik sensörü verisi güncelle (I2C'den direkt)
        if self.gpio_integration.depth_sensor_available:
            depth_data = self.gpio_integration.get_depth_data()
            if depth_data:
                with self.data_lock:
                    # I2C depth data'yı MAVLink ile birleştir
                    if not self.depth_data or not self.depth_data.get('connected', False):
                        self.depth_data = depth_data
                        self.depth_data['source'] = 'I2C'
        
        # GPIO buton durumu kontrol
        if self.gpio_integration.gpio_initialized:
            button_pressed = self.gpio_integration.read_button()
            if button_pressed:
                self.gpio_integration.emergency_button_pressed()
    
    def update_tcp_data(self):
        """TCP data update (backward compatibility için isim korundu)"""
        # Bu fonksiyon artık serial data'yı işliyor
        return self.serial_data_loop()
    
    def calculate_live_orientation(self, raw_imu):
        """Live roll/pitch/yaw hesapla"""
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
                roll_deg = getattr(self, 'last_roll', 0.0)
                pitch_deg = getattr(self, 'last_pitch', 0.0)
            
            # YAW integration
            dt = 0.1  # 10Hz
            gyro_z_deg = math.degrees(gyro_z)  # rad/s to deg/s
            
            # İlk kez çalışıyorsa YAW'ı 0'dan başlat
            if not hasattr(self, 'current_yaw'):
                self.current_yaw = 0.0
            
            # YAW güncelle
            yaw_change = gyro_z_deg * dt
            self.current_yaw += yaw_change
            
            # Yaw normalize (-180 to +180)
            while self.current_yaw > 180:
                self.current_yaw -= 360
            while self.current_yaw < -180:
                self.current_yaw += 360
            
            # Store values
            self.last_roll = roll_deg
            self.last_pitch = pitch_deg
            
            # Update live IMU structure for display compatibility
            if not hasattr(self, 'live_imu'):
                self.live_imu = {}
            
            self.live_imu.update({
                'roll': roll_deg,
                'pitch': pitch_deg,
                'yaw': self.current_yaw,
                'connected': True,
                'last_update': time.time(),
                'update_rate': 10
            })
            
        except Exception as e:
            self.log(f"❌ Orientation hesaplama hatası: {e}")
    
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
        title = f"🚀 TEKNOFEST ROV - Serial MAVLink Terminal [{self.serial_port}@{self.baud_rate}] 🚀"
        self.stdscr.addstr(0, max(0, (self.width - len(title)) // 2), title, curses.color_pair(4) | curses.A_BOLD)
        
        # Durum satırı - GPIO/I2C durumu eklendi
        status_line = 1
        serial_status = "✅ Serial BAĞLI" if self.tcp_data.get('connected', False) else "❌ Serial BAĞLI DEĞİL"
        armed_status = "🔴 ARMED" if (self.mavlink and self.mavlink.armed) else "🟢 DISARMED"
        gpio_status = "🔧 GPIO/I2C" if (self.gpio_integration.gpio_initialized or self.gpio_integration.depth_sensor_available) else "❌ NO GPIO"
        menu_status = f"📋 {self.current_menu.upper()}"
        
        self.stdscr.addstr(status_line, 2, f"Serial: {serial_status}", curses.color_pair(1 if self.tcp_data.get('connected', False) else 2))
        self.stdscr.addstr(status_line, 25, f"Durum: {armed_status}", curses.color_pair(2 if (self.mavlink and self.mavlink.armed) else 1))
        self.stdscr.addstr(status_line, 45, f"HW: {gpio_status}", curses.color_pair(1 if (self.gpio_integration.gpio_initialized or self.gpio_integration.depth_sensor_available) else 2))
        self.stdscr.addstr(status_line, 65, f"Kontrol: RAW", curses.color_pair(5))
        self.stdscr.addstr(status_line, 85, f"Menü: {menu_status}", curses.color_pair(4))
        
        # Ayırıcı çizgi
        self.stdscr.addstr(2, 0, "=" * self.width, curses.color_pair(6))
    
    def draw_live_imu_display(self):
        """Live IMU display - simplified"""
        start_row = 3
        self.stdscr.addstr(start_row, 4, "📊 LIVE IMU/ORIENTATION", curses.color_pair(4) | curses.A_BOLD)
        
        # Initialize live_imu if not exists
        if not hasattr(self, 'live_imu'):
            self.live_imu = {'connected': False, 'roll': 0, 'pitch': 0, 'yaw': 0, 'last_update': 0, 'update_rate': 0}
        
        # IMU verisi varsa göster
        if self.live_imu.get('connected', False):
            # Bağlantı durumu
            conn_status = "✅ Serial LIVE"
            conn_color = curses.color_pair(1)
            data_age = time.time() - self.live_imu.get('last_update', 0)
            
            if data_age > 2.0:
                conn_status = "⚠️ DATA OLD"
                conn_color = curses.color_pair(3)
            
            self.stdscr.addstr(start_row + 1, 4, f"Durum: {conn_status} ({self.live_imu.get('update_rate', 0)}Hz)", conn_color)
            
            # Orientation değerleri - angle bar ile
            roll_val = self.live_imu.get('roll', 0)
            pitch_val = self.live_imu.get('pitch', 0)
            yaw_val = self.live_imu.get('yaw', 0)
            
            self.stdscr.addstr(start_row + 2, 4, f"ROLL:  {roll_val:+6.1f} °", curses.color_pair(1 if abs(roll_val) < 10 else 3))
            self.stdscr.addstr(start_row + 3, 4, f"PITCH: {pitch_val:+6.1f} °", curses.color_pair(1 if abs(pitch_val) < 10 else 3))
            self.stdscr.addstr(start_row + 4, 4, f"YAW:   {yaw_val:+6.1f} °", curses.color_pair(1))
            
            # Angle bars
            self.draw_angle_bar(start_row + 2, 25, roll_val, "ROLL")
            self.draw_angle_bar(start_row + 3, 25, pitch_val, "PITCH")
            self.draw_angle_bar(start_row + 4, 25, yaw_val, "YAW")
            
        else:
            # Bağlantı yok
            self.stdscr.addstr(start_row + 1, 4, "Durum: ❌ BAĞLANTI YOK", curses.color_pair(2))
            self.stdscr.addstr(start_row + 2, 4, "ROLL:  ---- °", curses.color_pair(2))
            self.stdscr.addstr(start_row + 3, 4, "PITCH: ---- °", curses.color_pair(2))
            self.stdscr.addstr(start_row + 4, 4, "YAW:   ---- °", curses.color_pair(2))
            self.stdscr.addstr(start_row + 6, 4, "❌ Serial bağlantısı yok - IMU verisi alınamıyor", curses.color_pair(2))
    
    def draw_depth_sensor_display(self):
        """D300 Derinlik sensörü görüntüleme - YENİ!"""
        start_row = 9
        
        with self.data_lock:
            # Başlık
            self.stdscr.addstr(start_row, 55, "🌊 D300 DEPTH SENSOR (I2C)", curses.color_pair(4) | curses.A_BOLD)
            
            if self.gpio_integration.depth_sensor_available and self.depth_data['connected']:
                # Derinlik verisi
                depth_val = self.depth_data['depth_m']
                depth_color = curses.color_pair(1) if depth_val < 5 else curses.color_pair(3) if depth_val < 8 else curses.color_pair(2)
                self.stdscr.addstr(start_row + 1, 57, f"DEPTH:   {depth_val:7.2f}m", depth_color | curses.A_BOLD)
                
                # Sıcaklık verisi
                temp_val = self.depth_data['temperature_c']
                temp_color = curses.color_pair(1) if 15 < temp_val < 30 else curses.color_pair(3)
                self.stdscr.addstr(start_row + 2, 57, f"TEMP:    {temp_val:7.1f}°C", temp_color)
                
                # Basınç verisi
                pressure_val = self.depth_data['pressure_mbar']
                self.stdscr.addstr(start_row + 3, 57, f"PRESS:   {pressure_val:7.1f}mb", curses.color_pair(6))
                
                # I2C durum
                self.stdscr.addstr(start_row + 4, 57, "I2C: ✅ ACTIVE", curses.color_pair(1))
                
            else:
                self.stdscr.addstr(start_row + 1, 57, "DEPTH:   ----  m", curses.color_pair(2))
                self.stdscr.addstr(start_row + 2, 57, "TEMP:    ----  °C", curses.color_pair(2))
                self.stdscr.addstr(start_row + 3, 57, "PRESS:   ----  mb", curses.color_pair(2))
                if self.gpio_integration.depth_sensor_available:
                    self.stdscr.addstr(start_row + 4, 57, "I2C: ⚠️ NO DATA", curses.color_pair(3))
                else:
                    self.stdscr.addstr(start_row + 4, 57, "I2C: ❌ DISABLED", curses.color_pair(2))
    
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
        start_row = 14  # Depth sensor için yer açıldı
        
        if self.current_menu == "main":
            self.draw_main_controls(start_row)
        elif self.current_menu == "mission_plan":
            self.draw_mission_planning(start_row)
        elif self.current_menu == "test_scripts":
            self.draw_test_script_menu(start_row)
        elif self.current_menu == "gpio_test":
            self.draw_gpio_test_menu(start_row)
    
    def draw_main_controls(self, start_row):
        """Ana kontrol menüsü - MOTOR KONTROL EKLENDİ"""
        self.stdscr.addstr(start_row, 2, "⌨️  MANUEL KONTROL:", curses.color_pair(4) | curses.A_BOLD)
        
        # Real-time kontrol durumu
        self.stdscr.addstr(start_row + 1, 4, f"Roll:  {self.servo_values['roll']:+4.0f}° [A/D]", curses.color_pair(5))
        self.stdscr.addstr(start_row + 2, 4, f"Pitch: {self.servo_values['pitch']:+4.0f}° [W/S]", curses.color_pair(5))
        # YAW değeri daha belirgin görüntüleme
        yaw_display_color = curses.color_pair(3) | curses.A_BOLD if abs(self.servo_values['yaw']) > 0 else curses.color_pair(5)
        self.stdscr.addstr(start_row + 3, 4, f"Yaw:   {self.servo_values['yaw']:+4.0f}° [Q/E]", yaw_display_color)
        
        # MOTOR KONTROL - YENİ VE DETAYLI!
        motor_display_color = curses.color_pair(1) if abs(self.motor_value) < 30 else curses.color_pair(3) if abs(self.motor_value) < 70 else curses.color_pair(2)
        self.stdscr.addstr(start_row + 4, 4, f"Motor: {self.motor_value:+4.0f}% [J/K] [I/M boost]", motor_display_color | curses.A_BOLD)
        
        # GPIO durumu göster
        if self.gpio_integration.gpio_initialized:
            self.stdscr.addstr(start_row + 5, 4, "GPIO:  ✅ Buzzer/LED/Button aktif", curses.color_pair(1))
        else:
            self.stdscr.addstr(start_row + 5, 4, "GPIO:  ❌ Devre dışı", curses.color_pair(2))
        
        # Hızlı yüzdürme modu durumu göster
        if hasattr(self, 'quick_swim_mode') and self.quick_swim_mode:
            self.stdscr.addstr(start_row + 6, 4, "🏊 HIZLI YÜZDİRME MODU AKTİF!", curses.color_pair(2) | curses.A_BOLD)
            self.stdscr.addstr(start_row + 7, 6, "1-3: Yüz/Sığ/Derin  4-5: Sol/Sağ  6-7: İleri/Geri  ESC: Çıkış", curses.color_pair(3))
        
        # Komut menüsü - MOTOR KONTROL VE HIZLI YÜZDİRME EKLENDİ
        menu_start_row = start_row + 8 if hasattr(self, 'quick_swim_mode') and self.quick_swim_mode else start_row + 7
        self.stdscr.addstr(menu_start_row, 2, "📋 KONTROL KOMUTLARI:", curses.color_pair(4) | curses.A_BOLD)
        
        commands = [
            "W/S: Pitch ±",       "A/D: Roll ±",        "Q/E: Yaw ±",
            "J/K: Motor ±5%",     "I/M: Motor ±15%",    "N: Motor STOP",
            "R/F: RAW/PID Mode",  "X: Hızlı Yüzme",     "0: Mission Plan",
            "T: Test Scripts",    "G: GPIO Test",       "Z: Serial Debug",
            "B: Buzzer Test",     "Space: ARM/DISARM",  "C: Config",
        ]
        
        row = menu_start_row + 1
        col = 4
        for i, cmd in enumerate(commands):
            if i % 3 == 0 and i > 0:
                row += 1
                col = 4
            if cmd:
                self.stdscr.addstr(row, col, cmd[:18], curses.color_pair(6))
            col += 22
    
    def draw_gpio_test_menu(self, start_row):
        """GPIO test menüsü - YENİ!"""
        self.stdscr.addstr(start_row, 2, "🔧 GPIO/I2C TEST MENÜSÜ:", curses.color_pair(4) | curses.A_BOLD)
        
        # GPIO durum bilgisi
        gpio_status = "✅ AKTIF" if self.gpio_integration.gpio_initialized else "❌ DEVRE DIŞI"
        i2c_status = "✅ AKTIF" if self.gpio_integration.depth_sensor_available else "❌ DEVRE DIŞI"
        
        self.stdscr.addstr(start_row + 1, 4, f"GPIO Durum: {gpio_status}", curses.color_pair(1 if self.gpio_integration.gpio_initialized else 2))
        self.stdscr.addstr(start_row + 2, 4, f"I2C D300:   {i2c_status}", curses.color_pair(1 if self.gpio_integration.depth_sensor_available else 2))
        
        # GPIO test komutları
        self.stdscr.addstr(start_row + 4, 4, "GPIO TEST KOMUTLARI:", curses.color_pair(3))
        
        gpio_commands = [
            "1: Kırmızı LED",     "2: Yeşil LED",       "3: Mavi LED",
            "4: Warning LED",     "5: Status LED",      "6: RGB LED Test",
            "7: Buzzer Beep",     "8: Buzzer Alarm",    "9: Button Test",
        ]
        
        row = start_row + 5
        for i, cmd in enumerate(gpio_commands):
            if i % 3 == 0 and i > 0:
                row += 1
            col = 4 + (i % 3) * 20
            if row < self.height - 5:
                self.stdscr.addstr(row, col, cmd, curses.color_pair(6))
        
        # I2C test komutları
        self.stdscr.addstr(start_row + 9, 4, "I2C TEST KOMUTLARI:", curses.color_pair(3))
        self.stdscr.addstr(start_row + 10, 6, "D: D300 Depth Test", curses.color_pair(6))
        self.stdscr.addstr(start_row + 11, 6, "S: I2C Scanner", curses.color_pair(6))
        self.stdscr.addstr(start_row + 12, 6, "C: Calibrate Surface", curses.color_pair(6))
        
        # Menü kontrolü
        self.stdscr.addstr(start_row + 14, 4, "B: Ana menüye dön", curses.color_pair(4))
    
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
            if row < self.height - 8:
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
                if start_row + 12 + i < self.height - 2:
                    self.stdscr.addstr(start_row + 12 + i, 6, mission, curses.color_pair(5))
        else:
            self.stdscr.addstr(start_row + 11, 4, "Henüz görev eklenmemiş. Yukarıdaki komutları kullanın.", curses.color_pair(6))
    
    def draw_test_script_menu(self, start_row):
        """Test script menüsü"""
        self.stdscr.addstr(start_row, 2, "🔧 TEST SCRIPT YÖNETİCİSİ:", curses.color_pair(4) | curses.A_BOLD)
        
        # Test scriptleri listesi
        script_list = self.test_scripts.get_script_list()
        
        self.stdscr.addstr(start_row + 1, 4, "MEVCUT TEST SCRİPTLERİ:", curses.color_pair(3))
        
        for i, (script_id, name) in enumerate(script_list):
            row = start_row + 2 + i
            if row < self.height - 5:  # Ekran sınırı kontrolü
                status_text = "ÇALIŞIYOR..." if self.test_scripts.running_script == script_id else ""
                self.stdscr.addstr(row, 6, f"{script_id}: {name}", curses.color_pair(1 if not status_text else 3))
                self.stdscr.addstr(row, 35, f"- {script_list[script_id]}", curses.color_pair(6))
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
        elif self.current_menu == "gpio_test":
            self.handle_gpio_test(key)
    
    def handle_main_controls(self, key):
        """Ana kontrol tuşları - MOTOR KONTROL EKLENDİ"""
        # Real-time servo kontrol - DEBUG EKLENDI
        if key == ord('w'):
            self.active_keys.add('w')
            self.log("🎮 W tuşu basıldı (Pitch+)")
        elif key == ord('s'):
            self.active_keys.add('s')
            self.log("🎮 S tuşu basıldı (Pitch-)")
        elif key == ord('a'):
            self.active_keys.add('a')
            self.log("🎮 A tuşu basıldı (Roll+)")
        elif key == ord('d'):
            self.active_keys.add('d')
            self.log("🎮 D tuşu basıldı (Roll-)")
        elif key == ord('q') or key == ord('Q'):
            self.active_keys.add('q')
            self.log("🎯 Q tuşu basıldı - YAW SAĞ hareket başlatılıyor!")
        elif key == ord('e') or key == ord('E'):
            self.active_keys.add('e')
            self.log("🎯 E tuşu basıldı - YAW SOL hareket başlatılıyor!")
        
        # MOTOR KONTROL TUŞLARI - TAM YENİ SISTEM!
        elif key == ord('j') or key == ord('J'):
            old_motor = self.motor_value
            self.motor_value = min(100, self.motor_value + 5)
            self.send_motor_command()
            self.log(f"🚁 Motor İLERİ: {old_motor}% → {self.motor_value}% (J tuşu)")
            self.gpio_integration.beep_success()
        elif key == ord('k') or key == ord('K'):
            old_motor = self.motor_value
            self.motor_value = max(-100, self.motor_value - 5)
            self.send_motor_command()
            self.log(f"🚁 Motor GERİ: {old_motor}% → {self.motor_value}% (K tuşu)")
            self.gpio_integration.beep_success()
        elif key == ord('i') or key == ord('I'):
            old_motor = self.motor_value
            self.motor_value = min(100, self.motor_value + 15)
            self.send_motor_command()
            self.log(f"🚁 Motor BOOST İLERİ: {old_motor}% → {self.motor_value}% (I tuşu)")
            self.gpio_integration.beep_warning()
        elif key == ord('m') or key == ord('M'):
            old_motor = self.motor_value
            self.motor_value = max(-100, self.motor_value - 15)
            self.send_motor_command()
            self.log(f"🚁 Motor BOOST GERİ: {old_motor}% → {self.motor_value}% (M tuşu)")
            self.gpio_integration.beep_warning()
        
        # Motor STOP - SPACE haricinde
        elif key == ord('n') or key == ord('N'):
            old_motor = self.motor_value
            self.motor_value = 0
            self.send_motor_command()
            self.log(f"🚁 Motor STOP: {old_motor}% → 0% (N tuşu)")
            self.gpio_integration.beep_error()
        
        # ARM/DISARM
        elif key == ord(' '):  # Space
            self.toggle_arm()
        
        # Kontrol modu
        elif key == ord('r'):
            self.control_mode = "RAW"
            self.log("🎛️ Kontrol modu: RAW PWM")
            self.gpio_integration.beep_success()
        elif key == ord('f'):
            self.control_mode = "PID"
            self.log("🎛️ Kontrol modu: PID")
            self.gpio_integration.beep_success()
        
        # Serial CONNECTION DEBUG - YENİ! ⭐
        elif key == ord('z') or key == ord('Z'):
            self.serial_connection_debug()
        
        # Menü geçişleri
        elif key == ord('0'):
            self.current_menu = "mission_plan"
            self.log("🎯 Görev planlama menüsüne geçildi")
        elif key == ord('t') or key == ord('T'):
            self.current_menu = "test_scripts"
            self.log("🔧 Test script menüsüne geçildi")
        elif key == ord('g') or key == ord('G'):
            self.current_menu = "gpio_test"
            self.log("🔧 GPIO/I2C test menüsüne geçildi")
        
        # GPIO hızlı testler
        elif key == ord('b') or key == ord('B'):
            self.gpio_integration.beep_warning()
            self.log("🔊 Buzzer test - uyarı sesi çalındı")
        
        # Hızlı yüzdürme modu
        elif key == ord('x') or key == ord('X'):
            self.quick_swim_commands()
        
        # Hızlı yüzdürme komutları (sadece quick swim mode aktifse)
        elif hasattr(self, 'quick_swim_mode') and self.quick_swim_mode:
            if self.handle_quick_swim_command(key):
                return  # Komut başarılı, diğer kontrolleri atla
        
        # Diğer özellikler
        elif key == ord('c'):
            self.show_config_menu()
        elif key == ord('v'):
            self.show_vibration_data()
        elif key == ord('p'):
            self.show_gps_data()
    
    def serial_connection_debug(self):
        """Serial bağlantı debug - INSTANT TEST"""
        self.log("🔍 Serial CONNECTION DEBUG BAŞLATILIYOR...")
        
        # MAVLink durumu
        if self.mavlink_available and self.mavlink and self.mavlink.connected:
            self.log(f"📡 Serial MAVLink Object: ✅ EXISTS")
            self.log(f"📡 mavlink.connected: {getattr(self.mavlink, 'connected', 'N/A')}")
            self.log(f"📡 mavlink.master: {getattr(self.mavlink, 'master', 'N/A') is not None}")
            
            # Master durumu
            if hasattr(self.mavlink, 'master') and self.mavlink.master:
                try:
                    # Son mesajı kontrol et
                    msg = self.mavlink.master.recv_match(blocking=False, timeout=0.1)
                    if msg:
                        self.log(f"📡 Son mesaj tipi: {msg.get_type()}")
                    else:
                        self.log("📡 Hiç mesaj yok")
                        
                    # IMU verisi test et - VE ALTERNATIVE
                    imu_test = self.mavlink.get_imu_data()
                    if imu_test:
                        self.log(f"📡 IMU test: ✅ SUCCESS - {len(imu_test)} values")
                        self.log(f"📡 IMU sample: ax={imu_test[0]:.3f}, ay={imu_test[1]:.3f}, az={imu_test[2]:.3f}")
                    else:
                        self.log("📡 IMU test: ❌ FAILED - None return")
                        # Alternative test
                        if hasattr(self.mavlink, 'get_imu_data_alternative'):
                            imu_alt = self.mavlink.get_imu_data_alternative()
                            if imu_alt:
                                self.log(f"📡 IMU ALTERNATIVE: ✅ SUCCESS - {len(imu_alt)} values")
                                self.log(f"📡 ALT sample: ax={imu_alt[0]:.3f}, gy={imu_alt[4]:.3f}, gz={imu_alt[5]:.3f}")
                            else:
                                self.log("📡 IMU ALTERNATIVE: ❌ FAILED")
                        
                except Exception as e:
                    self.log(f"📡 Master test error: {e}")
            else:
                self.log("📡 Master: ❌ NOT EXISTS")
        else:
            self.log("📡 Serial MAVLink Object: ❌ NOT EXISTS")
        
        # Thread durumu
        with self.data_lock:
            self.log(f"🔄 Serial Data Connected: {self.tcp_data['connected']}")
            self.log(f"🔄 Live IMU Connected: {self.live_imu['connected']}")
            self.log(f"🔄 IMU Update Rate: {self.live_imu['update_rate']}Hz")
            
            last_packet = self.tcp_data.get('last_packet', 0)
            if last_packet > 0:
                data_age = time.time() - last_packet
                self.log(f"🔄 Son veri: {data_age:.1f} saniye önce")
            else:
                self.log("🔄 Hiç veri alınmamış")
        
        # Serial port kontrolü - shell command
        try:
            import subprocess
            result = subprocess.run(['ss', '-tlnp'], capture_output=True, text=True, timeout=3)
            for line in result.stdout.split('\n'):
                if '5777' in line:
                    self.log(f"🔌 Port 5777: {line.strip()}")
                    break
            else:
                self.log("🔌 Port 5777: ❌ NOT LISTENING")
        except Exception as e:
            self.log(f"🔌 Port check error: {e}")
        
        self.log("🔍 Serial CONNECTION DEBUG TAMAMLANDI")
        self.gpio_integration.beep_success()
    
    def handle_gpio_test(self, key):
        """GPIO test menüsü tuşları - YENİ!"""
        # LED testleri
        if key == ord('1'):
            self.gpio_integration.gpio.set_led('red', True, 100)
            self.log("🔴 Kırmızı LED açıldı")
            # 1 saniye sonra kapat
            threading.Timer(1.0, lambda: self.gpio_integration.gpio.set_led('red', False)).start()
        elif key == ord('2'):
            self.gpio_integration.gpio.set_led('green', True, 100)
            self.log("🟢 Yeşil LED açıldı")
            threading.Timer(1.0, lambda: self.gpio_integration.gpio.set_led('green', False)).start()
        elif key == ord('3'):
            self.gpio_integration.gpio.set_led('blue', True, 100)
            self.log("🔵 Mavi LED açıldı")
            threading.Timer(1.0, lambda: self.gpio_integration.gpio.set_led('blue', False)).start()
        elif key == ord('4'):
            self.gpio_integration.gpio.set_led('warning', True)
            self.log("⚠️ Warning LED açıldı")
            threading.Timer(1.0, lambda: self.gpio_integration.gpio.set_led('warning', False)).start()
        elif key == ord('5'):
            self.gpio_integration.gpio.set_led('status', True)
            self.log("📊 Status LED açıldı")
            threading.Timer(1.0, lambda: self.gpio_integration.gpio.set_led('status', False)).start()
        elif key == ord('6'):
            # RGB LED renk döngüsü
            def rgb_cycle():
                colors = [(100, 0, 0), (0, 100, 0), (0, 0, 100), (100, 100, 0), (100, 0, 100), (0, 100, 100)]
                for r, g, b in colors:
                    self.gpio_integration.gpio.set_rgb_led(r, g, b)
                    time.sleep(0.3)
                self.gpio_integration.gpio.set_rgb_led(0, 0, 0)
            threading.Thread(target=rgb_cycle, daemon=True).start()
            self.log("🌈 RGB LED renk döngüsü başlatıldı")
        
        # Buzzer testleri
        elif key == ord('7'):
            self.gpio_integration.gpio.buzzer_beep(1000, 0.3, 50)
            self.log("🔊 Buzzer beep testi (1000Hz)")
        elif key == ord('8'):
            # Alarm pattern
            def alarm_pattern():
                for _ in range(3):
                    self.gpio_integration.gpio.buzzer_beep(2000, 0.2, 70)
                    time.sleep(0.1)
                    self.gpio_integration.gpio.buzzer_beep(1000, 0.2, 70)
                    time.sleep(0.1)
            threading.Thread(target=alarm_pattern, daemon=True).start()
            self.log("🚨 Buzzer alarm pattern çalındı")
        elif key == ord('9'):
            # Button durumu göster
            button_state = self.gpio_integration.read_button()
            self.log(f"🔘 Button durumu: {'BASILDI' if button_state else 'BASILMADI'}")
        
        # I2C testleri
        elif key == ord('d') or key == ord('D'):
            if self.gpio_integration.depth_sensor_available:
                depth_data = self.gpio_integration.get_depth_data()
                if depth_data:
                    self.log(f"🌊 D300 Test: {depth_data['depth_m']:.2f}m, {depth_data['temperature_c']:.1f}°C")
                else:
                    self.log("❌ D300 veri alınamadı")
            else:
                self.log("❌ D300 sensörü hazır değil")
        elif key == ord('s') or key == ord('S'):
            self.log("🔍 I2C tarama başlatılıyor...")
            # I2C scanner çalıştır
            try:
                result = subprocess.run([sys.executable, "i2c_scanner.py"], capture_output=True, text=True, timeout=10)
                for line in result.stdout.strip().split('\n')[-3:]:  # Son 3 satır
                    if line.strip():
                        self.log(f"I2C: {line}")
            except Exception as e:
                self.log(f"❌ I2C tarama hatası: {e}")
        elif key == ord('c') or key == ord('C'):
            if self.gpio_integration.depth_sensor:
                success = self.gpio_integration.depth_sensor.calibrate_surface()
                if success:
                    self.log("✅ D300 yüzey kalibrasyonu tamamlandı")
                else:
                    self.log("❌ D300 kalibrasyon başarısız")
            else:
                self.log("❌ D300 sensörü yok")
        
        # Menü geçişi
        elif key == ord('b') or key == ord('B'):
            self.current_menu = "main"
            self.log("🔙 Ana menüye dönüldü")
    
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
                    self.gpio_integration.beep_success()
        
        # Görev kontrolü
        elif key == ord('\n') or key == ord('\r'):  # Enter
            if self.mission_planner.start_mission():
                self.log("🚀 Görev akışı başlatıldı!")
                self.gpio_integration.beep_warning()
                self.execute_mission_flow()
            else:
                self.log("❌ Başlatılacak görev yok!")
                self.gpio_integration.beep_error()
        
        elif key == ord('c') or key == ord('C'):
            self.mission_planner.clear_missions()
            self.log("🗑️ Tüm görevler temizlendi")
            self.gpio_integration.beep_success()
        
        elif key == ord('b') or key == ord('B'):
            self.current_menu = "main"
            self.log("🔙 Ana menüye dönüldü")
    
    def handle_test_scripts(self, key):
        """Test script tuşları"""
        # Script çalıştırma
        if key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5'), ord('6'), ord('7'), ord('8'), ord('9')]:
            script_id = chr(key)
            if self.test_scripts.run_script(script_id, self.test_script_callback):
                script_info = self.test_scripts.scripts.get(script_id, {})
                self.log(f"🔧 Test başlatıldı: {script_info.get('name', 'Unknown')}")
                self.gpio_integration.beep_success()
        
        # Script durdurma
        elif key == ord('s') or key == ord('S'):
            if self.test_scripts.running_script:
                self.log("⏹️ Çalışan script durduruluyor...")
                self.gpio_integration.beep_warning()
                # Script durdurma işlemi burada implement edilecek
            else:
                self.log("⚠️ Çalışan script yok")
        
        elif key == ord('b') or key == ord('B'):
            self.current_menu = "main"
            self.log("🔙 Ana menüye dönüldü")
    
    def get_mission_value_input(self, command):
        """Görev değeri girişi"""
        cmd_info = self.mission_planner.mission_commands.get(command, {})
        param_type = cmd_info.get('param_type', 'distance')
        
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
                for mission in self.mission_planner.missions:
                    if not self.mission_planner.mission_active:
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
                
                self.mission_planner.mission_active = False
                self.log("✅ Görev akışı tamamlandı!")
                
            except Exception as e:
                self.log(f"❌ Görev akışı hatası: {e}")
                self.mission_planner.mission_active = False
        
        # Mission executor thread
        mission_thread = threading.Thread(target=mission_executor, daemon=True)
        mission_thread.start()
    
    def execute_single_mission(self, mission):
        """Tek görev çalıştır"""
        try:
            action = mission['command']
            value = mission['value']
            
            if not self.mavlink or not self.mavlink.connected or not self.armed:
                self.log("❌ Görev için ARM gerekli!")
                return False
            
            # Görev tipine göre işlem
            if action == 'F':
                return self.execute_forward(value)
            elif action == 'B':
                return self.execute_backward(value)
            elif action == 'L':
                return self.execute_left(value)
            elif action == 'R':
                return self.execute_right(value)
            elif action == 'U':
                return self.execute_up(value)
            elif action == 'D':
                return self.execute_down(value)
            elif action == 'Y':
                return self.execute_yaw(value)
            elif action == 'W':
                return self.execute_wait(value)
            elif action == 'S':
                return self.execute_surface()
            
            return False
            
        except Exception as e:
            self.log(f"❌ Görev çalıştırma hatası: {e}")
            return False
    
    def execute_forward(self, distance):
        """İleri/geri hareket"""
        duration = abs(distance) * 2  # 2 saniye/metre
        motor_value = 30 if distance > 0 else -30
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_active:
                return False
            
            self.motor_value = motor_value
            self.send_motor_command()
            time.sleep(0.1)
        
        # Motor'u durdur
        self.motor_value = 0
        self.send_motor_command()
        return True
    
    def execute_backward(self, distance):
        """İleri/geri hareket"""
        return self.execute_forward(-distance)
    
    def execute_left(self, angle):
        """Sol dönüş"""
        duration = abs(angle) / 45  # 45°/saniye
        yaw_value = -angle * 30  # ±30° yaw servo
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_active:
                return False
            
            self.servo_values['yaw'] = yaw_value
            self.send_servo_commands()
            time.sleep(0.1)
        
        # Neutral'a getir
        self.servo_values['yaw'] = 0
        self.send_servo_commands()
        return True
    
    def execute_right(self, angle):
        """Sağ dönüş"""
        return self.execute_left(-angle)
    
    def execute_up(self, distance):
        """Yukarı hareket"""
        duration = distance * 3  # 3 saniye/metre
        pitch_value = distance * 20  # ±20° pitch
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_active:
                return False
            
            self.servo_values['pitch'] = pitch_value
            self.send_servo_commands()
            time.sleep(0.1)
        
        # Neutral'a getir
        self.servo_values['pitch'] = 0
        self.send_servo_commands()
        return True
    
    def execute_down(self, distance):
        """Aşağı hareket"""
        return self.execute_up(-distance)
    
    def execute_yaw(self, angle):
        """Yaw dönüşü"""
        duration = abs(angle) / 45  # 45°/saniye
        yaw_value = angle * 30  # ±30° yaw servo
        
        for i in range(int(duration * 10)):  # 10Hz
            if not self.mission_planner.mission_active:
                return False
            
            self.servo_values['yaw'] = yaw_value
            self.send_servo_commands()
            time.sleep(0.1)
        
        # Neutral'a getir
        self.servo_values['yaw'] = 0
        self.send_servo_commands()
        return True
    
    def execute_wait(self, duration):
        """Süre bekleme"""
        time.sleep(duration)
        return True
    
    def execute_surface(self):
        """Yüzeye çıkma"""
        return True
    
    def test_script_callback(self, script_id, success, stdout, stderr):
        """Test script callback"""
        script_info = self.test_scripts.scripts.get(script_id, {})
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
        """Real-time servo kontrolü - YAW PROBLEMİ DÜZELTİLDİ"""
        # Smooth control parameters
        pitch_step = 2.5
        roll_step = 2.5
        yaw_step = 4.0  # YAW için daha büyük adım
        decay_rate = 1.5  # Otomatik sıfırlama hızı
        
        # Pitch kontrol - SMOOTH
        if 'w' in self.active_keys:
            self.servo_values['pitch'] = min(45, self.servo_values['pitch'] + pitch_step)
        elif 's' in self.active_keys:
            self.servo_values['pitch'] = max(-45, self.servo_values['pitch'] - pitch_step)
        else:
            # Smooth otomatik sıfırlama
            if abs(self.servo_values['pitch']) > 0.5:
                if self.servo_values['pitch'] > 0:
                    self.servo_values['pitch'] = max(0, self.servo_values['pitch'] - decay_rate)
                else:
                    self.servo_values['pitch'] = min(0, self.servo_values['pitch'] + decay_rate)
            else:
                self.servo_values['pitch'] = 0
        
        # Roll kontrol - SMOOTH
        if 'a' in self.active_keys:
            self.servo_values['roll'] = min(45, self.servo_values['roll'] + roll_step)
        elif 'd' in self.active_keys:
            self.servo_values['roll'] = max(-45, self.servo_values['roll'] - roll_step)
        else:
            # Smooth otomatik sıfırlama
            if abs(self.servo_values['roll']) > 0.5:
                if self.servo_values['roll'] > 0:
                    self.servo_values['roll'] = max(0, self.servo_values['roll'] - decay_rate)
                else:
                    self.servo_values['roll'] = min(0, self.servo_values['roll'] + decay_rate)
            else:
                self.servo_values['roll'] = 0
        
        # YAW kontrol - TAMAMEN YENİ VE GÜÇLENDİRİLMİŞ SYSTEM
        yaw_changed = False
        
        if 'q' in self.active_keys:
            old_yaw = self.servo_values['yaw']
            self.servo_values['yaw'] = min(45, self.servo_values['yaw'] + yaw_step)
            yaw_changed = True
            
            # Sadece değer değiştiğinde log
            if self.servo_values['yaw'] != old_yaw:
                self.log(f"🎯 YAW RIGHT: {old_yaw:.1f}° → {self.servo_values['yaw']:.1f}° (Q key active)")
                
        elif 'e' in self.active_keys:
            old_yaw = self.servo_values['yaw']
            self.servo_values['yaw'] = max(-45, self.servo_values['yaw'] - yaw_step)
            yaw_changed = True
            
            # Sadece değer değiştiğinde log
            if self.servo_values['yaw'] != old_yaw:
                self.log(f"🎯 YAW LEFT: {old_yaw:.1f}° → {self.servo_values['yaw']:.1f}° (E key active)")
                
        else:
            # YAW Smooth auto-return to neutral - IMPROVED
            if abs(self.servo_values['yaw']) > 0.8:  # Threshold artırıldı
                old_yaw = self.servo_values['yaw']
                
                if self.servo_values['yaw'] > 0:
                    self.servo_values['yaw'] = max(0, self.servo_values['yaw'] - (decay_rate * 1.2))
                else:
                    self.servo_values['yaw'] = min(0, self.servo_values['yaw'] + (decay_rate * 1.2))
                
                # Neutral'a yakın değerler için debug
                if abs(old_yaw) > 1 and abs(self.servo_values['yaw']) <= 1:
                    self.log(f"🎯 YAW NEUTRAL: {old_yaw:.1f}° → {self.servo_values['yaw']:.1f}° (auto-return)")
                    
            elif abs(self.servo_values['yaw']) <= 0.8:
                # Küçük değerleri direkt sıfırla
                if self.servo_values['yaw'] != 0:
                    self.log(f"🎯 YAW ZERO: {self.servo_values['yaw']:.1f}° → 0.0° (force neutral)")
                self.servo_values['yaw'] = 0
        
        # Servo komutlarını gönder - YAW priority ile
        self.send_servo_commands(yaw_priority=yaw_changed)
        
        # Tuş durumunu temizle
        self.active_keys.clear()
    
    def send_servo_commands(self, yaw_priority=False):
        """Servo komutlarını TCP üzerinden gönder - YAW PROBLEMİ ÇÖZÜLDÜ"""
        
        # Logging strategy - sadece önemli değişiklikler
        if yaw_priority and abs(self.servo_values['yaw']) > 0:
            # YAW hareketi aktif - detaylı log
            self.log(f"🎯 YAW COMMAND: {self.servo_values['yaw']:.1f}° | R:{self.servo_values['roll']:.1f}° P:{self.servo_values['pitch']:.1f}°")
        elif not yaw_priority and (abs(self.servo_values['roll']) > 2 or abs(self.servo_values['pitch']) > 2):
            # Roll/Pitch hareketi - normal log
            self.log(f"📡 SERVO: R={self.servo_values['roll']:.1f}° P={self.servo_values['pitch']:.1f}° Y={self.servo_values['yaw']:.1f}°")
        
        # MAVLink bağlantı kontrolü - STREAMLINED
        if not self.mavlink or not self.mavlink.connected:
            # Sadece YAW aktifken uyarı göster (spam önleme)
            if yaw_priority:
                self.log("⚠️ Serial MAVLink disconnected - YAW command ignored!")
            return
            
        if not self.armed:
            # Sadece YAW aktifken uyarı göster (spam önleme)
            if yaw_priority:
                self.log("⚠️ DISARMED - YAW ignored! Press SPACE to ARM")
            return
        
        # GERÇEK SERVO KOMUTLARI - High Priority YAW
        try:
            # Real-time servo command transmission
            if self.control_mode == "RAW":
                success = self.mavlink.control_servos_raw(
                    self.servo_values['roll'],
                    self.servo_values['pitch'],
                    self.servo_values['yaw']
                )
            else:  # PID mode
                success = self.mavlink.control_servos_pid(
                    self.servo_values['roll'],
                    self.servo_values['pitch'],
                    self.servo_values['yaw']
                )
            
            # Success feedback - sadece YAW priority'sinde
            if yaw_priority and success:
                self.log(f"✅ YAW transmitted: {self.servo_values['yaw']:.1f}° via {self.control_mode}")
                
        except Exception as e:
            # Error feedback - sadece önemli hatalar
            if yaw_priority or abs(self.servo_values['roll']) > 10 or abs(self.servo_values['pitch']) > 10:
                self.log(f"❌ Servo transmission error: {e}")
                self.log(f"🔧 Values: R={self.servo_values['roll']:.1f}° P={self.servo_values['pitch']:.1f}° Y={self.servo_values['yaw']:.1f}°")
    
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
        """ARM/DISARM toggle - GPIO entegrasyonu ile"""
        if not self.mavlink or not self.mavlink.connected:
            self.log("❌ Serial MAVLink bağlantısı yok!")
            self.gpio_integration.beep_error()
            return
        
        try:
            # Basit ARM/DISARM toggle
            if self.armed:
                # DISARM
                self.log("🟢 DISARM ediliyor...")
                self.mavlink.disarm_system()
                self.armed = False
                self.log("🟢 GUI DISARM edildi (servo komutları artık gönderilmez)")
                
                # GPIO LED güncelle - YENİ!
                self.gpio_integration.set_connection_status_led(True, False)
                self.gpio_integration.beep_success()
                
                # Kontrolleri sıfırla
                self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
                self.motor_value = 0
                
            else:
                # ARM
                self.log("🔴 ARM ediliyor...")
                if self.mavlink.arm_system():
                    self.armed = True
                    self.log("🔴 GUI ARM edildi! Servo komutları MAVLink'e gönderilecek!")
                    self.log("🎮 Artık Q/E, W/S, A/D, J/K/I/M tuşları kontrolleri hareket ettirir!")
                    
                    # GPIO LED güncelle - YENİ!
                    self.gpio_integration.set_connection_status_led(True, True)
                    self.gpio_integration.beep_warning()  # ARM uyarısı
                    
                else:
                    self.log("⚠️ ARM başarısız ama GUI ARM moduna geçiyor (test için)")
                    self.armed = True  # Test için GUI'de ARM yap
                    self.log("💡 Manuel kontroller çalışır ama servo'lar hareket etmez")
                    
                    # GPIO uyarı - YENİ!
                    self.gpio_integration.set_connection_status_led(True, True)
                    self.gpio_integration.beep_error()
            
        except Exception as e:
            self.log(f"❌ ARM/DISARM hatası: {e}")
            # Hata olsa bile GUI durumunu değiştir
            self.armed = not self.armed
            status = "ARM" if self.armed else "DISARM"
            self.log(f"🔄 GUI {status} durumuna geçirildi (test için)")
            
            # GPIO hata sinyali - YENİ!
            self.gpio_integration.beep_error()
            self.gpio_integration.set_connection_status_led(True, self.armed)
    
    def show_config_menu(self):
        """Konfigürasyon menüsü"""
        self.log("🔧 Konfigürasyon menüsü açılıyor...")
        # Basit implementasyon - detayları daha sonra
    
    def show_vibration_data(self):
        """Vibration verilerini göster - OPSIYONEL"""
        if self.mavlink and self.mavlink.connected:
            self.log("📈 Vibration monitoring şimdilik devre dışı")
        else:
            self.log("⚠️ MAVLink bağlantısı gerekli")
    
    def show_gps_data(self):
        """GPS verilerini göster - OPSIYONEL"""
        if self.mavlink and self.mavlink.connected:
            gps_data = self.mavlink.get_gps_data()
            if gps_data:
                lat, lon, alt, sats = gps_data
                self.log(f"🗺️ GPS: Lat={lat:.6f}° Lon={lon:.6f}° Alt={alt:.1f}m Sats={sats}")
                self.gpio_integration.beep_success()
            else:
                self.log("🗺️ GPS verisi alınamadı")
                self.gpio_integration.beep_error()
        else:
            self.log("⚠️ MAVLink bağlantısı gerekli")
            self.gpio_integration.beep_error()
    
    def quick_swim_commands(self):
        """Hızlı yüzdürme komutları - Real-time kontrol"""
        if not self.mavlink or not self.mavlink.connected:
            self.log("❌ MAVLink bağlantısı gerekli!")
            return
        
        self.log("\n🏊 HIZLI YÜZDİRME KOMUTLARI AKTİF!")
        self.log("🎮 Kontroller:")
        self.log("   1,2,3 = Yüzdürme seviyeleri (yüzey, sığ, derin)")
        self.log("   4,5 = Sol/Sağ dönüş")
        self.log("   6,7 = İleri/Geri")
        self.log("   0 = Neutral (dur)")
        self.log("   ESC = Çıkış")
        
        # Quick swim mode flag
        self.quick_swim_mode = True
        
        # Pre-defined PWM values for quick commands
        self.quick_swim_pwm = {
            'neutral': 1500,
            'motor_stop': 1500,
            'surface_swim': {'fins': 1450, 'motor': 1520},   # Hafif yukarı + yavaş
            'shallow_dive': {'fins': 1550, 'motor': 1550},   # Hafif aşağı + orta
            'deep_dive': {'fins': 1600, 'motor': 1550},      # Orta aşağı + orta  
            'turn_left': {'left': 1550, 'right': 1450, 'motor': 1530},
            'turn_right': {'left': 1450, 'right': 1550, 'motor': 1530},
            'forward': {'fins': 1500, 'motor': 1600},        # Neutral fins + hızlı
            'reverse': {'fins': 1500, 'motor': 1450}         # Neutral fins + geri
        }
        
        self.log("✅ Hızlı yüzdürme modu aktif - sayı tuşlarını kullanın!")
    
    def handle_quick_swim_command(self, key):
        """Hızlı yüzdürme komutunu işle"""
        if not hasattr(self, 'quick_swim_mode') or not self.quick_swim_mode:
            return False
        
        if not self.mavlink or not self.mavlink.connected:
            return False
        
        try:
            # Servo channels (X-wing config)
            channels = {
                'front_left': 1,   # AUX1
                'front_right': 3,  # AUX3
                'rear_left': 4,    # AUX4
                'rear_right': 5,   # AUX5
                'motor': 6         # AUX6
            }
            
            if key == ord('0'):
                # Neutral position
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], self.quick_swim_pwm['neutral'])
                self.mavlink.set_servo_pwm(channels['motor'], self.quick_swim_pwm['motor_stop'])
                self.log("🏊 Neutral pozisyon - durma")
                
            elif key == ord('1'):
                # Surface swim
                pwm = self.quick_swim_pwm['surface_swim']['fins']
                motor_pwm = self.quick_swim_pwm['surface_swim']['motor']
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Su yüzeyinde yüzdürme - tüm finler hafif yukarı")
                
            elif key == ord('2'):
                # Shallow dive
                pwm = self.quick_swim_pwm['shallow_dive']['fins']
                motor_pwm = self.quick_swim_pwm['shallow_dive']['motor']
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Sığ dalış - 0.5m derinlik")
                
            elif key == ord('3'):
                # Deep dive
                pwm = self.quick_swim_pwm['deep_dive']['fins']
                motor_pwm = self.quick_swim_pwm['deep_dive']['motor']
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Derin dalış - 1m derinlik")
                
            elif key == ord('4'):
                # Turn left
                left_pwm = self.quick_swim_pwm['turn_left']['left']
                right_pwm = self.quick_swim_pwm['turn_left']['right']
                motor_pwm = self.quick_swim_pwm['turn_left']['motor']
                
                self.mavlink.set_servo_pwm(channels['front_left'], left_pwm)
                self.mavlink.set_servo_pwm(channels['rear_left'], left_pwm)
                self.mavlink.set_servo_pwm(channels['front_right'], right_pwm)
                self.mavlink.set_servo_pwm(channels['rear_right'], right_pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Sol dönüş - X-wing asimetrik kontrol")
                
            elif key == ord('5'):
                # Turn right
                left_pwm = self.quick_swim_pwm['turn_right']['left']
                right_pwm = self.quick_swim_pwm['turn_right']['right']
                motor_pwm = self.quick_swim_pwm['turn_right']['motor']
                
                self.mavlink.set_servo_pwm(channels['front_left'], left_pwm)
                self.mavlink.set_servo_pwm(channels['rear_left'], left_pwm)
                self.mavlink.set_servo_pwm(channels['front_right'], right_pwm)
                self.mavlink.set_servo_pwm(channels['rear_right'], right_pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Sağ dönüş - X-wing asimetrik kontrol")
                
            elif key == ord('6'):
                # Forward
                pwm = self.quick_swim_pwm['forward']['fins']
                motor_pwm = self.quick_swim_pwm['forward']['motor']
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Hızlı ileri - düz yüzdürme")
                
            elif key == ord('7'):
                # Reverse
                pwm = self.quick_swim_pwm['reverse']['fins']
                motor_pwm = self.quick_swim_pwm['reverse']['motor']
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], pwm)
                self.mavlink.set_servo_pwm(channels['motor'], motor_pwm)
                self.log("🏊 Geri hareket")
                
            elif key == 27:  # ESC key
                # Exit quick swim mode
                self.quick_swim_mode = False
                # Return to neutral
                for servo in ['front_left', 'front_right', 'rear_left', 'rear_right']:
                    self.mavlink.set_servo_pwm(channels[servo], self.quick_swim_pwm['neutral'])
                self.mavlink.set_servo_pwm(channels['motor'], self.quick_swim_pwm['motor_stop'])
                self.log("🏊 Hızlı yüzdürme modu kapatıldı - normal kontrollere dönüldü")
                return True
            else:
                return False
            
            # GPIO feedback
            if hasattr(self, 'gpio_integration') and self.gpio_integration:
                self.gpio_integration.beep_success()
            
            return True
            
        except Exception as e:
            self.log(f"❌ Hızlı yüzdürme komutu hatası: {e}")
            return False
    
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
                    self.draw_depth_sensor_display()
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
        """Temizlik işlemleri - GPIO/I2C DİL entegrasyonu"""
        self.log("🔄 Sistem kapatılıyor...")
        
        # Thread'leri durdur
        self.tcp_running = False
        if self.tcp_thread and self.tcp_thread.is_alive():
            self.tcp_thread.join(timeout=2)
        
        # Görevleri durdur
        self.mission_planner.mission_active = False
        
        # Servolar neutral'a
        if self.mavlink and self.mavlink.connected:
            try:
                self.mavlink.emergency_stop()
                self.mavlink.disconnect()
            except:
                pass
        
        # GPIO ve I2C temizliği - YENİ!
        if self.gpio_integration:
            try:
                # Son ses - sistem kapanıyor
                self.gpio_integration.beep_warning()
                self.gpio_integration.set_rgb_led(100, 100, 0)  # Turuncu
                time.sleep(0.2)
                self.gpio_integration.set_rgb_led(0, 0, 0)  # Kapat
                
                # GPIO/I2C cleanup
                self.gpio_integration.cleanup()
            except Exception as e:
                self.log(f"⚠️ GPIO cleanup hatası: {e}")
        
        self.log("✅ Sistem temizlendi!")
    
    def run(self):
        """Uygulamayı çalıştır"""
        # Curses uygulamasını başlat
        try:
            curses.wrapper(self._curses_main)
        except Exception as e:
            pass  # Curses içinde print yapma
        finally:
            self.cleanup()
    
    def _curses_main(self, stdscr):
        """Curses ana fonksiyonu"""
        self.init_curses(stdscr)
        # Sistem bileşenlerini curses SONRASI başlat
        self.init_systems()
        self.main_loop()

if __name__ == "__main__":
    # Çalışma dizinini kontrol et
    if not os.path.exists("config"):
        sys.exit(1)
    
    # Terminal GUI'yi başlat
    try:
        gui = AdvancedTerminalGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    except ImportError as e:
        sys.exit(1)
    except Exception as e:
        import traceback
        traceback.print_exc()
        sys.exit(1) 