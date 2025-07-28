#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Advanced Terminal GUI
TCP-Based Real-time Control & Mission Planning System
GPIO & I2C Full Integration (Buzzer, LED, Button, D300 Depth Sensor)
"""

import sys
import os

# Pi5 + PiOS curses desteği - BASİTLEŞTİRİLDİ
try:
    import curses
    # Terminal UI hazır - print yok
except ImportError as e:
    # Terminal UI hatası - print yok, sys.exit direkt
    sys.exit(1)

import threading
import time
import subprocess
import json
import math
from datetime import datetime
from collections import deque

# Local imports - BASİTLEŞTİRİLDİ VE GPIO/I2C EKLENDİ
try:
    from mavlink_handler import MAVLinkHandler
    
    # GPIO Controller - ZORUNİ DEĞIL AMA KULLANILACAK
    try:
        from gpio_controller import GPIOController
        HAS_GPIO = True
    except ImportError:
        HAS_GPIO = False
    
    # D300 Depth Sensor - I2C üzerinden
    try:
        from depth_sensor import D300DepthSensor
        HAS_DEPTH_SENSOR = True
    except ImportError:
        HAS_DEPTH_SENSOR = False
        
except ImportError as e:
    sys.exit(1)

class GPIOIntegration:
    """GPIO ve I2C entegrasyon yöneticisi"""
    
    def __init__(self, config):
        self.config = config
        
        # GPIO controller
        self.gpio = None
        if HAS_GPIO:
            try:
                self.gpio = GPIOController(config)
                if self.gpio.initialize():
                    # Button callback ayarla
                    self.gpio.setup_button_callback(self.emergency_button_pressed)
                    self.gpio_ready = True
                else:
                    self.gpio_ready = False
            except Exception as e:
                self.gpio_ready = False
        else:
            self.gpio_ready = False
        
        # D300 Depth sensor
        self.depth_sensor = None
        if HAS_DEPTH_SENSOR:
            try:
                # İlk önce gerçek sensörü dene
                self.depth_sensor = D300DepthSensor(config_path="config/hardware_config.json")
                if self.depth_sensor.connect():
                    self.depth_sensor.start_monitoring(interval=0.1)  # 10Hz
                    self.depth_ready = True
                else:
                    # Gerçek sensör yoksa simülasyon moduna geç
                    self.depth_sensor = D300DepthSensor(simulation_mode=True)
                    if self.depth_sensor.connect():
                        self.depth_sensor.start_monitoring(interval=0.1)
                        self.depth_ready = True
                    else:
                        self.depth_ready = False
            except Exception as e:
                self.depth_ready = False
        else:
            self.depth_ready = False
        
        # Emergency button callback function
        self.emergency_callback = None
    
    def set_emergency_callback(self, callback):
        """Acil durum butonu callback ayarla"""
        self.emergency_callback = callback
    
    def emergency_button_pressed(self):
        """Acil durum butonu basıldı"""
        # Log yerine direkt callback çağır
        if self.gpio_ready:
            # Acil durum LED/buzzer pattern
            self.gpio.emergency_led_pattern()
            self.gpio.buzzer_beep(2000, 0.3, 80)  # 2kHz, 300ms, 80% volume
        
        if self.emergency_callback:
            self.emergency_callback()
    
    def set_system_status_led(self, status):
        """Sistem durum LED ayarla"""
        if self.gpio_ready:
            self.gpio.status_led_pattern(status)
    
    def set_connection_status_led(self, connected, armed=False):
        """Bağlantı durumu LED'i"""
        if not self.gpio_ready:
            return
        
        if armed:
            self.gpio.set_rgb_led(100, 0, 0)  # ARMED: Kırmızı
        elif connected:
            self.gpio.set_rgb_led(0, 100, 0)  # CONNECTED: Yeşil
        else:
            self.gpio.set_rgb_led(100, 50, 0)  # DISCONNECTED: Turuncu
    
    def beep_success(self):
        """Başarı sesi"""
        if self.gpio_ready:
            self.gpio.buzzer_beep(1500, 0.1, 50)
    
    def beep_error(self):
        """Hata sesi"""
        if self.gpio_ready:
            self.gpio.buzzer_beep(500, 0.3, 70)
    
    def beep_warning(self):
        """Uyarı sesi"""
        if self.gpio_ready:
            self.gpio.buzzer_beep(1000, 0.2, 60)
    
    def read_button(self):
        """Buton durumu oku"""
        if self.gpio_ready:
            return self.gpio.read_button()
        return False
    
    def get_depth_data(self):
        """Derinlik sensörü verisi"""
        if self.depth_ready and self.depth_sensor:
            return self.depth_sensor.get_sensor_data()
        return None
    
    def cleanup(self):
        """GPIO/I2C temizliği"""
        if self.gpio and self.gpio_ready:
            self.gpio.cleanup()
        
        if self.depth_sensor and self.depth_ready:
            self.depth_sensor.disconnect()

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
            '1': {'name': 'Motor Testi', 'file': 'Test/test_motor_control.py', 'desc': 'Tüm motorları test et'},
            '2': {'name': 'Servo Kalibrasyon', 'file': 'scripts/servo_calibration.py', 'desc': 'Servo kalibrasyonu'},
            '3': {'name': 'IMU Kalibrasyon', 'file': 'scripts/imu_calibration.py', 'desc': 'IMU kalibrasyonu'},
            '4': {'name': 'Sistem Kontrolü', 'file': 'scripts/system_check.py', 'desc': 'Tam sistem kontrolü'},
            '5': {'name': 'D300 Test', 'file': 'Test/test_d300_depth_sensor.py', 'desc': 'D300 I2C derinlik test'},
            '6': {'name': 'GPIO Test', 'file': 'Test/test_gpio_button.py', 'desc': 'GPIO buton/LED test'},
            '7': {'name': 'Acil Durum Test', 'file': 'scripts/emergency_stop.py', 'desc': 'Acil durum protokolü'},
            '8': {'name': 'Stabilizasyon Test', 'file': 'Test/test_stabilization.py', 'desc': 'Stabilizasyon test'},
            '9': {'name': 'Full System Test', 'file': 'Test/test_full_system.py', 'desc': 'Tam sistem testi'}
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
        """Advanced Terminal GUI başlatıcı - FULL GPIO/I2C INTEGRATION"""
        
        # Config yükle
        self.load_config()
        
        # GPIO ve I2C entegrasyonu - YENİ!
        self.gpio_integration = GPIOIntegration(self.config)
        self.gpio_integration.set_emergency_callback(self.emergency_stop_callback)
        
        # Sistem bileşenleri - BASİTLEŞTİRİLDİ
        self.mavlink = None
        
        # Yeni sistem bileşenleri
        self.mission_planner = MissionPlanner()
        self.test_manager = TestScriptManager()
        
        # Kontrol durumu
        self.control_mode = "RAW"  # RAW veya PID
        self.navigation_mode = "IMU"  # GPS, IMU, HYBRID
        self.armed = False
        self.running = True
        
        # Real-time kontrol - MOTOR KONTROL EKLENDİ
        self.active_keys = set()
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0  # Motor değeri
        self.depth_target = 0
        
        # Terminal UI
        self.stdscr = None
        self.height = 0
        self.width = 0
        self.current_menu = "main"  # main, mission_plan, test_scripts, gpio_test
        
        # Logs - debug için artırıldı
        self.log_messages = deque(maxlen=200)
        
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
        
        # D300 Depth sensor data - YENİ!
        self.depth_data = {
            'depth_m': 0.0,
            'temperature_c': 20.0,
            'pressure_mbar': 1013.25,
            'connected': False
        }
        
        # Thread kontrolü
        self.data_lock = threading.Lock()
        self.data_thread = None
        self.data_thread_running = False
    
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
            # GERÇEK HARDWARE - Minimal config
            self.config = {
                "pixhawk": {
                    "servos": {
                        "front_left": 1,   # AUX1 → MAVLink 9 (Ön Sol)
                        "front_right": 3,  # AUX3 → MAVLink 11 (Ön Sağ)
                        "rear_left": 4,    # AUX4 → MAVLink 12 (Arka Sol)
                        "rear_right": 5    # AUX5 → MAVLink 13 (Arka Sağ)
                    },
                    "motor": 6,  # AUX6 → MAVLink 14 (Ana Motor)
                    "pwm_limits": {"servo_min": 1100, "servo_max": 1900, "servo_neutral": 1500, "motor_min": 1000, "motor_max": 2000, "motor_stop": 1500}
                },
                "mavlink": {"connection_string": "tcp:127.0.0.1:5777"},
                "raspberry_pi": {
                    "gpio": {
                        "buzzer": 7,
                        "control_button": 13,
                        "led_red": 4,
                        "led_green": 5,
                        "led_blue": 6,
                        "warning_led": 8,
                        "system_status_led": 10
                    },
                    "i2c": {"depth_sensor_address": "0x76", "bus_number": 1}
                }
            }
    
    def log(self, message):
        """Thread-safe log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        
        with self.data_lock:
            self.log_messages.append(log_entry)
    
    def emergency_stop_callback(self):
        """Acil durum butonu callback"""
        self.log("🚨 ACİL DURUM BUTONU BASILDI - TÜM SİSTEMLER DURDURULUYOR!")
        
        # Tüm kontrolleri sıfırla
        self.servo_values = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.motor_value = 0
        
        # Sistem disarm
        if self.armed:
            self.armed = False
            if self.mavlink and self.mavlink.connected:
                try:
                    self.mavlink.emergency_stop()
                except:
                    pass
        
        # Görevleri durdur
        self.mission_planner.mission_running = False
        
        # Buzzer pattern çal
        self.gpio_integration.beep_error()
        
        self.log("✅ Acil durum prosedürü tamamlandı!")
    
    def init_systems(self):
        """Sistem bileşenlerini başlat - TCP odaklı - DÜZELTİLDİ"""
        self.log("🚀 TEKNOFEST ROV Advanced Terminal GUI başlatılıyor...")
        
        # GPIO sistem durumunu belirt
        if self.gpio_integration.gpio_ready:
            self.log("✅ GPIO sistemleri hazır - Buzzer/LED/Button aktif!")
            self.gpio_integration.set_system_status_led('connecting')
        else:
            self.log("⚠️ GPIO sistemler devre dışı - sadece TCP kontrol")
        
        # D300 derinlik sensörü durumu
        if self.gpio_integration.depth_ready:
            self.log("✅ D300 derinlik sensörü hazır - I2C data akışı aktif!")
        else:
            self.log("⚠️ D300 derinlik sensörü devre dışı")
        
        # TCP MAVLink bağlantısı - DETAYLI DEBUG
        try:
            self.log("📡 TCP 127.0.0.1:5777 bağlantısı başlatılıyor...")
            self.log("🔧 MAVLinkHandler() oluşturuluyor...")
            self.mavlink = MAVLinkHandler()
            self.log("✅ MAVLinkHandler() oluşturuldu")
            
            # Bağlantı kurulmaya çalışılıyor - DETAYLI LOG
            self.log("⏳ mavlink.connect() çağrılıyor (timeout: 20s)...")
            self.log("🔧 DEBUG: mavlink.connect() çağrılıyor...")
            connect_result = self.mavlink.connect()
            self.log(f"🔧 DEBUG: connect() sonucu: {connect_result}")
            self.log(f"🔍 mavlink.connect() sonucu: {connect_result}")
            
            if connect_result:
                self.log("✅ TCP MAVLink bağlantısı kuruldu (127.0.0.1:5777)!")
                
                # GPIO LED güncelle
                self.gpio_integration.set_connection_status_led(True, self.armed)
                self.gpio_integration.beep_success()
                
                # Sistem durumunu kontrol et
                self.log("🔍 check_system_status() çağrılıyor...")
                self.log(f"🔧 DEBUG: connect() sonrası mavlink.connected = {self.mavlink.connected}")
                self.mavlink.check_system_status()
                self.log(f"🔧 DEBUG: check_system_status() sonrası mavlink.connected = {self.mavlink.connected}")
                self.log(f"📊 MAVLink durumu: Connected={self.mavlink.connected}, Armed={self.mavlink.armed}")
                
                # TCP data connected flag'i ayarla - DOĞRULAMA İLE
                self.log("🔧 TCP flags set ediliyor...")
                
                # Double check: Gerçekten bağlı mı?
                if self.mavlink and self.mavlink.connected:
                    # TCP flags'i init time'da set ET - DATA THREAD DEĞİL!
                    with self.data_lock:
                        self.tcp_data['connected'] = True
                        self.live_imu['connected'] = True
                    self.log(f"✅ TCP flags INIT'te set edildi: tcp_data={self.tcp_data['connected']}, live_imu={self.live_imu['connected']}")
                    
                    # İlk IMU test - çalışıyor mu kontrol et
                    test_imu = self.mavlink.get_imu_data()
                    if test_imu:
                        self.log("✅ İlk IMU verisi alındı - sistem hazır!")
                        self.log(f"🔧 İlk IMU: ax={test_imu[0]:.2f}, ay={test_imu[1]:.2f}, az={test_imu[2]:.2f}")
                    else:
                        self.log("⚠️ IMU verisi alınamadı - data thread beklenecek")
                else:
                    self.log("❌ mavlink.connected False - TCP flags set edilmedi!")
                    with self.data_lock:
                        self.tcp_data['connected'] = False
                        self.live_imu['connected'] = False
                
            else:
                self.log("❌ TCP MAVLink bağlantısı başarısız - connect() False döndü!")
                self.log("🔧 Debug: socat çalışıyor ama MAVLink connect edemedi")
                self.log("💡 Çözüm: mavlink_handler.py connection string veya timeout sorunu")
                with self.data_lock:
                    self.tcp_data['connected'] = False
                    self.live_imu['connected'] = False
                
                # GPIO hata sinyali
                self.gpio_integration.set_connection_status_led(False, False)
                self.gpio_integration.beep_error()
                
        except Exception as e:
            self.log(f"❌ TCP MAVLink exception hatası: {e}")
            self.log(f"🔧 Exception türü: {type(e).__name__}")
            import traceback
            self.log(f"🔍 Traceback: {traceback.format_exc()}")
            with self.data_lock:
                self.tcp_data['connected'] = False
                self.live_imu['connected'] = False
            
            # GPIO hata sinyali
            self.gpio_integration.set_connection_status_led(False, False)
            self.gpio_integration.beep_error()
        
        # TCP veri thread başlat
        self.start_tcp_data_thread()
        
        # ARM durumunu senkronize et
        if self.mavlink and self.mavlink.connected:
            self.armed = self.mavlink.armed
            self.log(f"🔐 ARM durumu senkronize edildi: {self.armed}")
            # GPIO LED güncelle
            self.gpio_integration.set_connection_status_led(True, self.armed)
        
        self.log("✅ Sistem başlatma tamamlandı!")
        
        # Başlangıç durumu özeti - DÜZELTİLDİ!
        self.log(f"🔧 DEBUG: Final tcp_data['connected'] = {self.tcp_data['connected']}")
        self.log(f"🔧 DEBUG: Final live_imu['connected'] = {self.live_imu['connected']}")
        
        # Son durum kontrolü - DOGRU BİLGİ VER!
        if self.tcp_data['connected'] and self.mavlink and self.mavlink.connected:
            self.log("🎯 HAZIR: TCP bağlı, IMU aktif, kontroller hazır!")
            if self.gpio_integration.gpio_ready:
                self.log("🎯 GPIO: Buzzer/LED/Button aktif, acil durum butonu hazır!")
            if self.gpio_integration.depth_ready:
                self.log("🎯 D300: I2C derinlik sensörü aktif!")
        else:
            self.log("⚠️ KISMÎ: TCP bağlantısı BAŞARISIZ - offline mod aktiv")
            self.log(f"🔧 Detay: mavlink={self.mavlink is not None}, connected={getattr(self.mavlink, 'connected', False)}")
    
    def start_tcp_data_thread(self):
        """TCP veri thread'ini başlat - yüksek frekanslı"""
        self.data_thread_running = True
        self.data_thread = threading.Thread(target=self.tcp_data_loop, daemon=True)
        self.data_thread.start()
        self.log("🔄 TCP veri thread'i başlatıldı (50Hz)")
    
    def tcp_data_loop(self):
        """TCP veri döngüsü - 50Hz live data + GPIO/I2C integration"""
        last_update = time.time()
        update_counter = 0
        
        while self.data_thread_running and self.running:
            try:
                current_time = time.time()
                dt = current_time - last_update
                
                # 50Hz veri güncelleme
                if dt >= 0.02:  # 20ms = 50Hz
                    self.update_tcp_data()
                    self.update_gpio_data()  # YENİ - GPIO/I2C data update
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
    
    def update_gpio_data(self):
        """GPIO ve I2C veri güncelleme - YENİ!"""
        # D300 derinlik sensörü verisi güncelle
        if self.gpio_integration.depth_ready:
            depth_data = self.gpio_integration.get_depth_data()
            if depth_data:
                with self.data_lock:
                    self.depth_data.update(depth_data)
        
        # GPIO buton durumu kontrol (interrupt dışında polling de yap)
        if self.gpio_integration.gpio_ready:
            button_pressed = self.gpio_integration.read_button()
            # Button state değişikliği için log gerekmez, interrupt callback var
    
    def update_tcp_data(self):
        """TCP'den live veri güncelle - DEBUG EKLENDI VE DÜZELTİLDİ"""
        # DEBUG: Bağlantı durumunu kontrol et
        if not hasattr(self, 'tcp_debug_counter'):
            self.tcp_debug_counter = 0
        self.tcp_debug_counter += 1
        
        # Her 250 call'da bir debug (5 saniyede bir @ 50Hz)
        debug_this_call = (self.tcp_debug_counter % 250 == 0)
        
        if debug_this_call:
            mavlink_exists = self.mavlink is not None
            mavlink_connected = self.mavlink.connected if self.mavlink else False
            self.log(f"🔍 TCP Thread Debug #{self.tcp_debug_counter}: mavlink={mavlink_exists}, connected={mavlink_connected}")
        
        # TCP Thread connection check - EARLY RETURN İF NO MAVLINK
        if not self.mavlink:
            if debug_this_call:
                self.log("❌ TCP Thread: MAVLink object yok")
            with self.data_lock:
                self.tcp_data['connected'] = False
                self.live_imu['connected'] = False
            return
        
        # MAVLink connected check - DAHA TOLERANSLI VE DEBUG'LI
        if not hasattr(self.mavlink, 'connected') or not self.mavlink.connected:
            if debug_this_call:
                has_connected_attr = hasattr(self.mavlink, 'connected')
                connected_value = getattr(self.mavlink, 'connected', 'N/A')
                self.log(f"⚠️ TCP Thread: mavlink.connected check failed - has_attr={has_connected_attr}, value={connected_value}")
            
            # Bağlantı yoksa ama master varsa yeniden kontrol et
            if hasattr(self.mavlink, 'master') and self.mavlink.master:
                try:
                    # Heartbeat kontrol et - timeout olmadan
                    msg = self.mavlink.master.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
                    if msg:
                        # Heartbeat varsa bağlantı var demektir
                        self.mavlink.connected = True
                        if debug_this_call:
                            self.log("✅ TCP Thread: Heartbeat bulundu - connected=True set edildi")
                    else:
                        if debug_this_call:
                            self.log("⚠️ TCP Thread: Heartbeat bulunamadı")
                except Exception as e:
                    if debug_this_call:
                        self.log(f"❌ TCP Thread: Heartbeat check hatası: {e}")
        
        # Hala bağlantı yoksa False set et
        if not getattr(self.mavlink, 'connected', False):
            with self.data_lock:
                # Sadece durum değişirse log
                was_connected = self.tcp_data.get('connected', False)
                if was_connected:
                    self.log("⚠️ TCP Thread: MAVLink bağlantısı KESİLDİ")
                    # GPIO LED güncelle
                    self.gpio_integration.set_connection_status_led(False, False)
                self.tcp_data['connected'] = False
                self.live_imu['connected'] = False
            return
        
        # MAVLink bağlı - IMU verisi almaya çalış
        try:
            # IMU verilerini TCP'den direkt al - DETAILED DEBUG + ALTERNATIVE
            raw_imu = self.mavlink.get_imu_data()
            
            # Eğer ana IMU yoksa alternatif dene - YENİ!
            if not raw_imu and hasattr(self.mavlink, 'get_imu_data_alternative'):
                raw_imu = self.mavlink.get_imu_data_alternative()
                if raw_imu and debug_this_call:
                    self.log("🔧 TCP Thread: ALTERNATIVE IMU source kullanıldı (ATTITUDE)")
            
            if debug_this_call:
                if raw_imu:
                    self.log(f"🔧 TCP Thread: get_imu_data() SUCCESS - len={len(raw_imu)}")
                else:
                    self.log("⚠️ TCP Thread: get_imu_data() FAILED - None return (her iki method da)")
            
            if raw_imu and len(raw_imu) >= 6:
                with self.data_lock:
                    # Raw veriyi sakla
                    self.tcp_data['imu_raw'] = raw_imu
                    
                    # Connection status update - sadece durum değişikliğinde
                    was_connected = self.tcp_data.get('connected', False)
                    self.tcp_data['connected'] = True
                    self.tcp_data['last_packet'] = time.time()
                    
                    if not was_connected:
                        self.log("✅ TCP Thread: IMU data akışı BAŞLADI!")
                        # GPIO LED güncelle
                        self.gpio_integration.set_connection_status_led(True, self.armed)
                    
                    # Live IMU hesapla - YAW dahil
                    self.calculate_live_orientation(raw_imu)
                    
                    # Bağlantı durumu güncelle
                    self.live_imu['connected'] = True
                    self.live_imu['last_update'] = time.time()
                    
                    # Data rate hesapla
                    if hasattr(self, 'last_data_time'):
                        dt = time.time() - self.last_data_time
                        if dt > 0:
                            current_rate = 1.0 / dt
                            # Smooth rate calculation
                            self.live_imu['update_rate'] = int(0.9 * self.live_imu['update_rate'] + 0.1 * current_rate)
                    else:
                        self.live_imu['update_rate'] = 1
                    
                    self.last_data_time = time.time()
                    
                    # Debug: Her 100 güncelleme de bir log (çok fazla log olmasın)
                    if not hasattr(self, 'data_debug_counter'):
                        self.data_debug_counter = 0
                    self.data_debug_counter += 1
                    
                    if self.data_debug_counter % 250 == 0:  # 5 saniyede bir
                        accel_x, accel_y, accel_z = raw_imu[0], raw_imu[1], raw_imu[2]
                        gyro_x, gyro_y, gyro_z = raw_imu[3], raw_imu[4], raw_imu[5]
                        self.log(f"🔧 TCP Data OK: Rate={self.live_imu['update_rate']}Hz, IMU=({accel_x:.2f},{accel_y:.2f},{accel_z:.2f})")
                        
            else:
                # IMU verisi gelmiyorsa durum güncelle
                with self.data_lock:
                    last_packet_time = self.tcp_data.get('last_packet', 0)
                    data_age = time.time() - last_packet_time
                    
                    if data_age > 2.0:  # 2 saniye timeout
                        was_connected = self.tcp_data.get('connected', False)
                        if was_connected:
                            self.log(f"⚠️ TCP Thread: IMU data timeout ({data_age:.1f}s)")
                            # GPIO LED güncelle
                            self.gpio_integration.set_connection_status_led(False, False)
                        
                        self.tcp_data['connected'] = False
                        self.live_imu['connected'] = False
                        self.live_imu['update_rate'] = 0
                        
        except Exception as e:
            with self.data_lock:
                was_connected = self.tcp_data.get('connected', False)
                if was_connected:
                    self.log(f"❌ TCP Thread: IMU data exception - bağlantı kesilecek")
                    # GPIO LED güncelle
                    self.gpio_integration.set_connection_status_led(False, False)
                
                self.tcp_data['connected'] = False
                self.live_imu['connected'] = False
                self.live_imu['update_rate'] = 0
            
            # Error logging (her hata için değil, sadece yeni hatalar için)
            if not hasattr(self, 'last_tcp_error') or time.time() - self.last_tcp_error > 5.0:
                self.log(f"❌ TCP data exception: {e}")
                self.last_tcp_error = time.time()
    
    def calculate_live_orientation(self, raw_imu):
        """Live roll/pitch/yaw hesapla - BASİTLEŞTİRİLDİ"""
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
            
            # YAW integration - BASİTLEŞTİRİLDİ VE DÜZELTİLDİ
            dt = 0.02  # 50Hz
            gyro_z_deg = math.degrees(gyro_z)  # rad/s to deg/s
            
            # İlk kez çalışıyorsa YAW'ı 0'dan başlat
            if not hasattr(self, 'yaw_initialized'):
                self.live_imu['yaw'] = 0.0
                self.yaw_initialized = True
                self.log(f"🎯 YAW sistemi başlatıldı: 0.0°")
            
            # GYRO THRESHOLD DÜŞÜRÜLDÜ - Her gyro değeri işlenir
            old_yaw = self.live_imu['yaw']
            
            # YAW her zaman güncellenir (threshold kaldırıldı)
            yaw_change = gyro_z_deg * dt
            yaw_deg = old_yaw + yaw_change
            
            # Yaw normalize (-180 to +180)
            while yaw_deg > 180:
                yaw_deg -= 360
            while yaw_deg < -180:
                yaw_deg += 360
            
            self.live_imu['yaw'] = yaw_deg
            
            # YAW Debug - sürekli görünür
            if not hasattr(self, 'yaw_debug_counter'):
                self.yaw_debug_counter = 0
            self.yaw_debug_counter += 1
            
            # Her 10 güncelleme de bir debug log
            if self.yaw_debug_counter % 10 == 0:
                self.log(f"🎯 YAW: {old_yaw:.1f}° → {yaw_deg:.1f}° (Δ{yaw_change:.2f}°, gyro_z={gyro_z_deg:.3f}°/s)")
            
            # Live IMU güncelle
            self.live_imu['roll'] = roll_deg
            self.live_imu['pitch'] = pitch_deg
            
        except Exception as e:
            self.log(f"❌ YAW hesaplama hatası: {e}")
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
        title = "🚀 TEKNOFEST ROV - Full GPIO/I2C Terminal [TCP:127.0.0.1:5777] 🚀"
        self.stdscr.addstr(0, max(0, (self.width - len(title)) // 2), title, curses.color_pair(4) | curses.A_BOLD)
        
        # Durum satırı - GPIO/I2C durumu eklendi
        status_line = 1
        tcp_status = "✅ TCP BAĞLI" if self.tcp_data['connected'] else "❌ TCP BAĞLI DEĞİL"
        arm_status = "🔴 ARMED" if self.armed else "🟢 DISARMED"
        gpio_status = "🔧 GPIO/I2C" if (self.gpio_integration.gpio_ready or self.gpio_integration.depth_ready) else "❌ NO GPIO"
        menu_status = f"📋 {self.current_menu.upper()}"
        
        self.stdscr.addstr(status_line, 2, f"TCP: {tcp_status}", curses.color_pair(1 if self.tcp_data['connected'] else 2))
        self.stdscr.addstr(status_line, 25, f"Durum: {arm_status}", curses.color_pair(2 if self.armed else 1))
        self.stdscr.addstr(status_line, 45, f"HW: {gpio_status}", curses.color_pair(1 if (self.gpio_integration.gpio_ready or self.gpio_integration.depth_ready) else 2))
        self.stdscr.addstr(status_line, 65, f"Kontrol: {self.control_mode}", curses.color_pair(5))
        self.stdscr.addstr(status_line, 85, f"Menü: {menu_status}", curses.color_pair(4))
        
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
                
                # Yaw - sarı/kırmızı renk kodlu (daha belirgin)
                yaw_val = self.live_imu['yaw']
                yaw_color = curses.color_pair(3) | curses.A_BOLD  # Sarı ve kalın
                if abs(yaw_val) > 90:  # Büyük açılar kırmızı
                    yaw_color = curses.color_pair(2) | curses.A_BOLD
                self.stdscr.addstr(start_row + 4, 4, f"YAW:   {yaw_val:+7.1f}°", yaw_color)
                
                # Görsel çubuklar (0-30-60-90 derece)
                self.draw_angle_bar(start_row + 2, 25, roll_val, "Roll")
                self.draw_angle_bar(start_row + 3, 25, pitch_val, "Pitch")
                self.draw_angle_bar(start_row + 4, 25, yaw_val, "Yaw")
                
            else:
                self.stdscr.addstr(start_row + 2, 4, "ROLL:  ---- °", curses.color_pair(2))
                self.stdscr.addstr(start_row + 3, 4, "PITCH: ---- °", curses.color_pair(2))
                self.stdscr.addstr(start_row + 4, 4, "YAW:   ---- °", curses.color_pair(2))
                self.stdscr.addstr(start_row + 6, 4, "❌ TCP bağlantısı yok - IMU verisi alınamıyor", curses.color_pair(2))
    
    def draw_depth_sensor_display(self):
        """D300 Derinlik sensörü görüntüleme - YENİ!"""
        start_row = 9
        
        with self.data_lock:
            # Başlık
            self.stdscr.addstr(start_row, 55, "🌊 D300 DEPTH SENSOR (I2C)", curses.color_pair(4) | curses.A_BOLD)
            
            if self.gpio_integration.depth_ready and self.depth_data['connected']:
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
                if self.gpio_integration.depth_ready:
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
        if self.gpio_integration.gpio_ready:
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
            "T: Test Scripts",    "G: GPIO Test",       "Z: TCP Debug",
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
        gpio_status = "✅ AKTIF" if self.gpio_integration.gpio_ready else "❌ DEVRE DIŞI"
        i2c_status = "✅ AKTIF" if self.gpio_integration.depth_ready else "❌ DEVRE DIŞI"
        
        self.stdscr.addstr(start_row + 1, 4, f"GPIO Durum: {gpio_status}", curses.color_pair(1 if self.gpio_integration.gpio_ready else 2))
        self.stdscr.addstr(start_row + 2, 4, f"I2C D300:   {i2c_status}", curses.color_pair(1 if self.gpio_integration.depth_ready else 2))
        
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
        
        # TCP CONNECTION DEBUG - YENİ! ⭐
        elif key == ord('z') or key == ord('Z'):
            self.tcp_connection_debug()
        
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
    
    def tcp_connection_debug(self):
        """TCP bağlantı debug - INSTANT TEST"""
        self.log("🔍 TCP CONNECTION DEBUG BAŞLATILIYOR...")
        
        # MAVLink durumu
        if self.mavlink:
            self.log(f"📡 MAVLink Object: ✅ EXISTS")
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
            self.log("📡 MAVLink Object: ❌ NOT EXISTS")
        
        # Thread durumu
        with self.data_lock:
            self.log(f"🔄 TCP Data Connected: {self.tcp_data['connected']}")
            self.log(f"🔄 Live IMU Connected: {self.live_imu['connected']}")
            self.log(f"🔄 IMU Update Rate: {self.live_imu['update_rate']}Hz")
            
            last_packet = self.tcp_data.get('last_packet', 0)
            if last_packet > 0:
                data_age = time.time() - last_packet
                self.log(f"🔄 Son veri: {data_age:.1f} saniye önce")
            else:
                self.log("🔄 Hiç veri alınmamış")
        
        # TCP port kontrolü - shell command
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
        
        self.log("🔍 TCP CONNECTION DEBUG TAMAMLANDI")
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
            if self.gpio_integration.depth_ready:
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
            if self.test_manager.run_script(script_id, self.test_script_callback):
                script_info = self.test_manager.available_scripts.get(script_id, {})
                self.log(f"🔧 Test başlatıldı: {script_info.get('name', 'Unknown')}")
                self.gpio_integration.beep_success()
        
        # Script durdurma
        elif key == ord('s') or key == ord('S'):
            if self.test_manager.running_script:
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
                self.log("⚠️ TCP MAVLink disconnected - YAW command ignored!")
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
            self.log("❌ TCP MAVLink bağlantısı yok!")
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