#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Web GUI
BlueOS entegrasyonu için Flask tabanlı arayüz
"""

import os
import sys
import json
import time
import threading
from flask import Flask, render_template, jsonify, request, send_from_directory
from flask_socketio import SocketIO, emit
import logging

# Local imports
from mavlink_handler import MAVLinkHandler
from navigation_engine import NavigationEngine
from vibration_monitor import VibrationMonitor
from control_module import ControlModule
from gpio_controller import GPIOController
from depth_sensor import D300DepthSensor

# Logging setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'teknofest_rov_2025'
socketio = SocketIO(app, cors_allowed_origins="*")

class ROVWebController:
    def __init__(self):
        """Web kontrolcü başlat"""
        # Sistem bileşenleri
        self.mavlink_handler = None
        self.navigation_engine = None
        self.vibration_monitor = None
        self.control_module = None
        self.gpio_controller = None
        self.depth_sensor = None
        
        # Durum değişkenleri
        self.connection_status = False
        self.armed_status = False
        self.current_mode = "raw"
        self.navigation_mode = "imu_only"
        
        # Data containers
        self.telemetry_data = {}
        self.system_status = {}
        
        # Thread güvenliği
        self.data_lock = threading.Lock()
        
        # Telemetry timer
        self.telemetry_timer = None
        self.start_telemetry_updates()
        
    def initialize_systems(self):
        """Sistem bileşenlerini başlat"""
        try:
            logger.info("Sistem bileşenleri başlatılıyor...")
            
            # MAVLink handler
            self.mavlink_handler = MAVLinkHandler()
            
            # MAVLink bağlantısını kur
            logger.info("MAVLink bağlantısı kuruluyor...")
            self.connection_status = self.mavlink_handler.connect()
            
            if self.connection_status:
                logger.info("✅ MAVLink bağlantısı kuruldu!")
            else:
                logger.warning("⚠️ MAVLink bağlantısı kurulamadı, offline modda çalışılıyor")
            
            # Navigation engine
            self.navigation_engine = NavigationEngine(self.mavlink_handler)
            
            # Vibration monitor
            self.vibration_monitor = VibrationMonitor(self.mavlink_handler)
            
            # Control module (manuel servo kontrolü)
            try:
                self.control_module = ControlModule(self.mavlink_handler, self.navigation_engine)
                logger.info("✅ Control module başlatıldı")
            except Exception as e:
                logger.warning(f"⚠️ Control module başlatılamadı: {e}")
            
            # GPIO controller
            try:
                # Load config for GPIO
                with open("config/hardware_config.json", 'r') as f:
                    config = json.load(f)
                self.gpio_controller = GPIOController(config)
                logger.info("✅ GPIO controller başlatıldı")
            except Exception as e:
                logger.warning(f"⚠️ GPIO controller başlatılamadı: {e}")
            
            # Depth sensor
            try:
                self.depth_sensor = D300DepthSensor()
                if self.depth_sensor.connect():
                    logger.info("✅ Derinlik sensörü bağlandı")
                else:
                    logger.warning("⚠️ Derinlik sensörü bağlanamadı")
            except Exception as e:
                logger.warning(f"⚠️ Derinlik sensörü hatası: {e}")
            
            logger.info("✅ Sistem bileşenleri başarıyla yüklendi!")
            return True
            
        except Exception as e:
            logger.error(f"❌ Sistem başlatma hatası: {e}")
            return False
    
    def start_telemetry_updates(self):
        """Telemetry güncellemelerini başlat"""
        def telemetry_worker():
            while True:
                try:
                    self.update_telemetry()
                    time.sleep(0.1)  # 10Hz
                except Exception as e:
                    logger.error(f"Telemetry error: {e}")
                    time.sleep(1.0)
        
        telemetry_thread = threading.Thread(target=telemetry_worker, daemon=True)
        telemetry_thread.start()
    
    def update_telemetry(self):
        """Telemetry verilerini güncelle"""
        if not self.mavlink_handler:
            return
        
        try:
            # IMU data
            imu_data = self.mavlink_handler.get_imu_data()
            
            # GPS data  
            gps_data = self.mavlink_handler.get_gps_data()
            
            # System status
            system_data = self.mavlink_handler.get_system_status()
            
            # Vibration data
            vibration_level = 0
            vibration_category = "Low"
            if self.vibration_monitor:
                vibration_level = self.vibration_monitor.get_vibration_level()
                vibration_category = self.vibration_monitor.get_vibration_category()
            
            # Depth sensor data
            depth_data = {'depth': 0.0, 'temperature': 20.0, 'pressure': 1013.25}
            if self.depth_sensor:
                try:
                    depth_reading = self.depth_sensor.read_depth()
                    if depth_reading:
                        depth_data = depth_reading
                except Exception as e:
                    logger.debug(f"Depth sensor read error: {e}")
            
            with self.data_lock:
                self.telemetry_data = {
                    'timestamp': time.time(),
                    'imu': {
                        'roll': imu_data[1] if imu_data else 0.0,
                        'pitch': imu_data[2] if imu_data else 0.0,
                        'yaw': imu_data[0] if imu_data else 0.0,
                        'connected': bool(imu_data)
                    },
                    'gps': {
                        'latitude': gps_data[0] if gps_data else 0.0,
                        'longitude': gps_data[1] if gps_data else 0.0,
                        'satellites': gps_data[3] if gps_data else 0,
                        'connected': bool(gps_data)
                    },
                    'system': {
                        'voltage': system_data.get('voltage', 0.0) if system_data else 0.0,
                        'current': system_data.get('current', 0.0) if system_data else 0.0,
                        'armed': self.armed_status,
                        'mode': self.current_mode
                    },
                    'vibration': {
                        'level': vibration_level,
                        'category': vibration_category
                    },
                    'depth': {
                        'depth': depth_data.get('depth', 0.0),
                        'temperature': depth_data.get('temperature', 20.0),
                        'pressure': depth_data.get('pressure', 1013.25),
                        'connected': self.depth_sensor is not None
                    }
                }
                
                self.system_status = {
                    'connected': self.connection_status,
                    'armed': self.armed_status,
                    'control_mode': self.current_mode,
                    'navigation_mode': self.navigation_mode
                }
            
            # WebSocket'e gönder
            socketio.emit('telemetry_update', self.telemetry_data)
            
        except Exception as e:
            logger.error(f"Telemetry update error: {e}")
    
    def toggle_connection(self):
        """MAVLink bağlantısını aç/kapat"""
        if not self.mavlink_handler:
            return False
        
        try:
            if self.connection_status:
                # Disconnect
                self.mavlink_handler.disconnect()
                self.connection_status = False
                self.armed_status = False
                logger.info("MAVLink bağlantısı kapatıldı")
            else:
                # Connect
                success = self.mavlink_handler.connect()
                self.connection_status = success
                if success:
                    logger.info("MAVLink bağlantısı kuruldu")
                else:
                    logger.error("MAVLink bağlantı hatası")
            
            return self.connection_status
            
        except Exception as e:
            logger.error(f"Connection toggle error: {e}")
            return False
    
    def toggle_arm(self):
        """ARM/DISARM durumunu değiştir"""
        if not self.mavlink_handler or not self.connection_status:
            return False
        
        try:
            if self.armed_status:
                success = self.mavlink_handler.disarm()
                if success:
                    self.armed_status = False
                    logger.info("Sistem DISARM edildi")
            else:
                success = self.mavlink_handler.arm()
                if success:
                    self.armed_status = True
                    logger.info("Sistem ARM edildi")
            
            return self.armed_status
            
        except Exception as e:
            logger.error(f"ARM toggle error: {e}")
            return False
    
    def set_control_mode(self, mode):
        """Kontrol modunu ayarla"""
        if mode in ["raw", "pid"]:
            self.current_mode = mode
            if self.mavlink_handler:
                self.mavlink_handler.set_control_mode(mode)
            logger.info(f"Kontrol modu: {mode}")
            return True
        return False
    
    def set_navigation_mode(self, mode):
        """Navigation modunu ayarla"""
        if mode in ["gps_only", "imu_only", "hybrid"]:
            self.navigation_mode = mode
            if self.navigation_engine:
                self.navigation_engine.set_navigation_mode(mode)
            logger.info(f"Navigation modu: {mode}")
            return True
        return False
    
    def execute_movement_command(self, command_type, parameter):
        """Hareket komutu çalıştır"""
        if not self.navigation_engine or not self.armed_status:
            return False
        
        try:
            success = self.navigation_engine.start_movement_mission(
                command_type, parameter, self.current_mode
            )
            logger.info(f"Movement command: {command_type} {parameter} -> {success}")
            return success
            
        except Exception as e:
            logger.error(f"Movement command error: {e}")
            return False
    
    def emergency_stop(self):
        """Acil durum durdur"""
        if self.mavlink_handler:
            self.mavlink_handler.emergency_stop()
        logger.warning("🚨 EMERGENCY STOP!")
        return True
    
    def run_script(self, script_id):
        """Script çalıştır"""
        try:
            import subprocess
            import os
            
            script_map = {
                'servo_calibration': 'scripts/servo_calibration.py',
                'motor_test': 'scripts/motor_test.py', 
                'imu_calibration': 'scripts/imu_calibration.py',
                'system_check': 'scripts/system_check.py',
                'emergency_test': 'scripts/emergency_stop.py'
            }
            
            if script_id not in script_map:
                logger.error(f"Script bulunamadı: {script_id}")
                return False
            
            script_path = script_map[script_id]
            if not os.path.exists(script_path):
                logger.error(f"Script dosyası yok: {script_path}")
                return False
            
            # Script'i arka planda çalıştır
            logger.info(f"Script çalıştırılıyor: {script_id}")
            process = subprocess.Popen(['python3', script_path], 
                                     stdout=subprocess.PIPE, 
                                     stderr=subprocess.PIPE)
            
            return True
            
        except Exception as e:
            logger.error(f"Script çalıştırma hatası: {e}")
            return False
    
    def control_servo(self, servo_id, position):
        """Manuel servo kontrolü"""
        try:
            if not self.control_module:
                logger.warning("Control module mevcut değil")
                return False
            
            # Servo pozisyonunu ayarla
            success = self.control_module.set_servo_position(servo_id, position)
            logger.info(f"Servo {servo_id} pozisyon {position} -> {success}")
            return success
            
        except Exception as e:
            logger.error(f"Servo kontrol hatası: {e}")
            return False
    
    def control_gpio(self, pin, state):
        """GPIO kontrolü"""
        try:
            if not self.gpio_controller:
                logger.warning("GPIO controller mevcut değil")
                return False
            
            # GPIO durumunu ayarla
            success = self.gpio_controller.set_pin_state(pin, state)
            logger.info(f"GPIO {pin} durum {state} -> {success}")
            return success
            
        except Exception as e:
            logger.error(f"GPIO kontrol hatası: {e}")
            return False
    
    def get_telemetry_data(self):
        """Telemetry verilerini döndür"""
        with self.data_lock:
            return self.telemetry_data.copy()
    
    def get_system_status(self):
        """Sistem durumunu döndür"""
        with self.data_lock:
            return self.system_status.copy()

# Global controller instance
rov_controller = ROVWebController()

# Web Routes
@app.route('/')
def index():
    """Ana sayfa"""
    return render_template('index.html')

@app.route('/api/status')
def api_status():
    """Sistem durumu API"""
    return jsonify(rov_controller.get_system_status())

@app.route('/api/telemetry')
def api_telemetry():
    """Telemetry API"""
    return jsonify(rov_controller.get_telemetry_data())

@app.route('/api/connect', methods=['POST'])
def api_connect():
    """Bağlantı kontrolü"""
    success = rov_controller.toggle_connection()
    return jsonify({'success': success, 'connected': rov_controller.connection_status})

@app.route('/api/arm', methods=['POST'])
def api_arm():
    """ARM/DISARM kontrolü"""
    success = rov_controller.toggle_arm()
    return jsonify({'success': success, 'armed': rov_controller.armed_status})

@app.route('/api/control_mode', methods=['POST'])
def api_control_mode():
    """Kontrol modu ayarla"""
    data = request.get_json()
    mode = data.get('mode', 'raw')
    success = rov_controller.set_control_mode(mode)
    return jsonify({'success': success, 'mode': mode})

@app.route('/api/navigation_mode', methods=['POST'])
def api_navigation_mode():
    """Navigation modu ayarla"""
    data = request.get_json()
    mode = data.get('mode', 'imu_only')
    success = rov_controller.set_navigation_mode(mode)
    return jsonify({'success': success, 'mode': mode})

@app.route('/api/movement', methods=['POST'])
def api_movement():
    """Hareket komutu"""
    data = request.get_json()
    command_type = data.get('command')
    parameter = data.get('parameter', 0)
    
    success = rov_controller.execute_movement_command(command_type, parameter)
    return jsonify({'success': success})

@app.route('/api/emergency', methods=['POST'])
def api_emergency():
    """Acil durum"""
    success = rov_controller.emergency_stop()
    return jsonify({'success': success})

@app.route('/api/scripts', methods=['GET'])
def api_get_scripts():
    """Mevcut scriptleri listele"""
    scripts = [
        {'id': 'servo_calibration', 'name': 'Servo Kalibrasyonu', 'description': 'Servo motorlarını kalibre et'},
        {'id': 'motor_test', 'name': 'Motor Testi', 'description': 'Ana motor testini çalıştır'},
        {'id': 'imu_calibration', 'name': 'IMU Kalibrasyonu', 'description': 'IMU sensörünü kalibre et'},
        {'id': 'system_check', 'name': 'Sistem Kontrolü', 'description': 'Tam sistem kontrolü yap'},
        {'id': 'emergency_test', 'name': 'Acil Durum Testi', 'description': 'Emergency stop sistemini test et'}
    ]
    return jsonify(scripts)

@app.route('/api/scripts/<script_id>/run', methods=['POST'])
def api_run_script(script_id):
    """Script çalıştır"""
    success = rov_controller.run_script(script_id)
    return jsonify({'success': success})

@app.route('/api/servo/control', methods=['POST'])
def api_servo_control():
    """Manuel servo kontrolü"""
    data = request.get_json()
    servo_id = data.get('servo_id')
    position = data.get('position', 90)
    
    success = rov_controller.control_servo(servo_id, position)
    return jsonify({'success': success})

@app.route('/api/gpio/control', methods=['POST'])
def api_gpio_control():
    """GPIO kontrolü"""
    data = request.get_json()
    pin = data.get('pin')
    state = data.get('state')
    
    success = rov_controller.control_gpio(pin, state)
    return jsonify({'success': success})

# WebSocket Events
@socketio.on('connect')
def handle_connect():
    """WebSocket bağlantısı"""
    logger.info('Web client connected')
    emit('system_status', rov_controller.get_system_status())

@socketio.on('disconnect')
def handle_disconnect():
    """WebSocket bağlantı kopması"""
    logger.info('Web client disconnected')

@socketio.on('control_command')
def handle_control_command(data):
    """Real-time kontrol komutu"""
    command = data.get('command')
    value = data.get('value', 0)
    
    logger.info(f"Control command: {command} = {value}")
    
    # Bu komutları mavlink_handler'a gönder
    if rov_controller.mavlink_handler and rov_controller.armed_status:
        # Real-time servo kontrolü burada implement edilecek
        pass

if __name__ == '__main__':
    logger.info("🚀 TEKNOFEST ROV Web GUI başlatılıyor...")
    
    # Sistem bileşenlerini başlat
    if rov_controller.initialize_systems():
        logger.info("Web server starting on http://0.0.0.0:5000")
        socketio.run(app, host='0.0.0.0', port=5000, debug=False)
    else:
        logger.error("Sistem başlatılamadı!")
        sys.exit(1) 