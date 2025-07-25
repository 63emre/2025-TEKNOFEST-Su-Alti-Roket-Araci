#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Ana GUI
PyQt5 Ana Arayüz Sistemi
"""

import sys
import os
import time
import json
import threading
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import pyqtgraph as pg

# Local imports
from mavlink_handler import MAVLinkHandler
from navigation_engine import NavigationEngine
from vibration_monitor import VibrationMonitor
from control_module import ControlModule
from gpio_controller import GPIOController
from depth_sensor import D300DepthSensor
from sensor_manager import SensorManager

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("TEKNOFEST 2025 - Su Altı ROV Kontrol Sistemi")
        self.setGeometry(100, 100, 1200, 800)
        
        # Sistem bileşenleri
        self.mavlink_handler = None
        self.navigation_engine = None
        self.vibration_monitor = None
        self.control_module = None
        self.gpio_controller = None
        self.depth_sensor = None
        self.sensor_manager = None
        
        # GUI durumu
        self.connection_status = False
        self.armed_status = False
        
        # Timers
        self.telemetry_timer = QTimer()
        self.telemetry_timer.timeout.connect(self.update_telemetry)
        
        # Setup GUI
        self.setup_ui()
        self.setup_system()
        
        # Status bar
        self.statusBar().showMessage("Sistem başlatılıyor...")
    
    def setup_ui(self):
        """Ana UI kurulumu"""
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Ana layout
        main_layout = QHBoxLayout(central_widget)
        
        # Sol panel - Kontrol ayarları
        left_panel = self.create_control_panel()
        main_layout.addWidget(left_panel, 1)
        
        # Orta panel - Telemetry ve grafikler
        center_panel = self.create_telemetry_panel()
        main_layout.addWidget(center_panel, 2)
        
        # Sağ panel - Scripts ve system
        right_panel = self.create_system_panel()
        main_layout.addWidget(right_panel, 1)
        
        # Alt panel - Terminal ve vibration
        self.create_bottom_panel()
        
        # Menu bar
        self.create_menu_bar()
    
    def create_control_panel(self):
        """Sol kontrol paneli"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Connection section
        conn_group = QGroupBox("Bağlantı")
        conn_layout = QVBoxLayout(conn_group)
        
        self.connect_btn = QPushButton("MAVLink Bağlan")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        
        self.arm_btn = QPushButton("ARM / DISARM")
        self.arm_btn.clicked.connect(self.toggle_arm)
        self.arm_btn.setEnabled(False)
        conn_layout.addWidget(self.arm_btn)
        
        layout.addWidget(conn_group)
        
        # Control mode section
        control_group = QGroupBox("Kontrol Modu")
        control_layout = QVBoxLayout(control_group)
        
        self.raw_control_cb = QCheckBox("RAW PWM Control (Titreşimsiz)")
        self.pid_control_cb = QCheckBox("PID Control (Filtreli)")
        self.raw_control_cb.setChecked(True)  # Default
        
        # Make them exclusive
        self.raw_control_cb.toggled.connect(self.on_control_mode_changed)
        self.pid_control_cb.toggled.connect(self.on_control_mode_changed)
        
        control_layout.addWidget(self.raw_control_cb)
        control_layout.addWidget(self.pid_control_cb)
        
        layout.addWidget(control_group)
        
        # Navigation mode section
        nav_group = QGroupBox("Navigation Modu")
        nav_layout = QVBoxLayout(nav_group)
        
        self.gps_nav_cb = QCheckBox("GPS Navigation")
        self.imu_nav_cb = QCheckBox("IMU Dead Reckoning")
        self.hybrid_nav_cb = QCheckBox("GPS + IMU Hybrid")
        self.imu_nav_cb.setChecked(True)  # Default
        
        # Make them exclusive
        self.gps_nav_cb.toggled.connect(self.on_navigation_mode_changed)
        self.imu_nav_cb.toggled.connect(self.on_navigation_mode_changed)
        self.hybrid_nav_cb.toggled.connect(self.on_navigation_mode_changed)
        
        nav_layout.addWidget(self.gps_nav_cb)
        nav_layout.addWidget(self.imu_nav_cb)
        nav_layout.addWidget(self.hybrid_nav_cb)
        
        layout.addWidget(nav_group)
        
        # Movement commands section
        movement_group = QGroupBox("Hareket Komutları")
        movement_layout = QGridLayout(movement_group)
        
        # T - Forward
        forward_btn = QPushButton("T - İleri Git")
        forward_btn.clicked.connect(lambda: self.show_movement_dialog('FORWARD'))
        movement_layout.addWidget(forward_btn, 0, 0)
        
        # Y - Yaw
        yaw_btn = QPushButton("Y - Yaw Dönme")
        yaw_btn.clicked.connect(lambda: self.show_movement_dialog('YAW_ROTATION'))
        movement_layout.addWidget(yaw_btn, 0, 1)
        
        # U - Ascend
        ascend_btn = QPushButton("U - Yukarı Çık")
        ascend_btn.clicked.connect(lambda: self.show_movement_dialog('ASCEND'))
        movement_layout.addWidget(ascend_btn, 1, 0)
        
        # G - Left
        left_btn = QPushButton("G - Sol Git")
        left_btn.clicked.connect(lambda: self.show_movement_dialog('STRAFE_LEFT'))
        movement_layout.addWidget(left_btn, 1, 1)
        
        # H - Right
        right_btn = QPushButton("H - Sağ Git")
        right_btn.clicked.connect(lambda: self.show_movement_dialog('STRAFE_RIGHT'))
        movement_layout.addWidget(right_btn, 2, 0)
        
        # Stop mission
        stop_btn = QPushButton("Görevi Durdur")
        stop_btn.clicked.connect(self.stop_mission)
        stop_btn.setStyleSheet("background-color: red; color: white;")
        movement_layout.addWidget(stop_btn, 2, 1)
        
        layout.addWidget(movement_group)
        
        # Real-time control button
        realtime_btn = QPushButton("Real-time Kontrol Penceresi")
        realtime_btn.clicked.connect(self.show_realtime_control)
        layout.addWidget(realtime_btn)
        
        # Emergency button
        emergency_btn = QPushButton("ACİL DURUM!")
        emergency_btn.clicked.connect(self.emergency_stop)
        emergency_btn.setStyleSheet("background-color: darkred; color: white; font-weight: bold;")
        layout.addWidget(emergency_btn)
        
        layout.addStretch()
        
        return panel
    
    def create_telemetry_panel(self):
        """Orta telemetry paneli"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Status indicators
        status_layout = QHBoxLayout()
        
        self.mavlink_status = QLabel("MAVLink: ❌")
        self.mavlink_status.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.mavlink_status)
        
        self.armed_status_label = QLabel("DISARMED")
        self.armed_status_label.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.armed_status_label)
        
        self.mission_status = QLabel("Görev: Yok")
        status_layout.addWidget(self.mission_status)
        
        status_layout.addStretch()
        layout.addLayout(status_layout)
        
        # Telemetry tabs
        tab_widget = QTabWidget()
        
        # IMU Graphs tab
        imu_tab = QWidget()
        imu_layout = QVBoxLayout(imu_tab)
        
        # YAW, PITCH, ROLL graphs
        self.yaw_graph = pg.PlotWidget(title="YAW (°)")
        self.pitch_graph = pg.PlotWidget(title="PITCH (°)")
        self.roll_graph = pg.PlotWidget(title="ROLL (°)")
        
        # Graph data
        self.graph_data_length = 100
        self.yaw_data = [0] * self.graph_data_length
        self.pitch_data = [0] * self.graph_data_length
        self.roll_data = [0] * self.graph_data_length
        
        # Graph lines
        self.yaw_line = self.yaw_graph.plot(self.yaw_data, pen='r')
        self.pitch_line = self.pitch_graph.plot(self.pitch_data, pen='g')
        self.roll_line = self.roll_graph.plot(self.roll_data, pen='b')
        
        imu_layout.addWidget(self.yaw_graph)
        imu_layout.addWidget(self.pitch_graph)
        imu_layout.addWidget(self.roll_graph)
        
        tab_widget.addTab(imu_tab, "IMU Grafikleri")
        
        # System info tab
        info_tab = QWidget()
        info_layout = QVBoxLayout(info_tab)
        
        # Telemetry display
        self.telemetry_text = QTextEdit()
        self.telemetry_text.setReadOnly(True)
        self.telemetry_text.setMaximumHeight(200)
        info_layout.addWidget(QLabel("Sistem Telemetry:"))
        info_layout.addWidget(self.telemetry_text)
        
        # GPS info (if available)
        self.gps_info = QLabel("GPS: Veri yok")
        info_layout.addWidget(self.gps_info)
        
        # Vibration monitor
        vibration_group = QGroupBox("Titreşim Monitörü")
        vib_layout = QVBoxLayout(vibration_group)
        
        self.vibration_bar = QProgressBar()
        self.vibration_bar.setRange(0, 100)
        self.vibration_bar.setValue(0)
        vib_layout.addWidget(QLabel("Titreşim Seviyesi:"))
        vib_layout.addWidget(self.vibration_bar)
        
        self.vibration_info = QLabel("Kategori: Düşük | Frekans: -- Hz")
        vib_layout.addWidget(self.vibration_info)
        
        info_layout.addWidget(vibration_group)
        
        tab_widget.addTab(info_tab, "Sistem Bilgisi")
        
        layout.addWidget(tab_widget)
        
        return panel
    
    def create_system_panel(self):
        """Sağ sistem paneli"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Scripts section
        scripts_group = QGroupBox("Uygulama Scriptleri")
        scripts_layout = QVBoxLayout(scripts_group)
        
        # Script buttons
        script_buttons = [
            ("Servo Kalibrasyonu", "servo_calibration"),
            ("Motor Testi", "motor_test"),
            ("IMU Kalibrasyonu", "imu_calibration"),
            ("Sistem Kontrolü", "system_check"),
            ("Acil Durum Testi", "emergency_test")
        ]
        
        for name, script in script_buttons:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, s=script: self.run_script(s))
            scripts_layout.addWidget(btn)
        
        layout.addWidget(scripts_group)
        
        # Configuration section
        config_group = QGroupBox("Konfigürasyon")
        config_layout = QVBoxLayout(config_group)
        
        config_btn = QPushButton("Pin Konfigürasyonu")
        config_btn.clicked.connect(self.show_pin_config)
        config_layout.addWidget(config_btn)
        
        reload_btn = QPushButton("Ayarları Yeniden Yükle")
        reload_btn.clicked.connect(self.reload_settings)
        config_layout.addWidget(reload_btn)
        
        layout.addWidget(config_group)
        
        # Comparison section
        comparison_group = QGroupBox("Titreşim Karşılaştırması")
        comp_layout = QVBoxLayout(comparison_group)
        
        self.comparison_btn = QPushButton("RAW vs PID Karşılaştır")
        self.comparison_btn.clicked.connect(self.show_vibration_comparison)
        comp_layout.addWidget(self.comparison_btn)
        
        layout.addWidget(comparison_group)
        
        layout.addStretch()
        
        return panel
    
    def create_bottom_panel(self):
        """Alt panel - Terminal ve log"""
        dock = QDockWidget("Terminal & Loglar", self)
        dock.setAllowedAreas(Qt.BottomDockWidgetArea)
        
        dock_widget = QWidget()
        dock_layout = QVBoxLayout(dock_widget)
        
        # Terminal emulator
        self.terminal_output = QTextEdit()
        self.terminal_output.setReadOnly(True)
        self.terminal_output.setStyleSheet("background-color: black; color: green; font-family: monospace;")
        self.terminal_output.setMaximumHeight(200)
        
        dock_layout.addWidget(QLabel("Script Output:"))
        dock_layout.addWidget(self.terminal_output)
        
        dock.setWidget(dock_widget)
        self.addDockWidget(Qt.BottomDockWidgetArea, dock)
    
    def create_menu_bar(self):
        """Menu bar oluştur"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu('Dosya')
        
        save_action = QAction('Ayarları Kaydet', self)
        save_action.triggered.connect(self.save_settings)
        file_menu.addAction(save_action)
        
        load_action = QAction('Ayarları Yükle', self)
        load_action.triggered.connect(self.load_settings)
        file_menu.addAction(load_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction('Çıkış', self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # Tools menu
        tools_menu = menubar.addMenu('Araçlar')
        
        realtime_action = QAction('Real-time Kontrol', self)
        realtime_action.triggered.connect(self.show_realtime_control)
        tools_menu.addAction(realtime_action)
        
        calib_action = QAction('Sistem Kalibrasyonu', self)
        calib_action.triggered.connect(lambda: self.run_script('system_calibration'))
        tools_menu.addAction(calib_action)
        
        # Help menu
        help_menu = menubar.addMenu('Yardım')
        
        about_action = QAction('Hakkında', self)
        about_action.triggered.connect(self.show_about)
        help_menu.addAction(about_action)
    
    def setup_system(self):
        """Sistem bileşenlerini başlat"""
        try:
            # Config yükle
            config = self.load_hardware_config()
            
            # MAVLink handler
            self.mavlink_handler = MAVLinkHandler()
            
            # GPIO Controller (Raspberry Pi)
            self.gpio_controller = GPIOController(config)
            gpio_status = self.gpio_controller.initialize()
            if gpio_status:
                self.log_message("✅ GPIO sistemi başlatıldı")
                # Buton callback ayarla
                self.gpio_controller.setup_button_callback(self.on_emergency_button)
            else:
                self.log_message("⚠️ GPIO simülasyon modunda")
            
            # D300 Derinlik Sensörü
            self.depth_sensor = D300DepthSensor()
            if self.depth_sensor.connect():
                self.depth_sensor.start_monitoring()
                self.log_message("✅ D300 derinlik sensörü başlatıldı")
            else:
                self.log_message("⚠️ D300 sensörü simülasyon modunda")
            
            # Adafruit Sensör Manager
            self.sensor_manager = SensorManager(config)
            if self.sensor_manager.initialize_sensors():
                self.log_message("✅ Adafruit sensörleri başlatıldı")
            else:
                self.log_message("⚠️ Adafruit sensörleri simülasyon modunda")
            
            # Navigation engine (güncellenmiş sensörlerle)
            self.navigation_engine = NavigationEngine(
                self.mavlink_handler, 
                self.sensor_manager, 
                self.depth_sensor
            )
            
            # Vibration monitor
            self.vibration_monitor = VibrationMonitor(self.mavlink_handler)
            self.vibration_monitor.add_callback(self.on_vibration_update)
            
            # Control module (Tkinter)
            self.control_module = ControlModule(
                self.mavlink_handler, 
                self.navigation_engine, 
                self.update_gui_from_control
            )
            
            self.log_message("🚀 Sistem bileşenleri başarıyla yüklendi!")
            
        except Exception as e:
            self.log_message(f"❌ Sistem başlatma hatası: {e}")
            QMessageBox.critical(self, "Hata", f"Sistem başlatılamadı: {e}")
    
    def load_hardware_config(self):
        """Hardware konfigürasyonunu yükle"""
        try:
            with open("config/hardware_config.json", 'r') as f:
                return json.load(f)
        except Exception as e:
            self.log_message(f"⚠️ Config yükleme hatası: {e}")
            # Varsayılan config döndür
            return {
                "raspberry_pi": {
                    "gpio": {
                        "buzzer": 7,
                        "control_button": 13,
                        "led_red": 4,
                        "led_green": 5,
                        "led_blue": 6,
                        "warning_led": 8,
                        "system_status_led": 10
                    }
                }
            }
    
    def on_emergency_button(self):
        """Acil durum butonu basıldı"""
        self.log_message("🚨 ACİL DURUM BUTONU BASILDI!")
        
        # Buzzer çal
        if self.gpio_controller:
            self.gpio_controller.buzzer_beep(2000, 0.5, 80)
            self.gpio_controller.emergency_led_pattern()
        
        # Emergency stop
        if self.mavlink_handler:
            self.mavlink_handler.emergency_stop()
        
        # GUI update
        QMessageBox.warning(self, "ACİL DURUM", "Acil durum butonu basıldı!\nTüm sistemler durduruldu!")
    
    def closeEvent(self, event):
        """Uygulama kapatılırken temizlik yap"""
        try:
            # GPIO temizle
            if self.gpio_controller:
                self.gpio_controller.cleanup()
            
            # Sensörleri kapat
            if self.depth_sensor:
                self.depth_sensor.disconnect()
            
            if self.sensor_manager:
                self.sensor_manager.shutdown_sensors()
            
            # MAVLink kapat
            if self.mavlink_handler:
                self.mavlink_handler.disconnect()
            
            self.log_message("🔄 Sistem temizliği tamamlandı")
            
        except Exception as e:
            print(f"Temizlik hatası: {e}")
        
        event.accept()
    
    def toggle_connection(self):
        """MAVLink bağlantısını aç/kapat"""
        if not self.mavlink_handler:
            return
        
        if not self.connection_status:
            # Bağlan
            self.connect_btn.setText("Bağlanıyor...")
            self.connect_btn.setEnabled(False)
            
            # Thread'de bağlan
            def connect_worker():
                success = self.mavlink_handler.connect()
                QTimer.singleShot(0, lambda: self.on_connection_result(success))
            
            threading.Thread(target=connect_worker, daemon=True).start()
            
        else:
            # Bağlantıyı kes
            self.mavlink_handler.disconnect()
            self.vibration_monitor.stop_monitoring()
            self.connection_status = False
            self.armed_status = False
            self.update_connection_ui()
            self.telemetry_timer.stop()
    
    def on_connection_result(self, success):
        """Bağlantı sonucu"""
        self.connection_status = success
        self.update_connection_ui()
        
        if success:
            self.log_message("MAVLink bağlantısı başarılı")
            self.vibration_monitor.start_monitoring()
            self.telemetry_timer.start(100)  # 10Hz
        else:
            self.log_message("MAVLink bağlantısı başarısız")
    
    def update_connection_ui(self):
        """Bağlantı UI güncelle"""
        if self.connection_status:
            self.connect_btn.setText("Bağlantıyı Kes")
            self.mavlink_status.setText("MAVLink: ✅")
            self.mavlink_status.setStyleSheet("color: green; font-weight: bold;")
            self.arm_btn.setEnabled(True)
        else:
            self.connect_btn.setText("MAVLink Bağlan")
            self.mavlink_status.setText("MAVLink: ❌")
            self.mavlink_status.setStyleSheet("color: red; font-weight: bold;")
            self.arm_btn.setEnabled(False)
            self.armed_status_label.setText("DISARMED")
            self.armed_status_label.setStyleSheet("color: red; font-weight: bold;")
        
        self.connect_btn.setEnabled(True)
    
    def toggle_arm(self):
        """ARM / DISARM"""
        if not self.mavlink_handler or not self.connection_status:
            return
        
        if not self.armed_status:
            # ARM
            success = self.mavlink_handler.arm_system()
            if success:
                self.armed_status = True
                self.armed_status_label.setText("ARMED")
                self.armed_status_label.setStyleSheet("color: green; font-weight: bold;")
                self.log_message("Sistem ARM edildi")
        else:
            # DISARM
            success = self.mavlink_handler.disarm_system()
            if success:
                self.armed_status = False
                self.armed_status_label.setText("DISARMED")
                self.armed_status_label.setStyleSheet("color: red; font-weight: bold;")
                self.log_message("Sistem DISARM edildi")
    
    def on_control_mode_changed(self):
        """Kontrol modu değişti"""
        if not self.mavlink_handler:
            return
        
        sender = self.sender()
        
        if sender == self.raw_control_cb and self.raw_control_cb.isChecked():
            self.pid_control_cb.setChecked(False)
            self.mavlink_handler.set_control_mode("raw")
            self.log_message("Kontrol modu: RAW PWM")
            
        elif sender == self.pid_control_cb and self.pid_control_cb.isChecked():
            self.raw_control_cb.setChecked(False)
            self.mavlink_handler.set_control_mode("pid")
            self.log_message("Kontrol modu: PID")
    
    def on_navigation_mode_changed(self):
        """Navigation modu değişti"""
        if not self.navigation_engine:
            return
        
        sender = self.sender()
        
        if sender == self.gps_nav_cb and self.gps_nav_cb.isChecked():
            self.imu_nav_cb.setChecked(False)
            self.hybrid_nav_cb.setChecked(False)
            self.navigation_engine.set_navigation_mode("gps_only")
            self.log_message("Navigation modu: GPS Only")
            
        elif sender == self.imu_nav_cb and self.imu_nav_cb.isChecked():
            self.gps_nav_cb.setChecked(False)
            self.hybrid_nav_cb.setChecked(False)
            self.navigation_engine.set_navigation_mode("imu_only")
            self.log_message("Navigation modu: IMU Only")
            
        elif sender == self.hybrid_nav_cb and self.hybrid_nav_cb.isChecked():
            self.gps_nav_cb.setChecked(False)
            self.imu_nav_cb.setChecked(False)
            self.navigation_engine.set_navigation_mode("hybrid")
            self.log_message("Navigation modu: Hybrid")
    
    def show_movement_dialog(self, command_type):
        """Hareket komutu dialogu"""
        if not self.navigation_engine:
            return
        
        # Command bilgilerini yükle
        try:
            with open('config/control_settings.json', 'r') as f:
                settings = json.load(f)
            command_info = settings.get('movement_commands', {}).get(command_type.split('_')[0], {})
        except:
            command_info = {'default_value': 5.0, 'min_value': 0.1, 'max_value': 50.0}
        
        # Dialog
        dialog = QInputDialog()
        dialog.setWindowTitle(f"{command_info.get('description', command_type)}")
        
        if 'ROTATION' in command_type:
            value, ok = dialog.getDouble(
                self, "Yaw Dönme", "Açı (derece):",
                command_info.get('default_value', 90.0),
                command_info.get('min_value', -180.0),
                command_info.get('max_value', 180.0), 1
            )
        else:
            value, ok = dialog.getDouble(
                self, "Mesafe", "Mesafe (metre):",
                command_info.get('default_value', 5.0),
                command_info.get('min_value', 0.1),
                command_info.get('max_value', 50.0), 1
            )
        
        if ok:
            control_mode = "raw" if self.raw_control_cb.isChecked() else "pid"
            self.navigation_engine.start_movement_mission(command_type, value, control_mode)
            self.log_message(f"Görev başlatıldı: {command_type} {value}")
    
    def stop_mission(self):
        """Aktif görevi durdur"""
        if self.navigation_engine:
            self.navigation_engine.stop_mission()
            self.log_message("Görev durduruldu")
    
    def emergency_stop(self):
        """Acil durum"""
        if self.mavlink_handler:
            self.mavlink_handler.emergency_stop()
            self.log_message("ACİL DURUM - Tüm kontroller durduruldu!")
            
        QMessageBox.warning(self, "ACİL DURUM", "Tüm kontroller durduruldu!")
    
    def show_realtime_control(self):
        """Real-time kontrol penceresini göster"""
        if self.control_module:
            self.control_module.show_control_window()
    
    def run_script(self, script_name):
        """Script çalıştır"""
        self.log_message(f"Script çalıştırılıyor: {script_name}")
        
        # Script dosya mapping
        script_files = {
            "servo_calibration": "scripts/servo_calibration.py",
            "motor_test": "scripts/motor_test.py", 
            "imu_calibration": "scripts/imu_calibration.py",
            "system_check": "scripts/system_check.py",
            "emergency_test": "scripts/emergency_stop.py"
        }
        
        if script_name not in script_files:
            error_msg = f"❌ Bilinmeyen script: {script_name}"
            self.log_message(error_msg)
            self.terminal_output.append(error_msg)
            return
        
        script_path = script_files[script_name]
        
        # Script dosyası var mı kontrol et
        if not os.path.exists(script_path):
            error_msg = f"❌ Script dosyası bulunamadı: {script_path}"
            self.log_message(error_msg)
            self.terminal_output.append(error_msg)
            return
        
        # Script'i thread'de çalıştır
        def run_script_worker():
            try:
                import subprocess
                
                # Python script'i çalıştır
                result = subprocess.run(
                    [sys.executable, script_path],
                    capture_output=True,
                    text=True,
                    cwd=".",
                    timeout=300  # 5 dakika timeout
                )
                
                # Sonuçları GUI'ye gönder
                QTimer.singleShot(0, lambda: self.on_script_finished(script_name, result))
                
            except subprocess.TimeoutExpired:
                error_msg = f"❌ {script_name} zaman aşımına uğradı (5 dakika)"
                QTimer.singleShot(0, lambda: self.terminal_output.append(error_msg))
            except Exception as e:
                error_msg = f"❌ {script_name} çalıştırma hatası: {e}"
                QTimer.singleShot(0, lambda: self.terminal_output.append(error_msg))
        
        # Thread başlat
        import threading
        threading.Thread(target=run_script_worker, daemon=True).start()
        
        # Başlatma mesajı
        start_msg = f"🚀 {script_name} script'i başlatıldı..."
        self.terminal_output.append(start_msg)
    
    def on_script_finished(self, script_name, result):
        """Script tamamlandığında çağrılır"""
        if result.returncode == 0:
            # Başarılı
            output = f"✅ {script_name} başarıyla tamamlandı\n"
            if result.stdout:
                output += f"📄 Çıktı:\n{result.stdout}\n"
        else:
            # Hata
            output = f"❌ {script_name} başarısız (return code: {result.returncode})\n"
            if result.stderr:
                output += f"🚨 Hata:\n{result.stderr}\n"
            if result.stdout:
                output += f"📄 Çıktı:\n{result.stdout}\n"
        
        self.terminal_output.append(output)
        self.log_message(f"Script tamamlandı: {script_name}")
    
    def update_telemetry(self):
        """Telemetry güncelle"""
        if not self.mavlink_handler or not self.connection_status:
            return
        
        # IMU verilerini al
        imu_data = self.mavlink_handler.get_imu_data()
        if imu_data:
            # IMU filtresi uygula
            roll, pitch, yaw = self.mavlink_handler.imu_filter.update(*imu_data)
            
            # Grafikleri güncelle
            self.yaw_data.append(yaw)
            self.pitch_data.append(pitch)
            self.roll_data.append(roll)
            
            # Buffer boyutunu koru
            if len(self.yaw_data) > self.graph_data_length:
                self.yaw_data = self.yaw_data[-self.graph_data_length:]
                self.pitch_data = self.pitch_data[-self.graph_data_length:]
                self.roll_data = self.roll_data[-self.graph_data_length:]
            
            # Grafikleri çiz
            self.yaw_line.setData(self.yaw_data)
            self.pitch_line.setData(self.pitch_data)
            self.roll_line.setData(self.roll_data)
            
            # Telemetry text güncelle
            telemetry_info = f"""IMU Verileri:
Roll:  {roll:+7.2f}°
Pitch: {pitch:+7.2f}°
Yaw:   {yaw:+7.2f}°

Kontrol Modu: {self.mavlink_handler.control_mode.upper()}
Navigation:   {self.navigation_engine.current_mode if self.navigation_engine else 'N/A'}
"""
            self.telemetry_text.setText(telemetry_info)
        
        # GPS verileri
        gps_data = self.mavlink_handler.get_gps_data()
        if gps_data:
            lat, lon, alt, satellites = gps_data
            self.gps_info.setText(f"GPS: {lat:.6f}, {lon:.6f} | Alt: {alt:.1f}m | Sat: {satellites}")
        else:
            self.gps_info.setText("GPS: Veri yok")
        
        # Mission status
        if self.navigation_engine and self.navigation_engine.mission_active:
            self.mission_status.setText("Görev: 🟢 Aktif")
        else:
            self.mission_status.setText("Görev: Yok")
    
    def on_vibration_update(self, vibration_data):
        """Vibration güncelleme callback"""
        level = vibration_data['level']
        color = vibration_data['color']
        category = vibration_data['category']
        freq = vibration_data['dominant_frequency']
        
        # Progress bar güncelle
        self.vibration_bar.setValue(int(level))
        
        # Renk ayarla
        if color == "green":
            style = "QProgressBar::chunk { background-color: green; }"
        elif color == "yellow":
            style = "QProgressBar::chunk { background-color: yellow; }"
        else:
            style = "QProgressBar::chunk { background-color: red; }"
        
        self.vibration_bar.setStyleSheet(style)
        
        # Info güncelle
        self.vibration_info.setText(f"Kategori: {category.title()} | Frekans: {freq:.1f} Hz")
    
    def show_vibration_comparison(self):
        """Vibration karşılaştırma penceresi"""
        if not self.vibration_monitor:
            QMessageBox.warning(self, "Uyarı", "Vibration monitor aktif değil!")
            return
        
        comparison_window = VibrationComparisonWindow(self.vibration_monitor, self)
        comparison_window.exec_()
    
    def show_pin_config(self):
        """Pin konfigürasyon penceresi"""
        pin_editor = PinConfigEditor(self)
        pin_editor.exec_()
    
    def update_gui_from_control(self, data):
        """Control module'den GUI güncelleme"""
        # Control module'den gelen verilerle GUI güncelle
        pass
    
    def reload_settings(self):
        """Ayarları yeniden yükle"""
        try:
            if self.mavlink_handler:
                self.mavlink_handler.load_config("config/hardware_config.json")
            if self.navigation_engine:
                self.navigation_engine.load_control_settings("config/control_settings.json")
            
            self.log_message("Ayarlar yeniden yüklendi")
            QMessageBox.information(self, "Başarılı", "Ayarlar yeniden yüklendi!")
            
        except Exception as e:
            self.log_message(f"Ayar yükleme hatası: {e}")
            QMessageBox.critical(self, "Hata", f"Ayarlar yüklenemedi: {e}")
    
    def save_settings(self):
        """Ayarları kaydet"""
        # TODO: Implement settings save
        QMessageBox.information(self, "Ayarları Kaydet", "Ayar kaydetme özelliği yakında!")
    
    def load_settings(self):
        """Ayarları yükle"""
        # TODO: Implement settings load
        QMessageBox.information(self, "Ayarları Yükle", "Ayar yükleme özelliği yakında!")
    
    def show_about(self):
        """Hakkında penceresi"""
        about_text = """
TEKNOFEST 2025 - Su Altı Roket Aracı Kontrol Sistemi

Bu uygulama aşağıdaki özellikleri içerir:
• Real-time ROV kontrolü (W,A,S,D tuşları)
• RAW PWM vs PID kontrol modları
• GPS ve IMU navigation sistemleri
• Titreşim analizi ve karşılaştırması
• Otomatik görev çalıştırma
• Script entegrasyonu

Geliştirici: TEKNOFEST Takımı
Sürüm: 1.0
Tarih: 2025
        """
        QMessageBox.about(self, "Hakkında", about_text)
    
    def log_message(self, message):
        """Log mesajı"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        self.terminal_output.append(log_entry)
        self.statusBar().showMessage(message)
        print(log_entry)  # Console'a da yaz
    
    def closeEvent(self, event):
        """Pencere kapatılırken"""
        # Sistem temizliği
        if self.vibration_monitor:
            self.vibration_monitor.stop_monitoring()
        
        if self.mavlink_handler:
            self.mavlink_handler.disconnect()
        
        if self.control_module:
            self.control_module.close_control_window()
        
        self.log_message("Sistem kapatılıyor...")
        event.accept()

class PinConfigEditor(QDialog):
    """Pin konfigürasyon editörü"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Pin Konfigürasyon Editörü")
        self.setGeometry(200, 200, 600, 500)
        self.setModal(True)
        
        self.config_file = "config/hardware_config.json"
        self.config_data = {}
        
        self.setup_ui()
        self.load_config()
    
    def setup_ui(self):
        """UI kurulumu"""
        layout = QVBoxLayout(self)
        
        # Başlık
        title = QLabel("🔧 Hardware Pin Konfigürasyonu")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Scroll area
        scroll = QScrollArea()
        scroll_widget = QWidget()
        self.form_layout = QFormLayout(scroll_widget)
        scroll.setWidget(scroll_widget)
        scroll.setWidgetResizable(True)
        layout.addWidget(scroll)
        
        # Pin input widgets
        self.pin_widgets = {}
        
        # Pixhawk Servo Section
        servo_group = QGroupBox("Pixhawk Servo Kanalları")
        servo_layout = QFormLayout(servo_group)
        
        servo_pins = [
            ("front_left", "Ön Sol Fin (AUX)"),
            ("rear_left", "Arka Sol Fin (AUX)"),
            ("rear_right", "Arka Sağ Fin (AUX)"),
            ("front_right", "Ön Sağ Fin (AUX)")
        ]
        
        for pin_key, description in servo_pins:
            spinbox = QSpinBox()
            spinbox.setRange(1, 16)
            spinbox.setValue(1)
            servo_layout.addRow(description, spinbox)
            self.pin_widgets[f"pixhawk_servos_{pin_key}"] = spinbox
        
        # Motor section
        motor_spinbox = QSpinBox()
        motor_spinbox.setRange(1, 16)
        motor_spinbox.setValue(6)
        servo_layout.addRow("Ana Motor (AUX)", motor_spinbox)
        self.pin_widgets["pixhawk_motor"] = motor_spinbox
        
        self.form_layout.addRow(servo_group)
        
        # PWM Limits Section
        pwm_group = QGroupBox("PWM Limitleri")
        pwm_layout = QFormLayout(pwm_group)
        
        pwm_limits = [
            ("servo_min", "Servo Minimum PWM", 1000, 1200, 1100),
            ("servo_max", "Servo Maximum PWM", 1800, 2000, 1900),
            ("servo_neutral", "Servo Neutral PWM", 1400, 1600, 1500),
            ("motor_min", "Motor Minimum PWM", 1000, 1200, 1000),
            ("motor_max", "Motor Maximum PWM", 1800, 2000, 2000),
            ("motor_stop", "Motor Stop PWM", 1400, 1600, 1500)
        ]
        
        for key, description, min_val, max_val, default in pwm_limits:
            spinbox = QSpinBox()
            spinbox.setRange(min_val, max_val)
            spinbox.setValue(default)
            spinbox.setSuffix(" µs")
            pwm_layout.addRow(description, spinbox)
            self.pin_widgets[f"pwm_limits_{key}"] = spinbox
        
        self.form_layout.addRow(pwm_group)
        
        # Raspberry Pi GPIO Section
        gpio_group = QGroupBox("Raspberry Pi GPIO")
        gpio_layout = QFormLayout(gpio_group)
        
        gpio_pins = [
            ("buzzer", "Buzzer GPIO", 7),
            ("control_button", "Kontrol Butonu GPIO", 13),
            ("led_red", "Kırmızı LED GPIO", 4),
            ("led_green", "Yeşil LED GPIO", 5),
            ("led_blue", "Mavi LED GPIO", 6),
            ("warning_led", "Uyarı LED GPIO", 8),
            ("system_status_led", "Durum LED GPIO", 10)
        ]
        
        for key, description, default in gpio_pins:
            spinbox = QSpinBox()
            spinbox.setRange(2, 27)  # Valid GPIO range
            spinbox.setValue(default)
            gpio_layout.addRow(description, spinbox)
            self.pin_widgets[f"raspberry_pi_gpio_{key}"] = spinbox
        
        self.form_layout.addRow(gpio_group)
        
        # MAVLink Section
        mavlink_group = QGroupBox("MAVLink Ayarları")
        mavlink_layout = QFormLayout(mavlink_group)
        
        self.connection_string = QLineEdit("tcp:127.0.0.1:5777")
        mavlink_layout.addRow("Bağlantı String", self.connection_string)
        
        self.heartbeat_timeout = QSpinBox()
        self.heartbeat_timeout.setRange(5, 120)
        self.heartbeat_timeout.setValue(30)
        self.heartbeat_timeout.setSuffix(" saniye")
        mavlink_layout.addRow("Heartbeat Timeout", self.heartbeat_timeout)
        
        self.command_timeout = QSpinBox()
        self.command_timeout.setRange(1, 30)
        self.command_timeout.setValue(5)
        self.command_timeout.setSuffix(" saniye")
        mavlink_layout.addRow("Komut Timeout", self.command_timeout)
        
        self.form_layout.addRow(mavlink_group)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        load_btn = QPushButton("🔄 Yenile")
        load_btn.clicked.connect(self.load_config)
        button_layout.addWidget(load_btn)
        
        save_btn = QPushButton("💾 Kaydet")
        save_btn.clicked.connect(self.save_config)
        button_layout.addWidget(save_btn)
        
        reset_btn = QPushButton("🔙 Varsayılan")
        reset_btn.clicked.connect(self.reset_to_defaults)
        button_layout.addWidget(reset_btn)
        
        cancel_btn = QPushButton("❌ İptal")
        cancel_btn.clicked.connect(self.reject)
        button_layout.addWidget(cancel_btn)
        
        layout.addLayout(button_layout)
    
    def load_config(self):
        """Konfigürasyonu yükle"""
        try:
            with open(self.config_file, 'r') as f:
                self.config_data = json.load(f)
            
            # Pixhawk servos
            if "pixhawk" in self.config_data and "servos" in self.config_data["pixhawk"]:
                servos = self.config_data["pixhawk"]["servos"]
                for servo_name, channel in servos.items():
                    widget_key = f"pixhawk_servos_{servo_name}"
                    if widget_key in self.pin_widgets:
                        self.pin_widgets[widget_key].setValue(channel)
            
            # Pixhawk motor
            if "pixhawk" in self.config_data and "motor" in self.config_data["pixhawk"]:
                self.pin_widgets["pixhawk_motor"].setValue(self.config_data["pixhawk"]["motor"])
            
            # PWM limits
            if "pixhawk" in self.config_data and "pwm_limits" in self.config_data["pixhawk"]:
                limits = self.config_data["pixhawk"]["pwm_limits"]
                for key, value in limits.items():
                    widget_key = f"pwm_limits_{key}"
                    if widget_key in self.pin_widgets:
                        self.pin_widgets[widget_key].setValue(value)
            
            # Raspberry Pi GPIO
            if "raspberry_pi" in self.config_data and "gpio" in self.config_data["raspberry_pi"]:
                gpio = self.config_data["raspberry_pi"]["gpio"]
                for key, value in gpio.items():
                    widget_key = f"raspberry_pi_gpio_{key}"
                    if widget_key in self.pin_widgets:
                        self.pin_widgets[widget_key].setValue(value)
            
            # MAVLink
            if "mavlink" in self.config_data:
                mavlink = self.config_data["mavlink"]
                if "connection_string" in mavlink:
                    self.connection_string.setText(mavlink["connection_string"])
                if "heartbeat_timeout" in mavlink:
                    self.heartbeat_timeout.setValue(mavlink["heartbeat_timeout"])
                if "command_timeout" in mavlink:
                    self.command_timeout.setValue(mavlink["command_timeout"])
            
            QMessageBox.information(self, "Başarılı", "Konfigürasyon yüklendi!")
            
        except FileNotFoundError:
            QMessageBox.warning(self, "Hata", f"Konfigürasyon dosyası bulunamadı: {self.config_file}")
        except json.JSONDecodeError as e:
            QMessageBox.critical(self, "JSON Hatası", f"Konfigürasyon dosyası geçersiz: {e}")
        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Konfigürasyon yükleme hatası: {e}")
    
    def save_config(self):
        """Konfigürasyonu kaydet"""
        try:
            # Yeni konfigürasyon oluştur
            new_config = {
                "pixhawk": {
                    "servos": {
                        "front_left": self.pin_widgets["pixhawk_servos_front_left"].value(),
                        "rear_left": self.pin_widgets["pixhawk_servos_rear_left"].value(),
                        "rear_right": self.pin_widgets["pixhawk_servos_rear_right"].value(),
                        "front_right": self.pin_widgets["pixhawk_servos_front_right"].value()
                    },
                    "motor": self.pin_widgets["pixhawk_motor"].value(),
                    "pwm_limits": {
                        "servo_min": self.pin_widgets["pwm_limits_servo_min"].value(),
                        "servo_max": self.pin_widgets["pwm_limits_servo_max"].value(),
                        "servo_neutral": self.pin_widgets["pwm_limits_servo_neutral"].value(),
                        "motor_min": self.pin_widgets["pwm_limits_motor_min"].value(),
                        "motor_max": self.pin_widgets["pwm_limits_motor_max"].value(),
                        "motor_stop": self.pin_widgets["pwm_limits_motor_stop"].value()
                    }
                },
                "raspberry_pi": {
                    "gpio": {
                        "buzzer": self.pin_widgets["raspberry_pi_gpio_buzzer"].value(),
                        "control_button": self.pin_widgets["raspberry_pi_gpio_control_button"].value(),
                        "led_red": self.pin_widgets["raspberry_pi_gpio_led_red"].value(),
                        "led_green": self.pin_widgets["raspberry_pi_gpio_led_green"].value(),
                        "led_blue": self.pin_widgets["raspberry_pi_gpio_led_blue"].value(),
                        "warning_led": self.pin_widgets["raspberry_pi_gpio_warning_led"].value(),
                        "system_status_led": self.pin_widgets["raspberry_pi_gpio_system_status_led"].value()
                    },
                    "i2c": {
                        "depth_sensor_address": "0x77"
                    }
                },
                "mavlink": {
                    "connection_string": self.connection_string.text(),
                    "heartbeat_timeout": self.heartbeat_timeout.value(),
                    "command_timeout": self.command_timeout.value()
                },
                "system_limits": {
                    "max_depth_meters": 10,
                    "max_distance_meters": 100,
                    "emergency_surface_depth": 5,
                    "low_battery_voltage": 19.8
                }
            }
            
            # Dosyaya kaydet
            os.makedirs(os.path.dirname(self.config_file), exist_ok=True)
            with open(self.config_file, 'w') as f:
                json.dump(new_config, f, indent=2)
            
            QMessageBox.information(self, "Başarılı", f"Konfigürasyon kaydedildi: {self.config_file}")
            self.accept()
            
        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Konfigürasyon kaydetme hatası: {e}")
    
    def reset_to_defaults(self):
        """Varsayılan değerlere sıfırla"""
        reply = QMessageBox.question(self, "Varsayılan Değerler", 
                                   "Tüm ayarları varsayılan değerlere sıfırlamak istiyor musunuz?",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            # Servo defaults
            self.pin_widgets["pixhawk_servos_front_left"].setValue(1)
            self.pin_widgets["pixhawk_servos_rear_left"].setValue(3)
            self.pin_widgets["pixhawk_servos_rear_right"].setValue(4)
            self.pin_widgets["pixhawk_servos_front_right"].setValue(5)
            self.pin_widgets["pixhawk_motor"].setValue(6)
            
            # PWM defaults
            self.pin_widgets["pwm_limits_servo_min"].setValue(1100)
            self.pin_widgets["pwm_limits_servo_max"].setValue(1900)
            self.pin_widgets["pwm_limits_servo_neutral"].setValue(1500)
            self.pin_widgets["pwm_limits_motor_min"].setValue(1000)
            self.pin_widgets["pwm_limits_motor_max"].setValue(2000)
            self.pin_widgets["pwm_limits_motor_stop"].setValue(1500)
            
            # GPIO defaults
            self.pin_widgets["raspberry_pi_gpio_buzzer"].setValue(7)
            self.pin_widgets["raspberry_pi_gpio_control_button"].setValue(13)
            self.pin_widgets["raspberry_pi_gpio_led_red"].setValue(4)
            self.pin_widgets["raspberry_pi_gpio_led_green"].setValue(5)
            self.pin_widgets["raspberry_pi_gpio_led_blue"].setValue(6)
            self.pin_widgets["raspberry_pi_gpio_warning_led"].setValue(8)
            self.pin_widgets["raspberry_pi_gpio_system_status_led"].setValue(10)
            
            # MAVLink defaults
            self.connection_string.setText("tcp:127.0.0.1:5777")
            self.heartbeat_timeout.setValue(30)
            self.command_timeout.setValue(5)

class VibrationComparisonWindow(QDialog):
    """Vibration karşılaştırma penceresi"""
    
    def __init__(self, vibration_monitor, parent=None):
        super().__init__(parent)
        self.setWindowTitle("RAW vs PID Titreşim Karşılaştırması")
        self.setGeometry(300, 300, 800, 600)
        self.setModal(True)
        
        self.vibration_monitor = vibration_monitor
        self.comparison_data = {
            "raw_samples": [],
            "pid_samples": [],
            "current_mode": None,
            "recording": False
        }
        
        self.setup_ui()
    
    def setup_ui(self):
        """UI kurulumu"""
        layout = QVBoxLayout(self)
        
        # Başlık
        title = QLabel("📊 RAW vs PID Titreşim Karşılaştırması")
        title.setFont(QFont("Arial", 14, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        layout.addWidget(title)
        
        # Control section
        control_group = QGroupBox("Kayıt Kontrolü")
        control_layout = QHBoxLayout(control_group)
        
        self.raw_record_btn = QPushButton("🔴 RAW Modu Kayıt Başlat")
        self.raw_record_btn.clicked.connect(lambda: self.start_recording("raw"))
        control_layout.addWidget(self.raw_record_btn)
        
        self.pid_record_btn = QPushButton("🔴 PID Modu Kayıt Başlat")
        self.pid_record_btn.clicked.connect(lambda: self.start_recording("pid"))
        control_layout.addWidget(self.pid_record_btn)
        
        self.stop_record_btn = QPushButton("⏹️ Kaydı Durdur")
        self.stop_record_btn.clicked.connect(self.stop_recording)
        self.stop_record_btn.setEnabled(False)
        control_layout.addWidget(self.stop_record_btn)
        
        layout.addWidget(control_group)
        
        # Status
        self.status_label = QLabel("📍 Durum: Hazır")
        layout.addWidget(self.status_label)
        
        # Data display
        data_group = QGroupBox("Kayıt Durumu")
        data_layout = QGridLayout(data_group)
        
        data_layout.addWidget(QLabel("RAW Modu Örnekleri:"), 0, 0)
        self.raw_count_label = QLabel("0")
        data_layout.addWidget(self.raw_count_label, 0, 1)
        
        data_layout.addWidget(QLabel("PID Modu Örnekleri:"), 1, 0)
        self.pid_count_label = QLabel("0")
        data_layout.addWidget(self.pid_count_label, 1, 1)
        
        layout.addWidget(data_group)
        
        # Results section
        results_group = QGroupBox("Sonuçlar")
        results_layout = QVBoxLayout(results_group)
        
        self.results_text = QTextEdit()
        self.results_text.setReadOnly(True)
        self.results_text.setMaximumHeight(200)
        results_layout.addWidget(self.results_text)
        
        layout.addWidget(results_group)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        analyze_btn = QPushButton("📈 Analiz Et")
        analyze_btn.clicked.connect(self.analyze_data)
        button_layout.addWidget(analyze_btn)
        
        clear_btn = QPushButton("🗑️ Verileri Temizle")
        clear_btn.clicked.connect(self.clear_data)
        button_layout.addWidget(clear_btn)
        
        export_btn = QPushButton("💾 Export")
        export_btn.clicked.connect(self.export_data)
        button_layout.addWidget(export_btn)
        
        close_btn = QPushButton("❌ Kapat")
        close_btn.clicked.connect(self.accept)
        button_layout.addWidget(close_btn)
        
        layout.addLayout(button_layout)
        
        # Timer for updates
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(500)  # 2Hz update
    
    def start_recording(self, mode):
        """Kayıt başlat"""
        if self.comparison_data["recording"]:
            QMessageBox.warning(self, "Uyarı", "Zaten kayıt devam ediyor!")
            return
        
        self.comparison_data["current_mode"] = mode
        self.comparison_data["recording"] = True
        
        # Button states
        self.raw_record_btn.setEnabled(False)
        self.pid_record_btn.setEnabled(False)
        self.stop_record_btn.setEnabled(True)
        
        self.status_label.setText(f"🔴 Kayıt: {mode.upper()} modu")
        
        QMessageBox.information(self, "Kayıt Başladı", 
                               f"{mode.upper()} modu için titreşim kaydı başladı.\n"
                               f"Sistemi {mode.upper()} moduna alın ve aracı test edin.")
    
    def stop_recording(self):
        """Kaydı durdur"""
        self.comparison_data["recording"] = False
        self.comparison_data["current_mode"] = None
        
        # Button states
        self.raw_record_btn.setEnabled(True)
        self.pid_record_btn.setEnabled(True)
        self.stop_record_btn.setEnabled(False)
        
        self.status_label.setText("📍 Durum: Hazır")
    
    def update_display(self):
        """Display güncelle"""
        # Sample counts
        self.raw_count_label.setText(str(len(self.comparison_data["raw_samples"])))
        self.pid_count_label.setText(str(len(self.comparison_data["pid_samples"])))
        
        # Recording data collection
        if self.comparison_data["recording"]:
            vibration_data = self.vibration_monitor.get_vibration_data()
            
            sample = {
                "timestamp": time.time(),
                "level": vibration_data["level"],
                "category": vibration_data["category"],
                "dominant_frequency": vibration_data["dominant_frequency"],
                "frequency_bands": vibration_data["frequency_bands"].copy()
            }
            
            if self.comparison_data["current_mode"] == "raw":
                self.comparison_data["raw_samples"].append(sample)
            elif self.comparison_data["current_mode"] == "pid":
                self.comparison_data["pid_samples"].append(sample)
    
    def analyze_data(self):
        """Veri analizi yap"""
        raw_samples = self.comparison_data["raw_samples"]
        pid_samples = self.comparison_data["pid_samples"]
        
        if not raw_samples or not pid_samples:
            QMessageBox.warning(self, "Uyarı", "Her iki mod için de veri gerekli!")
            return
        
        # Analysis
        raw_levels = [s["level"] for s in raw_samples]
        pid_levels = [s["level"] for s in pid_samples]
        
        raw_stats = {
            "mean": sum(raw_levels) / len(raw_levels),
            "max": max(raw_levels),
            "min": min(raw_levels),
            "count": len(raw_levels)
        }
        
        pid_stats = {
            "mean": sum(pid_levels) / len(pid_levels),
            "max": max(pid_levels),
            "min": min(pid_levels),
            "count": len(pid_levels)
        }
        
        # Comparison
        mean_diff = raw_stats["mean"] - pid_stats["mean"]
        improvement_percent = (mean_diff / raw_stats["mean"]) * 100 if raw_stats["mean"] > 0 else 0
        
        better_mode = "RAW" if raw_stats["mean"] < pid_stats["mean"] else "PID"
        
        # Results
        results = f"""📊 TİTREŞİM ANALİZ SONUÇLARI
{'='*50}

RAW PWM Control:
  • Ortalama Titreşim: {raw_stats['mean']:.1f}
  • Maksimum: {raw_stats['max']:.1f}
  • Minimum: {raw_stats['min']:.1f}
  • Örnek Sayısı: {raw_stats['count']}

PID Control:
  • Ortalama Titreşim: {pid_stats['mean']:.1f}
  • Maksimum: {pid_stats['max']:.1f}
  • Minimum: {pid_stats['min']:.1f}
  • Örnek Sayısı: {pid_stats['count']}

KARŞILAŞTIRMA:
  • Fark: {abs(mean_diff):.1f} puan
  • Değişim: {improvement_percent:+.1f}%
  • Daha İyi Mod: {better_mode}

ÖNERİ:
"""
        
        if abs(mean_diff) < 5:
            results += "Her iki mod da benzer titreşim gösteriyor. Tercihe göre seçilebilir."
        elif raw_stats["mean"] < pid_stats["mean"]:
            results += f"RAW control {pid_stats['mean'] - raw_stats['mean']:.1f} puan daha az titreşimli. RAW modu önerilir."
        else:
            results += f"PID control {raw_stats['mean'] - pid_stats['mean']:.1f} puan daha az titreşimli. PID modu önerilir."
        
        self.results_text.setText(results)
    
    def clear_data(self):
        """Verileri temizle"""
        reply = QMessageBox.question(self, "Veri Temizle", 
                                   "Tüm kayıtlı verileri silmek istiyor musunuz?",
                                   QMessageBox.Yes | QMessageBox.No)
        
        if reply == QMessageBox.Yes:
            self.comparison_data["raw_samples"].clear()
            self.comparison_data["pid_samples"].clear()
            self.results_text.clear()
            self.status_label.setText("📍 Durum: Veriler temizlendi")
    
    def export_data(self):
        """Verileri dışa aktar"""
        if not self.comparison_data["raw_samples"] and not self.comparison_data["pid_samples"]:
            QMessageBox.warning(self, "Uyarı", "Dışa aktarılacak veri yok!")
            return
        
        try:
            filename = f"vibration_comparison_{int(time.time())}.json"
            
            export_data = {
                "timestamp": time.time(),
                "raw_samples": self.comparison_data["raw_samples"],
                "pid_samples": self.comparison_data["pid_samples"],
                "analysis": self.results_text.toPlainText()
            }
            
            with open(filename, 'w') as f:
                json.dump(export_data, f, indent=2)
            
            QMessageBox.information(self, "Export Başarılı", 
                                  f"Veriler başarıyla dışa aktarıldı:\n{filename}")
                                  
        except Exception as e:
            QMessageBox.critical(self, "Export Hatası", f"Dışa aktarma hatası: {e}")

def main():
    """Ana fonksiyon"""
    app = QApplication(sys.argv)
    app.setApplicationName("TEKNOFEST ROV Controller")
    app.setOrganizationName("TEKNOFEST")
    
    # Ana pencere
    window = MainWindow()
    window.show()
    
    # Event loop
    sys.exit(app.exec_())

if __name__ == "__main__":
    main() 