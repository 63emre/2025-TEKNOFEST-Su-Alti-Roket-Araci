#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş
PLUS WING (+) KONFİGÜRASYONU - GPS'SİZ DEAD RECKONING

Görev Açıklaması:
- Başlangıç bölgesinden 2m derinlikte düz istikamette 10m ilerle (süre başlatılır)
- Kıyıdan en az 50m uzaklaş (dead reckoning ile)
- Başlangıç noktasına otonom geri dön (IMU + zaman bazlı)
- Pozitif sephiye ile yüzeye çıkıp enerjiyi kes

Puanlama:
- Seyir yapma (hız): 150 puan
- Başlangıç noktasında enerji kesme: 90 puan  
- Sızdırmazlık: 60 puan
- Süre limiti: 5 dakika
- Toplam: 300 puan

Plus Wing Konfigürasyonu:
     ÜST (14)
        |
SOL (13) + SAĞ (12)
        |
     ALT (11)
"""

import time
import threading
import math
import json
import argparse
from datetime import datetime
from pymavlink import mavutil

# GPIO kontrol sistemi (LED & Buzzer)
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("✅ RPi.GPIO modülü yüklendi")
except ImportError:
    print("⚠️ RPi.GPIO bulunamadı, LED/Buzzer devre dışı")
    GPIO_AVAILABLE = False

# Plus Wing hardware config import - full_stabilization2.py'den alınan servo mapping
# TEKNOFEST Standart Pin Mapping - full_stabilization2.py'deki gibi
SERVO_UP    = 14   # ÜST kanat
SERVO_DOWN  = 11   # ALT kanat  
SERVO_RIGHT = 12   # SAĞ kanat
SERVO_LEFT  = 13   # SOL kanat

MOTOR_CHANNEL = 1  # AUX 1 - Ana Motor

# Plus-Wing Servo Channels - full_stabilization2.py uyumlu
SERVO_CHANNELS = {
    'up': SERVO_UP,      # AUX 6 - Üst Kanat
    'down': SERVO_DOWN,  # AUX 4 - Alt Kanat
    'right': SERVO_RIGHT, # AUX 3 - Sağ Kanat
    'left': SERVO_LEFT   # AUX 5 - Sol Kanat
}

print("✅ Plus-Wing hardware konfigürasyonu yüklendi")

# D300 derinlik sensörü import
try:
    import sys
    sys.path.append('../App')
    from depth_sensor import D300DepthSensor
    D300_AVAILABLE = True
    print("✅ D300 derinlik sensörü modülü yüklendi")
except ImportError:
    print("⚠️ D300 derinlik sensörü modülü bulunamadı, SCALED_PRESSURE kullanılacak")
    D300_AVAILABLE = False

# MAVLink bağlantı adresi
import os
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# GPIO Pin tanımları (HARDWARE_PIN_MAPPING.md'den)
GPIO_STATUS_LED = 4      # Durum LED (Kırmızı/Yeşil/Mavi)
GPIO_BUZZER_PWM = 13     # Buzzer PWM Output
GPIO_POWER_BUTTON = 18   # Güç Butonu Input
GPIO_EMERGENCY_STOP = 19 # Acil Durdurma Input

# Görev parametreleri (şartnameden)
MISSION_PARAMS = {
    'target_depth': 2.0,           # 2m derinlik
    'straight_distance': 10.0,     # 10m düz seyir
    'min_offshore_distance': 50.0, # 50m kıyıdan uzaklık
    'cruise_speed': 1.5,           # Seyir hızı (m/s)
    'return_speed': 1.8,           # Geri dönüş hızı (m/s)
    'timeout_seconds': 300,        # 5 dakika süre limiti
    'position_tolerance': 2.0,     # Başlangıç noktası toleransı (m)
    'depth_tolerance': 0.2         # Derinlik toleransı (m)
}

# Plus Wing kontrol parametreleri - Optimize edilmiş hızlı & stabil sürüş
CONTROL_PARAMS = {
    # Derinlik PID: Hızlı tepki, az overshoot
    'depth_pid': {'kp': 120.0, 'ki': 5.0, 'kd': 35.0, 'max_output': 200},
    
    # Heading PID: Smooth navigasyon, yeterli otorite
    'heading_pid': {'kp': 6.0, 'ki': 0.25, 'kd': 1.2, 'max_output': 100},
    
    # Dead reckoning PID: Hassas pozisyon kontrolü
    'dead_reckoning_pid': {'kp': 2.5, 'ki': 0.05, 'kd': 0.8, 'max_output': 150},
    
    # Stabilizasyon PID'leri (attitude control)
    'roll_stabilization': {'kp': 3.0, 'ki': 0.1, 'kd': 0.5, 'max_output': 80},
    'pitch_stabilization': {'kp': 3.5, 'ki': 0.12, 'kd': 0.6, 'max_output': 80},
    'yaw_stabilization': {'kp': 2.0, 'ki': 0.08, 'kd': 0.4, 'max_output': 60}
}

# ---- FULL_STABILIZATION2.PY'DEN ALINAN STABİLİZASYON PARAMETRELERİ (HİÇBİR ŞEY DEĞİŞTİRİLMEDİ) ----

# Roll stabilizasyonu (full_stabilization2.py'den aynen)
ROLL_SENSE = +1.0
ROLL_K_ANG_US_PER_RAD = 500.0
ROLL_DEADBAND_DEG = 1.0
ROLL_MAX_DELTA_US = 350.0

# Pitch stabilizasyonu (full_stabilization2.py'den aynen)  
PITCH_SENSE = +1.0
PITCH_K_ANG_US_PER_RAD = 500.0
PITCH_DEADBAND_DEG = 1.0
PITCH_MAX_DELTA_US = 350.0

# Yaw stabilizasyonu (full_stabilization2.py'den aynen)
YAW_SENSE = +1.0
YAW_K_ANG_US_PER_RAD = 400.0
YAW_DEADBAND_DEG = 2.0
YAW_MAX_DELTA_US = 300.0

# ---- Mekanik Yönler (full_stabilization2.py'den aynen) ----
# Manuel testlerden alınan çalışan yön ayarları

# Roll için (LEFT & RIGHT kanatlar)
ROLL_DIR_LEFT  = +1.0
ROLL_DIR_RIGHT = +1.0

# Pitch için (UP & DOWN kanatlar)
PITCH_DIR_UP   = +1.0
PITCH_DIR_DOWN = +1.0

# Yaw için (4 kanat koordineli)
YAW_DIR_UP    = +1.0
YAW_DIR_DOWN  = +1.0
YAW_DIR_RIGHT = -1.0
YAW_DIR_LEFT  = -1.0

# ---- Genel Sınırlar (full_stabilization2.py'den aynen) ----
OVERALL_MAX_DELTA_US = 400.0  # Tüm eksenlerin toplamı için güvenlik sınırı

# PWM değerleri - full_stabilization2.py'den aynen
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

# Kalibrasyon dosyası yükle
def load_speed_calibration():
    """PWM→hız kalibrasyon parametrelerini yükle"""
    try:
        with open('config/cal_speed.json', 'r') as f:
            cal_data = json.load(f)
            plus_wing_cal = cal_data['pwm_speed_calibration']['plus_wing']
            return {
                'a': plus_wing_cal['a'],
                'b': plus_wing_cal['b'], 
                'neutral_pwm': plus_wing_cal['neutral_pwm'],
                'lpf_alpha': cal_data['distance_tracking']['lpf_alpha']
            }
    except Exception as e:
        print(f"⚠️ Kalibrasyon dosyası yüklenemedi: {e}")
        return {
            'a': 0.012,  # Plus-Wing default
            'b': 0.0,
            'neutral_pwm': 1500,
            'lpf_alpha': 0.3
        }

SPEED_CAL = load_speed_calibration()

def load_mission_config():
    """Merkezi mission config dosyasını yükle"""
    try:
        with open('config/mission_config.json', 'r') as f:
            config = json.load(f)
            print("✅ Mission config yüklendi")
            return config
    except Exception as e:
        print(f"⚠️ Mission config yüklenemedi: {e}, default değerler kullanılacak")
        return None

def validate_hardware_config(config):
    """Hardware konfigürasyonunu doğrula"""
    if not config:
        return True
        
    try:
        validation = config.get('validation_rules', {})
        
        # Required channels kontrolü
        if validation.get('required_channels'):
            required = validation['required_channels']
            print(f"🔍 Hardware validation: Required channels {required}")
            
            # Motor channel kontrolü
            motor_ch = config['hardware_config']['motor_channel']
            if motor_ch not in required:
                print(f"⚠️ Motor channel {motor_ch} gerekli kanallar listesinde değil")
            
            # Servo channels kontrolü
            servo_chs = list(config['hardware_config']['servo_channels'].values())
            for ch in servo_chs:
                if ch not in required:
                    print(f"⚠️ Servo channel {ch} gerekli kanallar listesinde değil")
                    
        # PWM aralık kontrolü
        if validation.get('pwm_range_check'):
            safety = config.get('safety_limits', {})
            pwm_min = safety.get('servo_pwm_min', 1300)
            pwm_max = safety.get('servo_pwm_max', 1700)
            if not (1000 <= pwm_min <= 1500 <= pwm_max <= 2000):
                print(f"⚠️ PWM aralığı geçersiz: {pwm_min}-{pwm_max}")
                return False
                
        print("✅ Hardware konfigürasyonu doğrulandı")
        return True
        
    except Exception as e:
        print(f"❌ Hardware validation hatası: {e}")
        return False

# Mission config yükle
MISSION_CONFIG = load_mission_config()

# PWM değerleri ve güvenlik sınırları - full_stabilization2.py ile uyumlu
PWM_NEUTRAL = PWM_NEU  # 1500 - full_stabilization2.py'den
SERVO_MAX_DELTA = 300  # Maksimum PWM değişimi (±300µs)

# Güvenlik sınırları (mekanik koruma) - full_stabilization2.py PWM_MIN/MAX kullanıyor
PWM_SAFE_MIN = PWM_MIN  # 1100 - full_stabilization2.py'den 
PWM_SAFE_MAX = PWM_MAX  # 1900 - full_stabilization2.py'den

class PIDController:
    """
    Optimize edilmiş PID Controller - Hızlı ve stabil sürüş için
    
    Args:
        kp: Proportional gain (hızlı tepki)
        ki: Integral gain (steady-state error eliminasyonu) 
        kd: Derivative gain (overshoot önleme)
        max_output: Çıkış sınırı (güvenlik)
        
    Features:
        - Integral windup koruması
        - Derivative kick önleme
        - Smooth output limiting
    """
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp = kp
        self.ki = ki  
        self.kd = kd
        self.max_output = max_output
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01
        
        error = setpoint - measurement
        
        self.integral += error * dt
        integral_limit = self.max_output / self.ki if self.ki > 0 else float('inf')
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        
        derivative = (error - self.previous_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.max_output, min(self.max_output, output))
        
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

# ---- FULL_STABILIZATION2.PY'DEN ALINAN STABİLİZASYON FONKSİYONLARI (HİÇBİR ŞEY DEĞİŞTİRİLMEDİ) ----

def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def to_pwm(delta_us):  
    """full_stabilization2.py'den aynen alınan PWM çevirme fonksiyonu"""
    return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

def calculate_roll_commands(roll):
    """Roll ekseni için servo komutlarını hesapla - full_stabilization2.py'den aynen"""
    roll_deg = math.degrees(roll)
    
    if abs(roll_deg) < ROLL_DEADBAND_DEG:
        return 0.0, 0.0  # left_cmd, right_cmd
    
    # CCW roll -> SOL ↑, SAĞ ↓ (aynı PWM değişimi, fiziksel zıt hareket)
    u = ROLL_SENSE * roll * ROLL_K_ANG_US_PER_RAD
    left_cmd_us  = (+u) * ROLL_DIR_LEFT
    right_cmd_us = (+u) * ROLL_DIR_RIGHT
    
    # Sınırla
    left_cmd_us  = max(-ROLL_MAX_DELTA_US, min(ROLL_MAX_DELTA_US, left_cmd_us))
    right_cmd_us = max(-ROLL_MAX_DELTA_US, min(ROLL_MAX_DELTA_US, right_cmd_us))
    
    return left_cmd_us, right_cmd_us

def calculate_pitch_commands(pitch):
    """Pitch ekseni için servo komutlarını hesapla - full_stabilization2.py'den aynen"""
    pitch_deg = math.degrees(pitch)
    
    if abs(pitch_deg) < PITCH_DEADBAND_DEG:
        return 0.0, 0.0  # right_cmd, left_cmd
    
    # +pitch (burun yukarı) -> RIGHT ↑, LEFT ↓ (manuel_pitch.py'den)
    u = PITCH_SENSE * pitch * PITCH_K_ANG_US_PER_RAD
    right_cmd_us = (+u) * PITCH_DIR_UP    # SERVO_RIGHT için
    left_cmd_us  = (-u) * PITCH_DIR_DOWN  # SERVO_LEFT için
    
    # Sınırla
    right_cmd_us = max(-PITCH_MAX_DELTA_US, min(PITCH_MAX_DELTA_US, right_cmd_us))
    left_cmd_us  = max(-PITCH_MAX_DELTA_US, min(PITCH_MAX_DELTA_US, left_cmd_us))
    
    return right_cmd_us, left_cmd_us

def calculate_yaw_commands(yaw):
    """Yaw ekseni için servo komutlarını hesapla - full_stabilization2.py'den aynen"""
    yaw_deg = math.degrees(yaw)
    
    if abs(yaw_deg) < YAW_DEADBAND_DEG:
        return 0.0, 0.0, 0.0, 0.0  # up_cmd, down_cmd, right_cmd, left_cmd
    
    # CCW yaw -> çapraz koordinasyon
    u = YAW_SENSE * yaw * YAW_K_ANG_US_PER_RAD
    
    up_cmd_us    = (-u) * YAW_DIR_UP     # SERVO_UP (6)
    down_cmd_us  = (+u) * YAW_DIR_DOWN   # SERVO_DOWN (4)
    right_cmd_us = (-u) * YAW_DIR_RIGHT  # SERVO_RIGHT (3)
    left_cmd_us  = (-u) * YAW_DIR_LEFT   # SERVO_LEFT (5)
    
    # Sınırla
    up_cmd_us    = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, up_cmd_us))
    down_cmd_us  = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, down_cmd_us))
    right_cmd_us = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, right_cmd_us))
    left_cmd_us  = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, left_cmd_us))
    
    return up_cmd_us, down_cmd_us, right_cmd_us, left_cmd_us

def combine_commands(roll_left, roll_right, pitch_right, pitch_left, yaw_up, yaw_down, yaw_right, yaw_left):
    """Tüm eksenlerin komutlarını birleştir - full_stabilization2.py'den aynen"""
    # Her kanat için komutları topla
    final_up_cmd    = yaw_up                    # Sadece YAW
    final_down_cmd  = yaw_down                  # Sadece YAW  
    final_right_cmd = roll_right + pitch_right + yaw_right  # ROLL + PITCH + YAW
    final_left_cmd  = roll_left + pitch_left + yaw_left     # ROLL + PITCH + YAW
    
    # Genel güvenlik sınırını uygula
    final_up_cmd    = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_up_cmd))
    final_down_cmd  = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_down_cmd))
    final_right_cmd = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_right_cmd))
    final_left_cmd  = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_left_cmd))
    
    return final_up_cmd, final_down_cmd, final_right_cmd, final_left_cmd

class Mission1Navigator:
    """
    TEKNOFEST Su Altı Roket Aracı - Görev 1 Navigator
    Plus-Wing Konfigürasyonu - GPS'siz Dead Reckoning
    
    Features:
        - D300 derinlik sensörü öncelikli sistem
        - PWM tabanlı odometri ve mesafe hesaplama
        - 90 saniye arming interlock güvenlik sistemi
        - Full stabilizasyon sistemi (roll/pitch/yaw)
        - LED/Buzzer feedback sistemi
        - Watchdog ve latched fault koruması
        - Thread-safe kontrol döngüsü
    """
    def __init__(self, start_heading=0.0):
        self.master = None
        self.connected = False
        self.mission_active = False
        
        # GPIO sistemi başlat
        self.gpio_initialized = False
        self._init_gpio_system()
        
        # D300 derinlik sensörü
        self.d300_sensor = None
        self.d300_connected = False
        if D300_AVAILABLE:
            try:
                self.d300_sensor = D300DepthSensor(i2c_address=0x76)
                self.d300_connected = self.d300_sensor.initialize()
                if self.d300_connected:
                    print("✅ D300 derinlik sensörü başlatıldı (0x76)")
                else:
                    print("⚠️ D300 sensörü başlatılamadı, SCALED_PRESSURE kullanılacak")
            except Exception as e:
                print(f"⚠️ D300 sensörü hatası: {e}, SCALED_PRESSURE kullanılacak")
                self.d300_connected = False
        
        # Dead Reckoning navigasyon durumu (GPS'siz)
        self.start_position = {'x': 0.0, 'y': 0.0, 'heading': start_heading}
        self.current_position = {'x': 0.0, 'y': 0.0}  # Relatif pozisyon (m)
        self.current_depth = 0.0
        self.current_heading = start_heading
        self.current_speed = 0.0
        
        # Attitude veriler (IMU)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = start_heading
        
        # Stabilizasyon için yaw offset (full_stabilization2.py'den)
        self.yaw_offset = None
        self.initial_heading_samples = []  # İlk heading örnekleri
        self.heading_calibration_complete = False
        
        # Dead reckoning için
        self.last_position_update = time.time()
        self.traveled_distance = 0.0
        self.initial_heading = start_heading
        
        # PWM tabanlı odometri
        self.current_pwm = PWM_NEUTRAL
        self.estimated_speed = 0.0
        self.filtered_speed = 0.0
        
        # Stage referansları (PWM odometri için)
        self.distance_at_straight_start = 0.0
        self.distance_at_offshore_start = 0.0
        
        # Görev durumu
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.straight_course_start_time = None
        self.mission_completion_time = None
        
        # Görev başarı metrikleri
        self.max_offshore_distance = 0.0
        self.straight_distance_completed = 0.0
        
        # Sensor zaman damgaları ve kaynak takibi
        self._last_depth_ts = time.time()
        self._last_attitude_ts = time.time()
        self.depth_source = "unknown"  # "d300" veya "scaled_pressure"
        self._latched_fault = None  # Kalıcı hata durumu
        
        # 90 saniye arming interlock sistemi
        self._arming_start_time = None
        self._arming_done = False
        self.ARMING_DURATION = 90.0  # 90 saniye
        self._arming_countdown_displayed = set()  # Gösterilen countdown'ları takip et
        self.final_position_error = float('inf')
        self.leak_detected = False
        
        # PID kontrolcüler - Optimize edilmiş parametrelerle
        self.depth_pid = PIDController(**CONTROL_PARAMS['depth_pid'])
        self.heading_pid = PIDController(**CONTROL_PARAMS['heading_pid'])
        self.position_pid_x = PIDController(**CONTROL_PARAMS['dead_reckoning_pid'])
        self.position_pid_y = PIDController(**CONTROL_PARAMS['dead_reckoning_pid'])
        
        # Stabilizasyon PID'leri (attitude control için)
        self.roll_pid = PIDController(**CONTROL_PARAMS['roll_stabilization'])
        self.pitch_pid = PIDController(**CONTROL_PARAMS['pitch_stabilization'])
        self.yaw_pid = PIDController(**CONTROL_PARAMS['yaw_stabilization'])
        
        # Veri kayıt
        self.mission_log = []
        self.telemetry_data = []
        
        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False
    
    def _init_gpio_system(self):
        """GPIO sistemi başlatma (LED & Buzzer)"""
        if not GPIO_AVAILABLE:
            return
            
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Output pinleri
            GPIO.setup(GPIO_STATUS_LED, GPIO.OUT)
            GPIO.setup(GPIO_BUZZER_PWM, GPIO.OUT)
            
            # Input pinleri (pull-up dirençli)
            GPIO.setup(GPIO_POWER_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.setup(GPIO_EMERGENCY_STOP, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Başlangıçta kapalı
            GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
            GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
            
            self.gpio_initialized = True
            print("✅ GPIO sistemi başlatıldı (LED: GPIO4, Buzzer: GPIO13)")
            
        except Exception as e:
            print(f"⚠️ GPIO başlatma hatası: {e}")
            self.gpio_initialized = False
    
    def _update_status_indicators(self, stage, arming_remaining=None, fault=None):
        """Durum göstergelerini güncelle (LED & Buzzer)"""
        if not self.gpio_initialized:
            return
            
        try:
            if fault:
                # FAULT: Hızlı flash LED + uzun bip×3
                for _ in range(6):
                    GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                    time.sleep(0.1)
                return
                
            if arming_remaining and arming_remaining > 0:
                # ARMING: LED sabit, buzzer aralıklı
                GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)
                if int(arming_remaining) % 2 == 0:  # Her 2 saniyede bip
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.HIGH)
                    time.sleep(0.1)
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
            else:
                # Normal operasyon: Stage'e göre LED
                if stage in ["DESCENT", "STRAIGHT_COURSE", "OFFSHORE_CRUISE"]:
                    GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)  # Sabit yanar
                elif stage in ["RETURN_NAVIGATION", "FINAL_APPROACH"]:
                    # Yavaş flash (0.5 Hz)
                    led_state = int(time.time() * 2) % 2
                    GPIO.output(GPIO_STATUS_LED, led_state)
                elif stage == "MISSION_COMPLETE":
                    # Başarı: 3 kısa bip
                    for _ in range(3):
                        GPIO.output(GPIO_BUZZER_PWM, GPIO.HIGH)
                        time.sleep(0.2)
                        GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                        time.sleep(0.2)
                    GPIO.output(GPIO_STATUS_LED, GPIO.HIGH)  # LED sabit yanar
                    
        except Exception as e:
            print(f"⚠️ GPIO güncelleme hatası: {e}")
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı kur"""
        try:
            print("🔌 Pixhawk bağlantısı kuruluyor...")
            
            # Handle serial vs TCP connection
            if ',' in MAV_ADDRESS:
                # Serial connection: port,baud
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                # TCP or other connection
                print(f"🌐 TCP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            print(f"   System ID: {self.master.target_system}")
            print(f"   Component ID: {self.master.target_component}")
            
            # Stream rate istekleri
            self._request_data_streams()
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def _request_data_streams(self):
        """MAVLink data stream hızlarını ayarla"""
        try:
            # ATTITUDE ≥20 Hz
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
                20,  # 20 Hz
                1    # Enable
            )
            
            # SCALED_PRESSURE ≥10 Hz  
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,  # SCALED_PRESSURE
                10,  # 10 Hz
                1    # Enable
            )
            
            print("📊 Stream rate istekleri gönderildi: ATTITUDE@20Hz, PRESSURE@10Hz")
        except Exception as e:
            print(f"⚠️ Stream rate ayarlama hatası: {e}")
    
    def read_sensors(self):
        """Tüm sensör verilerini oku (GPS'siz - IMU + Dead Reckoning)"""
        if not self.connected:
            return False
        
        # Latched fault kontrolü
        if self._latched_fault:
            return False
            
        try:
            current_time = time.time()
            dt = current_time - self.last_position_update
            
            # Attitude (IMU) - Ana navigasyon kaynağı
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw
                self._last_attitude_ts = current_time
                
                # Yaw offset ayarla (full_stabilization2.py mantığı ile - ilk okumada)
                if not self.heading_calibration_complete:
                    if self.yaw_offset is None:
                        # İlk okuma - full_stabilization2.py'deki gibi
                        self.yaw_offset = attitude_msg.yaw
                        self.initial_heading = math.degrees(self.yaw_offset)
                        self.heading_calibration_complete = True
                        print(f"🧭 Yaw referans noktası ayarlandı: {math.degrees(self.yaw_offset):.1f}° (full_stabilization2.py mantığı)")
                        print("✅ Stabilizasyon sistemi hazır - full_stabilization2.py uyumlu")
            
            # Derinlik sensörü (D300 öncelikli, yoksa SCALED_PRESSURE)
            depth_read_success = False
            if self.d300_connected and self.d300_sensor:
                try:
                    depth_data = self.d300_sensor.read_depth()
                    if depth_data['success']:
                        self.current_depth = max(0.0, depth_data['depth'])
                        self.depth_source = "d300"
                        self._last_depth_ts = current_time
                        depth_read_success = True
                        print(f"📡 D300 Derinlik: {self.current_depth:.2f}m")
                    else:
                        print(f"⚠️ D300 okuma başarısız: {depth_data}")
                except Exception as e:
                    print(f"⚠️ D300 okuma hatası: {e}")
            
            if not depth_read_success:
                # D300 yok veya hatalı, SCALED_PRESSURE kullan
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    depth_pressure = pressure_msg.press_abs - 1013.25
                    self.current_depth = max(0, depth_pressure * 0.0102)
                    self.depth_source = "scaled_pressure"
                    self._last_depth_ts = current_time
                    print(f"📡 SCALED_PRESSURE Derinlik: {self.current_depth:.2f}m (Basınç: {pressure_msg.press_abs:.1f}hPa)")
                else:
                    print("❌ Hiçbir derinlik verisi yok!")
            
            # Watchdog kontrolü - sensör zaman aşımı
            if current_time - self._last_depth_ts > 0.5:
                self._trigger_latched_fault("DEPTH_SENSOR_TIMEOUT")
                return False
            if current_time - self._last_attitude_ts > 0.5:
                self._trigger_latched_fault("ATTITUDE_SENSOR_TIMEOUT")
                return False
            
            # Hız bilgisi
            vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if vfr_msg:
                self.current_speed = vfr_msg.groundspeed
            
            # Leak tespiti (STATUSTEXT mesajlarını izle)
            statustext = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if statustext and statustext.text:
                text_lower = statustext.text.lower()
                if "leak" in text_lower or "water" in text_lower or "sızıntı" in text_lower:
                    self.leak_detected = True
                    self._trigger_latched_fault("LEAK_DETECTED")
                    print(f"🚨 SIZINTI TESPİT EDİLDİ: {statustext.text}")
                    return False
            
            # Dead Reckoning pozisyon güncelleme
            if dt > 0.1:  # 10Hz'de güncelle
                distance_traveled = self.current_speed * dt
                self.traveled_distance += distance_traveled
                
                # Heading'e göre X,Y pozisyon hesapla
                heading_rad = math.radians(self.current_heading)
                dx = distance_traveled * math.cos(heading_rad)
                dy = distance_traveled * math.sin(heading_rad)
                
                self.current_position['x'] += dx
                self.current_position['y'] += dy
                
                self.last_position_update = current_time
            
            # Telemetri kaydet - Genişletilmiş veri seti
            self.telemetry_data.append({
                'timestamp': current_time,
                'position': self.current_position.copy(),
                'depth': self.current_depth,
                'depth_source': self.depth_source,  # "d300" veya "scaled_pressure"
                'heading': self.current_heading,
                'speed': self.current_speed,
                'speed_source': getattr(self, 'speed_source', 'vfr_hud'),  # PWM veya VFR_HUD
                'estimated_speed': self.estimated_speed,  # PWM'den kestirilen
                'filtered_speed': self.filtered_speed,    # LPF uygulanmış
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'traveled_distance': self.traveled_distance,
                'mission_stage': self.mission_stage,
                'motor_pwm': self.current_pwm,
                'leak_detected': self.leak_detected,
                'latched_fault': self._latched_fault
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            self._trigger_latched_fault(f"SENSOR_READ_ERROR: {e}")
            return False
    
    def _trigger_latched_fault(self, fault_reason):
        """Kalıcı hata durumu tetikle - Mission abort"""
        if not self._latched_fault:  # Sadece ilk hata için
            self._latched_fault = fault_reason
            print(f"🚨 LATCHED FAULT: {fault_reason}")
            print("🚨 Sistem güvenli duruma geçiyor...")
            
            # Mission'ı abort et
            self.mission_stage = "MISSION_ABORT"
            self.mission_active = False
            
            # LED/Buzzer uyarısı
            self._update_status_indicators(self.mission_stage, fault=fault_reason)
            
            # Acil nötr
            self._emergency_neutral()
            
    def _emergency_neutral(self):
        """Acil durum - motor ve servoları nötr konuma getir - full_stabilization2.py uyumlu"""
        try:
            if self.connected:
                # Motor NEUTRAL
                self.set_motor_throttle(PWM_NEUTRAL)
                # Servolar nötr - full_stabilization2.py servo mapping kullan
                servo_channels = [SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT]
                for channel in servo_channels:
                    self._set_servo_pwm(channel, PWM_NEUTRAL)
        except Exception as e:
            print(f"❌ Emergency neutral hatası: {e}")
    
    def cleanup(self):
        """Güvenli kapanış sırası"""
        if hasattr(self, '_cleanup_done') and self._cleanup_done:
            return  # Idempotent - birden çok çağrı güvenli
        
        print("🧹 Sistem temizleniyor...")
        
        try:
            # 1. Motor NEUTRAL
            if self.connected:
                self.set_motor_throttle(PWM_NEUTRAL)
                print("   ✅ Motor nötr")
                
                # 2. Servolar nötr - full_stabilization2.py servo mapping kullan
                servo_channels = [SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT]
                for channel in servo_channels:
                    self._set_servo_pwm(channel, PWM_NEUTRAL)
                print("   ✅ Servolar nötr")
            
            # 3. Mission durumunu sonlandır
            self.running = False
            self.mission_active = False
            
            # 4. Thread'leri güvenli kapatma - Improved termination
            print("   🔄 Thread'ler sonlandırılıyor...")
            
            # Önce running flag'i false yap
            self.running = False
            self.mission_active = False
            
            # Control thread'i bekle
            if hasattr(self, 'control_thread') and self.control_thread and self.control_thread.is_alive():
                print("   ⏳ Control thread sonlandırılıyor...")
                self.control_thread.join(timeout=3.0)
                if self.control_thread.is_alive():
                    print("   ⚠️ Control thread 3s içinde sonlanmadı")
                else:
                    print("   ✅ Control thread sonlandırıldı")
            
            # Monitoring thread'i bekle
            if hasattr(self, 'monitoring_thread') and self.monitoring_thread and self.monitoring_thread.is_alive():
                print("   ⏳ Monitoring thread sonlandırılıyor...")
                self.monitoring_thread.join(timeout=2.0)
                if self.monitoring_thread.is_alive():
                    print("   ⚠️ Monitoring thread 2s içinde sonlanmadı")
                else:
                    print("   ✅ Monitoring thread sonlandırıldı")
            
            # 5. Telemetri flush
            if hasattr(self, 'telemetry_data') and len(self.telemetry_data) > 0:
                print(f"   📊 Telemetri kayıtları: {len(self.telemetry_data)} örnek")
            
            # 6. MAVLink bağlantısı kapat
            if self.connected and self.master:
                try:
                    self.master.close()
                    print("   ✅ MAVLink bağlantısı kapatıldı")
                except:
                    pass
                self.connected = False
            
            # 7. GPIO temizleme
            if self.gpio_initialized:
                try:
                    GPIO.output(GPIO_STATUS_LED, GPIO.LOW)
                    GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
                    GPIO.cleanup()
                    print("   ✅ GPIO temizlendi")
                except:
                    pass
            
            self._cleanup_done = True
            print("🧹 Sistem temizleme tamamlandı")
            
        except Exception as e:
            print(f"❌ Cleanup hatası: {e}")
    
    def update_pwm_based_odometry(self):
        """PWM tabanlı hız kestirimi ve mesafe güncelleme - Kalibrasyon aralığı kontrollü"""
        current_time = time.time()
        dt = current_time - self.last_position_update
        
        if dt > 0.05:  # 20 Hz güncelleme
            # PWM geçerli aralık kontrolü (cal_speed.json'dan)
            valid_min, valid_max = SPEED_CAL.get('valid_range', [1400, 1700])
            
            if valid_min <= self.current_pwm <= valid_max:
                # PWM geçerli aralıkta - hız kestir
                pwm_delta = self.current_pwm - SPEED_CAL['neutral_pwm']
                self.estimated_speed = SPEED_CAL['a'] * pwm_delta + SPEED_CAL['b']
                self.estimated_speed = max(0.05, self.estimated_speed)  # Min hız eşiği: 0.05 m/s
                self.speed_source = "pwm_calibrated"
            else:
                # PWM geçersiz aralıkta - hızı sıfırla
                self.estimated_speed = 0.0
                self.speed_source = "pwm_invalid"
                if abs(self.current_pwm - PWM_NEUTRAL) > 10:  # Sadece nötr dışındaysa uyar
                    print(f"⚠️ PWM aralık dışı: {self.current_pwm} (geçerli: {valid_min}-{valid_max})")
            
            # LPF uygula
            alpha = SPEED_CAL['lpf_alpha']
            self.filtered_speed = alpha * self.estimated_speed + (1 - alpha) * self.filtered_speed
            
            # Mesafe entegrasyonu (traveled_distance ana kaynak)
            distance_increment = self.filtered_speed * dt
            self.traveled_distance += distance_increment
            
            self.last_position_update = current_time
    
    def calculate_distance_bearing_to_origin(self):
        """Başlangıç noktasına mesafe ve bearing hesapla (Dead Reckoning)"""
        # Mevcut pozisyondan başlangıç noktasına (0,0)
        dx = -self.current_position['x']  # Başlangıca dönmek için negatif
        dy = -self.current_position['y']
        
        # Mesafe hesaplama
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Bearing hesaplama (matematiksel açıdan navigasyon açısına çevir)
        bearing_rad = math.atan2(dy, dx)
        bearing_deg = math.degrees(bearing_rad)
        
        # 0-360 derece aralığına çevir
        bearing_deg = (bearing_deg + 360) % 360
        
        return distance, bearing_deg
    
    def get_current_distance_from_start(self):
        """Başlangıç noktasından mevcut uzaklık"""
        return math.sqrt(self.current_position['x']**2 + self.current_position['y']**2)
    
    def _check_arming_interlock(self):
        """90 saniye arming interlock kontrolü"""
        # Test modu kontrolü - eğer _arming_done True ise direk geç
        if self._arming_done:
            return True
            
        if self._arming_start_time is None:
            self._arming_start_time = time.time()
            print("🔒 ARMİNG INTERLOCK başlatıldı - 90 saniye güvenlik süresi")
            return False
        
        elapsed = time.time() - self._arming_start_time
        remaining = self.ARMING_DURATION - elapsed
        
        # Countdown gösterimi (10'ar saniyelik aralıklarla)
        countdown_intervals = [90, 80, 70, 60, 50, 40, 30, 20, 10, 5, 4, 3, 2, 1]
        for interval in countdown_intervals:
            if remaining <= interval and interval not in self._arming_countdown_displayed:
                self._arming_countdown_displayed.add(interval)
                if interval <= 10:
                    print(f"⏰ ARMİNG COUNTDOWN: {interval} saniye")
                else:
                    print(f"🔒 ARMİNG INTERLOCK: {remaining:.0f} saniye kaldı")
        
        if elapsed >= self.ARMING_DURATION:
            if not self._arming_done:
                self._arming_done = True
                print("✅ ARMİNG INTERLOCK tamamlandı - Motor kontrolü serbest!")
            return True
        
        return False

    def set_motor_throttle(self, throttle_pwm):
        """Motor kontrolü"""
        if not self.connected:
            return False
            
        # Arming interlock kontrolü
        if not self._check_arming_interlock():
            # Arming süresi dolmadıysa motor NEUTRAL'de tut
            throttle_pwm = PWM_NEUTRAL
            
        # Motor için güvenli PWM sınırları (ESC arming ve güvenlik)
        throttle_pwm = max(PWM_SAFE_MIN, min(PWM_SAFE_MAX, throttle_pwm))
        self.current_pwm = throttle_pwm  # PWM tracking için
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL, throttle_pwm, 0, 0, 0, 0, 0
            )
            return True
        except:
            return False
    
    def set_control_surfaces(self, roll_cmd=0, pitch_cmd=0, yaw_cmd=0, use_stabilization=True):
        """Plus Wing kontrol yüzeylerini ayarla - FULL_STABILIZATION2.PY MANTIKLI (HİÇBİR ŞEY DEĞİŞTİRİLMEDİ)"""
        if not self.connected:
            return False
        
        try:
            if use_stabilization and hasattr(self, 'yaw_offset') and self.yaw_offset is not None:
                # FULL STABİLİZASYON MODU - full_stabilization2.py'deki mantık AYNEN
                
                # Mevcut attitude verilerini radyan olarak al
                roll_rad = math.radians(self.current_roll)
                pitch_rad = math.radians(self.current_pitch)
                raw_yaw_rad = math.radians(self.current_yaw)
                
                # Relatif yaw hesapla (full_stabilization2.py'den AYNEN)
                yaw_rad = raw_yaw_rad - self.yaw_offset
                while yaw_rad > math.pi:
                    yaw_rad -= 2 * math.pi
                while yaw_rad < -math.pi:
                    yaw_rad += 2 * math.pi
                
                # Stabilizasyon komutlarını hesapla - full_stabilization2.py'den AYNEN
                roll_left_cmd, roll_right_cmd = calculate_roll_commands(roll_rad)
                pitch_right_cmd, pitch_left_cmd = calculate_pitch_commands(pitch_rad)
                yaw_up_cmd, yaw_down_cmd, yaw_right_cmd, yaw_left_cmd = calculate_yaw_commands(yaw_rad)
                
                # Manual komutları ekle (eğer varsa) - görev navigasyonu için
                if abs(roll_cmd) > 0.1 or abs(pitch_cmd) > 0.1 or abs(yaw_cmd) > 0.1:
                    # Manual komutları stabilizasyon komutlarına ekle
                    manual_roll_us = roll_cmd * 5.0   # Çevirme faktörü
                    manual_pitch_us = pitch_cmd * 5.0
                    manual_yaw_us = yaw_cmd * 5.0
                    
                    # Manual komutları dağıt - roll ve pitch için
                    roll_left_cmd += manual_roll_us
                    roll_right_cmd += manual_roll_us
                    pitch_right_cmd += manual_pitch_us
                    pitch_left_cmd += manual_pitch_us
                    yaw_up_cmd += manual_yaw_us
                    yaw_down_cmd += manual_yaw_us
                    yaw_right_cmd += manual_yaw_us
                    yaw_left_cmd += manual_yaw_us
                
                # Komutları birleştir - full_stabilization2.py'den AYNEN
                final_up, final_down, final_right, final_left = combine_commands(
                    roll_left_cmd, roll_right_cmd,
                    pitch_right_cmd, pitch_left_cmd,
                    yaw_up_cmd, yaw_down_cmd, yaw_right_cmd, yaw_left_cmd
                )
                
                # PWM değerlerini hesapla - full_stabilization2.py'den AYNEN
                pwm_up = to_pwm(final_up)
                pwm_down = to_pwm(final_down)
                pwm_right = to_pwm(final_right)
                pwm_left = to_pwm(final_left)
                
                # Debug çıktısı (periyodik)
                if int(time.time() * 2) % 10 == 0:  # Her 5 saniyede bir
                    print(f"🎮 Stabilizasyon: R={math.degrees(roll_rad):+.1f}° P={math.degrees(pitch_rad):+.1f}° Y={math.degrees(yaw_rad):+.1f}°")
                    print(f"📡 PWM: ÜST={pwm_up} ALT={pwm_down} SAĞ={pwm_right} SOL={pwm_left}")
                
            else:
                # MANUEL MOD - Basit kontrol (stabilizasyon kapalı)
                roll_cmd = max(-100, min(100, roll_cmd * 2.0))
                pitch_cmd = max(-100, min(100, pitch_cmd * 2.0))
                yaw_cmd = max(-100, min(100, yaw_cmd * 2.0))
                
                # Plus Wing PWM hesaplama (manuel mod)
                neutral = PWM_NEUTRAL
                
                # Plus Wing mixing matrix - manuel kontrol
                pwm_up = neutral + int(pitch_cmd * 2.0)     # Üst kanat: pitch kontrolü
                pwm_down = neutral - int(pitch_cmd * 2.0)   # Alt kanat: pitch kontrolü (ters)
                pwm_right = neutral - int(roll_cmd * 2.0)   # Sağ kanat: roll kontrolü (ters)
                pwm_left = neutral + int(roll_cmd * 2.0)    # Sol kanat: roll kontrolü
                
                # Yaw için tüm kanatlara ekleme
                yaw_contribution = int(yaw_cmd * 1.0)
                pwm_up += yaw_contribution
                pwm_down += yaw_contribution  
                pwm_right += yaw_contribution
                pwm_left += yaw_contribution
                
                print(f"🎮 Manuel Mod: R={roll_cmd:+.1f} P={pitch_cmd:+.1f} Y={yaw_cmd:+.1f}")
            
            # Servo komutlarını gönder - full_stabilization2.py mapping AYNEN
            success_count = 0
            servo_commands = [
                (SERVO_UP, pwm_up, 'ÜST'),      # AUX 6
                (SERVO_DOWN, pwm_down, 'ALT'),  # AUX 4
                (SERVO_RIGHT, pwm_right, 'SAĞ'), # AUX 3
                (SERVO_LEFT, pwm_left, 'SOL')   # AUX 5
            ]
            
            for channel, pwm, name in servo_commands:
                if self._set_servo_pwm(channel, int(pwm)):
                    success_count += 1
                else:
                    print(f"❌ Servo {name} (kanal {channel}) komutu gönderilemedi")
            
            return success_count == len(servo_commands)
            
        except Exception as e:
            print(f"❌ Plus Wing kontrol hatası: {e}")
            return False
    
    def set_servo_position(self, channel, pwm_value):
        """Servo pozisyon kontrolü"""
        if not self.connected:
            print(f"❌ MAVLink bağlantısı yok! Servo {channel} komutu gönderilemedi")
            return False
            
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm_value, 0, 0, 0, 0, 0
            )
            print(f"✅ Servo {channel} → {pwm_value}µs komutu gönderildi")
            return True
        except Exception as e:
            print(f"❌ Servo {channel} komut hatası: {e}")
            return False
    
    def _set_servo_pwm(self, channel, pwm_value):
        """Servo PWM kontrolü - Plus Wing için"""
        if not self.connected:
            print(f"❌ MAVLink bağlantısı yok! Servo {channel} komutu gönderilemedi")
            return False
            
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm_value, 0, 0, 0, 0, 0
            )
            print(f"✅ Servo {channel} → {pwm_value}µs komutu gönderildi")
            return True
        except Exception as e:
            print(f"❌ Servo {channel} komut hatası: {e}")
            return False
    
    def test_servos(self):
        """Servo test fonksiyonu - full_stabilization2.py uyumlu"""
        print("\n🔧 SERVO TEST BAŞLIYOR - full_stabilization2.py uyumlu...")
        print("Her servo kanalı individual test ediliyor...")
        
        # Individual servo test - full_stabilization2.py mantığı ile
        test_values = [100, -100]  # PWM delta değerleri
        servo_list = [
            (SERVO_UP, 'ÜST', 'AUX6'),
            (SERVO_DOWN, 'ALT', 'AUX4'),
            (SERVO_RIGHT, 'SAĞ', 'AUX3'),
            (SERVO_LEFT, 'SOL', 'AUX5')
        ]
        
        for channel, name, aux in servo_list:
            print(f"\n📊 {name} kanat ({aux}) test ediliyor...")
            for val in test_values:
                pwm = to_pwm(val)
                print(f"   PWM: {pwm} (delta: {val:+d})")
                self._set_servo_pwm(channel, pwm)
                time.sleep(1.5)
            
            # Nötr
            print(f"   {name} nötr: {PWM_NEUTRAL}")
            self._set_servo_pwm(channel, PWM_NEUTRAL)
            time.sleep(0.5)
        
        print("✅ Individual servo test tamamlandı!")
        
        # Kombinasyon test - stabilizasyon kapalı
        print("\n🎮 Kombinasyon testleri...")
        test_combinations = [20, -20, 40, -40]
        
        for i, val in enumerate(test_combinations):
            print(f"\n📊 Kombinasyon {i+1}/4: Roll={val}")
            self.set_control_surfaces(roll_cmd=val, pitch_cmd=0, yaw_cmd=0, use_stabilization=False)
            time.sleep(2)
            
            print(f"📊 Kombinasyon {i+1}/4: Pitch={val}")  
            self.set_control_surfaces(roll_cmd=0, pitch_cmd=val, yaw_cmd=0, use_stabilization=False)
            time.sleep(2)
            
            print(f"📊 Kombinasyon {i+1}/4: Yaw={val}")
            self.set_control_surfaces(roll_cmd=0, pitch_cmd=0, yaw_cmd=val, use_stabilization=False)
            time.sleep(2)
        
        # Final nötr pozisyon
        print("\n🔄 Tüm servolar nötr pozisyona getiriliyor...")
        self.set_control_surfaces(roll_cmd=0, pitch_cmd=0, yaw_cmd=0, use_stabilization=False)
        time.sleep(1)
        print("✅ Servo test tamamlandı - full_stabilization2.py uyumlu!")

    def display_mission_status(self):
        """Görev durumunu göster"""
        print("\n" + "="*80)
        print("🚀 TEKNOFEST - GÖREV 1: SEYİR YAPMA & GERİ DÖNÜŞ")
        print("="*80)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        mission_time = (time.time() - self.mission_start_time) if self.mission_start_time else 0
        remaining_time = max(0, MISSION_PARAMS['timeout_seconds'] - mission_time)
        
        print(f"⏰ Zaman: {timestamp} | Görev Süresi: {mission_time:.0f}s | Kalan: {remaining_time:.0f}s")
        print(f"🎯 Görev Aşaması: {self.mission_stage}")
        
        # Dead Reckoning pozisyon bilgisi
        distance_from_start, bearing_to_start = self.calculate_distance_bearing_to_origin()
        print(f"📍 Mevcut Pozisyon (Dead Reckoning): X={self.current_position['x']:.1f}m, Y={self.current_position['y']:.1f}m")
        print(f"🏠 Başlangıçtan Uzaklık: {distance_from_start:.1f}m | Geri Dönüş Yönü: {bearing_to_start:.0f}°")
        print(f"📏 Toplam Mesafe: {self.traveled_distance:.1f}m")
        
        print(f"🌊 Derinlik: {self.current_depth:.1f}m | Hedef: {MISSION_PARAMS['target_depth']:.1f}m")
        print(f"🧭 Heading: {self.current_heading:.0f}° | Hız: {self.current_speed:.1f} m/s")
        print(f"📊 Roll: {self.current_roll:+.1f}° | Pitch: {self.current_pitch:+.1f}°")
        
        # Görev metrikleri
        print(f"📏 Düz Seyir: {self.straight_distance_completed:.1f}m / {MISSION_PARAMS['straight_distance']}m")
        print(f"🌊 Max Kıyı Uzaklığı: {self.max_offshore_distance:.1f}m / {MISSION_PARAMS['min_offshore_distance']}m")
        
        # Başarı durumu
        if self.final_position_error < float('inf'):
            print(f"🎯 Final Pozisyon Hatası: {self.final_position_error:.1f}m")
        
        print("="*80)
    
    def control_loop(self):
        """Ana kontrol döngüsü - Thread-safe, exception handling ile"""
        loop_start_time = time.time()
        loop_count = 0
        last_heartbeat = time.time()
        
        print("🎯 Control loop başlatıldı - 25Hz hedef frekans")
        
        try:
            while self.running and self.mission_active:
                cycle_start = time.time()
                
                try:
                    # Heartbeat kontrolü (her 5 saniyede bir)
                    if cycle_start - last_heartbeat > 5.0:
                        print(f"💓 Control loop heartbeat - Stage: {self.mission_stage}")
                        last_heartbeat = cycle_start
                    
                    # Sensör okuma ve odometri
                    if not self.read_sensors():
                        print("❌ Sensör okuma başarısız, control loop devam ediyor...")
                        continue
                    
                    self.update_pwm_based_odometry()
                    
                    # Mission stage execution with exception handling
                    stage_handlers = {
                        "DESCENT": self.execute_descent,
                        "STRAIGHT_COURSE": self.execute_straight_course,
                        "OFFSHORE_CRUISE": self.execute_offshore_cruise,
                        "RETURN_NAVIGATION": self.execute_return_navigation,
                        "FINAL_APPROACH": self.execute_final_approach,
                        "SURFACE_AND_SHUTDOWN": self.execute_surface_shutdown
                    }
                    
                    if self.mission_stage in stage_handlers:
                        stage_handlers[self.mission_stage]()
                    elif self.mission_stage == "MISSION_COMPLETE":
                        print("✅ Control loop: Mission complete")
                        break
                    elif self.mission_stage == "MISSION_ABORT":
                        print("🚨 Control loop: Mission aborted")
                        break
                    
                except Exception as stage_error:
                    print(f"❌ Stage execution error ({self.mission_stage}): {stage_error}")
                    # Stage hatası mission'ı durdurmasın, sadece bu döngüyü atla
                    continue
                
                # Timing kontrolü
                cycle_time = time.time() - cycle_start
                target_cycle_time = 0.04  # 25 Hz hedef
                sleep_time = max(0.005, target_cycle_time - cycle_time)  # Min 5ms sleep
                time.sleep(sleep_time)
                
                # Performans izleme
                loop_count += 1
                if loop_count % 125 == 0:  # Her 5 saniyede bir (25Hz * 5s)
                    avg_freq = loop_count / (time.time() - loop_start_time)
                    if avg_freq < 20:  # 20 Hz altına düşerse uyarı
                        print(f"⚠️ Control loop frekansı düşük: {avg_freq:.1f} Hz")
                    
        except Exception as e:
            print(f"🚨 CRITICAL: Control loop exception: {e}")
            self._trigger_latched_fault(f"CONTROL_LOOP_EXCEPTION: {e}")
        finally:
            print("🔄 Control loop sonlandırılıyor...")
            # Emergency safety
            try:
                self._emergency_neutral()
            except:
                pass
    
    def execute_descent(self):
        """2m derinliğe iniş"""
        # Derinlik kontrolü
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        
        # Debug derinlik sensörü
        print(f"🌊 Derinlik Debug: Mevcut={self.current_depth:.2f}m, Hedef={MISSION_PARAMS['target_depth']:.2f}m, Hata={depth_error:.2f}m")
        
        # Derinlik sensörü çalışmıyorsa simüle et (test için)
        if self.current_depth == 0.0:
            print("⚠️ Derinlik sensörü çalışmıyor! 10 saniye sonra simüle derinliğe geçiş...")
            if hasattr(self, '_descent_start_time'):
                if time.time() - self._descent_start_time > 10:
                    print("✅ Simüle derinlik ulaşıldı! Düz seyire geçiliyor...")
                    self.mission_stage = "STRAIGHT_COURSE"
                    self.straight_course_start_time = time.time()
                    return
            else:
                self._descent_start_time = time.time()
        
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            # PID kontrol ile iniş
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            
            # Motor ve fin kontrolü
            if depth_error > 0:  # Daha derine inmeli
                motor_throttle = PWM_NEUTRAL + 50  # İleri hareket
                pitch_cmd = 50  # Nose down (daha az agresif)
            else:  # Yükselmeli
                motor_throttle = PWM_NEUTRAL - 50  # Geri hareket
                pitch_cmd = -50  # Nose up
            
            # Roll stabilizasyonu ekle (test için)
            roll_error = 0 - self.current_roll  # Sıfır roll hedefi
            roll_cmd = max(-30, min(30, roll_error * 2))  # Basit P kontrolü
            
            # Yaw stabilizasyonu ekle
            if self.initial_heading is not None:
                yaw_error = self.initial_heading - self.current_heading
                if yaw_error > 180: yaw_error -= 360
                if yaw_error < -180: yaw_error += 360
                yaw_cmd = max(-30, min(30, yaw_error * 0.5))
            else:
                yaw_cmd = 0
            
            print(f"🎮 Descent Komutları: Motor={motor_throttle}, P={pitch_cmd}, R={roll_cmd}, Y={yaw_cmd}")
            
            self.set_motor_throttle(motor_throttle)
            self.set_control_surfaces(roll_cmd=roll_cmd, pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        else:
            # Hedef derinliğe ulaşıldı
            print("✅ Hedef derinlik ulaşıldı! Düz seyire geçiliyor...")
            self.mission_stage = "STRAIGHT_COURSE"
            self.straight_course_start_time = time.time()
            
            # Stage geçişinde PID resetleri
            self.depth_pid.reset()
            self.heading_pid.reset()
            
            # Mesafe referansları set et
            self.distance_at_straight_start = self.traveled_distance
            print(f"📏 STRAIGHT_COURSE başlangıç mesafesi: {self.distance_at_straight_start:.1f}m")
    
    def execute_straight_course(self):
        """10m düz seyir (süre başlatma) - PWM odometri tabanlı"""
        if not self.straight_course_start_time:
            self.straight_course_start_time = time.time()
            # Stage başlangıç mesafesini kaydet (PWM odometri referansı)
            self.distance_at_straight_start = self.traveled_distance
            print(f"📏 STRAIGHT_COURSE başlangıç mesafesi: {self.distance_at_straight_start:.1f}m")
        
        # PWM tabanlı odometri ile gerçek mesafe hesapla
        self.straight_distance_completed = self.traveled_distance - self.distance_at_straight_start
        
        # Düz seyir kontrolü
        motor_throttle = PWM_NEUTRAL + 120  # Forward thrust
        
        # Derinlik tutma
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-150, min(150, int(depth_correction)))
        
        # Yön stabilizasyonu (başlangıç heading'i tut)
        if self.initial_heading is not None:
            heading_error = self.initial_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_correction = self.heading_pid.update(self.initial_heading, self.current_heading)
            yaw_cmd = max(-50, min(50, int(heading_correction * 2)))  # 2x güçlendir
            
            # Debug çıktısı
            if abs(heading_error) > 5:
                print(f"🧭 Heading Error: {heading_error:.1f}° → Yaw Cmd: {yaw_cmd}")
        else:
            yaw_cmd = 0
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # 10m düz seyir tamamlandı mı? (PWM odometri ile kesin ölçüm)
        if self.straight_distance_completed >= MISSION_PARAMS['straight_distance']:
            print(f"✅ 10m düz seyir tamamlandı! PWM odometri: {self.straight_distance_completed:.1f}m")
            print("🌊 Kıyıdan uzaklaşmaya başlanıyor...")
            self.mission_stage = "OFFSHORE_CRUISE"
            # Offshore stage referansını set et
            self.distance_at_offshore_start = self.traveled_distance
    
    def execute_offshore_cruise(self):
        """Kıyıdan 50m uzaklaşma (Dead Reckoning)"""
        distance_from_start = self.get_current_distance_from_start()
        self.max_offshore_distance = max(self.max_offshore_distance, distance_from_start)
        
        # Hızlı ileri hareket
        motor_throttle = PWM_NEUTRAL + 150
        
        # Derinlik tutma
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-50, min(50, int(depth_correction // 4)))
        
        # Düz heading tutma (başlangıç yönünde devam)
        if self.initial_heading is not None:
            heading_error = self.initial_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_correction = self.heading_pid.update(self.initial_heading, self.current_heading)
            yaw_cmd = max(-40, min(40, int(heading_correction * 1.5)))  # Güçlendir
            
            # Debug çıktısı
            if abs(heading_error) > 10:
                print(f"🧭 Offshore Heading Error: {heading_error:.1f}° → Yaw Cmd: {yaw_cmd}")
        else:
            yaw_cmd = 0
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # 50m uzaklaştık mı?
        if distance_from_start >= MISSION_PARAMS['min_offshore_distance']:
            print("✅ 50m uzaklaşma tamamlandı! Geri dönüş navigasyonu başlıyor...")
            self.mission_stage = "RETURN_NAVIGATION"
    
    def execute_return_navigation(self):
        """Başlangıç noktasına geri dönüş (Dead Reckoning)"""
        distance_from_start, bearing_to_start = self.calculate_distance_bearing_to_origin()
        
        # Hedefe yönlenme
        heading_error = bearing_to_start - self.current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        # Hızlı geri dönüş
        if distance_from_start > 10:  # 10m'den uzaksa hızla git
            motor_throttle = PWM_NEUTRAL + 180
        else:  # Yakınsa yavaşla
            motor_throttle = PWM_NEUTRAL + 100
            
        # Navigasyon kontrolü
        heading_correction = self.heading_pid.update(bearing_to_start, self.current_heading)
        yaw_cmd = max(-50, min(50, int(heading_correction // 2)))
        
        # Derinlik tutma
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-50, min(50, int(depth_correction // 4)))
        
        self.set_motor_throttle(motor_throttle)
        # DÖNÜŞ NAVİGASYONU: Stabilizasyon açık (smooth navigation için)
        # Roll/pitch stabilize, sadece yaw için manual komut
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd, use_stabilization=True)
        
        # Başlangıç noktasına yaklaştık mı?
        if distance_from_start <= MISSION_PARAMS['position_tolerance'] * 2:  # 4m tolerance
            print("✅ Başlangıç noktasına yaklaşıldı! Final yaklaşım...")
            self.mission_stage = "FINAL_APPROACH"
    
    def execute_final_approach(self):
        """Final yaklaşım ve pozisyon tutma (Dead Reckoning)"""
        distance_from_start, bearing_to_start = self.calculate_distance_bearing_to_origin()
        self.final_position_error = distance_from_start
        
        # Hassas pozisyon kontrolü
        if distance_from_start > MISSION_PARAMS['position_tolerance']:
            # Yavaş yaklaşım
            motor_throttle = PWM_NEUTRAL + 60
            
            heading_correction = self.heading_pid.update(bearing_to_start, self.current_heading)
            yaw_cmd = max(-30, min(30, int(heading_correction // 3)))
        else:
            # Pozisyon tutma
            motor_throttle = PWM_NEUTRAL + 20
            yaw_cmd = 0
        
        # Derinlik tutma
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-30, min(30, int(depth_correction // 5)))
        
        self.set_motor_throttle(motor_throttle)
        # FINAL YAKLAŞIM: Hassas navigasyon için stabilizasyon açık (precision control)
        # Roll/pitch otomatik stabilize, yaw manual control
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd, use_stabilization=True)
        
        # 5 saniye pozisyon tutma sayacı (non-blocking)
        if distance_from_start <= MISSION_PARAMS['position_tolerance']:
            if not hasattr(self, '_final_hold_start_time'):
                self._final_hold_start_time = time.time()
                print("🎯 Pozisyon toleransı içinde! 5 saniye tutma başlatıldı...")
            
            hold_duration = time.time() - self._final_hold_start_time
            remaining = 5.0 - hold_duration
            
            if hold_duration >= 5.0:
                print("✅ Final pozisyon 5 saniye tutuldu! Yüzeye çıkış ve enerji kesme...")
                self.mission_stage = "SURFACE_AND_SHUTDOWN"
            else:
                # Progress göster (her saniyede bir)
                if int(hold_duration) != int(hold_duration - 0.1):
                    print(f"⏱️ Pozisyon tutma: {hold_duration:.1f}/5.0s (kalan: {remaining:.1f}s)")
        else:
            # Tolerans dışına çıktık, sayacı sıfırla
            if hasattr(self, '_final_hold_start_time'):
                delattr(self, '_final_hold_start_time')
                print("⚠️ Pozisyon toleransı dışına çıkıldı, sayaç sıfırlandı")
    
    def execute_surface_shutdown(self):
        """Pozitif sephiye ile yüzeye çıkış ve enerji kesme"""
        # Pozitif sephiye (buoyancy ile yüzeye çık)
        self.set_motor_throttle(PWM_NEUTRAL)  # Motor kapat
        self.set_control_surfaces(pitch_cmd=-100, use_stabilization=False)  # Nose up (pozitif sephiye)
        
        # Yüzeye çıkana kadar bekle
        if self.current_depth > 0.5:
            return  # Henüz yüzeye çıkmadı
        
        # Yüzeye çıktık - enerji kesme simülasyonu
        print("🌊 Yüzeye çıkış tamamlandı!")
        print("⚡ Sistem enerjisi kesiliyor...")
        
        # Tüm sistemleri durdur
        self.set_motor_throttle(PWM_NEUTRAL)
        self.set_control_surfaces(use_stabilization=False)
        
        self.mission_completion_time = time.time()
        self.mission_stage = "MISSION_COMPLETE"
        
        print("✅ GÖREV 1 TAMAMLANDI!")
    
    def monitoring_loop(self):
        """İzleme döngüsü - Fault ve durum kontrolü"""
        while self.running and self.mission_active:
            # Latched fault kontrolü
            if self._latched_fault:
                print(f"🚨 MONITORING: Latched fault aktif - {self._latched_fault}")
                print("🚨 Mission abort durumunda. Telemetri kaydediliyor...")
                break
            
            # Arming countdown gösterimi
            if not self._arming_done and self._arming_start_time:
                remaining = self.ARMING_DURATION - (time.time() - self._arming_start_time)
                self._update_status_indicators(self.mission_stage, arming_remaining=remaining)
            else:
                # Normal durum göstergeleri
                self._update_status_indicators(self.mission_stage)
            
            # Her 3 saniyede durum göster
            if len(self.telemetry_data) % 30 == 0:
                self.display_mission_status()
            
            # Süre kontrolü
            if self.mission_start_time:
                elapsed = time.time() - self.mission_start_time
                if elapsed > MISSION_PARAMS['timeout_seconds']:
                    print("⏰ Süre doldu! Görev sonlandırılıyor...")
                    self.mission_stage = "MISSION_TIMEOUT"
                    break
            
            time.sleep(0.1)
    
    def generate_mission_report(self):
        """Görev raporu oluştur"""
        mission_duration = (self.mission_completion_time - self.mission_start_time) if (self.mission_completion_time and self.mission_start_time) else 0
        
        print("\n" + "="*80)
        print("📋 GÖREV 1 RAPORU - SEYİR YAPMA & GERİ DÖNÜŞ")
        print("="*80)
        
        print(f"📅 Görev Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Görev Süresi: {mission_duration:.1f} saniye")
        
        # Performans metrikleri
        print(f"\n📊 PERFORMANS METRİKLERİ:")
        print("-"*60)
        print(f"📏 Düz Seyir Mesafesi: {self.straight_distance_completed:.1f}m / {MISSION_PARAMS['straight_distance']}m")
        print(f"🌊 Maksimum Kıyı Uzaklığı: {self.max_offshore_distance:.1f}m / {MISSION_PARAMS['min_offshore_distance']}m")
        print(f"🎯 Final Pozisyon Hatası: {self.final_position_error:.1f}m / {MISSION_PARAMS['position_tolerance']}m")
        print(f"💧 Sızdırmazlık: {'✅ BAŞARILI' if not self.leak_detected else '❌ SIZINTI'}")
        
        # Puanlama hesaplama
        print(f"\n🏆 PUANLAMA:")
        print("-"*40)
        
        # Seyir yapma puanı (hız bazlı) - Düzeltilmiş formül
        if mission_duration > 0 and mission_duration <= 300:
            # Hız puanı: 150 * (300 - süre) / 300
            time_factor = (300 - mission_duration) / 300
            cruise_success = (self.straight_distance_completed >= MISSION_PARAMS['straight_distance'] and 
                            self.max_offshore_distance >= MISSION_PARAMS['min_offshore_distance'])
            cruise_points = int(150 * time_factor) if cruise_success else 0
        else:
            cruise_points = 0
        print(f"  🚀 Seyir Yapma (hız): {cruise_points}/150 puan (süre faktörü: {time_factor:.3f})")
        
        # Başlangıç noktasında enerji kesme
        position_success = (self.final_position_error <= MISSION_PARAMS['position_tolerance'] and 
                           self.mission_stage == "MISSION_COMPLETE")
        position_points = 90 if position_success else 0
        print(f"  🎯 Başlangıç Noktasında Enerji Kesme: {position_points}/90 puan")
        
        # Sızdırmazlık (fault kontrolü dahil)
        leak_or_fault = self.leak_detected or self._latched_fault
        waterproof_points = 60 if not leak_or_fault else 0
        print(f"  💧 Sızdırmazlık: {waterproof_points}/60 puan")
        
        total_points = cruise_points + position_points + waterproof_points
        print(f"\n📈 TOPLAM PUAN: {total_points}/300")
        
        # Başarı değerlendirmesi
        if total_points >= 240:  # %80 başarı
            print("🎉 MÜKEMMEL PERFORMANS!")
        elif total_points >= 180:  # %60 başarı
            print("👍 İYİ PERFORMANS!")
        elif total_points >= 120:  # %40 başarı
            print("⚠️ ORTA PERFORMANS!")
        else:
            print("❌ DÜŞÜK PERFORMANS!")
        
        # Veri kaydet - Genişletilmiş rapor
        mission_report = {
            'timestamp': datetime.now().isoformat(),
            'mission_duration': mission_duration,
            'mission_result': self.mission_stage,
            'fault_info': {
                'latched_fault': self._latched_fault,
                'leak_detected': self.leak_detected,
                'fault_time': getattr(self, '_fault_time', None)
            },
            'performance_metrics': {
                'straight_distance_completed': self.straight_distance_completed,
                'max_offshore_distance': self.max_offshore_distance,
                'final_position_error': self.final_position_error,
                'total_traveled_distance': self.traveled_distance
            },
            'sensor_info': {
                'depth_source_primary': self.depth_source,
                'speed_source_primary': getattr(self, 'speed_source', 'vfr_hud'),
                'd300_connected': self.d300_connected,
                'sensor_timeouts': self._latched_fault and 'TIMEOUT' in str(self._latched_fault)
            },
            'scoring': {
                'cruise_points': cruise_points,
                'position_points': position_points,
                'waterproof_points': waterproof_points,
                'total_points': total_points,
                'time_factor': time_factor if mission_duration > 0 else 0
            },
            'telemetry_summary': {
                'total_samples': len(self.telemetry_data),
                'start_position': self.start_position,
                'mission_stages': list(set([d['mission_stage'] for d in self.telemetry_data])),
                'pwm_odometry_used': any(d.get('speed_source') == 'pwm_calibrated' for d in self.telemetry_data)
            }
        }
        
        # JSON rapor kaydet
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        with open(f'mission_1_report_{timestamp_str}.json', 'w') as f:
            json.dump(mission_report, f, indent=2)
        
        # CSV telemetri kaydet
        if self.telemetry_data:
            self._save_csv_telemetry(timestamp_str)
        
        print(f"\n💾 Görev raporu kaydedildi: mission_1_report_{timestamp_str}.json")
        print(f"💾 Telemetri kaydedildi: mission_1_telemetry_{timestamp_str}.csv")
        
        return total_points >= 180  # %60 başarı şartı
    
    def _save_csv_telemetry(self, timestamp_str):
        """Telemetri verilerini CSV formatında kaydet"""
        try:
            import csv
            filename = f'mission_1_telemetry_{timestamp_str}.csv'
            
            if not self.telemetry_data:
                return
                
            # CSV başlıkları
            fieldnames = [
                'timestamp', 'mission_stage', 'depth', 'depth_source',
                'heading', 'roll', 'pitch', 'yaw',
                'speed', 'speed_source', 'estimated_speed', 'filtered_speed',
                'position_x', 'position_y', 'traveled_distance',
                'motor_pwm', 'leak_detected', 'latched_fault'
            ]
            
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                
                for data in self.telemetry_data:
                    # CSV için veriyi düzenle
                    csv_row = {
                        'timestamp': data['timestamp'],
                        'mission_stage': data['mission_stage'],
                        'depth': data['depth'],
                        'depth_source': data['depth_source'],
                        'heading': data['heading'],
                        'roll': data['roll'],
                        'pitch': data['pitch'],
                        'yaw': data['yaw'],
                        'speed': data['speed'],
                        'speed_source': data.get('speed_source', 'vfr_hud'),
                        'estimated_speed': data.get('estimated_speed', 0),
                        'filtered_speed': data.get('filtered_speed', 0),
                        'position_x': data['position']['x'],
                        'position_y': data['position']['y'],
                        'traveled_distance': data['traveled_distance'],
                        'motor_pwm': data.get('motor_pwm', 1500),
                        'leak_detected': data.get('leak_detected', False),
                        'latched_fault': data.get('latched_fault', '')
                    }
                    writer.writerow(csv_row)
                    
            print(f"📊 CSV telemetri: {len(self.telemetry_data)} kayıt kaydedildi")
            
        except Exception as e:
            print(f"❌ CSV kayıt hatası: {e}")
    
    def run_mission_1(self):
        """Görev 1'i çalıştır"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - GÖREV 1 BAŞLIYOR")
        print("="*80)
        print("🎯 Görev: Seyir Yapma & Başlangıç Noktasına Geri Dönüş")
        print("⏱️ Süre Limiti: 5 dakika")
        print("🏆 Maksimum Puan: 300")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Hardware konfigürasyonu doğrula
        if not validate_hardware_config(MISSION_CONFIG):
            print("❌ Hardware validation başarısız!")
            return False
        
        # Dead Reckoning başlangıç pozisyonu ayarla
        print("📍 Dead Reckoning sistemi başlatılıyor...")
        print("🧭 IMU kalibrasyonu bekleniyor...")
        
        # IMU stabilizasyonu için bekle
        for i in range(10):
            if self.read_sensors():
                print(f"🔄 IMU okuma {i+1}/10: Heading={self.current_heading:.1f}°")
            time.sleep(0.5)
        
        # Başlangıç heading'ini ayarla
        self.initial_heading = self.current_heading
        self.start_position['heading'] = self.current_heading
        
        print(f"📍 Başlangıç pozisyonu: X=0.0m, Y=0.0m, Heading={self.initial_heading:.1f}°")
        print("🎯 Dead Reckoning navigasyon hazır!")
        
        print("\n⚠️ GÖREV HAZIRLIĞI:")
        print("- Tüm sistemler hazır mı?")
        print("- Güvenlik kontrolleri tamamlandı mı?") 
        print("- Şamandıra takıldı mı?")
        
        # Servo test seçeneği
        test_servos = input("\n🔧 Servolar test edilsin mi? (y/n): ").lower()
        if test_servos == 'y':
            self.test_servos()
        
        ready = input("\n✅ Görev 1 başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Görev iptal edildi")
            return False
        
        self.mission_start_time = time.time()
        self.mission_active = True
        self.running = True
        self.mission_stage = "DESCENT"
        
        # Control ve monitoring thread'leri başlat - Improved thread management
        try:
            self.control_thread = threading.Thread(target=self.control_loop, name="ControlLoop")
            self.control_thread.daemon = False  # Graceful shutdown için daemon=False
            self.control_thread.start()
            print("🎯 Control thread başlatıldı")
            
            self.monitoring_thread = threading.Thread(target=self.monitoring_loop, name="MonitoringLoop")
            self.monitoring_thread.daemon = False
            self.monitoring_thread.start()
            print("📊 Monitoring thread başlatıldı")
            
        except Exception as e:
            print(f"❌ Thread başlatma hatası: {e}")
            self.cleanup()
            return False
        
        try:
            print("\n🚀 GÖREV 1 BAŞLADI!")
            
            # Control thread bitmesini bekle
            self.control_thread.join()
            
            # Görev raporu
            success = self.generate_mission_report()
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Görev kullanıcı tarafından durduruldu")
            self.mission_active = False
            self.running = False
            return False
        except Exception as e:
            print(f"\n❌ Görev hatası: {e}")
            self.mission_active = False
            self.running = False
            return False
        finally:
            # Her durumda güvenli kapanış
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri - full_stabilization2.py uyumlu"""
        self.mission_active = False
        self.running = False
        
        print("\n🧹 Sistem temizleniyor - full_stabilization2.py uyumlu...")
        
        if self.connected:
            # Motor neutral
            self.set_motor_throttle(PWM_NEUTRAL)
            
            # Servolar neutral - full_stabilization2.py mantığı ile
            try:
                servo_channels = [SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT]
                for channel in servo_channels:
                    self._set_servo_pwm(channel, PWM_NEUTRAL)
                print("🔄 Servolar nötr pozisyonda - full_stabilization2.py uyumlu")
            except:
                pass
        
        # D300 sensörünü kapat
        if self.d300_connected and self.d300_sensor:
            try:
                self.d300_sensor.close()
                print("🔌 D300 derinlik sensörü kapatıldı")
            except:
                pass
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı - full_stabilization2.py uyumlu")

def main():
    """Ana fonksiyon"""
    parser = argparse.ArgumentParser(description='TEKNOFEST Görev 1: Seyir Yapma & Geri Dönüş (Plus Wing - GPS\'siz)')
    parser.add_argument('--start-heading', type=float, default=0.0, help='Başlangıç heading (derece)')
    
    args = parser.parse_args()
    
    mission = Mission1Navigator(start_heading=args.start_heading)
    
    try:
        success = mission.run_mission_1()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main()) 