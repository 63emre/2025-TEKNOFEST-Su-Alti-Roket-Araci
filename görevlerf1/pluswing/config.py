#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CONFIGURATION - Su Altı Roket Aracı (SARA) Konfigürasyon Dosyası
Deneme.txt belgesine göre hazırlanmıştır.
"""

# ---- MAVLink Bağlantı Ayarları ----
# Linux/Unix portları - dinamik tarama yapılacak ama fallback için
MAVLINK_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1", "/dev/ttyACM2", "/dev/ttyUSB0", "/dev/ttyUSB1"]
MAVLINK_PORT_WIN = "COM5"      # Windows için alternatif
MAVLINK_BAUD = 115200

# MAVLink bağlantı fallback ayarları
MAVLINK_CONNECTION_TIMEOUT = 5    # Bağlantı timeout (saniye)
MAVLINK_HEARTBEAT_TIMEOUT = 5     # Heartbeat timeout (saniye)
MAVLINK_RECONNECT_ATTEMPTS = 3    # Yeniden bağlantı denemeleri

# ---- Servo ve Motor Kanalları ----
# ArduSub'da: MAIN1-8 = kanal 1-8, AUX1 = kanal 9, AUX2 = kanal 10, vs.
MOTOR_MAIN = 9      # AUX1 → Ana Motor (DEGZ M5 + DEGZ BLU 30A ESC)
# AUX2 BOZUK - Kullanılmaz

SERVO_RIGHT = 11    # AUX3 → Fin Servo 1 (DS3230MG 30kg) - Plus: Sağ Kanat  
SERVO_DOWN  = 12    # AUX4 → Fin Servo 2 (DS3230MG 30kg) - Plus: Alt Kanat
SERVO_LEFT  = 13    # AUX5 → Fin Servo 3 (DS3230MG 30kg) - Plus: Sol Kanat
SERVO_UP    = 14    # AUX6 → Fin Servo 4 (DS3230MG 30kg) - Plus: Üst Kanat

# ---- GPIO Pin Tanımları ----
GPIO_LED_RED = 21        # Kırmızı LED
GPIO_BUZZER = 9          # Buzzer
GPIO_START_BUTTON = 11   # Başlatma/Soft-kill butonu
GPIO_SOLENOID = 10       # Selenoid karbondioksit (Sadece Görev 2'de kullanılır)

# ---- PWM Değerleri ----
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEUTRAL = 1500

# Motor PWM değerleri
MOTOR_STOP = 1500
MOTOR_FORWARD_MIN = 1550
MOTOR_FORWARD_MAX = 1900
MOTOR_REVERSE_MIN = 1450
MOTOR_REVERSE_MAX = 1100

# ---- D300 Derinlik Sensörü (MAVLink) ----
# D300 Pixhawk'a I2C ile bağlı, MAVLink SCALED_PRESSURE mesajları üzerinden veri alınır
D300_SOURCE = 2                    # Birincil kaynak: 2=SCALED_PRESSURE2, 3=SCALED_PRESSURE3
D300_FALLBACK_SOURCES = [2, 3]    # Fallback kaynakları (otomatik değişim)
D300_DATA_RATE_HZ = 10            # Veri alma hızı (Hz)
D300_SEAWATER_DENSITY = 1025.0    # Deniz suyu yoğunluğu (kg/m³) - GÖREVLER DENİZDE
D300_FRESHWATER_DENSITY = 997.0   # Tatlı su yoğunluğu (kg/m³)
D300_GRAVITY = 9.81               # Yerçekimi (m/s²)

# D300 Fallback ayarları
D300_MAX_FAILURES_BEFORE_SOURCE_SWITCH = 5    # Kaynak değiştirme öncesi başarısız okuma
D300_MAX_FAILURES_BEFORE_DISCONNECT = 15      # Bağlantı kesildi kabul etme
D300_MAX_RECONNECT_ATTEMPTS = 10               # Maksimum yeniden bağlantı denemesi
D300_RECONNECT_INTERVAL = 2.0                  # Yeniden bağlantı deneme aralığı (saniye)

# ---- D300 Kalibrasyon Ayarları ----
# Su yüzeyinde tutmadan kalibrasyon için
D300_CALIB_DURATION_SEAWATER = 6   # Deniz suyu kalibrasyonu süresi (saniye)
D300_CALIB_DURATION_FRESHWATER = 6 # Tatlı su kalibrasyonu süresi (saniye)
D300_USE_WATER_SURFACE_CALIB = False  # True: Su yüzeyinde tut, False: Havada kalibre et

# ---- Güvenlik Zamanları ----
ARMING_DELAY_SECONDS = 90    # 90 saniye arming gecikmesi (buton sonrası)
MISSION_TIMEOUT_SECONDS = 600  # 10 dakika maksimum görev süresi
D300_CALIBRATION_DURATION = 10  # 10 saniye D300 kalibrasyon süresi

# ---- Görev 1 Parametreleri ----
TARGET_DEPTH_RANGE_MIN = 2.0    # Minimum seyir derinliği
TARGET_DEPTH_RANGE_MAX = 2.5    # Maksimum seyir derinliği
TARGET_DEPTH_DEFAULT = 2.25     # Varsayılan seyir derinliği (2-2.5m ortası)
MISSION_DISTANCE = 50.0         # İleri gidiş mesafesi (10m + 40m)
RETURN_DISTANCE = 50.0          # Geri dönüş mesafesi
FIRST_PHASE_DISTANCE = 10.0     # İlk faz 10 metre
SECOND_PHASE_DISTANCE = 40.0    # İkinci faz 40 metre

# ---- Stabilizasyon Parametreleri ----
# Roll Kontrolü
ROLL_SENSE = +1.0
ROLL_K_ANG_US_PER_RAD = 500.0
ROLL_DEADBAND_DEG = 1.0
ROLL_MAX_DELTA_US = 350.0

# Pitch Kontrolü
PITCH_SENSE = +1.0
PITCH_K_ANG_US_PER_RAD = 500.0
PITCH_DEADBAND_DEG = 1.0
PITCH_MAX_DELTA_US = 350.0

# Yaw Kontrolü
YAW_SENSE = +1.0
YAW_K_ANG_US_PER_RAD = 400.0
YAW_DEADBAND_DEG = 2.0
YAW_MAX_DELTA_US = 300.0

# Derinlik Kontrolü (PID)
DEPTH_KP = 200.0        # P kontrolcü katsayısı
DEPTH_KI = 10.0         # I kontrolcü katsayısı
DEPTH_KD = 50.0         # D kontrolcü katsayısı
DEPTH_MAX_PITCH = 15.0  # Maksimum pitch açısı (derece)
DEPTH_DEADBAND = 0.2    # Derinlik deadband (metre)

# ---- Genel Güvenlik Sınırları ----
OVERALL_MAX_DELTA_US = 400.0

# ---- Servo Yön Ayarları ----
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

# ---- Görev Fazları ----
class MissionPhase:
    WAITING = "waiting"           # 90 saniye bekleme
    CALIBRATION = "calibration"   # Kalibrasyon
    PHASE_1 = "phase_1"          # İlk 10 metre (2m derinlik)
    PHASE_2 = "phase_2"          # Ana seyir (3m derinlik)
    TURNING = "turning"          # 180° dönüş
    RETURN = "return"            # Geri dönüş
    SURFACING = "surfacing"      # Yüzeye çıkış
    COMPLETED = "completed"      # Görev tamamlandı
    EMERGENCY = "emergency"      # Acil durum
    ROCKET_PREP = "rocket_prep"  # Roket hazırlığı (Görev 2)
    ROCKET_LAUNCH = "rocket_launch"  # Roket fırlatma (Görev 2)

# ---- Hız Profilleri ----
SPEED_SLOW = 1600      # Yavaş hız
SPEED_MEDIUM = 1700    # Orta hız  
SPEED_FAST = 1800      # Hızlı hız

# ---- Buzzer Sinyalleri ----
BUZZER_STARTUP = [0.5, 0.2, 0.5, 0.2, 0.5]  # Sistem başlangıç sinyali
BUZZER_CALIBRATION_LONG = 10.0               # D300 kalibrasyon uzun bip (10 saniye)
BUZZER_CALIBRATION_SUCCESS = [0.5, 0.2, 0.5, 0.2, 0.5]  # Kalibrasyon başarı
BUZZER_COUNTDOWN_SHORT = 0.1    # Kısa bip süresi  
BUZZER_COUNTDOWN_LONG = 0.5     # Uzun bip süresi
BUZZER_COUNTDOWN_PAUSE = 0.9    # Bip arası bekleme
BUZZER_MISSION_START = [0.2, 0.1] * 5        # Görev başlangıcı (5 saniye)
BUZZER_MISSION_END = [1.0, 0.5] * 3          # Görev bitişi (3 saniyede 1)
BUZZER_EMERGENCY = [0.1, 0.1] * 10           # Acil durum
BUZZER_90_SEC_COMPLETE = [0.5, 0.5, 0.5, 0.5, 0.5]  # 90 saniye tamamlandı

# ---- Hız Hesaplama Sabitleri ----
ESTIMATED_SPEED_SLOW = 1.0      # m/s
ESTIMATED_SPEED_MEDIUM = 1.5    # m/s
ESTIMATED_SPEED_FAST = 2.0      # m/s

# ---- Tolerans Değerleri ----
YAW_TURN_TOLERANCE = 10.0       # 180° dönüş için tolerans (derece)
SURFACE_DEPTH_THRESHOLD = 0.5   # Yüzeye çıktığını anlama için derinlik (metre)

# ---- Genel Yardımcı Fonksiyonlar ----
def get_servo_channels():
    """Servo kanallarını dict olarak döndür"""
    return {
        'up': SERVO_UP,
        'down': SERVO_DOWN, 
        'left': SERVO_LEFT,
        'right': SERVO_RIGHT
    }

def get_motor_channel():
    """Motor kanalını döndür"""
    return MOTOR_MAIN

def clamp(value, min_val, max_val):
    """Değeri belirtilen aralıkta sınırla"""
    if value is None:
        return (min_val + max_val) / 2  # Orta değer döndür
    return max(min_val, min(max_val, value))

def to_pwm(delta_us):
    """Delta mikrosekondeyi PWM değerine çevir"""
    if delta_us is None:
        return PWM_NEUTRAL
    return int(clamp(PWM_NEUTRAL + delta_us, PWM_MIN, PWM_MAX))

def get_speed_for_phase(phase):
    """Faza göre hız döndür"""
    if phase == MissionPhase.PHASE_1:
        return SPEED_MEDIUM
    elif phase == MissionPhase.PHASE_2:
        return SPEED_FAST
    elif phase == MissionPhase.RETURN:
        return SPEED_FAST
    elif phase == MissionPhase.TURNING:
        return SPEED_SLOW
    else:
        return SPEED_MEDIUM

def get_target_depth_for_phase(phase):
    """Faza göre hedef derinlik döndür"""
    if phase == MissionPhase.PHASE_1:
        return TARGET_DEPTH_DEFAULT  # 2.25m
    elif phase in [MissionPhase.PHASE_2, MissionPhase.RETURN]:
        return TARGET_DEPTH_DEFAULT  # 2.25m (aynı derinlikte)
    else:
        return TARGET_DEPTH_DEFAULT
