#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CONFIGURATION - Hava Yarışı Test Konfigürasyonu
D300 derinlik sensörü KALDIRILDI - Sadece havada test için
Pluswing klasöründen uyarlanmıştır
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

# ---- HAVA YARIŞI: D300 KALDIRILDI ----
# Derinlik kontrolü deaktif - sadece attitude kontrolü
USE_DEPTH_CONTROL = False
SIMULATE_DEPTH = True  # Test için sabit derinlik simüle et
SIMULATED_DEPTH = 0.0  # Hava testinde 0 metre derinlik

# ---- Güvenlik Zamanları ----
ARMING_DELAY_SECONDS = 10    # 10 saniye arming gecikmesi (hava testi için kısa)
MISSION_TIMEOUT_SECONDS = 300  # 5 dakika maksimum görev süresi (hava testi)

# ---- Görev Parametreleri (Hava Testi) ----
TARGET_DISTANCE_PHASE1 = 20.0    # İlk faz mesafesi (hava testinde kısa)
TARGET_DISTANCE_PHASE2 = 30.0    # İkinci faz mesafesi
TOTAL_FORWARD_DISTANCE = 50.0    # Toplam ileri mesafe
RETURN_DISTANCE = 50.0           # Geri dönüş mesafesi

# ---- Stabilizasyon Parametreleri ----
# Roll Kontrolü
ROLL_SENSE = +1.0
ROLL_K_ANG_US_PER_RAD = 500.0
ROLL_DEADBAND_DEG = 1.0
ROLL_MAX_DELTA_US = 350.0

# Pitch Kontrolü (Hava testinde daha hassas)
PITCH_SENSE = +1.0
PITCH_K_ANG_US_PER_RAD = 400.0  # Hava testinde daha yumuşak
PITCH_DEADBAND_DEG = 2.0        # Hava testinde daha geniş deadband
PITCH_MAX_DELTA_US = 300.0

# Yaw Kontrolü
YAW_SENSE = +1.0
YAW_K_ANG_US_PER_RAD = 400.0
YAW_DEADBAND_DEG = 2.0
YAW_MAX_DELTA_US = 300.0

# Altitude Kontrolü (Barometric - Hava testi için)
ALTITUDE_KP = 100.0      # P kontrolcü katsayısı (daha yumuşak)
ALTITUDE_KI = 5.0        # I kontrolcü katsayısı
ALTITUDE_KD = 20.0       # D kontrolcü katsayısı
ALTITUDE_MAX_PITCH = 10.0 # Maksimum pitch açısı (derece) - hava testi için düşük
ALTITUDE_DEADBAND = 0.5   # Altitude deadband (metre)

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
    WAITING = "waiting"           # Başlangıç bekleme
    CALIBRATION = "calibration"   # Kalibrasyon
    PHASE_1 = "phase_1"          # İlk faz
    PHASE_2 = "phase_2"          # Ana faz
    TURNING = "turning"          # 180° dönüş
    RETURN = "return"            # Geri dönüş
    LANDING = "landing"          # İniş (hava testi için)
    COMPLETED = "completed"      # Görev tamamlandı
    EMERGENCY = "emergency"      # Acil durum

# ---- Hız Profilleri (Hava Testi) ----
SPEED_SLOW = 1550      # Yavaş hız (hava testinde düşük)
SPEED_MEDIUM = 1650    # Orta hız  
SPEED_FAST = 1750      # Hızlı hız (hava testinde güvenli maksimum)

# ---- Buzzer Sinyalleri ----
BUZZER_STARTUP = [0.5, 0.2, 0.5, 0.2, 0.5]  # Sistem başlangıç sinyali
BUZZER_CALIBRATION_SUCCESS = [0.5, 0.2, 0.5, 0.2, 0.5]  # Kalibrasyon başarı
BUZZER_COUNTDOWN_SHORT = 0.1    # Kısa bip süresi  
BUZZER_COUNTDOWN_LONG = 0.5     # Uzun bip süresi
BUZZER_COUNTDOWN_PAUSE = 0.9    # Bip arası bekleme
BUZZER_MISSION_START = [0.2, 0.1] * 3        # Görev başlangıcı (3 saniye)
BUZZER_MISSION_END = [1.0, 0.5] * 2          # Görev bitişi (2 saniyede 1)
BUZZER_EMERGENCY = [0.1, 0.1] * 10           # Acil durum
BUZZER_TEST_COMPLETE = [0.3, 0.3, 0.3, 0.3]  # Test tamamlandı

# ---- Hız Hesaplama Sabitleri ----
ESTIMATED_SPEED_SLOW = 0.8      # m/s (hava testinde düşük)
ESTIMATED_SPEED_MEDIUM = 1.2    # m/s
ESTIMATED_SPEED_FAST = 1.6      # m/s

# ---- Tolerans Değerleri ----
YAW_TURN_TOLERANCE = 15.0       # 180° dönüş için tolerans (derece) - hava testinde geniş

# ---- Hava Testi Özel Parametreler ----
TEST_MODE = True                # Test modu aktif
GROUND_LEVEL_THRESHOLD = 0.2    # Yere iniş tespiti için (metre)
MAX_TEST_ALTITUDE = 5.0         # Maksimum test yüksekliği (metre)
AUTO_LAND_TIMEOUT = 30          # Otomatik iniş timeout (saniye)

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
