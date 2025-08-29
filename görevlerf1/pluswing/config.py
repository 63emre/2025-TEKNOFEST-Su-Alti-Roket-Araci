#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CONFIGURATION - Su Altı Roket Aracı (SARA) Konfigürasyon Dosyası
Deneme.txt belgesine göre hazırlanmıştır.
"""

# ---- MAVLink Bağlantı Ayarları ----
MAVLINK_PORTS = ["/dev/ttyACM0", "/dev/ttyACM1"]  # Pixhawk USB bağlantısı (Linux) - fallback ile
MAVLINK_PORT_WIN = "COM5"      # Windows için alternatif
MAVLINK_BAUD = 115200

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
# D300 Pixhawk'a bağlı, MAVLink SCALED_PRESSURE mesajları üzerinden veri alınır
D300_SOURCE = 1                    # 1=SCALED_PRESSURE, 2=SCALED_PRESSURE2, 3=SCALED_PRESSURE3
D300_DATA_RATE_HZ = 10            # Veri alma hızı (Hz)
D300_SEAWATER_DENSITY = 1025.0    # Deniz suyu yoğunluğu (kg/m³) - GÖREVLER DENİZDE
D300_FRESHWATER_DENSITY = 997.0   # Tatlı su yoğunluğu (kg/m³)
D300_GRAVITY = 9.81               # Yerçekimi (m/s²)

# ---- D300 Kalibrasyon Ayarları ----
# Otomatik kalibrasyon - güç verildiğinde butona basılmadan yapılır
D300_CALIB_DURATION_SEAWATER = 6   # Deniz suyu kalibrasyonu süresi (saniye)
D300_CALIB_DURATION_FRESHWATER = 6 # Tatlı su kalibrasyonu süresi (saniye)
D300_USE_WATER_SURFACE_CALIB = False  # True: Su yüzeyinde tut, False: Havada kalibre et
AUTO_CALIBRATION_ON_POWER = True     # Güç verildiğinde otomatik kalibrasyon

# ---- Güvenlik Zamanları ----
ARMING_DELAY_SECONDS = 90    # 90 saniye arming gecikmesi
MISSION_TIMEOUT_SECONDS = 600  # 10 dakika maksimum görev süresi

# ---- Görev 1 Parametreleri ----
TARGET_DEPTH_FIRST_10M = 2.0    # İlk 10 metre için 2m derinlik
TARGET_DEPTH_MAIN = 3.0         # Ana seyir için 3m derinlik  
MISSION_DISTANCE = 50.0         # Minimum 50 metre mesafe
FIRST_PHASE_DISTANCE = 10.0     # İlk faz 10 metre

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

# Derinlik Kontrolü (PID) - Deniz Koşulları İçin Optimize Edilmiş
DEPTH_KP = 150.0        # P kontrolcü katsayısı (deniz için yumuşak - önceki: 200.0)
DEPTH_KI = 6.0          # I kontrolcü katsayısı (windup önleme - önceki: 10.0)
DEPTH_KD = 80.0         # D kontrolcü katsayısı (daha iyi damping - önceki: 50.0)
DEPTH_MAX_PITCH = 20.0  # Maksimum pitch açısı (daha fazla otorite - önceki: 15.0)
DEPTH_DEADBAND = 0.2    # Derinlik deadband (metre)

# Gelişmiş PID Kontrol Parametreleri
DEPTH_INTEGRAL_CLAMP = 0.3      # Integral windup sınırı (radyan)
DEPTH_FILTER_CUTOFF = 5.0       # Sinyal filtresi kesim frekansı (Hz)
DEPTH_FAILSAFE_TIMEOUT = 5.0    # Max output için fail-safe süresi (saniye)
DEPTH_MAX_OUTPUT_THRESHOLD = 0.9 # Fail-safe tetikleme eşiği (%90)

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

# ---- Buzzer ve LED (Pinger) Sinyalleri ----
# Sistem başlangıç ve faz geçiş sinyalleri - DETAYLANDIRILMIŞ

# Sistem başlangıç
BUZZER_POWER_ON = [0.2, 0.1, 0.2, 0.1, 0.2]     # Güç verildiğinde (3 kısa bip)
BUZZER_CALIBRATION = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # Kalibrasyon sırasında (6 hızlı bip)
BUZZER_CALIBRATION_OK = [0.5, 0.2, 0.5]          # Kalibrasyon başarılı (2 uzun bip)
BUZZER_CALIBRATION_FAIL = [0.1, 0.1] * 5         # Kalibrasyon başarısız (10 hızlı bip)

# 90 saniye geri sayım (değiştirilmedi)
BUZZER_COUNTDOWN_SHORT = 0.1    # Kısa bip süresi  
BUZZER_COUNTDOWN_LONG = 0.5     # Uzun bip süresi
BUZZER_COUNTDOWN_PAUSE = 0.9    # Bip arası bekleme

# Görev fazları - HER FAZ İÇİN FARKLI SİNYAL
BUZZER_MISSION_START = [0.3, 0.1, 0.3, 0.1, 0.3]    # Görev başlangıcı (3 orta bip)
BUZZER_PHASE_1 = [0.2]                               # Faz 1 başlangıcı (1 bip)
BUZZER_PHASE_2 = [0.2, 0.1, 0.2]                    # Faz 2 başlangıcı (2 bip)
BUZZER_TURNING = [0.3, 0.1, 0.3, 0.1, 0.3]         # 180° dönüş (3 orta bip)
BUZZER_RETURN = [0.2, 0.1, 0.2, 0.1, 0.2, 0.1, 0.2]  # Geri dönüş (4 bip)
BUZZER_SURFACING = [1.0, 0.3, 1.0]                  # Yüzeye çıkış (2 uzun bip)

# Roket görevleri (Görev 2)
BUZZER_ROCKET_PREP = [0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5]  # Roket hazırlık (5 orta bip)
BUZZER_ROCKET_LAUNCH = [2.0]                         # Roket fırlatma (1 çok uzun bip)

# Görev sonucu
BUZZER_MISSION_SUCCESS = [0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5]  # Başarılı (4 orta bip)
BUZZER_MISSION_FAIL = [0.1, 0.1] * 8                 # Başarısız (16 hızlı bip)
BUZZER_EMERGENCY = [0.1, 0.1] * 10                   # Acil durum (20 hızlı bip)

# LED (Pinger) Durumları - DETAYLANDIRILMIŞ
LED_POWER_ON_BLINK = 0.5        # Güç verildiğinde yanıp sönme
LED_CALIBRATION_BLINK = 0.2     # Kalibrasyon sırasında hızlı yanıp sönme
LED_WAITING_BLINK = 1.0         # Buton bekleme - yavaş yanıp sönme
LED_COUNTDOWN_BLINK = 0.1       # Geri sayım - çok hızlı yanıp sönme
LED_MISSION_ON = True           # Görev sırasında sürekli açık
LED_PHASE_TRANSITION = 0.3      # Faz geçişlerinde orta hızda yanıp sönme
LED_EMERGENCY_BLINK = 0.05      # Acil durum - çok hızlı yanıp sönme
LED_SUCCESS_SLOW_BLINK = 2.0    # Görev başarılı - çok yavaş yanıp sönme

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
    return max(min_val, min(max_val, value))

def to_pwm(delta_us):
    """Delta mikrosekondeyi PWM değerine çevir"""
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
        return TARGET_DEPTH_FIRST_10M
    elif phase in [MissionPhase.PHASE_2, MissionPhase.RETURN]:
        return TARGET_DEPTH_MAIN
    else:
        return TARGET_DEPTH_MAIN

def get_buzzer_signal_for_phase(phase):
    """Faza göre buzzer sinyali döndür"""
    signals = {
        MissionPhase.CALIBRATION: BUZZER_CALIBRATION,
        MissionPhase.PHASE_1: BUZZER_PHASE_1,
        MissionPhase.PHASE_2: BUZZER_PHASE_2,
        MissionPhase.TURNING: BUZZER_TURNING,
        MissionPhase.RETURN: BUZZER_RETURN,
        MissionPhase.SURFACING: BUZZER_SURFACING,
        MissionPhase.ROCKET_PREP: BUZZER_ROCKET_PREP,
        MissionPhase.ROCKET_LAUNCH: BUZZER_ROCKET_LAUNCH,
        MissionPhase.COMPLETED: BUZZER_MISSION_SUCCESS,
        MissionPhase.EMERGENCY: BUZZER_EMERGENCY
    }
    return signals.get(phase, [0.2])  # Varsayılan: 1 kısa bip

def get_led_blink_for_phase(phase):
    """Faza göre LED yanıp sönme hızı döndür"""
    blink_rates = {
        MissionPhase.CALIBRATION: LED_CALIBRATION_BLINK,
        MissionPhase.WAITING: LED_WAITING_BLINK,
        MissionPhase.PHASE_1: LED_PHASE_TRANSITION,
        MissionPhase.PHASE_2: LED_PHASE_TRANSITION,
        MissionPhase.TURNING: LED_PHASE_TRANSITION,
        MissionPhase.RETURN: LED_PHASE_TRANSITION,
        MissionPhase.SURFACING: LED_PHASE_TRANSITION,
        MissionPhase.ROCKET_PREP: LED_PHASE_TRANSITION,
        MissionPhase.ROCKET_LAUNCH: LED_MISSION_ON,
        MissionPhase.COMPLETED: LED_SUCCESS_SLOW_BLINK,
        MissionPhase.EMERGENCY: LED_EMERGENCY_BLINK
    }
    return blink_rates.get(phase, LED_MISSION_ON)
