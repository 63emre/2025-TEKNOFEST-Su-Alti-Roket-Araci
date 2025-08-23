#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Pixhawk Pasif I/O Hub Hardware Konfigürasyonu
Bu dosya Pixhawk'ı sadece PWM çıkış + telemetri için kullanır.

Pixhawk Rolü (Pasif):
- PWM çıkışları (DO_SET_SERVO ile Pi kontrolü)
- ATTITUDE telemetrisi (20 Hz)
- SCALED_PRESSURE telemetrisi (10 Hz)  
- MANUAL mode (tek güvenli mod)

Pi Rolü (Aktif):
- Tüm PID kontrolleri
- Servo mixing (Plus/X-Wing)
- Odometri (PWM→hız→mesafe)
- Mission state machine
"""

import math
import numpy as np

# ============================================================================
# 1. SERVO KANAL MAPPING (Pixhawk çıkışları)
# ============================================================================

# Plus-Wing Konfigürasyon (4 fin + motor)
PLUS_WING_CHANNELS = {
    'right_fin': 1,    # SERVO1 - RIGHT fin (sağ)
    'left_fin': 2,     # SERVO2 - LEFT fin (sol)  
    'up_fin': 3,       # SERVO3 - UP fin (üst)
    'down_fin': 4,     # SERVO4 - DOWN fin (alt)
    'motor': 5         # SERVO5 - Motor (ESC)
}

# X-Wing Konfigürasyon (alternatif)
X_WING_CHANNELS = {
    'fin_1': 1,        # SERVO1 - Fin 1 (ön-sağ)
    'fin_2': 2,        # SERVO2 - Fin 2 (ön-sol)
    'fin_3': 3,        # SERVO3 - Fin 3 (arka-sağ) 
    'fin_4': 4,        # SERVO4 - Fin 4 (arka-sol)
    'motor': 5         # SERVO5 - Motor (ESC)
}

# Payload servolar (Mission 2 için)
PAYLOAD_CHANNELS = {
    'rocket_bay_1': 9,     # AUX1 - Roket bay 1
    'rocket_bay_2': 10,    # AUX2 - Roket bay 2
    'separation_1': 11,    # AUX3 - Ayrılma mekanizması 1
    'separation_2': 12     # AUX4 - Ayrılma mekanizması 2
}

# ============================================================================
# 2. PWM LİMİTLERİ (ArduPlane params ile uyumlu)
# ============================================================================

PWM_LIMITS = {
    # Servo limitleri (DS3230MG uyumlu)
    'servo_neutral': 1500,
    'servo_min': 1300,
    'servo_max': 1700,
    'servo_deadband': 10,      # ±10 PWM deadband
    
    # Motor limitleri (ESC uyumlu - tek yön: 1000=stop, 2000=full)
    'motor_neutral': 1000,
    'motor_min': 1000,         # ESC minimum (stop)
    'motor_max': 2000,         # ESC maximum (full)
    'motor_deadband': 20,      # ±20 PWM deadband
    
    # Güvenlik limitleri
    'overall_max_delta': 350,  # Maksimum PWM değişimi
    'rate_limit': 200          # PWM/saniye değişim limiti
}

# ============================================================================
# 3. SERVO MİXİNG MATRİSLERİ (Pi'de yapılır)
# ============================================================================

# Plus-Wing Mixing Matrix (4 fin)
# Roll/Pitch/Yaw komutlarını 4 fin PWM'ine çevirir
PLUS_WING_MATRIX = np.array([
    # [Right, Left, Up, Down] = f(Roll, Pitch, Yaw)
    [ 1.0, -1.0,  0.0,  0.0],  # Roll: Right+, Left-
    [ 0.0,  0.0,  1.0, -1.0],  # Pitch: Up+, Down-  
    [ 0.5,  0.5,  0.5,  0.5]   # Yaw: Tüm finler aynı yön
])

# X-Wing Mixing Matrix (4 fin)
X_WING_MATRIX = np.array([
    # [Fin1, Fin2, Fin3, Fin4] = f(Roll, Pitch, Yaw)  
    [ 1.0, -1.0, -1.0,  1.0],  # Roll: Çapraz kontrol
    [ 1.0,  1.0, -1.0, -1.0],  # Pitch: Ön/arka kontrol
    [ 1.0, -1.0,  1.0, -1.0]   # Yaw: Alternatif kontrol
])

# Mixing gain'leri (fine tuning için)
MIXING_GAINS = {
    'roll_gain': 1.0,
    'pitch_gain': 1.0, 
    'yaw_gain': 0.8,           # Yaw biraz daha yumuşak
    'overall_gain': 1.0
}

# ============================================================================
# 4. PID KONTROL PARAMETRELERİ (Pi'de çalışır)
# ============================================================================

# Derinlik PID (kritik - su altı kontrolü)
DEPTH_PID = {
    'kp': 120.0,
    'ki': 5.0,
    'kd': 35.0,
    'max_output': 200,         # PWM motor correction
    'integral_limit': 100,
    'derivative_filter': 0.1
}

# Heading PID (IMU yaw tabanlı)
HEADING_PID = {
    'kp': 6.0,
    'ki': 0.25,
    'kd': 1.2,
    'max_output': 100,         # PWM fin correction
    'integral_limit': 50,
    'derivative_filter': 0.05
}

# Roll/Pitch stabilization PID (opsiyonel - temel stabilizasyon)
ATTITUDE_PID = {
    'roll': {
        'kp': 3.0,
        'ki': 0.1,
        'kd': 0.5,
        'max_output': 80
    },
    'pitch': {
        'kp': 3.5,
        'ki': 0.12, 
        'kd': 0.6,
        'max_output': 80
    }
}

# ============================================================================
# 5. SENSÖR AYARLARI
# ============================================================================

# D300 Derinlik Sensörü (öncelikli)
D300_CONFIG = {
    'i2c_address': 0x76,
    'sample_rate': 10,         # Hz
    'filter_alpha': 0.8,       # Low-pass filter
    'calibration_offset': 0.0, # Manuel kalibrasyon
    'max_depth': 10.0,         # Maksimum ölçüm derinliği
    'timeout': 0.5             # Okuma timeout
}

# SCALED_PRESSURE Fallback
PRESSURE_CONFIG = {
    'sea_level_pressure': 1013.25,  # hPa
    'depth_conversion': 0.0102,      # hPa → metre (kritik katsayı)
    'filter_alpha': 0.7,
    'altitude_offset': 0.0           # Home altitude offset
}

# IMU/ATTITUDE ayarları
IMU_CONFIG = {
    'attitude_rate': 20,       # Hz - Pixhawk'tan istenecek
    'heading_filter': 0.9,     # Heading low-pass
    'tilt_compensation': True, # Roll/pitch'e göre yaw düzeltmesi
    'magnetic_declination': 0.0 # Bölgesel manyetik sapma
}

# ============================================================================
# 6. ODOMETRİ AYARLARI (GPS'siz navigasyon)
# ============================================================================

# PWM → Hız kalibrasyonu (cal_speed.json'dan yüklenecek)
ODOMETRY_CONFIG = {
    'speed_calibration_file': 'cal_speed.json',
    'integration_rate': 20,    # Hz - pozisyon entegrasyonu
    'velocity_filter': 0.9,    # Hız low-pass filter
    'position_filter': 0.8,    # Pozisyon low-pass filter
    'drift_correction': True,  # Drift kompensasyonu
    'heading_integration': True # Heading tabanlı vektör entegrasyonu
}

# Mesafe ölçüm ayarları
DISTANCE_CONFIG = {
    'straight_distance': 10.0,    # 10m düz seyir
    'offshore_distance': 50.0,    # 50m uzaklaşma
    'return_tolerance': 2.0,      # ±2m geri dönüş toleransı
    'bearing_tolerance': 10.0,    # ±10° bearing toleransı
    'distance_filter': 0.8        # Mesafe low-pass filter
}

# ============================================================================
# 7. MAVLink AYARLARI
# ============================================================================

MAVLINK_CONFIG = {
    'connection': {
        'port': '/dev/ttyACM0',
        'baudrate': 115200,
        'timeout': 15.0,
        'autoreconnect': True
    },
    
    # Telemetri stream oranları (SET_MESSAGE_INTERVAL ile)
    'stream_rates': {
        'ATTITUDE': 20,           # 20 Hz - kritik
        'SCALED_PRESSURE': 10,    # 10 Hz - derinlik için
        'SYS_STATUS': 2,          # 2 Hz - battery/sistem
        'HEARTBEAT': 1            # 1 Hz - bağlantı
    },
    
    # Komut timeout'ları
    'command_timeouts': {
        'DO_SET_SERVO': 1.0,      # Servo komutu timeout
        'SET_MODE': 2.0,          # Mode değişimi timeout
        'ARM_DISARM': 3.0         # Arming timeout
    }
}

# ============================================================================
# 8. GÜVENLİK SİSTEMLERİ
# ============================================================================

SAFETY_CONFIG = {
    # Arming sistemi (90s kuralı)
    'arming_duration': 90.0,      # 90 saniye arming gösterimi
    'arming_neutral_pwm': True,   # Arming süresince neutral PWM
    
    # Watchdog sistemi
    'watchdog_timeout': 0.5,      # 0.5s telemetri timeout
    'watchdog_actions': [
        'NEUTRAL_ALL_SERVOS',     # Tüm servolar neutral
        'EMERGENCY_SURFACE',      # Acil yüzeye çıkış
        'LOG_FAULT',              # Hata logu
        'CLEAN_SHUTDOWN'          # Temiz kapanış
    ],
    
    # Derinlik güvenliği
    'max_depth': 5.0,             # 5m maksimum derinlik
    'emergency_surface_depth': 0.5, # 0.5m acil yüzey eşiği
    
    # Battery güvenliği
    'min_battery_voltage': 20.0,  # 20V minimum battery
    'battery_warning_voltage': 22.0, # 22V uyarı voltajı
    
    # Latched fault'lar (bir kez tetiklenince mission abort)
    'latched_faults': [
        'SENSOR_TIMEOUT',
        'DEPTH_EXCEEDED',
        'BATTERY_LOW', 
        'LEAK_DETECTED',
        'WATCHDOG_TIMEOUT',
        'IMU_FAILURE'
    ]
}

# ============================================================================
# 9. KONTROL DÖNGÜSÜ AYARLARI
# ============================================================================

CONTROL_CONFIG = {
    'main_loop_rate': 20,         # 20 Hz ana kontrol döngüsü
    'servo_update_rate': 20,      # 20 Hz servo güncelleme
    'telemetry_log_rate': 10,     # 10 Hz telemetri kayıt
    'status_display_rate': 1,     # 1 Hz durum gösterimi
    
    # PID reset koşulları
    'pid_reset_on_stage_change': True,
    'integral_windup_limit': True,
    'derivative_kick_prevention': True
}

# ============================================================================
# 10. UTILITY FONKSİYONLARI
# ============================================================================

def apply_plus_wing_mixing(roll_cmd, pitch_cmd, yaw_cmd):
    """
    Plus-Wing mixing: Roll/Pitch/Yaw komutlarını 4 fin PWM'ine çevir
    
    Args:
        roll_cmd: Roll komutu (-1.0 to 1.0)
        pitch_cmd: Pitch komutu (-1.0 to 1.0)
        yaw_cmd: Yaw komutu (-1.0 to 1.0)
        
    Returns:
        dict: {'right_fin': pwm, 'left_fin': pwm, 'up_fin': pwm, 'down_fin': pwm}
    """
    # Komutları mixing matrix ile çarp
    commands = np.array([roll_cmd, pitch_cmd, yaw_cmd])
    fin_outputs = PLUS_WING_MATRIX.T @ commands
    
    # Gain'leri uygula
    fin_outputs[0] *= MIXING_GAINS['roll_gain']    # Right fin
    fin_outputs[1] *= MIXING_GAINS['roll_gain']    # Left fin  
    fin_outputs[2] *= MIXING_GAINS['pitch_gain']   # Up fin
    fin_outputs[3] *= MIXING_GAINS['pitch_gain']   # Down fin
    
    # PWM'e çevir (neutral + command)
    pwm_outputs = {}
    fin_names = ['right_fin', 'left_fin', 'up_fin', 'down_fin']
    
    for i, fin_name in enumerate(fin_names):
        pwm_delta = fin_outputs[i] * (PWM_LIMITS['servo_max'] - PWM_LIMITS['servo_neutral'])
        pwm_delta = np.clip(pwm_delta, -PWM_LIMITS['overall_max_delta'], PWM_LIMITS['overall_max_delta'])
        
        pwm_outputs[fin_name] = int(PWM_LIMITS['servo_neutral'] + pwm_delta)
        pwm_outputs[fin_name] = np.clip(pwm_outputs[fin_name], PWM_LIMITS['servo_min'], PWM_LIMITS['servo_max'])
    
    return pwm_outputs

def apply_x_wing_mixing(roll_cmd, pitch_cmd, yaw_cmd):
    """
    X-Wing mixing: Roll/Pitch/Yaw komutlarını 4 fin PWM'ine çevir
    """
    commands = np.array([roll_cmd, pitch_cmd, yaw_cmd])
    fin_outputs = X_WING_MATRIX.T @ commands
    
    # Gain'leri uygula
    fin_outputs *= MIXING_GAINS['overall_gain']
    
    # PWM'e çevir
    pwm_outputs = {}
    fin_names = ['fin_1', 'fin_2', 'fin_3', 'fin_4']
    
    for i, fin_name in enumerate(fin_names):
        pwm_delta = fin_outputs[i] * (PWM_LIMITS['servo_max'] - PWM_LIMITS['servo_neutral'])
        pwm_delta = np.clip(pwm_delta, -PWM_LIMITS['overall_max_delta'], PWM_LIMITS['overall_max_delta'])
        
        pwm_outputs[fin_name] = int(PWM_LIMITS['servo_neutral'] + pwm_delta)
        pwm_outputs[fin_name] = np.clip(pwm_outputs[fin_name], PWM_LIMITS['servo_min'], PWM_LIMITS['servo_max'])
    
    return pwm_outputs

def calculate_motor_pwm(throttle_cmd):
    """
    Throttle komutunu motor PWM'ine çevir
    
    Args:
        throttle_cmd: Throttle komutu (-1.0 to 1.0)
        
    Returns:
        int: Motor PWM (1100-1900)
    """
    if abs(throttle_cmd) < PWM_LIMITS['motor_deadband'] / 1000.0:
        return PWM_LIMITS['motor_neutral']  # Deadband içinde neutral
    
    # Throttle'ı PWM'e çevir
    if throttle_cmd > 0:
        pwm_range = PWM_LIMITS['motor_max'] - PWM_LIMITS['motor_neutral']
        motor_pwm = PWM_LIMITS['motor_neutral'] + int(throttle_cmd * pwm_range)
    else:
        pwm_range = PWM_LIMITS['motor_neutral'] - PWM_LIMITS['motor_min']
        motor_pwm = PWM_LIMITS['motor_neutral'] + int(throttle_cmd * pwm_range)
    
    # Limitleri uygula
    motor_pwm = np.clip(motor_pwm, PWM_LIMITS['motor_min'], PWM_LIMITS['motor_max'])
    
    return motor_pwm

def validate_pwm_limits(pwm_value, servo_type='servo'):
    """
    PWM değerinin limitler içinde olduğunu doğrula
    
    Args:
        pwm_value: PWM değeri
        servo_type: 'servo' veya 'motor'
        
    Returns:
        bool: Geçerli ise True
    """
    if servo_type == 'servo':
        return PWM_LIMITS['servo_min'] <= pwm_value <= PWM_LIMITS['servo_max']
    elif servo_type == 'motor':
        return PWM_LIMITS['motor_min'] <= pwm_value <= PWM_LIMITS['motor_max']
    else:
        return False

def get_channel_for_actuator(actuator_name, wing_config='plus_wing'):
    """
    Actuator adından Pixhawk servo kanalını al
    
    Args:
        actuator_name: 'right_fin', 'motor', etc.
        wing_config: 'plus_wing' veya 'x_wing'
        
    Returns:
        int: Servo kanal numarası (1-16)
    """
    if wing_config == 'plus_wing':
        return PLUS_WING_CHANNELS.get(actuator_name, 0)
    elif wing_config == 'x_wing':
        return X_WING_CHANNELS.get(actuator_name, 0)
    else:
        return 0

# ============================================================================
# 11. BAŞLATMA VE DOĞRULAMA
# ============================================================================

def validate_hardware_config():
    """Hardware konfigürasyonunu doğrula"""
    errors = []
    
    # PWM limit kontrolü
    if PWM_LIMITS['servo_min'] >= PWM_LIMITS['servo_max']:
        errors.append("Servo PWM limitleri hatalı")
    
    if PWM_LIMITS['motor_min'] >= PWM_LIMITS['motor_max']:
        errors.append("Motor PWM limitleri hatalı")
    
    # Kanal çakışma kontrolü
    plus_channels = set(PLUS_WING_CHANNELS.values())
    if len(plus_channels) != len(PLUS_WING_CHANNELS):
        errors.append("Plus-Wing kanal çakışması")
    
    # Matrix boyut kontrolü
    if PLUS_WING_MATRIX.shape != (3, 4):
        errors.append("Plus-Wing matrix boyutu hatalı")
    
    if X_WING_MATRIX.shape != (3, 4):
        errors.append("X-Wing matrix boyutu hatalı")
    
    return errors

# Başlangıçta doğrulama yap
_validation_errors = validate_hardware_config()
if _validation_errors:
    print("⚠️ Hardware config doğrulama hataları:")
    for error in _validation_errors:
        print(f"   - {error}")
else:
    print("✅ Pixhawk Pasif I/O Hub hardware konfigürasyonu yüklendi")
    print("   - Servo mixing: Pi'de yapılır")
    print("   - PWM kontrol: DO_SET_SERVO ile")
    print("   - Telemetri: ATTITUDE + SCALED_PRESSURE")
    print("   - Güvenlik: Watchdog + latched faults")
