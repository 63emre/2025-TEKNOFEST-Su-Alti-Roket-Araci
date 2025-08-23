#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Plus-Wing Hardware Konfigürasyonu
Bu dosya Plus-Wing konfigürasyonu için hardware tanımlarını içerir.
"""

# PWM Servo Kanalları (ArduSub AUX çıkışları)
SERVO_CHANNELS = {
    'fin_1': 3,  # AUX 3 - Sol Kanat (Port)
    'fin_2': 4,  # AUX 4 - Sağ Kanat (Starboard)  
    'fin_3': 5,  # AUX 5 - Üst Kanat (Top)
    'fin_4': 6   # AUX 6 - Alt Kanat (Bottom)
}

# Motor Kanalı
MOTOR_CHANNEL = 1  # AUX 1 - Ana Motor

# Plus-Wing Kontrol Matrisi
# Her satır bir kontrol ekseni: [fin_1, fin_2, fin_3, fin_4] katsayıları
PLUS_WING_MATRIX = {
    'roll':  [0, 0, 1, -1],   # Üst kanat (+), Alt kanat (-)
    'pitch': [1, -1, 0, 0],   # Sol kanat (+), Sağ kanat (-)
    'yaw':   [1, 1, 1, 1]     # Tüm kanatlar aynı yönde
}

# PWM Değerleri (microseconds)
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

# Güvenlik Sınırları (HARDWARE_PIN_MAPPING.md'den)
PWM_SAFE_MIN = 1300  # Mekanik güvenlik alt sınır
PWM_SAFE_MAX = 1700  # Mekanik güvenlik üst sınır

# Servo Hareket Sınırları
SERVO_MAX_DELTA = 300    # Maksimum servo hareketi (±300µs)
OVERALL_MAX_DELTA_US = 350  # Genel maksimum delta

# GPIO Pin Tanımları
GPIO_STATUS_LED = 4      # Durum LED
GPIO_BUZZER_PWM = 13     # Buzzer PWM
GPIO_POWER_BUTTON = 18   # Güç Butonu
GPIO_EMERGENCY_STOP = 19 # Acil Durdurma

# I2C Cihazları
I2C_D300_ADDRESS = 0x76  # D300 Derinlik Sensörü

# Kontrol Parametreleri
CONTROL_FREQUENCY = 25   # Hz - Kontrol döngü frekansı
DEADBAND_ROLL = 2.0      # Derece - Roll deadband
DEADBAND_PITCH = 2.0     # Derece - Pitch deadband  
DEADBAND_YAW = 5.0       # Derece - Yaw deadband

# Filtre Parametreleri
ATTITUDE_LPF_ALPHA = 0.7    # Attitude low-pass filter
DEPTH_LPF_ALPHA = 0.8       # Depth low-pass filter

print("✅ Plus-Wing hardware konfigürasyonu yüklendi")
