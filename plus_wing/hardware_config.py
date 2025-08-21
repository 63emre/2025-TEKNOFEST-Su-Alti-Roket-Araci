#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
Plus Wing (+) Konfigürasyonu Hardware Mapping
Pi5 Test Sistemi - Gerçek Hardware Bağlantıları
"""

# PLUS WING SERVO MAPPING (AUX → MAVLink Channel)
PLUS_WING_SERVO_CHANNELS = {
    'on_servo': 11,      # AUX 1 → Ön Servo (MAVLink Channel 9)
    'sol_servo': 12,    # AUX 3 → Sol Servo (MAVLink Channel 11)
    'sag_servo': 13,    # AUX 4 → Sağ Servo (MAVLink Channel 12)
    'arka_servo': 14,   # AUX 5 → Arka Servo (MAVLink Channel 13)
    'ana_motor': 9     # AUX 6 → Ana Motor (MAVLink Channel 14)
}

# PLUS WING HARDWARE CONFIGURATION
PLUS_WING_CONFIG = {
    # Servo Hardware Specs
    'SERVOS': {
        'on_servo': {
            'aux': 1, 
            'mavlink_channel': 9, 
            'name': 'Ön Servo',
            'model': 'DS3230MG 30kg',
            'position': 'front'
        },
        'sol_servo': {
            'aux': 3, 
            'mavlink_channel': 11, 
            'name': 'Sol Servo',
            'model': 'DS3230MG 30kg',
            'position': 'left'
        },
        'sag_servo': {
            'aux': 4, 
            'mavlink_channel': 12, 
            'name': 'Sağ Servo',
            'model': 'DS3230MG 30kg',
            'position': 'right'
        },
        'arka_servo': {
            'aux': 5, 
            'mavlink_channel': 13, 
            'name': 'Arka Servo',
            'model': 'DS3230MG 30kg',
            'position': 'rear'
        }
    },
    
    # Motor Hardware
    'MOTOR': {
        'ana_motor': {
            'aux': 6, 
            'mavlink_channel': 14, 
            'name': 'Ana İtki Motoru',
            'model': 'DEGZ M5 + DEGZ BLU 30A ESC'
        }
    },
    
    # Plus Wing Control Matrix
    'PLUS_MIXING': {
        'roll_sol': ['sol_servo'],                    # Roll sol: Sol servo MIN
        'roll_sag': ['sag_servo'],                    # Roll sağ: Sağ servo MIN
        'pitch_yukari': ['on_servo'],                 # Pitch yukarı: Ön servo MIN
        'pitch_asagi': ['arka_servo'],                # Pitch aşağı: Arka servo MIN
        'yaw_sol': ['on_servo', 'sol_servo', 'arka_servo', 'sag_servo'],  # Yaw sol: Tüm servo koordineli
        'yaw_sag': ['on_servo', 'sag_servo', 'arka_servo', 'sol_servo']   # Yaw sağ: Tüm servo ters
    },
    
    # PWM Signal Specifications
    'PWM_LIMITS': {
        'servo_min': 1000,
        'servo_neutral': 1500,
        'servo_max': 2000,
        'motor_min': 1000,
        'motor_neutral': 1500,
        'motor_max': 2000,
        'frequency': 333  # Hz
    },
    
    # Control Parameters
    'CONTROL_GAINS': {
        'pitch_gain': 12,    # Pitch hassasiyeti (Plus Wing optimized)
        'roll_gain': 15,     # Roll hassasiyeti (Plus Wing optimized)
        'yaw_gain': 8,       # Yaw hassasiyeti (Plus Wing optimized)
        'motor_gain': 1.0    # Motor güç çarpanı
    },
    
    # PID Controller Parameters (Plus Wing Tuned)
    'PID_PARAMS': {
        'roll': {'kp': 0.8, 'ki': 0.12, 'kd': 0.05, 'max_output': 400},
        'pitch': {'kp': 0.9, 'ki': 0.10, 'kd': 0.06, 'max_output': 400},
        'yaw': {'kp': 0.6, 'ki': 0.08, 'kd': 0.04, 'max_output': 300},
        'depth': {'kp': 1.2, 'ki': 0.15, 'kd': 0.08, 'max_output': 500}
    },
    
    # Performance Limits
    'LIMITS': {
        'max_servo_angle': 45,      # Maksimum servo açısı (derece)
        'max_motor_power': 100,     # Maksimum motor gücü (%)
        'control_rate': 50,         # Kontrol update rate (Hz)
        'response_time': 0.02,      # Sistem yanıt süresi (saniye)
        'stability_threshold': 2.0  # Kararlılık eşik değeri (derece)
    }
}

# Plus Wing Servo PWM Calculation Functions
def calculate_plus_wing_pwm(roll, pitch, yaw):
    """
    Plus Wing konfigürasyonu için servo PWM değerlerini hesapla
    
    Args:
        roll: Roll komutu (-100 to +100)
        pitch: Pitch komutu (-100 to +100) 
        yaw: Yaw komutu (-100 to +100)
    
    Returns:
        dict: Servo PWM değerleri
    """
    neutral = PLUS_WING_CONFIG['PWM_LIMITS']['servo_neutral']
    
    # Control gains
    pitch_gain = PLUS_WING_CONFIG['CONTROL_GAINS']['pitch_gain']
    roll_gain = PLUS_WING_CONFIG['CONTROL_GAINS']['roll_gain']
    yaw_gain = PLUS_WING_CONFIG['CONTROL_GAINS']['yaw_gain']
    
    # Plus Wing Mixing Matrix
    pwm_values = {
        'on_servo': neutral + int((pitch * pitch_gain) + (yaw * yaw_gain * 0.5)),
        'arka_servo': neutral - int((pitch * pitch_gain) - (yaw * yaw_gain * 0.5)),
        'sol_servo': neutral + int((roll * roll_gain) + (yaw * yaw_gain * 0.7)),
        'sag_servo': neutral - int((roll * roll_gain) - (yaw * yaw_gain * 0.7))
    }
    
    # PWM limit kontrolü
    pwm_min = PLUS_WING_CONFIG['PWM_LIMITS']['servo_min']
    pwm_max = PLUS_WING_CONFIG['PWM_LIMITS']['servo_max']
    
    for servo in pwm_values:
        pwm_values[servo] = max(pwm_min, min(pwm_max, pwm_values[servo]))
    
    return pwm_values

def get_plus_wing_channels():
    """Plus Wing servo kanallarını döndür"""
    return PLUS_WING_SERVO_CHANNELS

def get_plus_wing_config():
    """Tam Plus Wing konfigürasyonunu döndür"""
    return PLUS_WING_CONFIG

def compare_with_x_wing():
    """X Wing konfigürasyonu ile karşılaştırma bilgileri"""
    return {
        'plus_wing_advantages': [
            'Basit kontrol mantığı',
            'Doğrusal hareket kontrolü',
            'Az coupling etkisi',
            'Kolay kalibrasyon',
            'Başlangıç dostu'
        ],
        'x_wing_advantages': [
            'Yüksek manevra kabiliyeti',
            'Karmaşık hareket kombinasyonları',
            'İyi yaw kontrolü',
            'Profesyonel performans'
        ],
        'use_cases': {
            'plus_wing_ideal': [
                'Basit hareket görevleri',
                'Doğrusal navigasyon',
                'Eğitim amaçlı kullanım',
                'Kolay bakım gereksinimleri'
            ],
            'x_wing_ideal': [
                'Karmaşık manevra gereksinimleri',
                'Hassas konum kontrolü',
                'Profesyonel operasyonlar',
                'Gelişmiş otonom görevler'
            ]
        }
    }

def print_plus_wing_info():
    """Plus Wing konfigürasyon bilgilerini yazdır"""
    print("🔧 TEKNOFEST Plus Wing (+) Konfigürasyonu")
    print("=" * 50)
    
    print("\n📍 Servo Kanalları:")
    for servo_name, channel in PLUS_WING_SERVO_CHANNELS.items():
        servo_info = PLUS_WING_CONFIG['SERVOS'].get(servo_name, {'name': servo_name})
        print(f"   {servo_info.get('name', servo_name)}: Channel {channel}")
    
    print(f"\n⚙️ PWM Limitleri:")
    pwm = PLUS_WING_CONFIG['PWM_LIMITS']
    print(f"   Min: {pwm['servo_min']}µs")
    print(f"   Neutral: {pwm['servo_neutral']}µs")
    print(f"   Max: {pwm['servo_max']}µs")
    print(f"   Frequency: {pwm['frequency']}Hz")
    
    print(f"\n🎮 Kontrol Gains:")
    gains = PLUS_WING_CONFIG['CONTROL_GAINS']
    print(f"   Pitch Gain: {gains['pitch_gain']}")
    print(f"   Roll Gain: {gains['roll_gain']}")
    print(f"   Yaw Gain: {gains['yaw_gain']}")

if __name__ == "__main__":
    print_plus_wing_info()
    
    # Test PWM hesaplama
    print("\n🧪 Test PWM Hesaplaması:")
    test_commands = [
        (0, 0, 0, "Neutral"),
        (50, 0, 0, "Roll Sağ"),
        (0, 50, 0, "Pitch Yukarı"),
        (0, 0, 50, "Yaw Sağ"),
        (25, 25, 25, "Kombine Hareket")
    ]
    
    for roll, pitch, yaw, description in test_commands:
        pwm_values = calculate_plus_wing_pwm(roll, pitch, yaw)
        print(f"\n   {description} (R:{roll}, P:{pitch}, Y:{yaw}):")
        for servo, pwm in pwm_values.items():
            print(f"     {servo}: {pwm}µs")
