#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - GERÇEK HARDWARE MAPPING
X-Konfigürasyon Fin Sistemi + Motor Kontrol
"""

# GERÇEK HARDWARE CONFIGURATION - VIDEO TEST READY
HARDWARE_CONFIG = {
    # X-Configuration Fin System
    'FINS': {
        'front_left': {'aux': 1, 'mavlink_channel': 9, 'name': 'Ön Sol Fin'},
        'front_right': {'aux': 3, 'mavlink_channel': 11, 'name': 'Ön Sağ Fin'},
        'rear_left': {'aux': 4, 'mavlink_channel': 12, 'name': 'Arka Sol Fin'},
        'rear_right': {'aux': 5, 'mavlink_channel': 13, 'name': 'Arka Sağ Fin'}
    },
    
    # Motor System
    'MOTOR': {
        'main': {'aux': 6, 'mavlink_channel': 14, 'name': 'Ana İtki Motoru'}
    },
    
    # X-Mixing Matrix (Real-time control için)
    'X_MIXING': {
        'roll_left': ['front_left', 'rear_left'],     # Sol taraf finler
        'roll_right': ['front_right', 'rear_right'],  # Sağ taraf finler
        'pitch_up': ['front_left', 'front_right'],    # Ön finler
        'pitch_down': ['rear_left', 'rear_right'],    # Arka finler
        'yaw_right': ['front_left', 'rear_right'],    # X-diagonal 1
        'yaw_left': ['front_right', 'rear_left']      # X-diagonal 2
    },
    
    # PWM Limits
    'PWM_LIMITS': {
        'servo_min': 1000,
        'servo_neutral': 1500, 
        'servo_max': 2000,
        'motor_min': 1000,
        'motor_neutral': 1500,
        'motor_max': 2000
    },
    
    # Control Limits
    'CONTROL_LIMITS': {
        'max_fin_angle': 45,      # Maksimum fin açısı (derece)
        'max_motor_power': 100,   # Maksimum motor gücü (%)
        'response_rate': 50       # Control update rate (Hz)
    }
}

def get_fin_channels():
    """X-konfigürasyon fin kanallarını döndür"""
    return {
        'AUX1_FRONT_LEFT': HARDWARE_CONFIG['FINS']['front_left']['mavlink_channel'],
        'AUX3_FRONT_RIGHT': HARDWARE_CONFIG['FINS']['front_right']['mavlink_channel'], 
        'AUX4_REAR_LEFT': HARDWARE_CONFIG['FINS']['rear_left']['mavlink_channel'],
        'AUX5_REAR_RIGHT': HARDWARE_CONFIG['FINS']['rear_right']['mavlink_channel']
    }

def get_motor_channel():
    """Motor kanalını döndür"""
    return HARDWARE_CONFIG['MOTOR']['main']['mavlink_channel']

def calculate_x_mixing(roll, pitch, yaw, power_limit=45):
    """
    X-konfigürasyon mixing hesaplama
    Real-time control için optimize edilmiş
    """
    # Input normalization (-45 to +45 derece)
    roll = max(-power_limit, min(power_limit, roll))
    pitch = max(-power_limit, min(power_limit, pitch))  
    yaw = max(-power_limit, min(power_limit, yaw))
    
    # X-Mixing calculations
    fins = {
        'front_left': pitch + roll + yaw,    # Ön Sol (AUX1)
        'front_right': pitch - roll - yaw,   # Ön Sağ (AUX3)
        'rear_left': -pitch + roll - yaw,    # Arka Sol (AUX4)
        'rear_right': -pitch - roll + yaw    # Arka Sağ (AUX5)
    }
    
    # Limit checking and normalization
    for fin in fins:
        fins[fin] = max(-power_limit, min(power_limit, fins[fin]))
    
    return fins

def fins_to_pwm(fin_angles):
    """Fin açılarını PWM değerlerine çevir"""
    pwm_values = {}
    
    for fin_name, angle in fin_angles.items():
        # Linear interpolation: -45° → 1000µs, 0° → 1500µs, +45° → 2000µs
        pwm_value = int(HARDWARE_CONFIG['PWM_LIMITS']['servo_neutral'] + 
                       (angle / 45.0) * 500)
        
        # PWM limits
        pwm_value = max(HARDWARE_CONFIG['PWM_LIMITS']['servo_min'], 
                       min(HARDWARE_CONFIG['PWM_LIMITS']['servo_max'], pwm_value))
        
        pwm_values[fin_name] = pwm_value
    
    return pwm_values

def log_control_state(roll, pitch, yaw, motor_power, fins, pwm_values):
    """Real-time control durumunu logla - VIDEO TEST için optimize"""
    print(f"🎮 REAL-TIME CONTROL:")
    print(f"   📐 Input: R={roll:+5.1f}° P={pitch:+5.1f}° Y={yaw:+5.1f}° M={motor_power:+3.0f}%")
    print(f"   🎯 Fins:")
    print(f"      ÖN SOL (AUX1): {fins['front_left']:+5.1f}° → {pwm_values['front_left']}µs")
    print(f"      ÖN SAĞ (AUX3): {fins['front_right']:+5.1f}° → {pwm_values['front_right']}µs") 
    print(f"      ARKA SOL (AUX4): {fins['rear_left']:+5.1f}° → {pwm_values['rear_left']}µs")
    print(f"      ARKA SAĞ (AUX5): {fins['rear_right']:+5.1f}° → {pwm_values['rear_right']}µs")
    
    # Dinamik hareket bilgisi
    if abs(roll) > 2:
        direction = "SAĞ" if roll > 0 else "SOL"
        print(f"   ↔️ ROLL HAREKETİ: {direction} taraf aktif ({abs(roll):.1f}°)")
    
    if abs(pitch) > 2:
        direction = "YUKARI" if pitch > 0 else "AŞAĞI"  
        print(f"   ↕️ PITCH HAREKETİ: {direction} hareket ({abs(pitch):.1f}°)")
        
    if abs(yaw) > 2:
        direction = "SAĞ" if yaw > 0 else "SOL"
        print(f"   🔄 YAW HAREKETİ: {direction} dönüş ({abs(yaw):.1f}°)")

def get_hardware_status():
    """Hardware durumu özeti"""
    channels = get_fin_channels()
    motor_ch = get_motor_channel()
    
    status = {
        'configuration': 'X-Wing 4-Fin + Motor',
        'fins': {
            'front_left_aux1': channels['AUX1_FRONT_LEFT'],
            'front_right_aux3': channels['AUX3_FRONT_RIGHT'],
            'rear_left_aux4': channels['AUX4_REAR_LEFT'], 
            'rear_right_aux5': channels['AUX5_REAR_RIGHT']
        },
        'motor_aux6': motor_ch,
        'ready_for_video': True
    }
    
    return status

if __name__ == "__main__":
    print("🚀 TEKNOFEST ROV - Hardware Mapping Test")
    print("=" * 50)
    
    # Hardware status
    status = get_hardware_status()
    print(f"📋 Configuration: {status['configuration']}")
    print(f"🎯 Fins:")
    for fin, channel in status['fins'].items():
        print(f"   {fin.upper()}: MAVLink Channel {channel}")
    print(f"🚁 Motor: MAVLink Channel {status['motor_aux6']}")
    
    # Test X-mixing
    print(f"\n🧪 X-Mixing Test:")
    test_roll, test_pitch, test_yaw = 20, -15, 10
    
    fins = calculate_x_mixing(test_roll, test_pitch, test_yaw)
    pwm_values = fins_to_pwm(fins)
    
    log_control_state(test_roll, test_pitch, test_yaw, 50, fins, pwm_values)
    
    print(f"\n✅ Hardware mapping hazır - Video test için optimize edildi!") 