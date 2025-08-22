"""
TEKNOFEST 2025 Su Altı Roket Aracı
X Wing Konfigürasyon - Hardware Pin Haritası

Bu dosya X kanat konfigürasyonu için tüm hardware pin tanımlarını içerir.
Fin isimlendirme: Üst Sağ, Üst Sol, Alt Sol, Alt Sağ
"""

import os

# =============================================================================
# PIXHAWK PIN KONFIGÜRASYONU
# =============================================================================

class PixhawkConfig:
    """Pixhawk PX4 PIX 2.4.8 32 Bit Pin Konfigürasyonu"""
    
    # MAVLink Bağlantısı
    MAVLINK_PORT = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    MAVLINK_BAUD = int(os.getenv("MAV_BAUD", "115200"))
    
    # AUX Çıkışları (PWM)
    MOTOR_AUX = 1               # AUX 1 - Ana Motor (DEGZ M5 Su Geçirmez Motor)
    # AUX_2 = BOZUK - KULLANILMAYACAK
    FIN_UPPER_RIGHT_AUX = 3     # AUX 3 - Üst Sağ Fin (DS3230MG Servo)
    FIN_UPPER_LEFT_AUX = 4      # AUX 4 - Üst Sol Fin (DS3230MG Servo)
    FIN_LOWER_LEFT_AUX = 5      # AUX 5 - Alt Sol Fin (DS3230MG Servo)
    FIN_LOWER_RIGHT_AUX = 6     # AUX 6 - Alt Sağ Fin (DS3230MG Servo)
    
    # PWM Değerleri
    SERVO_MIN = 1000        # Minimum PWM değeri
    SERVO_NEUTRAL = 1500    # Nötr PWM değeri
    SERVO_MAX = 2000        # Maksimum PWM değeri
    SERVO_FREQUENCY = 330   # PWM frekansı (Hz)
    
    # Motor PWM Değerleri (ESC için)
    MOTOR_STOP = 1000       # Motor durdurma
    MOTOR_MIN = 1100        # Minimum motor hızı
    MOTOR_MAX = 2000        # Maksimum motor hızı

# =============================================================================
# RASPBERRY PI GPIO KONFIGÜRASYONU
# =============================================================================

class RaspberryPiConfig:
    """Raspberry Pi 5 GPIO Pin Konfigürasyonu"""
    
    # Kontrol Butonu
    BUTTON_PIN = 18         # GPIO 18 - Sistem kontrol butonu (16A P1Z-EC)
    
    # LED ve Buzzer
    LED_PIN = 22            # GPIO 22 - Durum LED'i
    BUZZER_PIN = 23         # GPIO 23 - Buzzer
    
    # I2C Bağlantısı (D300 Derinlik Sensörü için)
    I2C_BUS = 1             # I2C Bus 1
    I2C_SDA_PIN = 2         # GPIO 2 (I2C SDA)
    I2C_SCL_PIN = 3         # GPIO 3 (I2C SCL)

# =============================================================================
# SENSÖR KONFIGÜRASYONU
# =============================================================================

class SensorConfig:
    """Sensör Pin ve Adres Konfigürasyonu"""
    
    # D300 Derinlik ve Sıcaklık Sensörü
    D300_I2C_ADDRESS = 0x76  # I2C adresi
    D300_BUS = RaspberryPiConfig.I2C_BUS
    
    # Pixhawk'tan alınacak sensör verileri
    PIXHAWK_DISTANCE_SENSOR = True  # Pixhawk mesafe sensörü kullanılacak
    PIXHAWK_IMU_DATA = True        # IMU verileri (roll, pitch, yaw)

# =============================================================================
# FIN KONTROL KONFIGÜRASYONU (X WING)
# =============================================================================

class FinControlConfig:
    """X Wing Fin Kontrol Konfigürasyonu"""
    
    # Fin isimleri ve AUX port eşleşmeleri
    FINS = {
        "upper_right": {
            "name": "Üst Sağ Fin",
            "aux_port": PixhawkConfig.FIN_UPPER_RIGHT_AUX,
            "description": "Sağ üst çapraz hareket kontrolü"
        },
        "upper_left": {
            "name": "Üst Sol Fin", 
            "aux_port": PixhawkConfig.FIN_UPPER_LEFT_AUX,
            "description": "Sol üst çapraz hareket kontrolü"
        },
        "lower_left": {
            "name": "Alt Sol Fin",
            "aux_port": PixhawkConfig.FIN_LOWER_LEFT_AUX, 
            "description": "Sol alt çapraz hareket kontrolü"
        },
        "lower_right": {
            "name": "Alt Sağ Fin",
            "aux_port": PixhawkConfig.FIN_LOWER_RIGHT_AUX,
            "description": "Sağ alt çapraz hareket kontrolü"  
        }
    }
    
    # Hareket komutları (X konfigürasyonu için çapraz hareket)
    MOVEMENT_COMMANDS = {
        "yukarı": {
            "upper_right": PixhawkConfig.SERVO_MAX, 
            "upper_left": PixhawkConfig.SERVO_MAX,
            "lower_left": PixhawkConfig.SERVO_MIN, 
            "lower_right": PixhawkConfig.SERVO_MIN
        },
        "aşağı": {
            "upper_right": PixhawkConfig.SERVO_MIN, 
            "upper_left": PixhawkConfig.SERVO_MIN,
            "lower_left": PixhawkConfig.SERVO_MAX, 
            "lower_right": PixhawkConfig.SERVO_MAX
        },
        "sola": {
            "upper_right": PixhawkConfig.SERVO_MIN, 
            "upper_left": PixhawkConfig.SERVO_MAX,
            "lower_left": PixhawkConfig.SERVO_MAX, 
            "lower_right": PixhawkConfig.SERVO_MIN
        },
        "sağa": {
            "upper_right": PixhawkConfig.SERVO_MAX, 
            "upper_left": PixhawkConfig.SERVO_MIN,
            "lower_left": PixhawkConfig.SERVO_MIN, 
            "lower_right": PixhawkConfig.SERVO_MAX
        },
        "roll_sağ": {
            "upper_right": PixhawkConfig.SERVO_MAX, 
            "upper_left": PixhawkConfig.SERVO_MIN,
            "lower_left": PixhawkConfig.SERVO_MIN, 
            "lower_right": PixhawkConfig.SERVO_MAX
        },
        "roll_sol": {
            "upper_right": PixhawkConfig.SERVO_MIN, 
            "upper_left": PixhawkConfig.SERVO_MAX,
            "lower_left": PixhawkConfig.SERVO_MAX, 
            "lower_right": PixhawkConfig.SERVO_MIN
        },
        "nötr": {
            "upper_right": PixhawkConfig.SERVO_NEUTRAL, 
            "upper_left": PixhawkConfig.SERVO_NEUTRAL,
            "lower_left": PixhawkConfig.SERVO_NEUTRAL, 
            "lower_right": PixhawkConfig.SERVO_NEUTRAL
        }
    }

# =============================================================================
# PID KONTROL KONFIGÜRASYONU (X WING)
# =============================================================================

class PIDConfig:
    """PID Kontrol Parametreleri - X Wing Konfigürasyonu"""
    
    # Roll PID Parametreleri (X konfigürasyonu için çapraz fin kontrolü)
    ROLL_PID = {
        "kp": 2.5,      # Proportional gain
        "ki": 0.1,      # Integral gain  
        "kd": 0.8,      # Derivative gain
        "max_output": 500,  # Maksimum PWM çıkış farkı (±500 from neutral)
        "integral_limit": 100,  # Integral windup sınırı
        "setpoint": 0.0     # Hedef roll açısı (derece)
    }
    
    # Pitch PID Parametreleri
    PITCH_PID = {
        "kp": 2.8,
        "ki": 0.15,
        "kd": 0.9,
        "max_output": 500,
        "integral_limit": 100,
        "setpoint": 0.0     # Hedef pitch açısı (derece)
    }
    
    # Yaw PID Parametreleri
    YAW_PID = {
        "kp": 1.8,
        "ki": 0.05,
        "kd": 0.6,
        "max_output": 400,
        "integral_limit": 80,
        "setpoint": 0.0     # Hedef yaw açısı (derece)
    }
    
    # Derinlik PID Parametreleri
    DEPTH_PID = {
        "kp": 150.0,    # Derinlik için daha yüksek gain
        "ki": 5.0,
        "kd": 25.0,
        "max_output": 800,  # Motor hızı için maksimum değişim
        "integral_limit": 200,
        "setpoint": 1.0     # Hedef derinlik (metre)
    }
    
    # Hız PID Parametreleri (ileri/geri hareket)
    SPEED_PID = {
        "kp": 100.0,
        "ki": 2.0,
        "kd": 15.0,
        "max_output": 900,
        "integral_limit": 150,
        "setpoint": 0.5     # Hedef hız (m/s)
    }
    
    # Pozisyon Kontrolü PID (mesafe sensörü tabanlı)
    DISTANCE_PID = {
        "kp": 80.0,
        "ki": 1.5,
        "kd": 20.0,
        "max_output": 600,
        "integral_limit": 120,
        "setpoint": 2.0     # Hedef mesafe (metre)
    }
    
    # PID Güncellenme Frekansı
    UPDATE_FREQUENCY = 50   # Hz (20ms döngü)
    
    # Stabilizasyon Modları
    STABILIZATION_MODES = {
        "MANUAL": 0,        # Manuel kontrol, PID kapalı
        "STABILIZE": 1,     # Roll/Pitch stabilizasyonu aktif
        "DEPTH_HOLD": 2,    # Derinlik sabit tutma modu
        "AUTO_PILOT": 3,    # Tam otomatik pilot
        "HOVER": 4          # Sabit pozisyon hovering
    }

# =============================================================================
# FIN MIXİNG KONFIGÜRASYONU (X WING)
# =============================================================================

class FinMixingConfig:
    """X Wing Fin Karıştırma Matrisi"""
    
    # X konfigürasyon fin mixing matrisi
    # Her hareket için fin katkı oranları (-1.0 ile 1.0 arası)
    MIXING_MATRIX = {
        "roll": {
            "upper_right": +1.0,   # Roll sağa için pozitif katkı
            "upper_left": -1.0,    # Roll sağa için negatif katkı
            "lower_left": -1.0,    # Roll sağa için negatif katkı
            "lower_right": +1.0    # Roll sağa için pozitif katkı
        },
        "pitch": {
            "upper_right": +1.0,   # Pitch up için pozitif katkı
            "upper_left": +1.0,    # Pitch up için pozitif katkı
            "lower_left": -1.0,    # Pitch up için negatif katkı
            "lower_right": -1.0    # Pitch up için negatif katkı
        },
        "yaw": {
            "upper_right": +0.7,   # Yaw right için katkı
            "upper_left": -0.7,    # Yaw right için ters katkı
            "lower_left": +0.7,    # Yaw right için katkı  
            "lower_right": -0.7    # Yaw right için ters katkı
        }
    }
    
    # Fin efektiflik katsayıları (kalibrasyona göre ayarlanacak)
    FIN_EFFECTIVENESS = {
        "upper_right": 1.0,
        "upper_left": 1.0,
        "lower_left": 1.0,
        "lower_right": 1.0
    }

# =============================================================================
# POWER VE SAFETY KONFIGÜRASYONU
# =============================================================================

class PowerConfig:
    """Güç ve Güvenlik Konfigürasyonu"""
    
    # Batarya Özellikleri
    BATTERY_VOLTAGE = 22.2      # 6S LiPo batarya voltajı
    BATTERY_CAPACITY = 1800     # mAh kapasitesi
    BATTERY_DISCHARGE_RATE = 65 # C deşarj oranı
    
    # ESC Özellikleri
    ESC_CURRENT_RATING = 30     # 30A ESC
    
    # Güvenlik
    EMERGENCY_STOP_PHYSICAL = True  # Fiziksel güç kesimi
    LOW_VOLTAGE_THRESHOLD = 18.0    # Düşük voltaj uyarı eşiği
    
    # Power Port Besleme
    POWER_FROM_MAIN_OUTPUTS = True  # Main çıkışlardan jumper ile besleme
    POWER_FROM_POWER_PORT = True    # Power port'tan besleme seçeneği

# =============================================================================
# YARDIMCI FONKSİYONLAR
# =============================================================================

def get_fin_configuration():
    """X Wing fin konfigürasyonunu döndürür"""
    return FinControlConfig.FINS

def get_movement_command(direction):
    """Belirli bir yön için hareket komutunu döndürür"""
    return FinControlConfig.MOVEMENT_COMMANDS.get(direction.lower(), 
                                                 FinControlConfig.MOVEMENT_COMMANDS["nötr"])

def print_configuration():
    """Tüm konfigürasyonu konsola yazdırır"""
    print("="*50)
    print("X WING KONFIGÜRASYONU")  
    print("="*50)
    print(f"MAVLink Port: {PixhawkConfig.MAVLINK_PORT}")
    print(f"MAVLink Baud: {PixhawkConfig.MAVLINK_BAUD}")
    print(f"Motor AUX: {PixhawkConfig.MOTOR_AUX}")
    print(f"D300 I2C Adres: 0x{SensorConfig.D300_I2C_ADDRESS:02X}")
    print("\nFin Konfigürasyonu:")
    for key, fin in FinControlConfig.FINS.items():
        print(f"  {fin['name']}: AUX {fin['aux_port']}")
    print("="*50)

if __name__ == "__main__":
    print_configuration()
