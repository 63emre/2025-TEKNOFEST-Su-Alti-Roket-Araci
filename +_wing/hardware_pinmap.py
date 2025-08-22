"""
TEKNOFEST 2025 Su Altı Roket Aracı
+ Wing (Plus Wing) Konfigürasyon - Hardware Pin Haritası

Bu dosya + kanat konfigürasyonu için tüm hardware pin tanımlarını içerir.
Fin isimlendirme: Üst, Alt, Sol, Sağ
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
    MOTOR_AUX = 1           # AUX 1 - Ana Motor (DEGZ M5 Su Geçirmez Motor)
    # AUX_2 = BOZUK - KULLANILMAYACAK
    FIN_UPPER_AUX = 3       # AUX 3 - Üst Fin (DS3230MG Servo)
    FIN_LOWER_AUX = 4       # AUX 4 - Alt Fin (DS3230MG Servo)  
    FIN_LEFT_AUX = 5        # AUX 5 - Sol Fin (DS3230MG Servo)
    FIN_RIGHT_AUX = 6       # AUX 6 - Sağ Fin (DS3230MG Servo)
    
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
# FIN KONTROL KONFIGÜRASYONU (+ WING)
# =============================================================================

class FinControlConfig:
    """+ Wing Fin Kontrol Konfigürasyonu"""
    
    # Fin isimleri ve AUX port eşleşmeleri
    FINS = {
        "upper": {
            "name": "Üst Fin",
            "aux_port": PixhawkConfig.FIN_UPPER_AUX,
            "description": "Yukarı yönlü hareket kontrolü"
        },
        "lower": {
            "name": "Alt Fin", 
            "aux_port": PixhawkConfig.FIN_LOWER_AUX,
            "description": "Aşağı yönlü hareket kontrolü"
        },
        "left": {
            "name": "Sol Fin",
            "aux_port": PixhawkConfig.FIN_LEFT_AUX, 
            "description": "Sola yönlü hareket kontrolü"
        },
        "right": {
            "name": "Sağ Fin",
            "aux_port": PixhawkConfig.FIN_RIGHT_AUX,
            "description": "Sağa yönlü hareket kontrolü"  
        }
    }
    
    # Hareket komutları
    MOVEMENT_COMMANDS = {
        "yukarı": {"upper": PixhawkConfig.SERVO_MAX, "lower": PixhawkConfig.SERVO_MIN},
        "aşağı": {"upper": PixhawkConfig.SERVO_MIN, "lower": PixhawkConfig.SERVO_MAX},
        "sola": {"left": PixhawkConfig.SERVO_MAX, "right": PixhawkConfig.SERVO_MIN},
        "sağa": {"left": PixhawkConfig.SERVO_MIN, "right": PixhawkConfig.SERVO_MAX},
        "nötr": {"upper": PixhawkConfig.SERVO_NEUTRAL, "lower": PixhawkConfig.SERVO_NEUTRAL,
                "left": PixhawkConfig.SERVO_NEUTRAL, "right": PixhawkConfig.SERVO_NEUTRAL}
    }

# =============================================================================
# PID KONTROL KONFIGÜRASYONU (+ WING)
# =============================================================================

class PIDConfig:
    """PID Kontrol Parametreleri - + Wing Konfigürasyonu"""
    
    # Roll PID Parametreleri (+ konfigürasyonu için dikey fin kontrolü)
    ROLL_PID = {
        "kp": 3.2,      # + wing için biraz daha yüksek gain
        "ki": 0.12,
        "kd": 1.0,
        "max_output": 500,
        "integral_limit": 100,
        "setpoint": 0.0
    }
    
    # Pitch PID Parametreleri
    PITCH_PID = {
        "kp": 3.5,      # + wing pitch kontrolü daha doğrudan
        "ki": 0.18,
        "kd": 1.1,
        "max_output": 500,
        "integral_limit": 100,
        "setpoint": 0.0
    }
    
    # Yaw PID Parametreleri  
    YAW_PID = {
        "kp": 2.2,
        "ki": 0.08,
        "kd": 0.7,
        "max_output": 400,
        "integral_limit": 80,
        "setpoint": 0.0
    }
    
    # Derinlik PID Parametreleri
    DEPTH_PID = {
        "kp": 150.0,
        "ki": 5.0,
        "kd": 25.0,
        "max_output": 800,
        "integral_limit": 200,
        "setpoint": 1.0
    }
    
    # Hız PID Parametreleri
    SPEED_PID = {
        "kp": 100.0,
        "ki": 2.0,
        "kd": 15.0,
        "max_output": 900,
        "integral_limit": 150,
        "setpoint": 0.5
    }
    
    # Pozisyon Kontrolü PID
    DISTANCE_PID = {
        "kp": 80.0,
        "ki": 1.5,
        "kd": 20.0,
        "max_output": 600,
        "integral_limit": 120,
        "setpoint": 2.0
    }
    
    # PID Güncellenme Frekansı
    UPDATE_FREQUENCY = 50
    
    # Stabilizasyon Modları
    STABILIZATION_MODES = {
        "MANUAL": 0,
        "STABILIZE": 1,
        "DEPTH_HOLD": 2,
        "AUTO_PILOT": 3,
        "HOVER": 4
    }

# =============================================================================
# FIN MIXİNG KONFIGÜRASYONU (+ WING)
# =============================================================================

class FinMixingConfig:
    """+ Wing Fin Karıştırma Matrisi"""
    
    # + konfigürasyon fin mixing matrisi
    MIXING_MATRIX = {
        "roll": {
            "upper": 0.0,      # Üst fin roll'a katkı vermiyor
            "lower": 0.0,      # Alt fin roll'a katkı vermiyor  
            "left": -1.0,      # Sol fin roll sağa için negatif
            "right": +1.0      # Sağ fin roll sağa için pozitif
        },
        "pitch": {
            "upper": +1.0,     # Üst fin pitch up için pozitif
            "lower": -1.0,     # Alt fin pitch up için negatif
            "left": 0.0,       # Sol fin pitch'e katkı vermiyor
            "right": 0.0       # Sağ fin pitch'e katkı vermiyor
        },
        "yaw": {
            "upper": +0.5,     # Üst fin yaw'a kısmi katkı
            "lower": -0.5,     # Alt fin yaw'a ters katkı
            "left": +0.5,      # Sol fin yaw'a kısmi katkı
            "right": -0.5      # Sağ fin yaw'a ters katkı
        }
    }
    
    # Fin efektiflik katsayıları
    FIN_EFFECTIVENESS = {
        "upper": 1.0,
        "lower": 1.0,
        "left": 1.0,
        "right": 1.0
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
    """+ Wing fin konfigürasyonunu döndürür"""
    return FinControlConfig.FINS

def get_movement_command(direction):
    """Belirli bir yön için hareket komutunu döndürür"""
    return FinControlConfig.MOVEMENT_COMMANDS.get(direction.lower(), 
                                                 FinControlConfig.MOVEMENT_COMMANDS["nötr"])

def print_configuration():
    """Tüm konfigürasyonu konsola yazdırır"""
    print("="*50)
    print("+ WING KONFIGÜRASYONU")  
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
