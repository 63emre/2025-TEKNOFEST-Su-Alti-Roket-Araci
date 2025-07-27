#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
X Kanat Konfigürasyonu Gerçek Zamanlı Servo Kontrolü

AUX 1, 3, 4, 5 servo kontrolü ve GPIO 13 buzzer ile interaktif test
Klavye ile gerçek zamanlı X kanat kontrolü

Hardware: 
- AUX 1: Ön Sol Fin (DS3230MG 30kg)
- AUX 3: Arka Sol Fin (DS3230MG 30kg)  
- AUX 4: Arka Sağ Fin (DS3230MG 30kg)
- AUX 5: Ekstra Kontrol Fin
- GPIO 7: PWM Buzzer (Yüksek Voltaj Pin)

Pin Mapping: HARDWARE_PIN_MAPPING.md standardına göre
"""

import time
import threading
import sys
import termios
import tty
import select
from pymavlink import mavutil
import RPi.GPIO as GPIO

# MAVLink bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
try:
    from connection_config import get_test_constants
    CONFIG = get_test_constants()
    MAV_ADDRESS = CONFIG['MAV_ADDRESS']
    print(f"📡 Using dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to static config
    MAV_ADDRESS = 'tcp:127.0.0.1:5777'
    print(f"⚠️ Using fallback connection: {MAV_ADDRESS}")

# Servo kanalları (AUX to MAVLink channel mapping)
SERVO_CHANNELS = {
    'aux1': 9,   # AUX 1 = Servo channel 9 (Ön Sol)
    'aux3': 11,  # AUX 3 = Servo channel 11 (Arka Sol)
    'aux4': 12,  # AUX 4 = Servo channel 12 (Arka Sağ)
    'aux5': 13   # AUX 5 = Servo channel 13 (Ekstra)
}

# X Kanat Konfigürasyonu
X_WING_CONFIG = {
    'front_left': 'aux1',   # Ön Sol
    'rear_left': 'aux3',    # Arka Sol
    'rear_right': 'aux4',   # Arka Sağ
    'extra': 'aux5'         # Ekstra kontrol
}

# GPIO Pin Tanımları
GPIO_BUZZER_PWM = 7         # PWM Buzzer (Yüksek voltaj pin)

# Servo ayarları
SERVO_FREQUENCY = 330       # 330Hz servo frekansı
PWM_MIN = 1000             # Minimum PWM (µs)
PWM_MID = 1500             # Orta PWM (µs) 
PWM_MAX = 2000             # Maksimum PWM (µs)
PWM_STEP = 50              # PWM artış adımı

# Buzzer ayarları
BUZZER_FREQUENCY = 2000     # 2kHz buzzer frequency
PWM_FREQ = 1000            # 1kHz PWM frequency

# Musical Notes (Hz) - Feedback için
NOTES = {
    'move': 1000,
    'neutral': 1500, 
    'limit': 500,
    'mode_change': 2000,
    'error': 300,
    'success': 880
}

class XWingRealtimeController:
    """X Kanat Gerçek Zamanlı Kontrol Sınıfı"""
    
    def __init__(self):
        # MAVLink
        self.master = None
        self.connected = False
        
        # GPIO
        self.gpio_initialized = False
        self.buzzer_pwm = None
        
        # Servo pozisyonları
        self.servo_positions = {
            'aux1': PWM_MID,
            'aux3': PWM_MID,
            'aux4': PWM_MID,
            'aux5': PWM_MID
        }
        
        # Kontrol modu
        self.control_mode = 'individual'  # 'individual', 'x_pattern', 'synchronized'
        self.running = False
        
        # Terminal ayarları
        self.old_settings = None
        
    def setup_gpio(self):
        """GPIO pinlerini başlat"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Buzzer pin setup
            GPIO.setup(GPIO_BUZZER_PWM, GPIO.OUT)
            GPIO.output(GPIO_BUZZER_PWM, GPIO.LOW)
            
            # Setup PWM for Buzzer
            self.buzzer_pwm = GPIO.PWM(GPIO_BUZZER_PWM, BUZZER_FREQUENCY)
            self.buzzer_pwm.start(0)
            
            self.gpio_initialized = True
            print("✅ GPIO pinleri başlatıldı")
            return True
            
        except Exception as e:
            print(f"❌ GPIO başlatma hatası: {e}")
            return False
    
    def cleanup_gpio(self):
        """GPIO temizliği"""
        try:
            if self.gpio_initialized and self.buzzer_pwm:
                self.buzzer_pwm.stop()
                GPIO.cleanup()
                print("🔄 GPIO temizlendi")
        except Exception as e:
            print(f"⚠️ GPIO temizleme uyarısı: {e}")
    
    def play_tone(self, frequency, duration=0.1, volume=30):
        """Buzzer ton çalma"""
        if not self.gpio_initialized:
            return
            
        try:
            self.buzzer_pwm.ChangeFrequency(frequency)
            self.buzzer_pwm.ChangeDutyCycle(volume)
            time.sleep(duration)
            self.buzzer_pwm.ChangeDutyCycle(0)
        except Exception as e:
            print(f"⚠️ Buzzer hatası: {e}")
    
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Servo frekansını ayarla
            self.set_servo_frequency(SERVO_FREQUENCY)
            
            # Başarı tonu
            self.play_tone(NOTES['success'], 0.2)
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            self.play_tone(NOTES['error'], 0.3)
            return False
    
    def set_servo_frequency(self, frequency):
        """Servo frekansını ayarla"""
        if not self.connected:
            return False
            
        try:
            print(f"🔧 Servo frekansı ayarlanıyor: {frequency}Hz")
            
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'PWM_AUX_RATE',
                frequency,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            
            time.sleep(0.5)
            print(f"✅ Servo frekansı {frequency}Hz olarak ayarlandı")
            return True
            
        except Exception as e:
            print(f"❌ Frekans ayarlama hatası: {e}")
            return False
    
    def set_servo_pwm(self, aux_name, pwm_value):
        """Servo PWM ayarı"""
        if not self.connected or aux_name not in SERVO_CHANNELS:
            return False
            
        # PWM değer kontrolü
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            channel = SERVO_CHANNELS[aux_name]
            
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            
            self.servo_positions[aux_name] = pwm_value
            return True
            
        except Exception as e:
            print(f"❌ Servo kontrol hatası: {e}")
            return False
    
    def setup_terminal(self):
        """Terminal raw mode ayarı"""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def restore_terminal(self):
        """Terminal ayarlarını geri yükle"""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def get_char(self):
        """Karakter okuma (non-blocking)"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        return None
    
    def print_status(self):
        """Mevcut durumu göster"""
        print(f"\r🎮 Mod: {self.control_mode.upper()} | "
              f"AUX1:{self.servo_positions['aux1']} | "
              f"AUX3:{self.servo_positions['aux3']} | "
              f"AUX4:{self.servo_positions['aux4']} | "
              f"AUX5:{self.servo_positions['aux5']} ", end='', flush=True)
    
    def individual_control(self, key):
        """Tekil servo kontrolü"""
        moved = False
        
        # AUX1 kontrolü (Q/A)
        if key == 'Q':
            new_pos = min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP)
            self.set_servo_pwm('aux1', new_pos)
            moved = True
        elif key == 'A':
            new_pos = max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP)
            self.set_servo_pwm('aux1', new_pos)
            moved = True
        
        # AUX3 kontrolü (W/S)
        elif key == 'W':
            new_pos = min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP)
            self.set_servo_pwm('aux3', new_pos)
            moved = True
        elif key == 'S':
            new_pos = max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP)
            self.set_servo_pwm('aux3', new_pos)
            moved = True
        
        # AUX4 kontrolü (E/D)
        elif key == 'E':
            new_pos = min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP)
            self.set_servo_pwm('aux4', new_pos)
            moved = True
        elif key == 'D':
            new_pos = max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP)
            self.set_servo_pwm('aux4', new_pos)
            moved = True
        
        # AUX5 kontrolü (R/F)
        elif key == 'R':
            new_pos = min(PWM_MAX, self.servo_positions['aux5'] + PWM_STEP)
            self.set_servo_pwm('aux5', new_pos)
            moved = True
        elif key == 'F':
            new_pos = max(PWM_MIN, self.servo_positions['aux5'] - PWM_STEP)
            self.set_servo_pwm('aux5', new_pos)
            moved = True
        
        if moved:
            self.play_tone(NOTES['move'], 0.05, 20)
    
    def x_pattern_control(self, key):
        """X pattern kontrolü"""
        moved = False
        
        # Roll kontrolü (A/D - Sol/Sağ finler birlikte)
        if key == 'A':  # Sol roll
            # Sol finler yukarı, sağ finler aşağı
            self.set_servo_pwm('aux1', min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP))
            self.set_servo_pwm('aux3', min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP))
            self.set_servo_pwm('aux4', max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP))
            moved = True
        elif key == 'D':  # Sağ roll
            # Sağ finler yukarı, sol finler aşağı
            self.set_servo_pwm('aux1', max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP))
            self.set_servo_pwm('aux3', max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP))
            self.set_servo_pwm('aux4', min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP))
            moved = True
        
        # Pitch kontrolü (W/S - Ön/Arka finler birlikte)
        elif key == 'W':  # Pitch up
            # Ön finler aşağı, arka finler yukarı
            self.set_servo_pwm('aux1', max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP))
            self.set_servo_pwm('aux3', min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP))
            self.set_servo_pwm('aux4', min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP))
            moved = True
        elif key == 'S':  # Pitch down
            # Ön finler yukarı, arka finler aşağı
            self.set_servo_pwm('aux1', min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP))
            self.set_servo_pwm('aux3', max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP))
            self.set_servo_pwm('aux4', max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP))
            moved = True
        
        # Yaw kontrolü (Q/E - X diagonal)
        elif key == 'Q':  # Yaw left
            # AUX1 & AUX4 bir yön, AUX3 diğer yön
            self.set_servo_pwm('aux1', min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP))
            self.set_servo_pwm('aux4', min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP))
            self.set_servo_pwm('aux3', max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP))
            moved = True
        elif key == 'E':  # Yaw right
            # AUX3 & AUX1 bir yön, AUX4 diğer yön
            self.set_servo_pwm('aux1', max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP))
            self.set_servo_pwm('aux4', max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP))
            self.set_servo_pwm('aux3', min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP))
            moved = True
        
        if moved:
            self.play_tone(NOTES['move'], 0.05, 25)
    
    def synchronized_control(self, key):
        """Senkronize kontrol - tüm finler birlikte"""
        moved = False
        
        if key == 'W':  # Tüm finler yukarı
            for aux in ['aux1', 'aux3', 'aux4', 'aux5']:
                new_pos = min(PWM_MAX, self.servo_positions[aux] + PWM_STEP)
                self.set_servo_pwm(aux, new_pos)
            moved = True
        elif key == 'S':  # Tüm finler aşağı
            for aux in ['aux1', 'aux3', 'aux4', 'aux5']:
                new_pos = max(PWM_MIN, self.servo_positions[aux] - PWM_STEP)
                self.set_servo_pwm(aux, new_pos)
            moved = True
        
        if moved:
            self.play_tone(NOTES['move'], 0.08, 30)
    
    def reset_all_servos(self):
        """Tüm servoları orta pozisyona getir"""
        print("\n🔄 Tüm servolar orta pozisyona getiriliyor...")
        for aux in ['aux1', 'aux3', 'aux4', 'aux5']:
            self.set_servo_pwm(aux, PWM_MID)
        
        self.play_tone(NOTES['neutral'], 0.2, 40)
        print("✅ Reset tamamlandı!")
    
    def print_help(self):
        """Yardım menüsü"""
        print("\n" + "="*80)
        print("🎮 X KANAT GERÇEK ZAMANLI KONTROL - YARDIM MENÜSÜ")
        print("="*80)
        print(f"📡 Mevcut Mod: {self.control_mode.upper()}")
        print()
        
        if self.control_mode == 'individual':
            print("🔧 TEKİL SERVO KONTROLÜ:")
            print("   Q/A: AUX1 (Ön Sol) +/-")
            print("   W/S: AUX3 (Arka Sol) +/-") 
            print("   E/D: AUX4 (Arka Sağ) +/-")
            print("   R/F: AUX5 (Ekstra) +/-")
            
        elif self.control_mode == 'x_pattern':
            print("✈️ X PATTERN KONTROLÜ:")
            print("   A/D: Roll (Sol/Sağ)")
            print("   W/S: Pitch (Yukarı/Aşağı)")
            print("   Q/E: Yaw (Sol/Sağ)")
            
        elif self.control_mode == 'synchronized':
            print("🔄 SENKRONİZE KONTROL:")
            print("   W/S: Tüm finler birlikte +/-")
        
        print()
        print("🎛️ GENEL KOMUTLAR:")
        print("   1/2/3: Kontrol modu değiştir")
        print("   0: Tüm servoları orta pozisyona getir")
        print("   H: Bu yardım menüsünü göster")
        print("   ESC: Çıkış")
        print("="*80)
        
        self.play_tone(NOTES['mode_change'], 0.15, 35)
    
    def run_realtime_control(self):
        """Gerçek zamanlı kontrol ana döngüsü"""
        if not self.setup_gpio():
            return False
            
        if not self.connect_pixhawk():
            return False
        
        try:
            self.setup_terminal()
            self.running = True
            
            print("\n🚀 X KANAT GERÇEK ZAMANLI KONTROL BAŞLATILDI")
            print("H tuşuna basarak yardım menüsünü açabilirsiniz")
            print("ESC tuşu ile çıkış yapabilirsiniz")
            
            # Başlangıç tonu
            self.play_tone(NOTES['success'], 0.3, 40)
            
            # Tüm servoları orta pozisyona getir
            self.reset_all_servos()
            
            while self.running:
                self.print_status()
                
                key = self.get_char()
                if key:
                    # ESC tuşu (çıkış)
                    if ord(key) == 27:  # ESC
                        self.play_tone(NOTES['mode_change'], 0.2, 30)
                        break
                    
                    key = key.upper()
                    
                    # Mod değiştirme
                    if key == '1':
                        self.control_mode = 'individual'
                        self.play_tone(NOTES['mode_change'], 0.1)
                        print(f"\n🔧 Mod: TEKİL KONTROL")
                    elif key == '2':
                        self.control_mode = 'x_pattern'
                        self.play_tone(NOTES['mode_change'], 0.1)
                        print(f"\n✈️ Mod: X PATTERN KONTROL")
                    elif key == '3':
                        self.control_mode = 'synchronized'
                        self.play_tone(NOTES['mode_change'], 0.1)
                        print(f"\n🔄 Mod: SENKRONİZE KONTROL")
                    
                    # Reset
                    elif key == '0':
                        self.reset_all_servos()
                    
                    # Yardım
                    elif key == 'H':
                        self.print_help()
                    
                    # Kontrol komutları
                    else:
                        if self.control_mode == 'individual':
                            self.individual_control(key)
                        elif self.control_mode == 'x_pattern':
                            self.x_pattern_control(key)
                        elif self.control_mode == 'synchronized':
                            self.synchronized_control(key)
                
                time.sleep(0.05)  # 20Hz update rate
            
            print("\n\n✅ Gerçek zamanlı kontrol sonlandırıldı")
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Program kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Program hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.running = False
        
        # Tüm servoları orta pozisyona getir
        if self.connected:
            for aux in ['aux1', 'aux3', 'aux4', 'aux5']:
                self.set_servo_pwm(aux, PWM_MID)
        
        # MAVLink bağlantısını kapat
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        # GPIO temizle
        self.cleanup_gpio()
        
        # Terminal ayarlarını geri yükle
        self.restore_terminal()

def main():
    """Ana fonksiyon"""
    print("🎮 TEKNOFEST 2025 - X Kanat Gerçek Zamanlı Kontrol")
    print("=" * 60)
    
    controller = XWingRealtimeController()
    
    try:
        return 0 if controller.run_realtime_control() else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program kullanıcı tarafından durduruldu")
        return 1
    except Exception as e:
        print(f"❌ Program hatası: {e}")
        return 1
    finally:
        controller.cleanup()

if __name__ == "__main__":
    sys.exit(main()) 