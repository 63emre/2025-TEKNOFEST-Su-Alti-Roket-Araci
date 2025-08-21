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
# Pi5 GPIO Support - RPi.GPIO yerine gpiozero kullan
try:
    import RPi.GPIO as GPIO
    HAS_RPI_GPIO = True
    print("✅ RPi.GPIO library loaded")
except ImportError:
    HAS_RPI_GPIO = False
    print("⚠️ RPi.GPIO not available")

# Pi5 için alternatif GPIO
try:
    from gpiozero import PWMOutputDevice, LED
    HAS_GPIOZERO = True
    print("✅ gpiozero library loaded (Pi5 compatible)")
except ImportError:
    HAS_GPIOZERO = False
    print("⚠️ gpiozero not available")

# MAVLink Serial bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
import os
try:
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Using dynamic serial connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to serial config with environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Using fallback serial connection: {MAV_ADDRESS}")

# GERÇEK HARDWARE - Servo kanalları (AUX to MAVLink channel mapping)
SERVO_CHANNELS = {
    'aux3': 11,   # AUX 1 = Ön Sol Fin (MAVLink 9)
    'aux4': 12,  # AUX 3 = Ön Sağ Fin (MAVLink 11)
    'aux5': 13,  # AUX 4 = Arka Sol Fin (MAVLink 12)
    'aux6': 14,  # AUX 5 = Arka Sağ Fin (MAVLink 13)
    'aux1': 9   # AUX 6 = Ana Motor (MAVLink 14)
}

# GERÇEK X-Wing Konfigürasyonu
X_WING_CONFIG = {
    'front_left': 'aux3',   # Ön Sol (AUX1)
    'front_right': 'aux4',  # Ön Sağ (AUX3)
    'rear_left': 'aux5',    # Arka Sol (AUX4)
    'rear_right': 'aux6',   # Arka Sağ (AUX5)
    'motor': 'aux1'         # Ana Motor (AUX6)
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
        
        # GPIO - Pi5 compatible
        self.gpio_initialized = False
        self.buzzer_pwm = None
        self.buzzer_device = None  # gpiozero device
        self.gpio_method = "none"  # "rpi_gpio", "gpiozero", or "none"
        
        # GERÇEK HARDWARE - Servo pozisyonları
        self.servo_positions = {
            'aux1': PWM_MID,  # Ön Sol Fin
            'aux3': PWM_MID,  # Ön Sağ Fin
            'aux4': PWM_MID,  # Arka Sol Fin
            'aux5': PWM_MID,  # Arka Sağ Fin
            'aux6': PWM_MID   # Ana Motor
        }
        
        # Real-time control state tracking
        self.last_control_time = time.time()
        self.control_rate = 0
        self.movement_state = {
            'roll': 0, 'pitch': 0, 'yaw': 0, 'motor': 0
        }
        
        # Kontrol modu
        self.control_mode = 'individual'  # 'individual', 'x_pattern', 'synchronized'
        self.running = False
        
        # Terminal ayarları
        self.old_settings = None
        
    def setup_gpio(self):
        """GPIO pinlerini başlat - Pi5 compatible"""
        # Pi5 için gpiozero dene
        if HAS_GPIOZERO:
            try:
                self.buzzer_device = PWMOutputDevice(GPIO_BUZZER_PWM, frequency=BUZZER_FREQUENCY)
                self.gpio_initialized = True
                self.gpio_method = "gpiozero"
                print("✅ GPIO pinleri başlatıldı (gpiozero - Pi5 compatible)")
                return True
            except Exception as e:
                print(f"⚠️ gpiozero GPIO hatası: {e}")
        
        # Fallback: RPi.GPIO dene
        if HAS_RPI_GPIO:
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
                self.gpio_method = "rpi_gpio"
                print("✅ GPIO pinleri başlatıldı (RPi.GPIO - eski Pi)")
                return True
                
            except Exception as e:
                print(f"⚠️ RPi.GPIO hatası: {e}")
        
        # GPIO başarısız - ama devam et
        print("⚠️ GPIO başlatılamadı - buzzer devre dışı ama test devam ediyor")
        self.gpio_initialized = False
        self.gpio_method = "none"
        return True  # Test'e devam etsin
    
    def cleanup_gpio(self):
        """GPIO temizliği - Pi5 compatible"""
        try:
            if self.gpio_initialized:
                if self.gpio_method == "gpiozero" and self.buzzer_device:
                    self.buzzer_device.close()
                    print("🔄 GPIO temizlendi (gpiozero)")
                elif self.gpio_method == "rpi_gpio" and self.buzzer_pwm:
                    self.buzzer_pwm.stop()
                    GPIO.cleanup()
                    print("🔄 GPIO temizlendi (RPi.GPIO)")
        except Exception as e:
            print(f"⚠️ GPIO temizleme uyarısı: {e}")
    
    def play_tone(self, frequency, duration=0.1, volume=30):
        """Buzzer ton çalma - Pi5 compatible"""
        if not self.gpio_initialized:
            return
            
        try:
            if self.gpio_method == "gpiozero" and self.buzzer_device:
                # gpiozero ile buzzer
                pwm_value = volume / 100.0  # 0-1 range
                self.buzzer_device.pulse(fade_in_time=0, fade_out_time=duration, n=1)
            elif self.gpio_method == "rpi_gpio" and self.buzzer_pwm:
                # RPi.GPIO ile buzzer
                self.buzzer_pwm.ChangeFrequency(frequency)
                self.buzzer_pwm.ChangeDutyCycle(volume)
                time.sleep(duration)
                self.buzzer_pwm.ChangeDutyCycle(0)
        except Exception as e:
            # Buzzer hatası önemli değil - sessizce devam et
            pass
    
    def log_realtime_movement(self, movement_type, details):
        """Real-time hareket logları - VIDEO TEST için optimize"""
        current_time = time.time()
        
        # Control rate hesapla
        rate = 1.0 / (current_time - self.last_control_time) if current_time > self.last_control_time else 0
        self.control_rate = rate
        
        print(f"🎮 {movement_type}: {details}")
        print(f"   📊 Control Rate: {rate:.1f}Hz")
        print(f"   📍 Current PWM Values:")
        print(f"      AUX1(Ön Sol): {self.servo_positions['aux1']}µs")
        print(f"      AUX3(Ön Sağ): {self.servo_positions['aux3']}µs")
        print(f"      AUX4(Arka Sol): {self.servo_positions['aux4']}µs")
        print(f"      AUX5(Arka Sağ): {self.servo_positions['aux5']}µs")
        print(f"      AUX6(Motor): {self.servo_positions['aux6']}µs")
        
        # Dinamik hareket durumu
        active_movements = []
        if self.movement_state['roll'] != 0:
            direction = "SAĞ" if self.movement_state['roll'] > 0 else "SOL" 
            active_movements.append(f"ROLL {direction}")
        if self.movement_state['pitch'] != 0:
            direction = "YUKARI" if self.movement_state['pitch'] > 0 else "AŞAĞI"
            active_movements.append(f"PITCH {direction}")
        if self.movement_state['yaw'] != 0:
            direction = "SAĞ" if self.movement_state['yaw'] > 0 else "SOL"
            active_movements.append(f"YAW {direction}")
        if self.movement_state['motor'] != 0:
            direction = "İLERİ" if self.movement_state['motor'] > 0 else "GERİ"
            active_movements.append(f"MOTOR {direction}")
        
        if active_movements:
            print(f"   🚀 Aktif Hareketler: {' + '.join(active_movements)}")
        
        print("-" * 60)
    
    def update_control_rate(self):
        """Control rate güncelle"""
        self.last_control_time = time.time()
    
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor...")
            
            # Handle serial vs TCP connection
            if ',' in MAV_ADDRESS:
                # Serial connection: port,baud
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                # TCP or other connection
                print(f"🌐 TCP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            
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
        """X pattern kontrolü - GERÇEK HARDWARE MAPPING"""
        moved = False
        
        # GERÇEK HARDWARE MAPPING:
        # AUX1: Ön Sol,  AUX3: Ön Sağ,  AUX4: Arka Sol,  AUX5: Arka Sağ
        
        # Roll kontrolü (A/D - Sol/Sağ taraf finleri)
        if key == 'A':  # Sol roll
            # Sol taraf finler (AUX1+AUX4) yukarı, sağ taraf (AUX3+AUX5) aşağı
            self.set_servo_pwm('aux1', min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP))  # Ön Sol +
            self.set_servo_pwm('aux4', min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP))  # Arka Sol +
            self.set_servo_pwm('aux3', max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP))  # Ön Sağ -
            self.set_servo_pwm('aux5', max(PWM_MIN, self.servo_positions['aux5'] - PWM_STEP))  # Arka Sağ -
            self.movement_state['roll'] = -1  # Sol tarafa roll
            self.log_realtime_movement("ROLL SOL", f"Sol finler YUKARİ, Sağ finler AŞAĞI")
            moved = True
        elif key == 'D':  # Sağ roll
            # Sağ taraf finler (AUX3+AUX5) yukarı, sol taraf (AUX1+AUX4) aşağı
            self.set_servo_pwm('aux1', max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP))  # Ön Sol -
            self.set_servo_pwm('aux4', max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP))  # Arka Sol -
            self.set_servo_pwm('aux3', min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP))  # Ön Sağ +
            self.set_servo_pwm('aux5', min(PWM_MAX, self.servo_positions['aux5'] + PWM_STEP))  # Arka Sağ +
            self.movement_state['roll'] = 1   # Sağ tarafa roll
            self.log_realtime_movement("ROLL SAĞ", f"Sağ finler YUKARİ, Sol finler AŞAĞI")
            moved = True
        
        # Pitch kontrolü (W/S - Ön/Arka finler)
        elif key == 'W':  # Pitch up (burun yukarı)
            # Ön finler (AUX1+AUX3) aşağı, arka finler (AUX4+AUX5) yukarı
            self.set_servo_pwm('aux1', max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP))  # Ön Sol -
            self.set_servo_pwm('aux3', max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP))  # Ön Sağ -
            self.set_servo_pwm('aux4', min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP))  # Arka Sol +
            self.set_servo_pwm('aux5', min(PWM_MAX, self.servo_positions['aux5'] + PWM_STEP))  # Arka Sağ +
            self.movement_state['pitch'] = 1  # Pitch up
            self.log_realtime_movement("PITCH YUKARI", f"Ön finler AŞAĞI, Arka finler YUKARİ")
            moved = True
        elif key == 'S':  # Pitch down (burun aşağı)
            # Ön finler (AUX1+AUX3) yukarı, arka finler (AUX4+AUX5) aşağı
            self.set_servo_pwm('aux1', min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP))  # Ön Sol +
            self.set_servo_pwm('aux3', min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP))  # Ön Sağ +
            self.set_servo_pwm('aux4', max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP))  # Arka Sol -
            self.set_servo_pwm('aux5', max(PWM_MIN, self.servo_positions['aux5'] - PWM_STEP))  # Arka Sağ -
            self.movement_state['pitch'] = -1 # Pitch down
            self.log_realtime_movement("PITCH AŞAĞI", f"Ön finler YUKARİ, Arka finler AŞAĞI")
            moved = True
        
        # Yaw kontrolü (Q/E - X diagonal)
        elif key == 'Q':  # Yaw left (sola döüş)
            # X-diagonal: AUX1(Ön Sol)+AUX5(Arka Sağ) vs AUX3(Ön Sağ)+AUX4(Arka Sol)
            self.set_servo_pwm('aux1', min(PWM_MAX, self.servo_positions['aux1'] + PWM_STEP))  # Ön Sol +
            self.set_servo_pwm('aux5', min(PWM_MAX, self.servo_positions['aux5'] + PWM_STEP))  # Arka Sağ +
            self.set_servo_pwm('aux3', max(PWM_MIN, self.servo_positions['aux3'] - PWM_STEP))  # Ön Sağ -
            self.set_servo_pwm('aux4', max(PWM_MIN, self.servo_positions['aux4'] - PWM_STEP))  # Arka Sol -
            self.movement_state['yaw'] = -1   # Yaw left
            self.log_realtime_movement("YAW SOL", f"X-diagonal: AUX1+AUX5 vs AUX3+AUX4")
            moved = True
        elif key == 'E':  # Yaw right (sağa dönüş)
            # X-diagonal: AUX3(Ön Sağ)+AUX4(Arka Sol) vs AUX1(Ön Sol)+AUX5(Arka Sağ)
            self.set_servo_pwm('aux1', max(PWM_MIN, self.servo_positions['aux1'] - PWM_STEP))  # Ön Sol -
            self.set_servo_pwm('aux5', max(PWM_MIN, self.servo_positions['aux5'] - PWM_STEP))  # Arka Sağ -
            self.set_servo_pwm('aux3', min(PWM_MAX, self.servo_positions['aux3'] + PWM_STEP))  # Ön Sağ +
            self.set_servo_pwm('aux4', min(PWM_MAX, self.servo_positions['aux4'] + PWM_STEP))  # Arka Sol +
            self.movement_state['yaw'] = 1    # Yaw right
            self.log_realtime_movement("YAW SAĞ", f"X-diagonal: AUX3+AUX4 vs AUX1+AUX5")
            moved = True
        
        # Motor kontrolü (O/L)
        elif key == 'O':  # Motor ileri
            self.set_servo_pwm('aux6', min(PWM_MAX, self.servo_positions['aux6'] + PWM_STEP))
            self.movement_state['motor'] = 1
            self.log_realtime_movement("MOTOR İLERİ", f"PWM: {self.servo_positions['aux6']}")
            moved = True
        elif key == 'L':  # Motor geri
            self.set_servo_pwm('aux6', max(PWM_MIN, self.servo_positions['aux6'] - PWM_STEP))
            self.movement_state['motor'] = -1
            self.log_realtime_movement("MOTOR GERİ", f"PWM: {self.servo_positions['aux6']}")
            moved = True
        
        if moved:
            self.play_tone(NOTES['move'], 0.05, 25)
            self.update_control_rate()
    
    def synchronized_control(self, key):
        """Senkronize kontrol - tüm finler birlikte - GERÇEK HARDWARE"""
        moved = False
        
        # GERÇEK HARDWARE: AUX1, AUX3, AUX4, AUX5, AUX6
        
        if key == 'W':  # Tüm finler yukarı
            for aux in ['aux1', 'aux3', 'aux4', 'aux5']:
                new_pos = min(PWM_MAX, self.servo_positions[aux] + PWM_STEP)
                self.set_servo_pwm(aux, new_pos)
            self.log_realtime_movement("TÜM FİNLER YUKARI", f"Senkronize hareket: +{PWM_STEP}µs")
            moved = True
        elif key == 'S':  # Tüm finler aşağı
            for aux in ['aux1', 'aux3', 'aux4', 'aux5']:
                new_pos = max(PWM_MIN, self.servo_positions[aux] - PWM_STEP)
                self.set_servo_pwm(aux, new_pos)
            self.log_realtime_movement("TÜM FİNLER AŞAĞI", f"Senkronize hareket: -{PWM_STEP}µs")
            moved = True
        elif key == 'O':  # Motor ileri
            new_pos = min(PWM_MAX, self.servo_positions['aux6'] + PWM_STEP)
            self.set_servo_pwm('aux6', new_pos)
            self.log_realtime_movement("MOTOR İLERİ (SYNC)", f"PWM: {new_pos}µs")
            moved = True
        elif key == 'L':  # Motor geri
            new_pos = max(PWM_MIN, self.servo_positions['aux6'] - PWM_STEP)
            self.set_servo_pwm('aux6', new_pos)
            self.log_realtime_movement("MOTOR GERİ (SYNC)", f"PWM: {new_pos}µs")
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