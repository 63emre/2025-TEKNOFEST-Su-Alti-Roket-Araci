#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UTILS AIR - Hava Yarışı için Yardımcı Fonksiyonlar
GPIO kontrolü, zamanlayıcılar ve loglama fonksiyonları
Pluswing/utils.py'den uyarlanmıştır
"""

import time
import threading
from datetime import datetime
from config_air import *

# GPIO import - Raspberry Pi 5 uyumlu
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
    print("✅ RPi.GPIO sistemi yüklendi")
except ImportError:
    try:
        import lgpio as GPIO
        GPIO_AVAILABLE = True
        print("✅ lgpio sistemi yüklendi")
    except ImportError:
        GPIO_AVAILABLE = False
        print("❌ GPIO sistemi mevcut değil - simülasyon modunda çalışacak")

class LEDController:
    """Kırmızı LED kontrol sınıfı"""
    
    def __init__(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(GPIO_LED_RED, GPIO.OUT)
                self.led_state = False
                self.blink_thread = None
                self.blink_running = False
            except Exception as e:
                print(f"LED GPIO kurulum hatası: {e}")
                GPIO_AVAILABLE = False
        
    def set_led(self, state):
        """LED'i aç/kapat"""
        if GPIO_AVAILABLE:
            try:
                self.led_state = state
                GPIO.output(GPIO_LED_RED, state)
            except:
                pass
        else:
            print(f"LED Simülasyon: {'ON' if state else 'OFF'}")
        
    def turn_on(self):
        """LED'i aç"""
        self.set_led(True)
        
    def turn_off(self):
        """LED'i kapat"""
        self.set_led(False)
        
    def blink(self, interval=0.5, count=None):
        """LED'i yanıp söndür"""
        self.stop_blink()
        
        def blink_worker():
            blink_count = 0
            while self.blink_running:
                if count is not None and blink_count >= count * 2:
                    break
                    
                self.set_led(not getattr(self, 'led_state', False))
                time.sleep(interval)
                blink_count += 1
                
            self.blink_running = False
            
        self.blink_running = True
        self.blink_thread = threading.Thread(target=blink_worker)
        self.blink_thread.daemon = True
        self.blink_thread.start()
        
    def stop_blink(self):
        """Yanıp sönmeyi durdur"""
        self.blink_running = False
        if hasattr(self, 'blink_thread') and self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join(timeout=1.0)
            
    def cleanup(self):
        """Temizlik"""
        self.stop_blink()
        self.turn_off()

class BuzzerController:
    """Buzzer kontrol sınıfı"""
    
    def __init__(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(GPIO_BUZZER, GPIO.OUT)
                self.buzzer_thread = None
                self.buzzer_running = False
            except Exception as e:
                print(f"Buzzer GPIO kurulum hatası: {e}")
        
    def beep(self, duration):
        """Tek bip çıkar"""
        if GPIO_AVAILABLE:
            try:
                GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                time.sleep(duration)
                GPIO.output(GPIO_BUZZER, GPIO.LOW)
            except:
                pass
        else:
            print(f"Buzzer Simülasyon: BEEP {duration}s")
            time.sleep(duration)
        
    def beep_pattern(self, pattern):
        """Bip patterni çal"""
        self.stop_buzzer()
        
        def pattern_worker():
            for i, duration in enumerate(pattern):
                if not self.buzzer_running:
                    break
                    
                if GPIO_AVAILABLE:
                    try:
                        state = GPIO.HIGH if i % 2 == 0 else GPIO.LOW
                        GPIO.output(GPIO_BUZZER, state)
                    except:
                        pass
                else:
                    if i % 2 == 0:
                        print(f"Buzzer: ON {duration}s")
                    
                time.sleep(duration)
                
            if GPIO_AVAILABLE:
                try:
                    GPIO.output(GPIO_BUZZER, GPIO.LOW)
                except:
                    pass
            self.buzzer_running = False
            
        self.buzzer_running = True
        self.buzzer_thread = threading.Thread(target=pattern_worker)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
        
    def test_complete_buzzer(self):
        """Test tamamlandı buzzer'ı"""
        self.beep_pattern(BUZZER_TEST_COMPLETE)
        
    def stop_buzzer(self):
        """Buzzer'ı durdur"""
        self.buzzer_running = False
        if hasattr(self, 'buzzer_thread') and self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=1.0)
        if GPIO_AVAILABLE:
            try:
                GPIO.output(GPIO_BUZZER, GPIO.LOW)
            except:
                pass
        
    def cleanup(self):
        """Temizlik"""
        self.stop_buzzer()

class ButtonController:
    """Buton kontrol sınıfı"""
    
    def __init__(self):
        if GPIO_AVAILABLE:
            try:
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(GPIO_START_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                self.last_press_time = 0
                self.debounce_time = 1.0
            except Exception as e:
                print(f"Button GPIO kurulum hatası: {e}")
        
    def is_pressed(self):
        """Buton basıldı mı kontrol et"""
        if not GPIO_AVAILABLE:
            return False
            
        current_time = time.time()
        
        # Debounce kontrolü
        if (current_time - self.last_press_time) < self.debounce_time:
            return False
            
        try:
            # Mevcut durum
            current_state = GPIO.input(GPIO_START_BUTTON)  # 1=basıldı, 0=serbest
            
            # Önceki durumu kontrol et
            if not hasattr(self, 'previous_state'):
                self.previous_state = 0  # İlk çalışmada serbest kabul et
            
            # EDGE tespiti: 0→1 geçişi (serbest→basıldı)
            if self.previous_state == 0 and current_state == 1:
                # Gerçek basma anı tespit edildi
                self.last_press_time = current_time
                self.previous_state = current_state
                return True
            
            # Durumu güncelle
            self.previous_state = current_state
            return False
                
        except Exception as e:
            return False
        
    def wait_for_press(self, timeout=None):
        """Buton basılmasını bekle"""
        start_time = time.time()
        
        while True:
            if self.is_pressed():
                return True
                
            if timeout and (time.time() - start_time) > timeout:
                return False
                
            time.sleep(0.1)

class Timer:
    """Zamanlayıcı sınıfı"""
    
    def __init__(self):
        self.start_time = None
        self.pause_time = None
        self.paused_duration = 0
        
    def start(self):
        """Zamanlayıcıyı başlat"""
        self.start_time = time.time()
        self.pause_time = None
        self.paused_duration = 0
        
    def pause(self):
        """Zamanlayıcıyı duraklat"""
        if self.start_time and not self.pause_time:
            self.pause_time = time.time()
            
    def resume(self):
        """Zamanlayıcıyı devam ettir"""
        if self.pause_time:
            self.paused_duration += time.time() - self.pause_time
            self.pause_time = None
            
    def elapsed(self):
        """Geçen süreyi döndür (saniye)"""
        if not self.start_time:
            return 0
            
        current_time = time.time()
        if self.pause_time:
            # Duraklatılmış
            return (self.pause_time - self.start_time) - self.paused_duration
        else:
            # Çalışıyor
            return (current_time - self.start_time) - self.paused_duration
            
    def reset(self):
        """Zamanlayıcıyı sıfırla"""
        self.start_time = None
        self.pause_time = None
        self.paused_duration = 0
        
    def is_running(self):
        """Zamanlayıcı çalışıyor mu"""
        return self.start_time is not None and self.pause_time is None

class Logger:
    """Basit loglama sınıfı - Hava yarışı için"""
    
    def __init__(self, log_file=None):
        self.log_file = log_file
        
    def log(self, message, level="INFO"):
        """Log mesajı yazdır"""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        log_message = f"[{timestamp}] [{level}] {message}"
        
        print(log_message)
        
        if self.log_file:
            try:
                with open(self.log_file, "a", encoding="utf-8") as f:
                    f.write(log_message + "\n")
            except Exception as e:
                print(f"Log yazma hatası: {e}")
                
    def info(self, message):
        """Info seviyesi log"""
        self.log(message, "INFO")
        
    def warning(self, message):
        """Warning seviyesi log"""
        self.log(message, "WARNING")
        
    def error(self, message):
        """Error seviyesi log"""
        self.log(message, "ERROR")
        
    def debug(self, message):
        """Debug seviyesi log"""
        self.log(message, "DEBUG")
        
    def critical(self, message):
        """Critical seviyesi log"""
        self.log(message, "CRITICAL")

class SystemStatus:
    """Sistem durumu takip sınıfı - Hava yarışı için"""
    
    def __init__(self):
        self.led = LEDController()
        self.buzzer = BuzzerController()
        self.button = ButtonController()
        self.timer = Timer()
        self.logger = Logger("air_race_test.log")
        
        # Durum değişkenleri
        self.mission_phase = MissionPhase.WAITING
        self.is_emergency = False
        
    def set_phase(self, phase):
        """Görev fazını değiştir"""
        self.mission_phase = phase
        self.logger.info(f"Test fazı değişti: {phase}")
        
        # Faza göre LED ve buzzer kontrolü
        if phase == MissionPhase.WAITING:
            self.led.blink(0.5)  # Yavaş yanıp sön
        elif phase == MissionPhase.CALIBRATION:
            self.led.turn_on()
        elif phase in [MissionPhase.PHASE_1, MissionPhase.PHASE_2, MissionPhase.RETURN]:
            self.led.turn_on()
        elif phase == MissionPhase.COMPLETED:
            self.led.blink(0.2, 10)  # Hızlı 10 kez yanıp sön
            self.buzzer.test_complete_buzzer()
        elif phase == MissionPhase.EMERGENCY:
            self.led.blink(0.1)  # Çok hızlı yanıp sön
            
    def check_start_button(self):
        """Başlatma butonunu kontrol et"""
        if self.button.is_pressed():
            self.logger.info("🔘 Test butonu basıldı!")
            return "restart"
                
        return None
        
    def emergency_stop(self):
        """Acil durum prosedürü"""
        self.is_emergency = True
        self.set_phase(MissionPhase.EMERGENCY)
        self.logger.error("ACİL DURUM AKTIF!")
        
    def cleanup(self):
        """Sistem temizliği"""
        self.logger.info("Sistem temizleniyor...")
        self.led.cleanup()
        self.buzzer.cleanup()
        if GPIO_AVAILABLE:
            try:
                GPIO.cleanup()
            except:
                pass
        self.logger.info("Sistem temizliği tamamlandı")

# Yardımcı fonksiyonlar
def wait_with_button_check(duration, button_controller, logger=None):
    """Belirtilen süre bekle ama buton kontrolü yap"""
    start_time = time.time()
    
    while time.time() - start_time < duration:
        if button_controller.is_pressed():
            if logger:
                logger.info("Bekleme sırasında buton basıldı")
            return False
        time.sleep(0.1)
        
    return True

def establish_mavlink_connection(connection_string, retries=5, logger=None):
    """MAVLink bağlantısını güvenilir şekilde kur"""
    import sys
    from pymavlink import mavutil
    
    for attempt in range(retries):
        try:
            if logger:
                logger.info(f"MAVLink bağlantı denemesi {attempt+1}/{retries}: {connection_string}")
                
            mavlink_connection = mavutil.mavlink_connection(connection_string, baud=MAVLINK_BAUD)
            
            # Heartbeat bekle
            if logger:
                logger.info("Heartbeat bekleniyor...")
            heartbeat = mavlink_connection.wait_heartbeat(timeout=10)
            
            if heartbeat:
                if logger:
                    logger.info(f"MAVLink bağlantısı başarılı! Sistem ID: {mavlink_connection.target_system}")
                return True, mavlink_connection
            else:
                raise Exception("Heartbeat alınamadı")
                
        except Exception as e:
            if logger:
                logger.warning(f"Bağlantı denemesi {attempt+1}/{retries} başarısız: {e}")
            if attempt < retries - 1:
                time.sleep(2)
                
    if logger:
        logger.error("MAVLink bağlantısı kurulamadı!")
    return False, None

def estimate_distance(speed_pwm, elapsed_time):
    """Hız ve zamandan mesafe tahmini"""
    # PWM değerine göre hız tahmini
    if speed_pwm <= MOTOR_STOP:
        estimated_speed = 0
    elif speed_pwm <= SPEED_SLOW:
        estimated_speed = ESTIMATED_SPEED_SLOW
    elif speed_pwm <= SPEED_MEDIUM:
        estimated_speed = ESTIMATED_SPEED_MEDIUM
    else:
        estimated_speed = ESTIMATED_SPEED_FAST
        
    return estimated_speed * elapsed_time

def format_time(seconds):
    """Saniyeyi mm:ss formatına çevir"""
    minutes = int(seconds // 60)
    seconds = int(seconds % 60)
    return f"{minutes:02d}:{seconds:02d}"

def safe_gpio_cleanup():
    """Güvenli GPIO temizleme"""
    if GPIO_AVAILABLE:
        try:
            GPIO.cleanup()
            print("✓ GPIO temizliği tamamlandı")
        except Exception as e:
            print(f"⚠️ GPIO temizleme hatası: {e}")
    else:
        print("✓ GPIO simülasyon - temizlik atlandı")

# Global sistem durumu
system_status = None

def init_system_status():
    """Sistem durumunu başlat"""
    global system_status
    if system_status is None:
        system_status = SystemStatus()
    return system_status

def get_system_status():
    """Sistem durumunu döndür"""
    global system_status
    if system_status is None:
        system_status = SystemStatus()
    return system_status
