#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UTILS - Yardımcı Fonksiyonlar
LED kontrolü, buzzer kontrolü, zamanlayıcılar ve loglama fonksiyonları
"""

import time
import threading
import RPi.GPIO as GPIO
from datetime import datetime
from config import *

class LEDController:
    """Kırmızı LED kontrol sınıfı"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_LED_RED, GPIO.OUT)
        self.led_state = False
        self.blink_thread = None
        self.blink_running = False
        
    def set_led(self, state):
        """LED'i aç/kapat"""
        self.led_state = state
        GPIO.output(GPIO_LED_RED, state)
        
    def turn_on(self):
        """LED'i aç"""
        self.set_led(True)
        
    def turn_off(self):
        """LED'i kapat"""
        self.set_led(False)
        
    def blink(self, interval=0.5, count=None):
        """LED'i yanıp söndür
        Args:
            interval: Yanıp sönme aralığı (saniye)
            count: Kaç kez yanıp söneceği (None=sürekli)
        """
        self.stop_blink()
        
        def blink_worker():
            blink_count = 0
            while self.blink_running:
                if count is not None and blink_count >= count * 2:
                    break
                    
                self.set_led(not self.led_state)
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
        if self.blink_thread and self.blink_thread.is_alive():
            self.blink_thread.join(timeout=1.0)
            
    def cleanup(self):
        """Temizlik"""
        self.stop_blink()
        self.turn_off()

class BuzzerController:
    """Buzzer kontrol sınıfı"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_BUZZER, GPIO.OUT)
        self.buzzer_thread = None
        self.buzzer_running = False
        
    def beep(self, duration):
        """Tek bip çıkar"""
        GPIO.output(GPIO_BUZZER, GPIO.HIGH)
        time.sleep(duration)
        GPIO.output(GPIO_BUZZER, GPIO.LOW)
        
    def beep_pattern(self, pattern):
        """Bip patterni çal
        Args:
            pattern: [süre1, süre2, süre3, ...] listesi
                    Çift indeksler HIGH, tek indeksler LOW süresi
        """
        self.stop_buzzer()
        
        def pattern_worker():
            for i, duration in enumerate(pattern):
                if not self.buzzer_running:
                    break
                    
                state = GPIO.HIGH if i % 2 == 0 else GPIO.LOW
                GPIO.output(GPIO_BUZZER, state)
                time.sleep(duration)
                
            GPIO.output(GPIO_BUZZER, GPIO.LOW)
            self.buzzer_running = False
            
        self.buzzer_running = True
        self.buzzer_thread = threading.Thread(target=pattern_worker)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
        
    def countdown_buzzer(self, total_seconds):
        """90 saniye geri sayım buzzer'ı
        90 saniye = 10 döngü x 9 saniye
        Her döngü: 9 kısa bip + 1 uzun bip
        """
        self.stop_buzzer()
        
        def countdown_worker():
            cycles = total_seconds // 9  # 90/9 = 10 döngü
            
            for cycle in range(cycles):
                if not self.buzzer_running:
                    break
                    
                # 9 kısa bip
                for short_beep in range(9):
                    if not self.buzzer_running:
                        break
                    
                    GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                    time.sleep(BUZZER_COUNTDOWN_SHORT)
                    GPIO.output(GPIO_BUZZER, GPIO.LOW)
                    time.sleep(BUZZER_COUNTDOWN_PAUSE)
                    
                # 1 uzun bip (10. saniye)
                if self.buzzer_running:
                    GPIO.output(GPIO_BUZZER, GPIO.HIGH)
                    time.sleep(BUZZER_COUNTDOWN_LONG)
                    GPIO.output(GPIO_BUZZER, GPIO.LOW)
                    
                    # Son döngü değilse 8.5 saniye bekle
                    if cycle < cycles - 1:
                        time.sleep(8.5)
                        
            GPIO.output(GPIO_BUZZER, GPIO.LOW)
            self.buzzer_running = False
            
        self.buzzer_running = True
        self.buzzer_thread = threading.Thread(target=countdown_worker)
        self.buzzer_thread.daemon = True
        self.buzzer_thread.start()
        
    def mission_start_buzzer(self):
        """Görev başlangıcı - 5 saniyede bip"""
        self.beep_pattern(BUZZER_MISSION_START)
        
    def mission_end_buzzer(self):
        """Görev bitişi - 3 saniyede 1 bip"""
        self.beep_pattern(BUZZER_MISSION_END)
        
    def emergency_buzzer(self):
        """Acil durum buzzer'ı"""
        self.beep_pattern(BUZZER_EMERGENCY)
        
    def stop_buzzer(self):
        """Buzzer'ı durdur"""
        self.buzzer_running = False
        if self.buzzer_thread and self.buzzer_thread.is_alive():
            self.buzzer_thread.join(timeout=1.0)
        GPIO.output(GPIO_BUZZER, GPIO.LOW)
        
    def cleanup(self):
        """Temizlik"""
        self.stop_buzzer()

class ButtonController:
    """Buton kontrol sınıfı"""
    
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_START_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self.last_press_time = 0
        self.debounce_time = 0.2  # 200ms debounce
        
    def is_pressed(self):
        """Buton basılı mı kontrol et (debounce ile)"""
        current_time = time.time()
        button_state = not GPIO.input(GPIO_START_BUTTON)  # Pull-up, basıldığında LOW
        
        if button_state and (current_time - self.last_press_time) > self.debounce_time:
            self.last_press_time = current_time
            return True
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
    """Basit loglama sınıfı"""
    
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

class SystemStatus:
    """Sistem durumu takip sınıfı"""
    
    def __init__(self):
        self.led = LEDController()
        self.buzzer = BuzzerController()
        self.button = ButtonController()
        self.timer = Timer()
        self.logger = Logger("sara_mission.log")
        
        # Durum değişkenleri
        self.mission_phase = MissionPhase.WAITING
        self.is_emergency = False
        self.button_toggle_state = False  # Soft-kill butonu toggle durumu
        
    def set_phase(self, phase):
        """Görev fazını değiştir"""
        self.mission_phase = phase
        self.logger.info(f"Görev fazı değişti: {phase}")
        
        # Faza göre LED ve buzzer kontrolü
        if phase == MissionPhase.WAITING:
            self.led.blink(0.5)  # Yavaş yanıp sön
        elif phase == MissionPhase.CALIBRATION:
            self.led.turn_on()
        elif phase in [MissionPhase.PHASE_1, MissionPhase.PHASE_2, MissionPhase.RETURN]:
            self.led.turn_on()
        elif phase == MissionPhase.COMPLETED:
            self.led.blink(0.2, 10)  # Hızlı 10 kez yanıp sön
            self.buzzer.mission_end_buzzer()
        elif phase == MissionPhase.EMERGENCY:
            self.led.blink(0.1)  # Çok hızlı yanıp sön
            self.buzzer.emergency_buzzer()
            
    def check_start_button(self):
        """Başlatma butonunu kontrol et ve toggle durumunu yönet"""
        if self.button.is_pressed():
            self.button_toggle_state = not self.button_toggle_state
            self.logger.info(f"Başlatma butonu basıldı, durum: {self.button_toggle_state}")
            
            if self.button_toggle_state:
                self.logger.info("Görev başlatma modu")
                return "start"
            else:
                self.logger.info("Görev durdurma modu")
                return "stop"
                
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
        GPIO.cleanup()
        self.logger.info("Sistem temizliği tamamlandı")

# Yardımcı fonksiyonlar
def wait_with_button_check(duration, button_controller, logger=None):
    """Belirtilen süre bekle ama buton kontrolü yap
    Args:
        duration: Bekleme süresi (saniye)
        button_controller: ButtonController instance
        logger: Logger instance (opsiyonel)
    Returns:
        True: Normal süre doldu
        False: Buton basıldı
    """
    start_time = time.time()
    
    while time.time() - start_time < duration:
        if button_controller.is_pressed():
            if logger:
                logger.info("Bekleme sırasında buton basıldı")
            return False
        time.sleep(0.1)
        
    return True

def estimate_distance(speed_pwm, elapsed_time):
    """Hız ve zamandan mesafe tahmini
    Args:
        speed_pwm: Motor PWM değeri
        elapsed_time: Geçen süre (saniye)
    Returns:
        Tahmini mesafe (metre)
    """
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
    try:
        GPIO.cleanup()
    except Exception as e:
        print(f"GPIO temizleme hatası: {e}")

# Global sistem durumu (ana dosyadan kullanılabilir)
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
