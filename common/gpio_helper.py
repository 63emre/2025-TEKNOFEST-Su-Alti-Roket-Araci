"""
TEKNOFEST 2025 Su Altı Roket Aracı
GPIO Control Helper Modülü

Bu modül LED, buzzer ve buton kontrolü için kullanılır.
"""

import time
import logging
import threading
try:
    import RPi.GPIO as GPIO
except ImportError:
    # Test ortamında mock GPIO kullan
    import sys
    from unittest.mock import MagicMock
    sys.modules['RPi.GPIO'] = MagicMock()
    import RPi.GPIO as GPIO

class GPIOController:
    """GPIO kontrol sınıfı"""
    
    def __init__(self, button_pin: int = 18, led_pin: int = 22, buzzer_pin: int = 23):
        """
        GPIO controller'ı başlat
        
        Args:
            button_pin: Buton GPIO pin numarası
            led_pin: LED GPIO pin numarası  
            buzzer_pin: Buzzer GPIO pin numarası
        """
        self.button_pin = button_pin
        self.led_pin = led_pin
        self.buzzer_pin = buzzer_pin
        
        self.setup_complete = False
        self.button_callback = None
        
        # LED durumu
        self.led_state = False
        self.led_blink_thread = None
        self.led_blink_stop = threading.Event()
        
        # Logging
        self.logger = logging.getLogger(__name__)
        
    def setup_gpio(self):
        """GPIO pinlerini ayarla"""
        try:
            # GPIO mod ayarla
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Pin ayarları
            GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Buton (pull-up)
            GPIO.setup(self.led_pin, GPIO.OUT)      # LED çıkışı
            GPIO.setup(self.buzzer_pin, GPIO.OUT)   # Buzzer çıkışı
            
            # Başlangıç durumları
            GPIO.output(self.led_pin, GPIO.LOW)     # LED kapalı
            GPIO.output(self.buzzer_pin, GPIO.LOW)  # Buzzer kapalı
            
            self.setup_complete = True
            self.logger.info(f"GPIO ayarlandı - Buton:{self.button_pin}, LED:{self.led_pin}, Buzzer:{self.buzzer_pin}")
            
        except Exception as e:
            self.logger.error(f"GPIO ayar hatası: {e}")
            self.setup_complete = False
    
    def cleanup_gpio(self):
        """GPIO temizliği yap"""
        if self.setup_complete:
            self.stop_led_blink()
            self.buzzer_off()
            self.led_off()
            GPIO.cleanup()
            self.logger.info("GPIO temizlendi")
    
    def set_button_callback(self, callback_function):
        """
        Buton basma callback fonksiyonu ayarla
        
        Args:
            callback_function: Buton basıldığında çalışacak fonksiyon
        """
        if not self.setup_complete:
            self.logger.warning("GPIO ayarlanmamış, callback ayarlanamıyor")
            return
            
        self.button_callback = callback_function
        
        # GPIO interrupt ayarla (düşen kenar - buton basıldığında)
        GPIO.add_event_detect(
            self.button_pin, 
            GPIO.FALLING, 
            callback=self._button_pressed_handler,
            bouncetime=300  # 300ms debounce
        )
        
        self.logger.info("Buton callback ayarlandı")
    
    def _button_pressed_handler(self, channel):
        """Buton basma interrupt handler'ı"""
        self.logger.info("Buton basıldı!")
        if self.button_callback:
            try:
                self.button_callback()
            except Exception as e:
                self.logger.error(f"Buton callback hatası: {e}")
    
    def is_button_pressed(self) -> bool:
        """Buton durumunu kontrol et"""
        if not self.setup_complete:
            return False
        return GPIO.input(self.button_pin) == GPIO.LOW  # Pull-up kullanıyoruz
    
    def led_on(self):
        """LED'i aç"""
        if self.setup_complete:
            GPIO.output(self.led_pin, GPIO.HIGH)
            self.led_state = True
    
    def led_off(self):
        """LED'i kapat"""
        if self.setup_complete:
            GPIO.output(self.led_pin, GPIO.LOW)
            self.led_state = False
    
    def led_toggle(self):
        """LED durumunu değiştir"""
        if self.led_state:
            self.led_off()
        else:
            self.led_on()
    
    def led_blink(self, on_time: float = 0.5, off_time: float = 0.5, count: int = 0):
        """
        LED'i yanıp söndür
        
        Args:
            on_time: Açık kalma süresi (saniye)
            off_time: Kapalı kalma süresi (saniye) 
            count: Yanıp sönme sayısı (0 = sonsuz)
        """
        self.stop_led_blink()  # Önceki blink'i durdur
        
        self.led_blink_stop.clear()
        self.led_blink_thread = threading.Thread(
            target=self._led_blink_loop,
            args=(on_time, off_time, count)
        )
        self.led_blink_thread.daemon = True
        self.led_blink_thread.start()
    
    def _led_blink_loop(self, on_time: float, off_time: float, count: int):
        """LED blink döngüsü"""
        blink_count = 0
        
        while not self.led_blink_stop.is_set():
            # LED aç
            self.led_on()
            if self.led_blink_stop.wait(on_time):
                break
                
            # LED kapat
            self.led_off()
            if self.led_blink_stop.wait(off_time):
                break
            
            blink_count += 1
            if count > 0 and blink_count >= count:
                break
        
        self.led_off()  # Son durumda LED kapalı olsun
    
    def stop_led_blink(self):
        """LED blink'ini durdur"""
        if self.led_blink_thread and self.led_blink_thread.is_alive():
            self.led_blink_stop.set()
            self.led_blink_thread.join(timeout=1)
    
    def buzzer_on(self):
        """Buzzer'ı aç"""
        if self.setup_complete:
            GPIO.output(self.buzzer_pin, GPIO.HIGH)
    
    def buzzer_off(self):
        """Buzzer'ı kapat"""  
        if self.setup_complete:
            GPIO.output(self.buzzer_pin, GPIO.LOW)
    
    def buzzer_beep(self, duration: float = 0.5):
        """
        Buzzer bip sesi
        
        Args:
            duration: Bip süresi (saniye)
        """
        self.buzzer_on()
        time.sleep(duration)
        self.buzzer_off()
    
    def buzzer_beep_pattern(self, pattern: list):
        """
        Buzzer bip pattern'i çal
        
        Args:
            pattern: [(on_time, off_time), ...] listesi
        """
        for on_time, off_time in pattern:
            self.buzzer_on()
            time.sleep(on_time)
            self.buzzer_off()
            if off_time > 0:
                time.sleep(off_time)
    
    def startup_sequence(self):
        """Başlangıç ses ve ışık gösterisi"""
        self.logger.info("Başlangıç sequence başlatılıyor...")
        
        # 3 kısa bip + LED blink
        for i in range(3):
            self.led_on()
            self.buzzer_on()
            time.sleep(0.2)
            
            self.led_off()
            self.buzzer_off()
            time.sleep(0.2)
        
        # Uzun bip + LED açık kal
        self.led_on()
        self.buzzer_on()
        time.sleep(1.0)
        self.buzzer_off()
        
        self.logger.info("Başlangıç sequence tamamlandı")
    
    def error_sequence(self):
        """Hata durumu ses ve ışık gösterisi"""
        self.logger.warning("Hata sequence başlatılıyor...")
        
        # Hızlı yanıp sönme + alarm sesi
        for i in range(5):
            self.led_on()
            self.buzzer_on()
            time.sleep(0.1)
            
            self.led_off()
            self.buzzer_off()
            time.sleep(0.1)
        
        self.logger.info("Hata sequence tamamlandı")
    
    def success_sequence(self):
        """Başarı durumu ses ve ışık gösterisi"""
        self.logger.info("Başarı sequence başlatılıyor...")
        
        # Yükselen ton taklidi + yavaş blink
        beep_pattern = [
            (0.1, 0.05),  # Kısa
            (0.15, 0.05),  # Biraz uzun
            (0.2, 0.05),   # Daha uzun
            (0.5, 0)       # En uzun
        ]
        
        for on_time, off_time in beep_pattern:
            self.led_on()
            self.buzzer_on()
            time.sleep(on_time)
            self.buzzer_off()
            time.sleep(off_time)
            self.led_off()
            if off_time > 0:
                time.sleep(0.1)
        
        # LED açık kalsın (başarı durumu)
        self.led_on()
        self.logger.info("Başarı sequence tamamlandı")
    
    def get_status(self) -> dict:
        """GPIO durumunu al"""
        return {
            "setup_complete": self.setup_complete,
            "button_pressed": self.is_button_pressed(),
            "led_state": self.led_state,
            "pins": {
                "button": self.button_pin,
                "led": self.led_pin,
                "buzzer": self.buzzer_pin
            }
        }

# Test fonksiyonu
if __name__ == "__main__":
    import signal
    import sys
    
    def signal_handler(sig, frame):
        print("\nÇıkılıyor...")
        gpio.cleanup_gpio()
        sys.exit(0)
    
    # Signal handler ayarla
    signal.signal(signal.SIGINT, signal_handler)
    
    # GPIO controller oluştur ve test et
    gpio = GPIOController()
    
    try:
        # GPIO ayarla
        gpio.setup_gpio()
        
        if not gpio.setup_complete:
            print("GPIO ayarlanamadı!")
            sys.exit(1)
        
        # Başlangıç sequence
        gpio.startup_sequence()
        
        def button_pressed():
            print("Buton basıldı! LED durumu değişiyor...")
            gpio.led_toggle()
            gpio.buzzer_beep(0.2)
        
        # Buton callback ayarla
        gpio.set_button_callback(button_pressed)
        
        print("Test başladı. Butona basın. Ctrl+C ile çıkın.")
        print("10 saniye sonra blink testi yapılacak...")
        
        # 10 saniye bekle
        time.sleep(10)
        
        # Blink testi
        print("LED blink testi...")
        gpio.led_blink(0.3, 0.3, 5)  # 5 kez yanıp sönsün
        time.sleep(4)
        
        # Buzzer pattern testi
        print("Buzzer pattern testi...")
        pattern = [(0.1, 0.1), (0.1, 0.1), (0.3, 0.2), (0.5, 0)]
        gpio.buzzer_beep_pattern(pattern)
        
        # Başarı sequence
        gpio.success_sequence()
        
        print("Test tamamlandı. Ctrl+C ile çıkın.")
        
        # Sonsuz döngü
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        signal_handler(None, None)
    finally:
        gpio.cleanup_gpio()
