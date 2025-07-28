#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - GPIO Kontrol Modülü
Raspberry Pi 5 Uyumlu Sürüm
"""

import time
import threading
from typing import Optional, Callable

# Pi5 uyumlu GPIO kütüphaneleri
try:
    import lgpio
    GPIO_LIBRARY = "lgpio"
    print("✅ lgpio kütüphanesi yüklendi (Pi5 uyumlu)")
except ImportError:
    try:
        import gpiozero
        from gpiozero import LED, Button, Buzzer, PWMOutputDevice
        GPIO_LIBRARY = "gpiozero"
        print("✅ gpiozero kütüphanesi yüklendi (Pi5 uyumlu)")
    except ImportError:
        try:
            import RPi.GPIO as GPIO
            GPIO_LIBRARY = "RPi.GPIO"
            print("⚠️ RPi.GPIO yüklendi (Pi5'te sorunlu olabilir)")
        except ImportError:
            GPIO_LIBRARY = "simulation"
            print("⚠️ GPIO kütüphanesi bulunamadı - simülasyon modunda")

class GPIOController:
    """Raspberry Pi GPIO kontrol sınıfı"""
    
    def __init__(self, config):
        """GPIO controller başlat"""
        self.config = config
        self.gpio_pins = config.get("raspberry_pi", {}).get("gpio", {})
        
        # GPIO durumu
        self.initialized = False
        self.pwm_objects = {}
        
        # Pin tanımları
        self.buzzer_pin = self.gpio_pins.get("buzzer", 7)
        self.button_pin = self.gpio_pins.get("control_button", 13)
        self.led_red = self.gpio_pins.get("led_red", 4)
        self.led_green = self.gpio_pins.get("led_green", 5)
        self.led_blue = self.gpio_pins.get("led_blue", 6)
        self.warning_led = self.gpio_pins.get("warning_led", 8)
        self.status_led = self.gpio_pins.get("system_status_led", 10)
        
        # Button callback
        self.button_callback = None
        
        # Thread safety
        self.gpio_lock = threading.Lock()
        
    def initialize(self):
        """GPIO sistemini başlat"""
        if GPIO_LIBRARY == "simulation":
            print("⚠️ GPIO simülasyon modunda çalışıyor")
            self.initialized = True
            return True
        
        try:
            with self.gpio_lock:
                if GPIO_LIBRARY == "lgpio":
                    # lgpio için pin modu ayarla
                    lgpio.gpio_open()
                    lgpio.gpio_set_mode(self.buzzer_pin, lgpio.OUTPUT)
                    lgpio.gpio_set_mode(self.led_red, lgpio.OUTPUT)
                    lgpio.gpio_set_mode(self.led_green, lgpio.OUTPUT)
                    lgpio.gpio_set_mode(self.led_blue, lgpio.OUTPUT)
                    lgpio.gpio_set_mode(self.warning_led, lgpio.OUTPUT)
                    lgpio.gpio_set_mode(self.status_led, lgpio.OUTPUT)
                    lgpio.gpio_write(self.buzzer_pin, lgpio.LOW)
                    lgpio.gpio_write(self.led_red, lgpio.LOW)
                    lgpio.gpio_write(self.led_green, lgpio.LOW)
                    lgpio.gpio_write(self.led_blue, lgpio.LOW)
                    lgpio.gpio_write(self.warning_led, lgpio.LOW)
                    lgpio.gpio_write(self.status_led, lgpio.LOW)
                    
                    # PWM objelerini oluştur
                    self.pwm_objects['buzzer'] = PWMOutputDevice(self.buzzer_pin, frequency=1000)
                    self.pwm_objects['led_red'] = PWMOutputDevice(self.led_red, frequency=100)
                    self.pwm_objects['led_green'] = PWMOutputDevice(self.led_green, frequency=100)
                    self.pwm_objects['led_blue'] = PWMOutputDevice(self.led_blue, frequency=100)
                    
                    # PWM'leri başlat (0% duty cycle)
                    for pwm in self.pwm_objects.values():
                        pwm.value = 0
                
                elif GPIO_LIBRARY == "gpiozero":
                    # gpiozero için pin modu ayarla
                    self.buzzer_pin = Button(self.buzzer_pin, pull_up=True)
                    self.button_pin = Button(self.button_pin, pull_up=True)
                    self.led_red = LED(self.led_red)
                    self.led_green = LED(self.led_green)
                    self.led_blue = LED(self.led_blue)
                    self.warning_led = LED(self.warning_led)
                    self.status_led = LED(self.status_led)
                    
                    # PWM objelerini oluştur
                    self.pwm_objects['buzzer'] = PWMOutputDevice(self.buzzer_pin, frequency=1000)
                    self.pwm_objects['led_red'] = PWMOutputDevice(self.led_red, frequency=100)
                    self.pwm_objects['led_green'] = PWMOutputDevice(self.led_green, frequency=100)
                    self.pwm_objects['led_blue'] = PWMOutputDevice(self.led_blue, frequency=100)
                    
                    # PWM'leri başlat (0% duty cycle)
                    for pwm in self.pwm_objects.values():
                        pwm.value = 0
                
                elif GPIO_LIBRARY == "RPi.GPIO":
                    # RPi.GPIO için pin modu ayarla
                    import RPi.GPIO as GPIO
                    GPIO.setmode(GPIO.BCM)
                    GPIO.setwarnings(False)
                    
                    # Output pinleri ayarla
                    output_pins = [
                        self.buzzer_pin, self.led_red, self.led_green, 
                        self.led_blue, self.warning_led, self.status_led
                    ]
                    
                    for pin in output_pins:
                        GPIO.setup(pin, GPIO.OUT)
                        GPIO.output(pin, GPIO.LOW)
                    
                    # Button input ayarla
                    GPIO.setup(self.button_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                    
                    # PWM objelerini oluştur
                    self.pwm_objects['buzzer'] = GPIO.PWM(self.buzzer_pin, 1000)  # 1kHz
                    self.pwm_objects['led_red'] = GPIO.PWM(self.led_red, 100)     # 100Hz
                    self.pwm_objects['led_green'] = GPIO.PWM(self.led_green, 100)
                    self.pwm_objects['led_blue'] = GPIO.PWM(self.led_blue, 100)
                    
                    # PWM'leri başlat (0% duty cycle)
                    for pwm in self.pwm_objects.values():
                        pwm.start(0)
                
                self.initialized = True
                print("✅ GPIO sistemi başlatıldı")
                return True
                
        except Exception as e:
            print(f"❌ GPIO başlatma hatası: {e}")
            return False
    
    def cleanup(self):
        """GPIO temizliği"""
        if GPIO_LIBRARY == "simulation":
            return
        
        try:
            with self.gpio_lock:
                if GPIO_LIBRARY == "lgpio":
                    lgpio.gpio_close(self.buzzer_pin)
                    lgpio.gpio_close(self.led_red)
                    lgpio.gpio_close(self.led_green)
                    lgpio.gpio_close(self.led_blue)
                    lgpio.gpio_close(self.warning_led)
                    lgpio.gpio_close(self.status_led)
                    lgpio.gpio_close() # Ana GPIO kapat
                
                elif GPIO_LIBRARY == "gpiozero":
                    self.buzzer_pin.close()
                    self.button_pin.close()
                    self.led_red.close()
                    self.led_green.close()
                    self.led_blue.close()
                    self.warning_led.close()
                    self.status_led.close()
                
                elif GPIO_LIBRARY == "RPi.GPIO":
                    import RPi.GPIO as GPIO
                    GPIO.cleanup()
                
                self.initialized = False
                print("🔄 GPIO temizlendi")
                
        except Exception as e:
            print(f"⚠️ GPIO temizleme uyarısı: {e}")
    
    def set_led(self, led_name, state, brightness=100):
        """LED kontrol (on/off veya PWM)"""
        if not self.initialized:
            return False
        
        led_map = {
            'red': self.led_red,
            'green': self.led_green, 
            'blue': self.led_blue,
            'warning': self.warning_led,
            'status': self.status_led
        }
        
        if led_name not in led_map:
            return False
        
        try:
            pin = led_map[led_name]
            
            if GPIO_LIBRARY == "simulation":
                print(f"🔨 [SIM] LED {led_name}: {'ON' if state else 'OFF'} ({brightness}%)")
                return True
            
            with self.gpio_lock:
                if led_name in ['red', 'green', 'blue']:
                    # PWM kontrollü LED'ler
                    if state:
                        self.pwm_objects[f'led_{led_name}'].value = brightness / 100.0
                    else:
                        self.pwm_objects[f'led_{led_name}'].value = 0
                else:
                    # Normal dijital LED'ler
                    pin.value = GPIO.HIGH if state else GPIO.LOW
            
            return True
            
        except Exception as e:
            print(f"❌ LED kontrol hatası ({led_name}): {e}")
            return False
    
    def set_rgb_led(self, red, green, blue):
        """RGB LED renk ayarla (0-100%)"""
        self.set_led('red', red > 0, red)
        self.set_led('green', green > 0, green)
        self.set_led('blue', blue > 0, blue)
    
    def buzzer_beep(self, frequency=1000, duration=0.2, volume=50):
        """Buzzer ses çıkar"""
        if not self.initialized:
            return False
        
        try:
            if GPIO_LIBRARY == "simulation":
                print(f"🔊 [SIM] Buzzer: {frequency}Hz, {duration}s, {volume}%")
                return True
            
            with self.gpio_lock:
                # Frekansı ayarla
                self.pwm_objects['buzzer'].frequency = frequency
                # Ses aç
                self.pwm_objects['buzzer'].value = volume / 100.0
                
            # Süre kadar bekle
            time.sleep(duration)
            
            with self.gpio_lock:
                # Sesi kapat
                self.pwm_objects['buzzer'].value = 0
            
            return True
            
        except Exception as e:
            print(f"❌ Buzzer hatası: {e}")
            return False
    
    def buzzer_start(self, frequency=1000, volume=50):
        """Buzzer sürekli çal"""
        if not self.initialized:
            return False
        
        try:
            if GPIO_LIBRARY == "simulation":
                print(f"🔊 [SIM] Buzzer START: {frequency}Hz, {volume}%")
                return True
            
            with self.gpio_lock:
                self.pwm_objects['buzzer'].frequency = frequency
                self.pwm_objects['buzzer'].value = volume / 100.0
            
            return True
            
        except Exception as e:
            print(f"❌ Buzzer başlatma hatası: {e}")
            return False
    
    def buzzer_stop(self):
        """Buzzer durdur"""
        if not self.initialized:
            return False
        
        try:
            if GPIO_LIBRARY == "simulation":
                print("🔇 [SIM] Buzzer STOP")
                return True
            
            with self.gpio_lock:
                self.pwm_objects['buzzer'].value = 0
            
            return True
            
        except Exception as e:
            print(f"❌ Buzzer durdurma hatası: {e}")
            return False
    
    def read_button(self):
        """Buton durumunu oku"""
        if not self.initialized:
            return False
        
        try:
            if GPIO_LIBRARY == "simulation":
                return False  # Simülasyonda buton yok
            
            with self.gpio_lock:
                # Pull-up olduğu için LOW = basılı
                return not self.button_pin.is_pressed
            
        except Exception as e:
            print(f"❌ Buton okuma hatası: {e}")
            return False
    
    def setup_button_callback(self, callback_function):
        """Buton basma callback ayarla"""
        if not self.initialized:
            return False
        
        try:
            self.button_callback = callback_function
            
            # Edge detection ayarla
            if GPIO_LIBRARY == "gpiozero":
                self.button_pin.when_pressed = self._button_interrupt
                self.button_pin.when_released = self._button_interrupt # gpiozero'da released değil, pressed olur
            elif GPIO_LIBRARY == "lgpio":
                lgpio.gpio_event_detect(self.button_pin, lgpio.FALLING, self._button_interrupt)
                lgpio.gpio_event_detect(self.button_pin, lgpio.RISING, self._button_interrupt) # gpiozero'da released değil, pressed olur
            elif GPIO_LIBRARY == "RPi.GPIO":
                GPIO.add_event_detect(
                    self.button_pin, 
                    GPIO.FALLING,  # Pull-up'ta basılma = FALLING
                    callback=self._button_interrupt,
                    bouncetime=200  # 200ms debounce
                )
            
            print("✅ Buton callback ayarlandı")
            return True
            
        except Exception as e:
            print(f"❌ Buton callback hatası: {e}")
            return False
    
    def _button_interrupt(self, channel):
        """Buton interrupt handler"""
        if self.button_callback:
            try:
                self.button_callback()
            except Exception as e:
                print(f"❌ Buton callback error: {e}")
    
    def emergency_led_pattern(self):
        """Acil durum LED deseni"""
        def emergency_worker():
            for i in range(10):
                self.set_led('warning', True)
                self.set_rgb_led(100, 0, 0)  # Kırmızı
                time.sleep(0.1)
                
                self.set_led('warning', False)
                self.set_rgb_led(0, 0, 0)
                time.sleep(0.1)
        
        emergency_thread = threading.Thread(target=emergency_worker, daemon=True)
        emergency_thread.start()
    
    def status_led_pattern(self, pattern):
        """Durum LED deseni"""
        patterns = {
            'connecting': [(True, 0.5), (False, 0.5)],  # Yanıp söner
            'connected': [(True, 2.0), (False, 0.1)],   # Uzun yanık
            'error': [(True, 0.1), (False, 0.1)] * 5,   # Hızlı yanıp sönme
            'armed': [(True, 1.0)]                       # Sürekli yanık
        }
        
        if pattern not in patterns:
            return
        
        def pattern_worker():
            sequence = patterns[pattern]
            for state, duration in sequence:
                self.set_led('status', state)
                time.sleep(duration)
        
        pattern_thread = threading.Thread(target=pattern_worker, daemon=True)
        pattern_thread.start()
    
    def get_gpio_status(self):
        """GPIO durumunu döndür"""
        return {
            'initialized': self.initialized,
            'gpio_library': GPIO_LIBRARY,
            'pins': {
                'buzzer': self.buzzer_pin,
                'button': self.button_pin,
                'led_red': self.led_red,
                'led_green': self.led_green,
                'led_blue': self.led_blue,
                'warning_led': self.warning_led,
                'status_led': self.status_led
            }
        }

if __name__ == "__main__":
    # Test kodu
    print("🔬 GPIO Controller Test")
    
    # Dummy config
    config = {
        "raspberry_pi": {
            "gpio": {
                "buzzer": 7,
                "control_button": 13,
                "led_red": 4,
                "led_green": 5,
                "led_blue": 6,
                "warning_led": 8,
                "system_status_led": 10
            }
        }
    }
    
    gpio = GPIOController(config)
    
    if gpio.initialize():
        print("GPIO test başlıyor...")
        
        # LED test
        for color in ['red', 'green', 'blue']:
            print(f"Testing {color} LED...")
            gpio.set_led(color, True, 50)
            time.sleep(0.5)
            gpio.set_led(color, False)
        
        # RGB test
        print("RGB color test...")
        gpio.set_rgb_led(100, 0, 0)    # Kırmızı
        time.sleep(0.5)
        gpio.set_rgb_led(0, 100, 0)    # Yeşil
        time.sleep(0.5)
        gpio.set_rgb_led(0, 0, 100)    # Mavi
        time.sleep(0.5)
        gpio.set_rgb_led(0, 0, 0)      # Kapat
        
        # Buzzer test
        print("Buzzer test...")
        gpio.buzzer_beep(1000, 0.2, 30)
        time.sleep(0.3)
        gpio.buzzer_beep(2000, 0.2, 30)
        
        # Status pattern test
        print("Status LED pattern test...")
        gpio.status_led_pattern('connecting')
        time.sleep(3)
        
        gpio.cleanup()
    else:
        print("❌ GPIO başlatma başarısız!") 