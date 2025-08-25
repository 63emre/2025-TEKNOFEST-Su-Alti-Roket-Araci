#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPIO WRAPPER - Raspberry Pi 5 Uyumlu GPIO Kontrolü
RPi.GPIO yerine rpi-lgpio kullanarak Pi 5 uyumluluğu sağlar
"""

import time
import threading

# Pi 5 için rpi-lgpio kullan
try:
    import lgpio
    PI5_COMPATIBLE = True
    print("✅ rpi-lgpio kütüphanesi yüklü - Pi 5 uyumlu mod aktif")
except ImportError:
    PI5_COMPATIBLE = False
    # Fallback için RPi.GPIO dene (Pi 4 ve öncesi)
    try:
        import RPi.GPIO as GPIO
        print("⚠️ rpi-lgpio bulunamadı, RPi.GPIO kullanılıyor (Pi 4 ve öncesi)")
    except ImportError:
        print("❌ Ne rpi-lgpio ne de RPi.GPIO bulunamadı!")
        raise ImportError("GPIO kütüphanesi bulunamadı!")

class GPIOWrapper:
    """Raspberry Pi 5 uyumlu GPIO wrapper sınıfı"""
    
    # GPIO konstları (RPi.GPIO uyumluluğu için)
    BCM = "BCM"
    HIGH = True
    LOW = False
    OUT = "OUT"
    IN = "IN"
    PUD_UP = "PUD_UP"
    PUD_DOWN = "PUD_DOWN"
    
    def __init__(self):
        self.chip_handle = None
        self.mode_set = False
        self.setup_pins = {}  # Pin -> (mode, pull) mapping
        
        if PI5_COMPATIBLE:
            # lgpio chip handle'ı aç
            self.chip_handle = lgpio.gpiochip_open(0)
            print(f"✅ lgpio chip handle açıldı: {self.chip_handle}")
        else:
            print("⚠️ Fallback RPi.GPIO modu")
            
    def setmode(self, mode):
        """GPIO mod ayarla (BCM/BOARD)"""
        if PI5_COMPATIBLE:
            # lgpio'da mod ayarı gerekmez, her zaman BCM
            self.mode_set = True
        else:
            GPIO.setmode(GPIO.BCM if mode == "BCM" else GPIO.BOARD)
            self.mode_set = True
            
    def setup(self, pin, mode, pull_up_down=None):
        """GPIO pin ayarla"""
        if not self.mode_set:
            raise RuntimeError("GPIO modu ayarlanmamış! setmode() çağırın.")
            
        if PI5_COMPATIBLE:
            if mode == "OUT":
                lgpio.gpio_claim_output(self.chip_handle, pin)
            elif mode == "IN":
                if pull_up_down == "PUD_UP":
                    lgpio.gpio_claim_input(self.chip_handle, pin, lgpio.SET_PULL_UP)
                elif pull_up_down == "PUD_DOWN":
                    lgpio.gpio_claim_input(self.chip_handle, pin, lgpio.SET_PULL_DOWN)
                else:
                    lgpio.gpio_claim_input(self.chip_handle, pin)
                    
            self.setup_pins[pin] = (mode, pull_up_down)
        else:
            # RPi.GPIO fallback
            gpio_mode = GPIO.OUT if mode == "OUT" else GPIO.IN
            gpio_pull = None
            
            if pull_up_down == "PUD_UP":
                gpio_pull = GPIO.PUD_UP
            elif pull_up_down == "PUD_DOWN":
                gpio_pull = GPIO.PUD_DOWN
                
            if gpio_pull:
                GPIO.setup(pin, gpio_mode, pull_up_down=gpio_pull)
            else:
                GPIO.setup(pin, gpio_mode)
                
    def output(self, pin, value):
        """GPIO çıkış değeri ayarla"""
        if PI5_COMPATIBLE:
            lgpio_value = 1 if value else 0
            lgpio.gpio_write(self.chip_handle, pin, lgpio_value)
        else:
            GPIO.output(pin, GPIO.HIGH if value else GPIO.LOW)
            
    def input(self, pin):
        """GPIO giriş değeri oku"""
        if PI5_COMPATIBLE:
            return bool(lgpio.gpio_read(self.chip_handle, pin))
        else:
            return GPIO.input(pin)
            
    def cleanup(self):
        """GPIO temizleme"""
        if PI5_COMPATIBLE:
            if self.chip_handle is not None:
                # Tüm setup edilmiş pinleri serbest bırak
                for pin in self.setup_pins.keys():
                    try:
                        lgpio.gpio_free(self.chip_handle, pin)
                    except:
                        pass
                        
                # Chip handle'ı kapat
                try:
                    lgpio.gpiochip_close(self.chip_handle)
                except:
                    pass
                    
                self.chip_handle = None
                self.setup_pins.clear()
        else:
            GPIO.cleanup()

# Global GPIO instance (singleton pattern)
_gpio_instance = None

def get_gpio():
    """GPIO instance döndür (singleton)"""
    global _gpio_instance
    if _gpio_instance is None:
        _gpio_instance = GPIOWrapper()
    return _gpio_instance

# GPIO sınıfı (RPi.GPIO uyumluluğu için)
class GPIO:
    """RPi.GPIO uyumlu GPIO sınıfı"""
    
    # Konstlar
    BCM = GPIOWrapper.BCM
    HIGH = GPIOWrapper.HIGH
    LOW = GPIOWrapper.LOW
    OUT = GPIOWrapper.OUT
    IN = GPIOWrapper.IN
    PUD_UP = GPIOWrapper.PUD_UP
    PUD_DOWN = GPIOWrapper.PUD_DOWN
    
    @staticmethod
    def setmode(mode):
        """GPIO mod ayarla"""
        gpio = get_gpio()
        gpio.setmode(mode)
    
    @staticmethod
    def setup(pin, mode, pull_up_down=None):
        """GPIO pin ayarla"""
        gpio = get_gpio()
        gpio.setup(pin, mode, pull_up_down)
    
    @staticmethod
    def output(pin, value):
        """GPIO çıkış değeri ayarla"""
        gpio = get_gpio()
        gpio.output(pin, value)
    
    @staticmethod
    def input(pin):
        """GPIO giriş değeri oku"""
        gpio = get_gpio()
        return gpio.input(pin)
    
    @staticmethod
    def cleanup():
        """GPIO temizleme"""
        gpio = get_gpio()
        gpio.cleanup()

# RPi.GPIO uyumluluğu için fonksiyon wrapper'ları
def setmode(mode):
    """GPIO mod ayarla"""
    GPIO.setmode(mode)

def setup(pin, mode, pull_up_down=None):
    """GPIO pin ayarla"""
    GPIO.setup(pin, mode, pull_up_down)

def output(pin, value):
    """GPIO çıkış değeri ayarla"""
    GPIO.output(pin, value)

def input(pin):
    """GPIO giriş değeri oku"""
    return GPIO.input(pin)

def cleanup():
    """GPIO temizleme"""
    GPIO.cleanup()

# RPi.GPIO konstları (uyumluluk için)
BCM = GPIOWrapper.BCM
HIGH = GPIOWrapper.HIGH
LOW = GPIOWrapper.LOW
OUT = GPIOWrapper.OUT
IN = GPIOWrapper.IN
PUD_UP = GPIOWrapper.PUD_UP
PUD_DOWN = GPIOWrapper.PUD_DOWN

# Bilgi fonksiyonu
def get_gpio_info():
    """GPIO sistem bilgilerini döndür"""
    return {
        'pi5_compatible': PI5_COMPATIBLE,
        'library': 'rpi-lgpio' if PI5_COMPATIBLE else 'RPi.GPIO',
        'chip_handle': _gpio_instance.chip_handle if _gpio_instance else None,
        'setup_pins': _gpio_instance.setup_pins if _gpio_instance else {}
    }

def print_gpio_info():
    """GPIO sistem bilgilerini yazdır"""
    info = get_gpio_info()
    print("\n" + "="*50)
    print("GPIO SİSTEM BİLGİLERİ")
    print("="*50)
    print(f"Pi 5 Uyumlu: {'✅ Evet' if info['pi5_compatible'] else '❌ Hayır'}")
    print(f"Kullanılan Kütüphane: {info['library']}")
    if info['pi5_compatible'] and info['chip_handle']:
        print(f"Chip Handle: {info['chip_handle']}")
    if info['setup_pins']:
        print("Ayarlı Pinler:")
        for pin, (mode, pull) in info['setup_pins'].items():
            pull_str = f" ({pull})" if pull else ""
            print(f"  GPIO {pin}: {mode}{pull_str}")
    print("="*50 + "\n")

if __name__ == "__main__":
    # Test kodu
    print("GPIO Wrapper Test")
    print_gpio_info()
    
    try:
        # Basit LED test (GPIO 18)
        test_pin = 18
        print(f"GPIO {test_pin} test ediliyor...")
        
        setmode(BCM)
        setup(test_pin, OUT)
        
        print("LED test: 3 kez yanıp söndürülüyor...")
        for i in range(3):
            output(test_pin, HIGH)
            time.sleep(0.5)
            output(test_pin, LOW)
            time.sleep(0.5)
            
        cleanup()
        print("✅ GPIO test başarılı!")
        
    except Exception as e:
        print(f"❌ GPIO test hatası: {e}")
        cleanup()
