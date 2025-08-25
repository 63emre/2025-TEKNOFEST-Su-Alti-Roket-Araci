#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPIO_COMPAT - Raspberry Pi 5 Uyumlu GPIO Katmanı
RPi.GPIO yerine rpi-lgpio kullanarak Pi 5 uyumluluğu sağlar
"""

import sys
import time
import logging

# Raspberry Pi 5 için rpi-lgpio kullan
try:
    import lgpio as gpio_lib
    GPIO_LIB = "lgpio"
    print("✓ rpi-lgpio kütüphanesi yüklendi (Pi 5 uyumlu)")
except ImportError:
    # Fallback: Eski Pi modelleri için RPi.GPIO dene
    try:
        import RPi.GPIO as gpio_lib
        GPIO_LIB = "RPi.GPIO"
        print("✓ RPi.GPIO kütüphanesi yüklendi (eski Pi modelleri)")
    except ImportError:
        print("❌ Ne lgpio ne de RPi.GPIO bulunamadı!")
        sys.exit(1)

class GPIOCompat:
    """GPIO uyumluluk katmanı - RPi.GPIO API'sini taklit eder"""
    
    # Sabitler
    BCM = "BCM"
    OUT = "OUT" 
    IN = "IN"
    HIGH = 1
    LOW = 0
    PUD_UP = "PUD_UP"
    PUD_DOWN = "PUD_DOWN"
    
    def __init__(self):
        self.chip = None
        self.mode_set = False
        self.setup_pins = set()
        
        if GPIO_LIB == "lgpio":
            try:
                # GPIO chip'i aç
                self.chip = gpio_lib.gpiochip_open(0)  # /dev/gpiochip0
                print(f"✓ GPIO chip açıldı: {self.chip}")
                
                # Sistem başlangıcında kritik pinleri temizle
                self._force_cleanup_critical_pins()
                
            except Exception as e:
                print(f"❌ GPIO chip açma hatası: {e}")
                raise
    
    def _force_cleanup_critical_pins(self):
        """Kritik pinleri zorla temizle"""
        critical_pins = [21, 9, 11, 10]  # LED, BUZZER, BUTTON, SOLENOID
        
        for pin in critical_pins:
            try:
                gpio_lib.gpio_free(self.chip, pin)
            except:
                pass  # Pin zaten serbest veya kullanımda değil
    
    def setmode(self, mode):
        """GPIO modunu ayarla"""
        if mode == self.BCM:
            self.mode_set = True
            return
        else:
            raise ValueError("Sadece BCM modu destekleniyor")
    
    def setup(self, pin, direction, pull_up_down=None):
        """Pin kurulumu"""
        if not self.mode_set:
            raise RuntimeError("setmode() önce çağrılmalı")
            
        if GPIO_LIB == "lgpio":
            try:
                # Pin zaten kurulmuşsa önce serbest bırak
                if pin in self.setup_pins:
                    try:
                        gpio_lib.gpio_free(self.chip, pin)
                        self.setup_pins.discard(pin)
                    except:
                        pass
                
                # Pin'i yeniden kur
                if direction == self.OUT:
                    gpio_lib.gpio_claim_output(self.chip, pin, self.LOW)
                elif direction == self.IN:
                    flags = 0
                    if pull_up_down == self.PUD_UP:
                        flags = gpio_lib.SET_PULL_UP
                    elif pull_up_down == self.PUD_DOWN:
                        flags = gpio_lib.SET_PULL_DOWN
                    
                    gpio_lib.gpio_claim_input(self.chip, pin, flags)
                    
                self.setup_pins.add(pin)
                
            except Exception as e:
                print(f"❌ Pin {pin} kurulum hatası: {e}")
                raise
                
        else:  # RPi.GPIO
            gpio_lib.setup(pin, getattr(gpio_lib, direction), 
                          pull_up_down=getattr(gpio_lib, pull_up_down) if pull_up_down else None)
            self.setup_pins.add(pin)
    
    def output(self, pin, state):
        """Pin çıkışı ayarla"""
        if pin not in self.setup_pins:
            raise RuntimeError(f"Pin {pin} önce setup() ile ayarlanmalı")
            
        if GPIO_LIB == "lgpio":
            try:
                gpio_lib.gpio_write(self.chip, pin, state)
            except Exception as e:
                print(f"❌ Pin {pin} output hatası: {e}")
                raise
        else:  # RPi.GPIO
            gpio_lib.output(pin, state)
    
    def input(self, pin):
        """Pin girişini oku"""
        if pin not in self.setup_pins:
            raise RuntimeError(f"Pin {pin} önce setup() ile ayarlanmalı")
            
        if GPIO_LIB == "lgpio":
            try:
                return gpio_lib.gpio_read(self.chip, pin)
            except Exception as e:
                print(f"❌ Pin {pin} input hatası: {e}")
                raise
        else:  # RPi.GPIO
            return gpio_lib.input(pin)
    
    def cleanup(self):
        """GPIO temizliği"""
        if GPIO_LIB == "lgpio":
            try:
                # Kurulu pinleri serbest bırak
                for pin in self.setup_pins:
                    try:
                        gpio_lib.gpio_free(self.chip, pin)
                    except:
                        pass  # Hata olsa da devam et
                        
                # Chip'i kapat
                if self.chip is not None:
                    gpio_lib.gpiochip_close(self.chip)
                    self.chip = None
                    
            except Exception as e:
                print(f"⚠️ GPIO cleanup hatası: {e}")
                
        else:  # RPi.GPIO
            try:
                gpio_lib.cleanup()
            except Exception as e:
                print(f"⚠️ GPIO cleanup hatası: {e}")
                
        self.setup_pins.clear()
        self.mode_set = False

# Global GPIO instance
GPIO = GPIOCompat()

def safe_gpio_cleanup():
    """Güvenli GPIO temizliği"""
    try:
        GPIO.cleanup()
        print("✓ GPIO temizliği tamamlandı")
    except Exception as e:
        print(f"⚠️ GPIO temizlik hatası: {e}")

# Test fonksiyonu
def test_gpio_compat():
    """GPIO uyumluluk testi"""
    print("🧪 GPIO uyumluluk testi başlıyor...")
    
    try:
        # Test pin (LED için)
        test_pin = 21
        
        # Kurulum
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(test_pin, GPIO.OUT)
        
        # Test
        print(f"Pin {test_pin} HIGH...")
        GPIO.output(test_pin, GPIO.HIGH)
        time.sleep(0.5)
        
        print(f"Pin {test_pin} LOW...")
        GPIO.output(test_pin, GPIO.LOW)
        time.sleep(0.5)
        
        print("✅ GPIO testi başarılı")
        return True
        
    except Exception as e:
        print(f"❌ GPIO testi başarısız: {e}")
        return False
        
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    # Test çalıştır
    test_gpio_compat()
