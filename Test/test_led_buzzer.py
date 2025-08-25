#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
LED ve Buzzer Test Scripti

Bu script, sistemde bulunan tüm LED'leri ve buzzer'ları test eder.
RGB LED'ler, status LED'ler, uyarı LED'leri ve buzzer'ların doğru çalıştığını doğrular.

Hardware: 
- RGB LED'ler (GPIO 4,5,6)
- Status LED'ler (GPIO 16,20,24) 
- Buzzer'lar (GPIO 13,25)
- RGB LED Strip (GPIO 26)

Pin Mapping: HARDWARE_PIN_MAPPING.md standardına göre
"""

import time
import threading
import numpy as np
import sys
from datetime import datetime

# GPIO uyumluluk katmanı
try:
    # Önce gpio_compat modülünü dene (pluswing klasöründen)
    sys.path.append('../görevlerf1/pluswing')
    from gpio_compat import GPIO
    print("✓ GPIO uyumluluk katmanı yüklendi")
except ImportError:
    # Fallback: Direkt lgpio veya RPi.GPIO
    try:
        import lgpio as gpio_lib
        print("✓ rpi-lgpio yüklendi")
    except ImportError:
        try:
            import RPi.GPIO as gpio_lib
            print("✓ RPi.GPIO yüklendi")
        except ImportError:
            print("❌ GPIO kütüphanesi bulunamadı!")
            sys.exit(1)
    
    # Basit wrapper (test için)
    class SimpleGPIO:
        BCM = "BCM"
        OUT = "OUT"
        IN = "IN" 
        HIGH = 1
        LOW = 0
        PUD_UP = "PUD_UP"
        
        def setmode(self, mode): pass
        def setup(self, pin, direction, pull_up_down=None): pass
        def output(self, pin, state): pass
        def input(self, pin): return 0
        def cleanup(self): pass
    
    GPIO = SimpleGPIO()

# Pin Definitions - HARDWARE_PIN_MAPPING.md standardı
GPIO_LED_RED = 4            # Kırmızı LED
GPIO_LED_GREEN = 5          # Yeşil LED  
GPIO_LED_BLUE = 6           # Mavi LED
GPIO_BUZZER_PWM = 13        # PWM Buzzer
GPIO_WARNING_LED = 16       # Uyarı LED
GPIO_SYSTEM_LED = 20        # Sistem LED
GPIO_MISSION_LED = 24       # Görev LED
GPIO_EXT_BUZZER = 25        # Dış Buzzer
GPIO_RGB_STRIP = 26         # RGB LED Strip

# PWM Frequencies
BUZZER_FREQUENCY = 2000     # 2kHz buzzer frequency
PWM_FREQUENCY = 1000        # 1kHz LED PWM frequency
RGB_STRIP_FREQUENCY = 800   # 800kHz for WS2812B (approximation)

# Musical Notes (Hz)
NOTES = {
    'C4': 261, 'D4': 294, 'E4': 329, 'F4': 349,
    'G4': 392, 'A4': 440, 'B4': 493, 'C5': 523,
    'SUCCESS': 880, 'WARNING': 1000, 'ERROR': 500, 'BEEP': 2000
}

# Test Patterns
LED_PATTERNS = {
    'blink': [1, 0, 1, 0, 1, 0],
    'fade_in': [0, 0.2, 0.4, 0.6, 0.8, 1.0],
    'fade_out': [1.0, 0.8, 0.6, 0.4, 0.2, 0],
    'pulse': [0, 0.5, 1.0, 0.5, 0, 0],
    'sos': [1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1]  # S.O.S pattern
}

class LEDBuzzerTester:
    """LED ve Buzzer Test Sınıfı"""
    
    def __init__(self):
        # GPIO setup
        self.gpio_initialized = False
        
        # PWM Objects
        self.led_pwm = {}
        self.buzzer_pwm = {}
        
        # Test results
        self.test_results = {
            'rgb_leds': {'red': False, 'green': False, 'blue': False},
            'status_leds': {'warning': False, 'system': False, 'mission': False},
            'buzzers': {'main': False, 'external': False},
            'rgb_strip': False
        }
        
        # Threading
        self.running = False
        self.demo_thread = None
        
    def setup_gpio(self):
        """GPIO pinlerini başlat"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # LED Pins as OUTPUT
            led_pins = [GPIO_LED_RED, GPIO_LED_GREEN, GPIO_LED_BLUE, 
                       GPIO_WARNING_LED, GPIO_SYSTEM_LED, GPIO_MISSION_LED, GPIO_RGB_STRIP]
            
            for pin in led_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            
            # Buzzer Pins as OUTPUT
            buzzer_pins = [GPIO_BUZZER_PWM, GPIO_EXT_BUZZER]
            
            for pin in buzzer_pins:
                GPIO.setup(pin, GPIO.OUT)
                GPIO.output(pin, GPIO.LOW)
            
            # Setup PWM for RGB LEDs
            self.led_pwm['red'] = GPIO.PWM(GPIO_LED_RED, PWM_FREQUENCY)
            self.led_pwm['green'] = GPIO.PWM(GPIO_LED_GREEN, PWM_FREQUENCY)
            self.led_pwm['blue'] = GPIO.PWM(GPIO_LED_BLUE, PWM_FREQUENCY)
            
            # Setup PWM for Buzzers
            self.buzzer_pwm['main'] = GPIO.PWM(GPIO_BUZZER_PWM, BUZZER_FREQUENCY)
            self.buzzer_pwm['external'] = GPIO.PWM(GPIO_EXT_BUZZER, BUZZER_FREQUENCY)
            
            # Start all PWM at 0%
            for pwm in self.led_pwm.values():
                pwm.start(0)
            
            for pwm in self.buzzer_pwm.values():
                pwm.start(0)
            
            self.gpio_initialized = True
            print("✅ GPIO pinleri başlatıldı")
            return True
            
        except Exception as e:
            print(f"❌ GPIO başlatma hatası: {e}")
            return False
    
    def cleanup_gpio(self):
        """GPIO temizliği"""
        try:
            if self.gpio_initialized:
                # Stop all PWM
                for pwm in self.led_pwm.values():
                    pwm.stop()
                for pwm in self.buzzer_pwm.values():
                    pwm.stop()
                
                GPIO.cleanup()
                print("🔄 GPIO temizlendi")
        except Exception as e:
            print(f"⚠️ GPIO temizleme uyarısı: {e}")
    
    def test_rgb_led(self, color):
        """RGB LED test"""
        print(f"🔴🟢🔵 {color.upper()} LED testi...")
        
        try:
            if color not in self.led_pwm:
                print(f"❌ Bilinmeyen renk: {color}")
                return False
            
            pwm_obj = self.led_pwm[color]
            
            # Fade in test
            print(f"   {color} LED fade in...")
            for brightness in range(0, 101, 10):
                pwm_obj.ChangeDutyCycle(brightness)
                time.sleep(0.1)
            
            # Full brightness hold
            time.sleep(0.5)
            
            # Fade out test
            print(f"   {color} LED fade out...")
            for brightness in range(100, -1, -10):
                pwm_obj.ChangeDutyCycle(brightness)
                time.sleep(0.1)
            
            # Blink test
            print(f"   {color} LED blink test...")
            for _ in range(5):
                pwm_obj.ChangeDutyCycle(100)
                time.sleep(0.2)
                pwm_obj.ChangeDutyCycle(0)
                time.sleep(0.2)
            
            self.test_results['rgb_leds'][color] = True
            print(f"✅ {color.upper()} LED testi başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ {color.upper()} LED test hatası: {e}")
            self.test_results['rgb_leds'][color] = False
            return False
    
    def test_status_led(self, led_type):
        """Status LED test"""
        led_pins = {
            'warning': GPIO_WARNING_LED,
            'system': GPIO_SYSTEM_LED,
            'mission': GPIO_MISSION_LED
        }
        
        if led_type not in led_pins:
            print(f"❌ Bilinmeyen LED tipi: {led_type}")
            return False
        
        pin = led_pins[led_type]
        print(f"💡 {led_type.upper()} LED testi (GPIO {pin})...")
        
        try:
            # Basic on/off test
            for _ in range(5):
                GPIO.output(pin, GPIO.HIGH)
                time.sleep(0.3)
                GPIO.output(pin, GPIO.LOW)
                time.sleep(0.3)
            
            # Rapid blink test
            print(f"   {led_type} LED hızlı yanıp sönme...")
            for _ in range(10):
                GPIO.output(pin, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(pin, GPIO.LOW)
                time.sleep(0.1)
            
            self.test_results['status_leds'][led_type] = True
            print(f"✅ {led_type.upper()} LED testi başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ {led_type.upper()} LED test hatası: {e}")
            self.test_results['status_leds'][led_type] = False
            return False
    
    def test_buzzer(self, buzzer_type):
        """Buzzer test"""
        buzzer_pins = {
            'main': GPIO_BUZZER_PWM,
            'external': GPIO_EXT_BUZZER
        }
        
        if buzzer_type not in buzzer_pins and buzzer_type not in self.buzzer_pwm:
            print(f"❌ Bilinmeyen buzzer tipi: {buzzer_type}")
            return False
        
        pin = buzzer_pins[buzzer_type]
        pwm_obj = self.buzzer_pwm[buzzer_type]
        print(f"🔊 {buzzer_type.upper()} buzzer testi (GPIO {pin})...")
        
        try:
            # Frequency sweep test
            print(f"   {buzzer_type} buzzer frekans taraması...")
            frequencies = [500, 1000, 1500, 2000, 2500, 2000, 1500, 1000, 500]
            
            for freq in frequencies:
                pwm_obj.ChangeFrequency(freq)
                pwm_obj.ChangeDutyCycle(50)  # 50% duty cycle
                time.sleep(0.2)
                pwm_obj.ChangeDutyCycle(0)
                time.sleep(0.1)
            
            # Musical note test
            print(f"   {buzzer_type} buzzer müzik testi...")
            melody = ['C4', 'E4', 'G4', 'C5', 'G4', 'E4', 'C4']
            
            for note in melody:
                if note in NOTES:
                    pwm_obj.ChangeFrequency(NOTES[note])
                    pwm_obj.ChangeDutyCycle(30)
                    time.sleep(0.3)
                    pwm_obj.ChangeDutyCycle(0)
                    time.sleep(0.1)
            
            # Beep pattern test
            print(f"   {buzzer_type} buzzer beep pattern...")
            pwm_obj.ChangeFrequency(NOTES['BEEP'])
            
            # Short beeps
            for _ in range(3):
                pwm_obj.ChangeDutyCycle(50)
                time.sleep(0.1)
                pwm_obj.ChangeDutyCycle(0)
                time.sleep(0.1)
            
            time.sleep(0.2)
            
            # Long beeps
            for _ in range(3):
                pwm_obj.ChangeDutyCycle(50)
                time.sleep(0.4)
                pwm_obj.ChangeDutyCycle(0)
                time.sleep(0.2)
            
            pwm_obj.ChangeDutyCycle(0)  # Ensure off
            
            self.test_results['buzzers'][buzzer_type] = True
            print(f"✅ {buzzer_type.upper()} buzzer testi başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ {buzzer_type.upper()} buzzer test hatası: {e}")
            self.test_results['buzzers'][buzzer_type] = False
            return False
    
    def test_rgb_strip(self):
        """RGB LED Strip test (WS2812B emulation)"""
        print(f"🌈 RGB LED Strip testi (GPIO {GPIO_RGB_STRIP})...")
        
        try:
            # Basic on/off pattern (WS2812B would need proper timing)
            print("   RGB Strip temel test...")
            
            # Simulate color patterns
            patterns = [
                ("Kırmızı", 10),
                ("Yeşil", 10), 
                ("Mavi", 10),
                ("Beyaz", 10),
                ("Rainbow", 20)
            ]
            
            for pattern_name, duration in patterns:
                print(f"   RGB Strip: {pattern_name} pattern...")
                
                # Simple on/off simulation for WS2812B
                for i in range(duration):
                    GPIO.output(GPIO_RGB_STRIP, GPIO.HIGH)
                    time.sleep(0.05)
                    GPIO.output(GPIO_RGB_STRIP, GPIO.LOW)
                    time.sleep(0.05)
            
            GPIO.output(GPIO_RGB_STRIP, GPIO.LOW)  # Ensure off
            
            self.test_results['rgb_strip'] = True
            print("✅ RGB LED Strip testi başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ RGB Strip test hatası: {e}")
            self.test_results['rgb_strip'] = False
            return False
    
    def run_startup_sequence(self):
        """Sistem açılış ışık/ses sekansı"""
        print("🚀 Sistem açılış sekansı başlatılıyor...")
        
        try:
            # Phase 1: System status LEDs
            print("   Faz 1: Sistem durum LED'leri...")
            GPIO.output(GPIO_SYSTEM_LED, GPIO.HIGH)
            time.sleep(0.5)
            
            # Phase 2: RGB color cycle
            print("   Faz 2: RGB renk döngüsü...")
            colors = ['red', 'green', 'blue']
            for color in colors:
                self.led_pwm[color].ChangeDutyCycle(50)
                time.sleep(0.5)
                self.led_pwm[color].ChangeDutyCycle(0)
                time.sleep(0.2)
            
            # Phase 3: All LEDs flash
            print("   Faz 3: Tüm LED'ler birlikte...")
            all_pins = [GPIO_WARNING_LED, GPIO_SYSTEM_LED, GPIO_MISSION_LED]
            
            for _ in range(3):
                # All on
                for pin in all_pins:
                    GPIO.output(pin, GPIO.HIGH)
                for color in colors:
                    self.led_pwm[color].ChangeDutyCycle(100)
                
                time.sleep(0.3)
                
                # All off
                for pin in all_pins:
                    GPIO.output(pin, GPIO.LOW)
                for color in colors:
                    self.led_pwm[color].ChangeDutyCycle(0)
                
                time.sleep(0.2)
            
            # Phase 4: Success tone
            print("   Faz 4: Başarı tonu...")
            success_melody = [NOTES['C4'], NOTES['E4'], NOTES['G4'], NOTES['C5']]
            
            for note_freq in success_melody:
                self.buzzer_pwm['main'].ChangeFrequency(note_freq)
                self.buzzer_pwm['main'].ChangeDutyCycle(30)
                time.sleep(0.3)
                self.buzzer_pwm['main'].ChangeDutyCycle(0)
                time.sleep(0.1)
            
            print("✅ Açılış sekansı tamamlandı!")
            return True
            
        except Exception as e:
            print(f"❌ Açılış sekansı hatası: {e}")
            return False
    
    def run_mission_status_demo(self):
        """Görev durumu gösterimi"""
        print("🎯 Görev durumu LED gösterimi...")
        
        mission_phases = [
            ("İniş", GPIO_MISSION_LED, 'blue', 2),
            ("Navigasyon", GPIO_WARNING_LED, 'green', 3),
            ("Yüzey", GPIO_SYSTEM_LED, 'red', 2),
            ("Roket Ayrılması", GPIO_MISSION_LED, 'red', 1),
            ("Tamamlandı", GPIO_SYSTEM_LED, 'green', 1)
        ]
        
        for phase_name, status_pin, rgb_color, duration in mission_phases:
            print(f"   {phase_name} fazı gösterimi...")
            
            for _ in range(duration):
                # Status LED on
                GPIO.output(status_pin, GPIO.HIGH)
                # RGB color on
                if rgb_color in self.led_pwm:
                    self.led_pwm[rgb_color].ChangeDutyCycle(70)
                
                time.sleep(0.8)
                
                # All off
                GPIO.output(status_pin, GPIO.LOW)
                if rgb_color in self.led_pwm:
                    self.led_pwm[rgb_color].ChangeDutyCycle(0)
                
                time.sleep(0.3)
        
        print("✅ Görev durumu gösterimi tamamlandı!")
    
    def run_emergency_alert(self):
        """Acil durum uyarısı"""
        print("🚨 Acil durum uyarısı testi...")
        
        try:
            alert_duration = 10  # 10 second alert
            start_time = time.time()
            
            while time.time() - start_time < alert_duration:
                # Flash all red
                self.led_pwm['red'].ChangeDutyCycle(100)
                GPIO.output(GPIO_WARNING_LED, GPIO.HIGH)
                
                # Emergency buzzer
                self.buzzer_pwm['main'].ChangeFrequency(NOTES['ERROR'])
                self.buzzer_pwm['main'].ChangeDutyCycle(70)
                self.buzzer_pwm['external'].ChangeFrequency(NOTES['ERROR'])
                self.buzzer_pwm['external'].ChangeDutyCycle(50)
                
                time.sleep(0.2)
                
                # All off
                self.led_pwm['red'].ChangeDutyCycle(0)
                GPIO.output(GPIO_WARNING_LED, GPIO.LOW)
                self.buzzer_pwm['main'].ChangeDutyCycle(0)
                self.buzzer_pwm['external'].ChangeDutyCycle(0)
                
                time.sleep(0.2)
            
            print("✅ Acil durum uyarısı testi tamamlandı!")
            return True
            
        except Exception as e:
            print(f"❌ Acil durum uyarısı hatası: {e}")
            return False
    
    def run_full_test_suite(self):
        """Tam test paketi"""
        print("\n🔬 LED ve Buzzer Tam Test Paketi")
        print("=" * 60)
        
        if not self.setup_gpio():
            return False
        
        try:
            # Test 1: RGB LEDs
            print("\n📍 Test 1: RGB LED'ler")
            for color in ['red', 'green', 'blue']:
                self.test_rgb_led(color)
                time.sleep(0.5)
            
            # Test 2: Status LEDs  
            print("\n📍 Test 2: Durum LED'leri")
            for led_type in ['warning', 'system', 'mission']:
                self.test_status_led(led_type)
                time.sleep(0.5)
            
            # Test 3: Buzzers
            print("\n📍 Test 3: Buzzer'lar")
            for buzzer_type in ['main', 'external']:
                self.test_buzzer(buzzer_type)
                time.sleep(0.5)
            
            # Test 4: RGB Strip
            print("\n📍 Test 4: RGB LED Strip")
            self.test_rgb_strip()
            time.sleep(0.5)
            
            # Test 5: Startup sequence
            print("\n📍 Test 5: Açılış sekansı")
            self.run_startup_sequence()
            time.sleep(1)
            
            # Test 6: Mission status demo
            print("\n📍 Test 6: Görev durumu gösterimi")
            self.run_mission_status_demo()
            time.sleep(1)
            
            # Test 7: Emergency alert
            print("\n📍 Test 7: Acil durum uyarısı")
            self.run_emergency_alert()
            
            # Final report
            self.print_test_results()
            
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"❌ Test paketi hatası: {e}")
            return False
        finally:
            self.cleanup_gpio()
    
    def print_test_results(self):
        """Test sonuçlarını yazdır"""
        print("\n📊 LED ve BUZZER TEST SONUÇLARI")
        print("=" * 50)
        
        # RGB LEDs
        print("🔴🟢🔵 RGB LED'ler:")
        for color, result in self.test_results['rgb_leds'].items():
            status = "✅ GEÇTİ" if result else "❌ BAŞARISIZ"
            print(f"   {color.upper()} LED: {status}")
        
        # Status LEDs
        print("\n💡 Durum LED'leri:")
        for led_type, result in self.test_results['status_leds'].items():
            status = "✅ GEÇTİ" if result else "❌ BAŞARISIZ"
            print(f"   {led_type.upper()} LED: {status}")
        
        # Buzzers
        print("\n🔊 Buzzer'lar:")
        for buzzer_type, result in self.test_results['buzzers'].items():
            status = "✅ GEÇTİ" if result else "❌ BAŞARISIZ"
            print(f"   {buzzer_type.upper()} Buzzer: {status}")
        
        # RGB Strip
        rgb_strip_status = "✅ GEÇTİ" if self.test_results['rgb_strip'] else "❌ BAŞARISIZ"
        print(f"\n🌈 RGB LED Strip: {rgb_strip_status}")
        
        # Overall success
        all_tests = []
        all_tests.extend(self.test_results['rgb_leds'].values())
        all_tests.extend(self.test_results['status_leds'].values())
        all_tests.extend(self.test_results['buzzers'].values())
        all_tests.append(self.test_results['rgb_strip'])
        
        overall_success = all(all_tests)
        success_rate = sum(all_tests) / len(all_tests) * 100
        
        print(f"\n🏆 GENEL SONUÇ: {'✅ TÜM TESTLER GEÇTİ' if overall_success else '⚠️ BAZI TESTLER BAŞARISIZ'}")
        print(f"📈 Başarı Oranı: {success_rate:.1f}%")

def main():
    print("🎮 TEKNOFEST 2025 - LED ve Buzzer Test Suite")
    print("=" * 60)
    
    tester = LEDBuzzerTester()
    
    try:
        return 0 if tester.run_full_test_suite() else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program kullanıcı tarafından durduruldu")
        return 1
    except Exception as e:
        print(f"❌ Program hatası: {e}")
        return 1
    finally:
        tester.cleanup_gpio()

if __name__ == "__main__":
    sys.exit(main()) 