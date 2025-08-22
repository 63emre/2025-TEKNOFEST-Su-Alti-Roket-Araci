"""
TEKNOFEST 2025 Su Altı Roket Aracı
GPIO Bileşenleri Test Scripti

Bu script LED, buzzer ve buton gibi GPIO bileşenlerini test eder.
"""

import os
import sys
import time
import signal

# Proje dizinini path'e ekle
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from common.gpio_helper import GPIOController

class GPIOTester:
    """GPIO bileşenleri test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.gpio = None
        
        # Signal handler ayarla
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Ctrl+C ile çıkış"""
        print("\n\nTest durduruluyor...")
        self.running = False
        if self.gpio:
            self.gpio.cleanup_gpio()
        sys.exit(0)
    
    def test_gpio_setup(self):
        """GPIO ayarlarını test et"""
        print("="*60)
        print("GPIO BİLEŞENLERİ TEST")
        print("="*60)
        print("Buton Pin: GPIO 18")
        print("LED Pin: GPIO 22") 
        print("Buzzer Pin: GPIO 23")
        print("-"*60)
        
        # GPIO controller oluştur
        self.gpio = GPIOController(button_pin=18, led_pin=22, buzzer_pin=23)
        
        # GPIO ayarla
        print("GPIO pinleri ayarlanıyor...")
        self.gpio.setup_gpio()
        
        if not self.gpio.setup_complete:
            print("❌ GPIO AYARLANAMADI!")
            print("Kontrol edilecekler:")
            print("- Raspberry Pi üzerinde çalışıyor musunuz?")
            print("- GPIO pinleri başka bir uygulama tarafından kullanılıyor mu?")
            print("- Root yetkisi var mı? (sudo ile çalıştırın)")
            return False
        
        print("✅ GPIO AYARLANDI!")
        print("-"*60)
        return True
    
    def test_led(self):
        """LED testini yap"""
        print("\nLED TESİ")
        print("LED'in yanıp söndüğünü kontrol edin...")
        print("-"*60)
        
        # Temel LED testleri
        tests = [
            ("LED açık", lambda: self.gpio.led_on()),
            ("LED kapalı", lambda: self.gpio.led_off()),
            ("LED açık", lambda: self.gpio.led_on()),
            ("LED toggle", lambda: self.gpio.led_toggle()),
            ("LED toggle", lambda: self.gpio.led_toggle()),
        ]
        
        for test_name, test_func in tests:
            print(f"🔸 {test_name}")
            test_func()
            time.sleep(1.5)
        
        # Blink testi
        print("🔸 LED blink testi (5 kez)")
        self.gpio.led_blink(0.3, 0.3, 5)
        time.sleep(4)
        
        # Hızlı blink testi
        print("🔸 LED hızlı blink (10 kez)")
        self.gpio.led_blink(0.1, 0.1, 10) 
        time.sleep(3)
        
        print("✅ LED testi tamamlandı")
    
    def test_buzzer(self):
        """Buzzer testini yap"""
        print("\nBUZZER TESİ")
        print("Buzzer'dan ses geldiğini kontrol edin...")
        print("-"*60)
        
        # Temel buzzer testleri
        tests = [
            ("Kısa bip", lambda: self.gpio.buzzer_beep(0.2)),
            ("Uzun bip", lambda: self.gpio.buzzer_beep(1.0)),
            ("Çok kısa bip", lambda: self.gpio.buzzer_beep(0.1)),
        ]
        
        for test_name, test_func in tests:
            print(f"🔊 {test_name}")
            test_func()
            time.sleep(0.5)
        
        # Pattern testleri
        print("🔊 SOS pattern")
        sos_pattern = [
            (0.1, 0.1), (0.1, 0.1), (0.1, 0.3),  # S
            (0.3, 0.1), (0.3, 0.1), (0.3, 0.3),  # O  
            (0.1, 0.1), (0.1, 0.1), (0.1, 0.3),  # S
        ]
        self.gpio.buzzer_beep_pattern(sos_pattern)
        
        time.sleep(1)
        
        print("🔊 Yükselen ton pattern")
        rising_pattern = [(0.1, 0.05), (0.15, 0.05), (0.2, 0.05), (0.3, 0)]
        self.gpio.buzzer_beep_pattern(rising_pattern)
        
        print("✅ Buzzer testi tamamlandı")
    
    def test_button(self):
        """Buton testini yap"""
        print("\nBUTON TESİ")
        print("Butona basıp serbest bırakın. 15 saniye sürecek.")
        print("-"*60)
        
        button_press_count = 0
        
        def button_callback():
            nonlocal button_press_count
            button_press_count += 1
            print(f"✅ Buton basıldı! (Toplam: {button_press_count})")
            self.gpio.led_toggle()
            self.gpio.buzzer_beep(0.1)
        
        # Buton callback ayarla
        self.gpio.set_button_callback(button_callback)
        
        # 15 saniye bekle
        start_time = time.time()
        while self.running and (time.time() - start_time) < 15:
            button_state = self.gpio.is_button_pressed()
            print(f"📊 Buton durumu: {'BASILI' if button_state else 'SERBEST'} | "
                  f"Basılma sayısı: {button_press_count}", end='\r')
            time.sleep(0.5)
        
        print(f"\n✅ Buton testi tamamlandı - Toplam {button_press_count} basış")
    
    def test_sequences(self):
        """Özel sequence testleri"""
        print("\nSEQUENCE TESTLERİ")
        print("Başlangıç, hata ve başarı sequence'ları test ediliyor...")
        print("-"*60)
        
        sequences = [
            ("Başlangıç Sequence", lambda: self.gpio.startup_sequence()),
            ("Hata Sequence", lambda: self.gpio.error_sequence()),
            ("Başarı Sequence", lambda: self.gpio.success_sequence()),
        ]
        
        for seq_name, seq_func in sequences:
            print(f"🎵 {seq_name}")
            seq_func()
            time.sleep(2)
        
        print("✅ Sequence testleri tamamlandı")
    
    def test_combined_functionality(self):
        """Kombine fonksiyonalite testi"""
        print("\nKOMBİNE FONKSİYONALİTE TESİ")
        print("LED + Buzzer + Buton birlikte test ediliyor...")
        print("Butona bastığınızda LED ve buzzer çalışacak.")
        print("20 saniye sürecek. Ctrl+C ile durdurun.")
        print("-"*60)
        
        press_count = 0
        
        def combined_callback():
            nonlocal press_count
            press_count += 1
            
            # Farklı basış sayılarında farklı efektler
            if press_count % 3 == 1:
                # Tek bip + LED açık
                self.gpio.led_on()
                self.gpio.buzzer_beep(0.2)
            elif press_count % 3 == 2:
                # Çift bip + LED blink
                self.gpio.buzzer_beep_pattern([(0.1, 0.1), (0.1, 0)])
                self.gpio.led_blink(0.2, 0.2, 3)
            else:
                # Uzun bip + LED kapalı
                self.gpio.buzzer_beep(0.5)
                self.gpio.led_off()
            
            print(f"\n🎉 Kombine efekt #{press_count}")
        
        self.gpio.set_button_callback(combined_callback)
        
        # 20 saniye test
        start_time = time.time()
        while self.running and (time.time() - start_time) < 20:
            elapsed = time.time() - start_time
            remaining = 20 - elapsed
            print(f"⏱️  Kalan süre: {remaining:.1f}s | Basış: {press_count}", end='\r')
            time.sleep(0.1)
        
        print(f"\n✅ Kombine test tamamlandı - Toplam {press_count} basış")
        self.gpio.led_off()  # Son durumda LED kapalı
    
    def test_stress_test(self):
        """Stress testi"""
        print("\nSTRESS TESİ")
        print("Hızlı LED/Buzzer değişimleri test ediliyor...")
        print("-"*60)
        
        # Hızlı LED toggle testi
        print("🔸 Hızlı LED toggle (100 kez)")
        for i in range(100):
            self.gpio.led_toggle()
            time.sleep(0.02)  # 50Hz
            if i % 10 == 0:
                print(f"Progress: {i+1}/100", end='\r')
        
        print("\n🔊 Hızlı buzzer test (50 kez)")
        for i in range(50):
            self.gpio.buzzer_on()
            time.sleep(0.01)
            self.gpio.buzzer_off()
            time.sleep(0.01)
            if i % 10 == 0:
                print(f"Progress: {i+1}/50", end='\r')
        
        print("\n✅ Stress testi tamamlandı")
        self.gpio.led_off()
    
    def run_full_test(self):
        """Tam test senaryosu"""
        try:
            # GPIO ayarlama testi
            if not self.test_gpio_setup():
                return
            
            # LED testi
            self.test_led()
            
            if not self.running:
                return
            
            # Buzzer testi
            self.test_buzzer()
            
            if not self.running:
                return
            
            # Buton testi
            self.test_button()
            
            if not self.running:
                return
            
            # Sequence testleri
            self.test_sequences()
            
            if not self.running:
                return
            
            # Kombine test
            self.test_combined_functionality()
            
            if not self.running:
                return
            
            # Stress test (isteğe bağlı)
            stress_test = input("\nStress testi yapılsın mı? (e/h): ").lower()
            if stress_test == 'e':
                self.test_stress_test()
            
            print("\n" + "="*60)
            print("🎉 TÜM GPIO TESTLER BAŞARIYLA TAMAMLANDI!")
            print("="*60)
            
        except Exception as e:
            print(f"\n❌ TEST HATASI: {e}")
            
        finally:
            if self.gpio:
                self.gpio.cleanup_gpio()

def main():
    """Ana test fonksiyonu"""
    tester = GPIOTester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
