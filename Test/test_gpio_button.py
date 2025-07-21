#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - GPIO Buton & Güvenlik Sistemi Testi
16mm metal buton + 90 saniye güvenlik gecikme + acil durdurma testi
"""

import RPi.GPIO as GPIO
import time
import threading
import signal
import sys
from datetime import datetime

# GPIO pin tanımları
POWER_BUTTON_PIN = 18    # Ana güç butonu (16mm metal buton)
EMERGENCY_STOP_PIN = 19  # Acil durdurma butonu (opsiyonel)
STATUS_LED_PIN = 20      # Durum LED'i
MOTOR_ENABLE_PIN = 21    # Motor enable rölesi

class SafetyButtonSystem:
    def __init__(self):
        self.system_powered = False
        self.motor_enabled = False
        self.safety_timer = 90  # 90 saniye güvenlik gecikmesi
        self.emergency_stop = False
        self.button_press_count = 0
        self.last_button_press = 0
        
        # GPIO kurulum
        self.setup_gpio()
        
        # Status threads
        self.status_thread = None
        self.timer_thread = None
        self.running = True
        
    def setup_gpio(self):
        """GPIO pinlerini kur"""
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Input pinleri (pull-up dirençli)
        GPIO.setup(POWER_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(EMERGENCY_STOP_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Output pinleri
        GPIO.setup(STATUS_LED_PIN, GPIO.OUT)
        GPIO.setup(MOTOR_ENABLE_PIN, GPIO.OUT)
        
        # Başlangıçta her şey kapalı
        GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        GPIO.output(MOTOR_ENABLE_PIN, GPIO.LOW)
        
        # Interrupt callback'leri
        GPIO.add_event_detect(POWER_BUTTON_PIN, GPIO.FALLING, 
                            callback=self.power_button_callback, bouncetime=300)
        GPIO.add_event_detect(EMERGENCY_STOP_PIN, GPIO.FALLING,
                            callback=self.emergency_stop_callback, bouncetime=300)
        
        print("🔧 GPIO kurulumu tamamlandı")
    
    def power_button_callback(self, channel):
        """Ana güç butonu callback"""
        current_time = time.time()
        
        # Debounce kontrolü (300ms)
        if current_time - self.last_button_press < 0.3:
            return
            
        self.last_button_press = current_time
        self.button_press_count += 1
        
        print(f"\n🔘 Güç butonu basıldı! (#{self.button_press_count})")
        
        if not self.system_powered:
            self.start_system()
        else:
            self.stop_system()
    
    def emergency_stop_callback(self, channel):
        """Acil durdurma butonu callback"""
        print("\n🚨 ACİL DURDURMA AKTİF!")
        self.emergency_stop = True
        self.stop_system()
    
    def start_system(self):
        """Sistem başlatma"""
        if self.emergency_stop:
            print("❌ Acil durdurma aktif - sistem başlatılamıyor!")
            return
            
        self.system_powered = True
        print("✅ Sistem enerjilendi - 90 saniye güvenlik gecikmesi başladı")
        
        # Status LED'i yavaş yanıp sön (güvenlik gecikmesi)
        self.start_status_thread()
        
        # 90 saniye timer başlat
        self.timer_thread = threading.Thread(target=self.safety_timer_countdown)
        self.timer_thread.daemon = True
        self.timer_thread.start()
    
    def stop_system(self):
        """Sistem durdurma"""
        self.system_powered = False
        self.motor_enabled = False
        self.emergency_stop = False
        
        # Tüm çıkışları kapat
        GPIO.output(STATUS_LED_PIN, GPIO.LOW)
        GPIO.output(MOTOR_ENABLE_PIN, GPIO.LOW)
        
        print("⛔ Sistem durduruldu - tüm güç kesildi")
        
        # Thread'leri durdur
        if self.timer_thread and self.timer_thread.is_alive():
            self.running = False
    
    def safety_timer_countdown(self):
        """90 saniye güvenlik gecikmesi countdown"""
        for remaining in range(self.safety_timer, 0, -1):
            if not self.system_powered or self.emergency_stop:
                return
                
            if remaining % 10 == 0 or remaining <= 10:
                print(f"⏱️  Motor aktivasyon: {remaining} saniye")
                
            time.sleep(1)
        
        if self.system_powered and not self.emergency_stop:
            self.enable_motors()
    
    def enable_motors(self):
        """Motorları aktif et"""
        self.motor_enabled = True
        GPIO.output(MOTOR_ENABLE_PIN, GPIO.HIGH)
        print("🚀 MOTORLAR AKTİF EDILDI!")
        print("⚠️  Artık motor komutları gönderilebilir")
    
    def start_status_thread(self):
        """Status LED thread başlat"""
        if self.status_thread and self.status_thread.is_alive():
            return
            
        self.running = True
        self.status_thread = threading.Thread(target=self.status_led_worker)
        self.status_thread.daemon = True
        self.status_thread.start()
    
    def status_led_worker(self):
        """Status LED kontrol thread"""
        while self.running and self.system_powered:
            if self.emergency_stop:
                # Acil durdurma: Hızlı yanıp sön
                GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
                time.sleep(0.1)
                GPIO.output(STATUS_LED_PIN, GPIO.LOW)
                time.sleep(0.1)
            elif not self.motor_enabled:
                # Güvenlik gecikmesi: Yavaş yanıp sön
                GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(STATUS_LED_PIN, GPIO.LOW)
                time.sleep(0.5)
            else:
                # Motor aktif: Sürekli yak
                GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
                time.sleep(0.1)
    
    def get_system_status(self):
        """Sistem durumunu döndür"""
        return {
            'system_powered': self.system_powered,
            'motor_enabled': self.motor_enabled,
            'emergency_stop': self.emergency_stop,
            'button_press_count': self.button_press_count,
            'safety_timer_active': self.system_powered and not self.motor_enabled
        }
    
    def test_sequence(self):
        """Test sırası çalıştır"""
        print("🧪 GPIO BUTON SİSTEMİ TEST BAŞLADI")
        print("=" * 50)
        
        print("\n📋 TEST TALİMATLARI:")
        print("1. Ana güç butonuna basın (sistem enerjilenir)")
        print("2. 90 saniye bekleyin (status LED yavaş yanıp söner)")
        print("3. Motorlar aktif olduğunda LED sürekli yanar")
        print("4. Tekrar güç butonuna basın (sistem durur)")
        print("5. Acil durdurma butonunu test edin")
        print("\n⌨️ CTRL+C ile testi bitirin\n")
        
        try:
            while True:
                # Her 5 saniyede durum raporu
                status = self.get_system_status()
                timestamp = datetime.now().strftime("%H:%M:%S")
                
                print(f"\n[{timestamp}] 📊 SİSTEM DURUMU:")
                print(f"  💡 Sistem Aktif: {'✅' if status['system_powered'] else '❌'}")
                print(f"  🚀 Motor Aktif: {'✅' if status['motor_enabled'] else '❌'}")
                print(f"  🚨 Acil Durdurma: {'🔴' if status['emergency_stop'] else '🟢'}")
                print(f"  🔘 Buton Basım: {status['button_press_count']} kez")
                print(f"  ⏱️  Güvenlik Timer: {'🔶' if status['safety_timer_active'] else '🔸'}")
                
                time.sleep(5)
                
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        print("\n🧹 Temizlik işlemleri...")
        self.running = False
        self.stop_system()
        GPIO.cleanup()
        print("✅ GPIO temizlendi")

def signal_handler(sig, frame):
    """SIGINT handler"""
    print("\n🛑 Program sonlandırılıyor...")
    sys.exit(0)

def main():
    """Ana fonksiyon"""
    # Signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    # Test sistemi oluştur
    safety_system = SafetyButtonSystem()
    
    try:
        # Test sırasını çalıştır
        safety_system.test_sequence()
        
    except Exception as e:
        print(f"❌ Beklenmeyen hata: {e}")
        
    finally:
        safety_system.cleanup()

if __name__ == "__main__":
    main() 