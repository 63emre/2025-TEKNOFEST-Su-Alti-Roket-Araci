"""
TEKNOFEST 2025 Su Altı Roket Aracı
MAVLink Bağlantı Test Scripti

Bu script Pixhawk ile MAVLink bağlantısını test eder ve 
roll, pitch, yaw verilerini okur.
"""

import os
import sys
import time
import signal

# Proje dizinini path'e ekle
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from common.mavlink_helper import MAVLinkController

class MAVLinkTester:
    """MAVLink bağlantı test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.mav = None
        
        # Bağlantı ayarları
        self.connection_string = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
        self.baud_rate = int(os.getenv("MAV_BAUD", "115200"))
        
        # Signal handler ayarla
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Ctrl+C ile çıkış"""
        print("\n\nTest durduruluyor...")
        self.running = False
        if self.mav:
            self.mav.disconnect()
        sys.exit(0)
    
    def test_connection(self):
        """MAVLink bağlantısını test et"""
        print("="*60)
        print("MAVLink BAĞLANTI TESİ")
        print("="*60)
        print(f"Bağlantı: {self.connection_string}")
        print(f"Baud Rate: {self.baud_rate}")
        print("-"*60)
        
        # MAVLink controller oluştur
        self.mav = MAVLinkController(self.connection_string, self.baud_rate)
        
        # Bağlantıyı dene
        print("Pixhawk'a bağlanılıyor...")
        if not self.mav.connect():
            print("❌ BAĞLANTI BAŞARISIZ!")
            print("Kontrol edilecekler:")
            print("- Pixhawk bağlı mı?")
            print("- USB kablo çalışıyor mu?")
            print("- Port adresi doğru mu? (/dev/ttyACM0)")
            print("- Pixhawk açık mı?")
            return False
        
        print("✅ BAĞLANTI BAŞARILI!")
        print("-"*60)
        return True
    
    def test_data_reading(self):
        """Veri okuma testini yap"""
        print("\nVERİ OKUMA TESİ BAŞLATIYOR...")
        print("30 saniye boyunca veri okunacak. Ctrl+C ile durdurun.")
        print("-"*60)
        
        start_time = time.time()
        data_count = 0
        
        try:
            while self.running and (time.time() - start_time) < 30:
                # Durum verilerini al
                status = self.mav.get_status_summary()
                
                if not status["connected"]:
                    print("❌ Bağlantı koptu!")
                    break
                
                # Attitude verileri
                attitude = status["attitude"]
                roll = attitude["roll"]
                pitch = attitude["pitch"] 
                yaw = attitude["yaw"]
                
                # Distance verisi
                distance = status["distance"]
                
                # Battery verisi
                battery = status["battery"]
                voltage = battery["voltage"]
                current = battery["current"]
                remaining = battery["remaining"]
                
                # Ekrana yazdır
                print(f"📊 Veri #{data_count + 1:3d} | "
                      f"Roll: {roll:6.1f}° | "
                      f"Pitch: {pitch:6.1f}° | "
                      f"Yaw: {yaw:6.1f}° | "
                      f"Mesafe: {distance:5.2f}m | "
                      f"Batarya: {voltage:4.1f}V ({remaining:3d}%)")
                
                data_count += 1
                time.sleep(1)  # 1 Hz veri gösterimi
                
        except KeyboardInterrupt:
            print("\n\nTest kullanıcı tarafından durduruldu")
            
        print("-"*60)
        print(f"✅ TEST TAMAMLANDI!")
        print(f"Toplam {data_count} veri paketi alındı")
    
    def test_servo_commands(self):
        """Servo komutlarını test et"""
        print("\nSERVO KOMUT TESİ")
        print("Servo komutları Pixhawk'a gönderiliyor...")
        print("Gerçek servolar bağlı değilse hareket göremezsiniz.")
        print("-"*60)
        
        servo_tests = [
            (3, 1500, "AUX 3 - Nötr"),
            (3, 1000, "AUX 3 - Minimum"), 
            (3, 2000, "AUX 3 - Maksimum"),
            (3, 1500, "AUX 3 - Nötr"),
            (4, 1500, "AUX 4 - Test"),
            (5, 1500, "AUX 5 - Test"),
            (6, 1500, "AUX 6 - Test")
        ]
        
        for servo_num, pwm_value, description in servo_tests:
            print(f"📤 {description} - PWM: {pwm_value}")
            self.mav.set_servo_pwm(servo_num, pwm_value)
            time.sleep(1)
        
        print("✅ Servo komut testi tamamlandı")
    
    def test_motor_commands(self):
        """Motor komutlarını test et"""
        print("\nMOTOR KOMUT TESİ")
        print("⚠️  DİKKAT: Motor bağlıysa düşük hızda çalışacak!")
        print("İptal etmek için Ctrl+C basın...")
        
        # 3 saniye bekle, kullanıcı iptal etme şansı
        for i in range(3, 0, -1):
            print(f"Başlama: {i} saniye...")
            time.sleep(1)
        
        if not self.running:
            return
        
        print("-"*60)
        
        motor_tests = [
            (0, "Motor Durdur"),
            (10, "Motor %10 Hız"),
            (20, "Motor %20 Hız"),
            (10, "Motor %10 Hız"),
            (0, "Motor Durdur")
        ]
        
        for speed_percent, description in motor_tests:
            print(f"🔧 {description}")
            self.mav.set_motor_speed(speed_percent)
            time.sleep(2)
        
        print("✅ Motor komut testi tamamlandı")
    
    def run_full_test(self):
        """Tam test senaryosu"""
        try:
            # Bağlantı testi
            if not self.test_connection():
                return
            
            # Veri okuma testi
            self.test_data_reading()
            
            if not self.running:
                return
            
            # Servo testi
            self.test_servo_commands()
            
            if not self.running:
                return
            
            # Motor testi (isteğe bağlı)
            test_motor = input("\nMotor komutları test edilsin mi? (e/h): ").lower()
            if test_motor == 'e':
                self.test_motor_commands()
            
            print("\n" + "="*60)
            print("🎉 TÜM TESTLER BAŞARIYLA TAMAMLANDI!")
            print("="*60)
            
        except Exception as e:
            print(f"\n❌ TEST HATASI: {e}")
            
        finally:
            if self.mav:
                self.mav.disconnect()

def main():
    """Ana test fonksiyonu"""
    tester = MAVLinkTester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
