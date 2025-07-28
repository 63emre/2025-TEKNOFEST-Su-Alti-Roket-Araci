#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Headless Servo Test
Pi5 Hardware Test - Terminal üzerinden servo kontrolü
"""

import sys
import time
import json
from mavlink_handler import MAVLinkHandler

class HeadlessServoTest:
    def __init__(self):
        """Headless servo test başlat"""
        print("🎮 TEKNOFEST ROV - Headless Servo Test")
        print("=" * 50)
        print("🔗 X-Wing Konfigürasyonu:")
        print("   • Ön Sol Fin: AUX1    • Ön Sağ Fin: AUX3")
        print("   • Arka Sol Fin: AUX4  • Arka Sağ Fin: AUX5")
        print("   • Ana Motor: AUX6")
        print("=" * 50)
        
        # MAVLink handler
        self.mavlink = MAVLinkHandler()
        self.connected = False
        
        # Servo mapping (Pi5 Test konfigürasyonu)
        self.servo_map = {
            '1': {'name': 'Ön Sol Fin', 'channel': 1},
            '2': {'name': 'Ön Sağ Fin', 'channel': 3},
            '3': {'name': 'Arka Sol Fin', 'channel': 4},  
            '4': {'name': 'Arka Sağ Fin', 'channel': 5},
            '6': {'name': 'Ana Motor', 'channel': 6}
        }
        
        # PWM limitleri
        self.pwm_limits = {
            'servo_min': 1100,
            'servo_max': 1900,
            'servo_neutral': 1500,
            'motor_min': 1100,
            'motor_max': 1900,
            'motor_stop': 1500
        }
    
    def connect_mavlink(self):
        """MAVLink bağlantısı"""
        print("\n🔌 MAVLink bağlantısı kuruluyor...")
        
        if self.mavlink.connect():
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # IMU test
            time.sleep(1)
            imu_data = self.mavlink.get_imu_data()
            if imu_data:
                print(f"✅ IMU: Roll={imu_data[0]:.1f}°, Pitch={imu_data[1]:.1f}°, Yaw={imu_data[2]:.1f}°")
            
            return True
        else:
            print("❌ MAVLink bağlantısı başarısız!")
            return False
    
    def show_menu(self):
        """Ana menü"""
        print("\n" + "=" * 30)
        print("🎮 SERVO KONTROL MENÜSÜ")
        print("=" * 30)
        print("1) Ön Sol Fin (AUX1)")
        print("2) Ön Sağ Fin (AUX3)")  
        print("3) Arka Sol Fin (AUX4)")
        print("4) Arka Sağ Fin (AUX5)")
        print("6) Ana Motor (AUX6)")
        print("")
        print("t) Tüm Servolar Test Et")
        print("n) Tüm Servolar Neutral")
        print("s) Sweep Test (Tüm servolar)")
        print("i) IMU Verilerini Göster")
        print("q) Çıkış")
        print("=" * 30)
    
    def test_single_servo(self, servo_key):
        """Tek servo test"""
        if not self.connected:
            print("❌ MAVLink bağlı değil!")
            return
        
        servo = self.servo_map[servo_key]
        print(f"\n🎯 {servo['name']} (AUX{servo['channel']}) Test Ediliyor")
        
        while True:
            try:
                print(f"\nPWM değeri girin (1100-1900, Enter=1500, q=çık): ", end="")
                user_input = input().strip()
                
                if user_input.lower() == 'q':
                    break
                
                if user_input == "":
                    pwm_value = 1500
                else:
                    pwm_value = int(user_input)
                
                if pwm_value < 1100 or pwm_value > 1900:
                    print("❌ PWM değeri 1100-1900 arasında olmalı!")
                    continue
                
                # Servo komutunu gönder (test için ARM kontrolü olmadan)
                success = self.mavlink.set_servo_pwm(servo['channel'], pwm_value)
                
                if success:
                    print(f"✅ {servo['name']}: PWM {pwm_value} µs gönderildi")
                else:
                    print(f"❌ {servo['name']}: Komut gönderme hatası")
                
            except ValueError:
                print("❌ Geçerli bir sayı girin!")
            except KeyboardInterrupt:
                break
        
        # Neutral position
        self.mavlink.set_servo_pwm(servo['channel'], 1500)
        print(f"🔄 {servo['name']} neutral pozisyona alındı")
    
    def test_all_servos(self):
        """Tüm servoları test et"""
        if not self.connected:
            print("❌ MAVLink bağlı değil!")
            return
        
        print("\n🎯 TÜM SERVOLAR TEST EDİLİYOR")
        
        test_positions = [1200, 1500, 1800, 1500]  # Sol, Neutral, Sağ, Neutral
        position_names = ["Sol/Aşağı", "Neutral", "Sağ/Yukarı", "Neutral"]
        
        for i, (pwm, name) in enumerate(zip(test_positions, position_names)):
            print(f"\n📍 Pozisyon {i+1}/4: {name} ({pwm} µs)")
            
            # Tüm servo kanallarına gönder (motor hariç)
            for key in ['1', '2', '3', '4']:
                servo = self.servo_map[key]
                self.mavlink.set_servo_pwm(servo['channel'], pwm)
                print(f"   ✅ {servo['name']}: {pwm} µs")
            
            time.sleep(2)  # 2 saniye bekle
        
        print("\n✅ Tüm servo testi tamamlandı")
    
    def neutral_all_servos(self):
        """Tüm servoları neutral yap"""
        if not self.connected:
            print("❌ MAVLink bağlı değil!")
            return
        
        print("\n🔄 TÜM SERVOLAR NEUTRAL POZİSYONA ALIYOR...")
        
        for key in ['1', '2', '3', '4', '6']:  # Motor dahil
            servo = self.servo_map[key]
            self.mavlink.set_servo_pwm(servo['channel'], 1500)
            print(f"   ✅ {servo['name']}: 1500 µs (neutral)")
        
        print("✅ Tüm servolar neutral pozisyonda")
    
    def sweep_test(self):
        """Smooth sweep testi"""
        if not self.connected:
            print("❌ MAVLink bağlı değil!")
            return
        
        print("\n🌊 SWEEP TEST BAŞLATILIYOR...")
        print("   (Servolar yavaşça hareket edecek)")
        
        try:
            for cycle in range(3):  # 3 döngü
                print(f"\n🔄 Döngü {cycle + 1}/3")
                
                # 1500'den 1800'e (2 saniyede)
                for pwm in range(1500, 1801, 10):
                    for key in ['1', '2', '3', '4']:
                        servo = self.servo_map[key]
                        self.mavlink.set_servo_pwm(servo['channel'], pwm)
                    time.sleep(0.05)  # 50ms aralık
                
                time.sleep(0.5)
                
                # 1800'den 1200'ye (3 saniyede) 
                for pwm in range(1800, 1199, -10):
                    for key in ['1', '2', '3', '4']:
                        servo = self.servo_map[key]
                        self.mavlink.set_servo_pwm(servo['channel'], pwm)
                    time.sleep(0.05)
                
                time.sleep(0.5)
                
                # 1200'den 1500'e (2 saniyede)
                for pwm in range(1200, 1501, 10):
                    for key in ['1', '2', '3', '4']:
                        servo = self.servo_map[key]
                        self.mavlink.set_servo_pwm(servo['channel'], pwm)
                    time.sleep(0.05)
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\n⏹️ Sweep test durduruldu")
        
        # Neutral'a dön
        self.neutral_all_servos()
        print("✅ Sweep test tamamlandı")
    
    def show_imu_data(self):
        """IMU verilerini göster"""
        if not self.connected:
            print("❌ MAVLink bağlı değil!")
            return
        
        print("\n📊 IMU VERİLERİ (Ctrl+C ile dur)")
        print("-" * 40)
        
        try:
            while True:
                imu_data = self.mavlink.get_imu_data()
                
                if imu_data:
                    roll, pitch, yaw = imu_data
                    print(f"\rRoll: {roll:+6.1f}° | Pitch: {pitch:+6.1f}° | Yaw: {yaw:+6.1f}°", end="", flush=True)
                else:
                    print(f"\r❌ IMU verisi alınamıyor", end="", flush=True)
                
                time.sleep(0.1)  # 10Hz
                
        except KeyboardInterrupt:
            print("\n📊 IMU gösterimi durduruldu")
    
    def run(self):
        """Ana döngü"""
        # MAVLink bağlantısı
        if not self.connect_mavlink():
            print("❌ MAVLink bağlantısı kurulamadı. Çıkılıyor...")
            return
        
        try:
            while True:
                self.show_menu()
                
                try:
                    choice = input("\nSeçiminiz: ").strip().lower()
                    
                    if choice == 'q':
                        break
                    elif choice in ['1', '2', '3', '4', '6']:
                        self.test_single_servo(choice)
                    elif choice == 't':
                        self.test_all_servos()
                    elif choice == 'n':
                        self.neutral_all_servos()
                    elif choice == 's':
                        self.sweep_test()
                    elif choice == 'i':
                        self.show_imu_data()
                    else:
                        print("❌ Geçersiz seçim!")
                        
                except KeyboardInterrupt:
                    print("\n⏹️ İşlem durduruldu")
                    continue
                    
        except KeyboardInterrupt:
            print("\n\n👋 Çıkılıyor...")
        
        finally:
            # Temizlik
            self.neutral_all_servos()
            if self.connected:
                self.mavlink.disconnect()
                print("🔌 MAVLink bağlantısı kapatıldı")
            
            print("🏁 Servo test tamamlandı!")

def main():
    servo_test = HeadlessServoTest()
    servo_test.run()

if __name__ == "__main__":
    main() 