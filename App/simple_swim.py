#!/usr/bin/env python3
"""
TEKNOFEST 2025 - BASİT YÜZDÜRME SCRİPTİ
2m x 1m x 2m ROV için X-Wing Yüzdürme
RAW PWM Kontrol - Karmaşıklık YOK!
"""

import time
import json
from mavlink_handler import MAVLinkHandler

class SimpleSwimmer:
    def __init__(self):
        """Basit yüzdürücü"""
        print("🏊 TEKNOFEST ROV - BASİT YÜZDÜRME")
        print("=" * 40)
        print("🎯 2m x 1m x 2m ROV Yüzdürme Modu")
        print("🔗 X-Wing: AUX1,3,4,5 + Motor AUX6")
        print("=" * 40)
        
        # MAVLink
        self.mavlink = MAVLinkHandler()
        self.connected = False
        
        # X-Wing Servo Kanalları (Pi5 Test)
        self.servos = {
            'front_left': 1,   # AUX1 - Ön Sol
            'front_right': 3,  # AUX3 - Ön Sağ  
            'rear_left': 4,    # AUX4 - Arka Sol
            'rear_right': 5,   # AUX5 - Arka Sağ
            'motor': 6         # AUX6 - Ana Motor
        }
        
        # YÜZDÜRİCİ PWM DEĞERLERİ (2x1x2m ROV için optimize)
        self.pwm_values = {
            'neutral': 1500,      # Orta pozisyon
            'lift_angle': 1400,   # Yüzdürme açısı (100 mikrosaniye aşağı)
            'motor_slow': 1530,   # Yavaş ileri (30 mikrosaniye)
            'motor_stop': 1500,   # Motor dur
            'stable_trim': 1480,  # Stabilizasyon trim
        }
        
        # Yüzdürme durumu
        self.swimming = False
        
    def connect(self):
        """MAVLink bağlan"""
        print("\n🔌 Pixhawk'a bağlanıyor...")
        
        if self.mavlink.connect():
            self.connected = True
            print("✅ Bağlantı başarılı!")
            print("📡 MAVLink TCP aktif")
            return True
        else:
            print("❌ Bağlantı başarısız!")
            return False
    
    def set_all_servos(self, pwm_dict):
        """Tüm servoları ayarla"""
        for servo_name, channel in self.servos.items():
            if servo_name in pwm_dict:
                pwm = pwm_dict[servo_name]
                success = self.mavlink.set_servo_pwm(channel, pwm)
                if success:
                    print(f"   ✅ {servo_name.upper()}: {pwm} µs")
                else:
                    print(f"   ❌ {servo_name.upper()}: HATA")
    
    def neutral_position(self):
        """Tüm servoları neutral yap"""
        print("\n🔄 NEUTRAL POZİSYON")
        
        neutral_pwm = {
            'front_left': self.pwm_values['neutral'],
            'front_right': self.pwm_values['neutral'],
            'rear_left': self.pwm_values['neutral'],
            'rear_right': self.pwm_values['neutral'],
            'motor': self.pwm_values['motor_stop']
        }
        
        self.set_all_servos(neutral_pwm)
        print("✅ Tüm sistemler neutral")
    
    def swimming_position(self):
        """Yüzdürme pozisyonu - tüm finler hafif aşağı"""
        print("\n🏊 YÜZDÜRME POZİSYONU")
        print("   Tüm finler hafıf aşağı açılı → Yüzdürme kuvveti")
        
        swim_pwm = {
            'front_left': self.pwm_values['lift_angle'],   # Hafif aşağı
            'front_right': self.pwm_values['lift_angle'],  # Hafif aşağı
            'rear_left': self.pwm_values['lift_angle'],    # Hafif aşağı
            'rear_right': self.pwm_values['lift_angle'],   # Hafif aşağı
            'motor': self.pwm_values['motor_slow']         # Yavaş ileri
        }
        
        self.set_all_servos(swim_pwm)
        print("🎯 ROV yüzmeye başlamalı!")
    
    def emergency_surface(self):
        """ACİL YÜKSEL - maksimum yüzdürme"""
        print("\n🚨 ACİL YÜKSELME!")
        print("   Tüm finler maksimum aşağı → Hızlı yüzme")
        
        surface_pwm = {
            'front_left': 1350,   # Daha fazla aşağı
            'front_right': 1350,  # Daha fazla aşağı
            'rear_left': 1350,    # Daha fazla aşağı
            'rear_right': 1350,   # Daha fazla aşağı
            'motor': 1550         # Daha hızlı ileri
        }
        
        self.set_all_servos(surface_pwm)
        print("⬆️ ROV hızla yükselmeli!")
    
    def test_fins_individual(self):
        """Finleri tek tek test et"""
        print("\n🧪 FİN TESTİ - Tek tek hareket")
        
        test_sequence = [
            ('front_left', 'Ön Sol Fin'),
            ('front_right', 'Ön Sağ Fin'), 
            ('rear_left', 'Arka Sol Fin'),
            ('rear_right', 'Arka Sağ Fin')
        ]
        
        # Önce tümünü neutral yap
        self.neutral_position()
        time.sleep(1)
        
        # Her fini tek tek test et
        for servo_name, display_name in test_sequence:
            print(f"\n📍 {display_name} Test Ediliyor...")
            
            # Bu servo aşağı, diğerleri neutral
            test_pwm = {
                'front_left': self.pwm_values['neutral'],
                'front_right': self.pwm_values['neutral'],
                'rear_left': self.pwm_values['neutral'],
                'rear_right': self.pwm_values['neutral'],
                'motor': self.pwm_values['motor_stop']
            }
            
            # Test edilen servoyu hareket ettir
            test_pwm[servo_name] = 1300  # Aşağı hareket
            
            self.set_all_servos(test_pwm)
            time.sleep(2)  # 2 saniye bekle
            
            # Neutral'a dön
            test_pwm[servo_name] = self.pwm_values['neutral']
            self.set_all_servos(test_pwm)
            time.sleep(1)
        
        print("✅ Fin testi tamamlandı")
    
    def auto_swim_test(self):
        """Otomatik yüzdürme testi - 30 saniye"""
        print("\n🤖 OTOMATİK YÜZDÜRme TESTİ")
        print("   30 saniye boyunca yüzdürme pozisyonunda kalacak")
        
        # Yüzdürme pozisyonuna geç
        self.swimming_position()
        
        # 30 saniye bekle
        for i in range(30):
            remaining = 30 - i
            print(f"\r⏱️  Yüzdürme devam ediyor... {remaining} saniye kaldı", end="", flush=True)
            time.sleep(1)
        
        print("\n✅ Otomatik yüzdürme testi tamamlandı")
        
        # Neutral'a dön
        self.neutral_position()
    
    def manual_control(self):
        """Manuel kontrol menüsü"""
        print("\n" + "=" * 30)
        print("🎮 MANUEL KONTROL MENÜSÜ")
        print("=" * 30)
        print("1) Neutral Pozisyon")
        print("2) Yüzdürme Pozisyonu")
        print("3) Acil Yükselme")
        print("4) Fin Testi (Tek tek)")
        print("5) Otomatik Yüzdürme (30sn)")
        print("9) Ana Menüye Dön")
        print("=" * 30)
        
        while True:
            try:
                choice = input("\nSeçim (1-5,9): ").strip()
                
                if choice == '1':
                    self.neutral_position()
                elif choice == '2':
                    self.swimming_position()
                elif choice == '3':
                    self.emergency_surface()
                elif choice == '4':
                    self.test_fins_individual()
                elif choice == '5':
                    self.auto_swim_test()
                elif choice == '9':
                    break
                else:
                    print("❌ Geçersiz seçim!")
                    
            except KeyboardInterrupt:
                print("\n⏹️ Manuel kontrol durduruldu")
                break
    
    def run(self):
        """Ana program döngüsü"""
        # Bağlantı kur
        if not self.connect():
            print("❌ Bağlantı kurulamadı. Çıkılıyor...")
            return
        
        try:
            while True:
                print("\n" + "=" * 50)
                print("🏊 BASİT ROV YÜZDÜRİCİ - 2x1x2m")
                print("=" * 50)
                print("s) HEMEN YÜZDÜR! (Otomatik)")
                print("n) Neutral Pozisyon")
                print("e) Acil Yükselme")
                print("t) Fin Testi")
                print("m) Manuel Kontrol Menüsü")
                print("q) Çıkış")
                print("=" * 50)
                
                choice = input("\nSeçim: ").strip().lower()
                
                if choice == 's':
                    print("\n🚀 HEMEN YÜZDÜRİYOR!")
                    self.swimming_position()
                    input("\nYüzdürme aktif! Enter'a basınca dur...")
                    self.neutral_position()
                    
                elif choice == 'n':
                    self.neutral_position()
                    
                elif choice == 'e':
                    self.emergency_surface()
                    input("\nAcil yükselme aktif! Enter'a basınca dur...")
                    self.neutral_position()
                    
                elif choice == 't':
                    self.test_fins_individual()
                    
                elif choice == 'm':
                    self.manual_control()
                    
                elif choice == 'q':
                    break
                    
                else:
                    print("❌ Geçersiz seçim!")
                    
        except KeyboardInterrupt:
            print("\n\n👋 Çıkılıyor...")
        
        finally:
            # Güvenlik - tümünü neutral yap
            print("\n🔄 Güvenlik: Tüm servoları neutral yapıyor...")
            self.neutral_position()
            
            if self.connected:
                self.mavlink.disconnect()
                print("🔌 Bağlantı kapatıldı")
            
            print("🏁 Yüzdürme tamamlandı!")

def main():
    swimmer = SimpleSwimmer()
    swimmer.run()

if __name__ == "__main__":
    main() 