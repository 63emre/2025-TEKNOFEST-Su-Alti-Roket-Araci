#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - RAW PWM Kontrol Yüzdürme
X-Wing Konfigürasyonu ile Manuel Kontrol
AUX1: Ön Sol, AUX3: Ön Sağ, AUX4: Arka Sol, AUX5: Arka Sağ, AUX6: Motor
"""

import time
import json
import threading
import sys
import os

# Ana klasörden modülleri import et
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from mavlink_handler import MAVLinkHandler

class RawSwimControl:
    def __init__(self):
        """Raw PWM tabanlı yüzdürme kontrolcüsü"""
        print("🏊 RAW PWM SWIM CONTROL - X-WING")
        print("=" * 50)
        print("🎯 2x2m Havuz, 1m Derinlik - Manual Kontrol")
        print("🔗 AUX1,3,4,5 (X-Wing) + AUX6 (Motor)")
        print("=" * 50)
        
        # MAVLink handler
        self.mavlink = MAVLinkHandler()
        self.connected = False
        
        # X-Wing servo mapping
        self.servos = {
            'front_left': 1,   # AUX1 - Ön Sol Fin
            'front_right': 3,  # AUX3 - Ön Sağ Fin  
            'rear_left': 4,    # AUX4 - Arka Sol Fin
            'rear_right': 5,   # AUX5 - Arka Sağ Fin
            'motor': 6         # AUX6 - Ana Tahrik Motoru
        }
        
        # PWM değerleri (2x2m havuz için optimize)
        self.pwm = {
            'neutral': 1500,
            'servo_min': 1100,
            'servo_max': 1900,
            'motor_stop': 1500,
            'motor_min': 1000,
            'motor_max': 2000,
            
            # Hareket PWM değerleri
            'lift_small': 1450,   # Hafif yukarı (yüzdürme)
            'lift_medium': 1400,  # Orta yukarı
            'lift_strong': 1350,  # Güçlü yukarı
            
            'dive_small': 1550,   # Hafif aşağı
            'dive_medium': 1600,  # Orta aşağı
            'dive_strong': 1650,  # Güçlü aşağı
            
            'motor_slow': 1520,   # Yavaş ileri
            'motor_medium': 1550, # Orta hız
            'motor_fast': 1600,   # Hızlı
            'motor_reverse': 1450 # Geri
        }
        
        # Kontrol durumu
        self.control_active = False
        self.current_position = "neutral"
        
    def connect(self):
        """Pixhawk'a bağlan"""
        print("\n🔌 Pixhawk TCP bağlantısı kuruluyor...")
        
        if self.mavlink.connect():
            self.connected = True
            print("✅ MAVLink TCP bağlantısı başarılı!")
            return True
        else:
            print("❌ Bağlantı başarısız!")
            return False
    
    def set_servo_position(self, servo_name, pwm_value):
        """Tek servo pozisyonu ayarla"""
        if servo_name not in self.servos:
            print(f"❌ Bilinmeyen servo: {servo_name}")
            return False
            
        channel = self.servos[servo_name]
        
        # PWM limitlerini kontrol et
        if servo_name == 'motor':
            pwm_value = max(self.pwm['motor_min'], min(self.pwm['motor_max'], pwm_value))
        else:
            pwm_value = max(self.pwm['servo_min'], min(self.pwm['servo_max'], pwm_value))
        
        success = self.mavlink.set_servo_pwm(channel, pwm_value)
        if success:
            print(f"✅ {servo_name.upper()}: {pwm_value} µs")
        else:
            print(f"❌ {servo_name.upper()}: Ayarlama hatası")
        
        return success
    
    def set_all_servos(self, position_dict):
        """Tüm servoları aynı anda ayarla"""
        print(f"\n🎯 SERVO POZİSYONU: {self.current_position}")
        
        for servo_name, pwm_value in position_dict.items():
            if servo_name in self.servos:
                self.set_servo_position(servo_name, pwm_value)
                time.sleep(0.05)  # Servo arası kısa bekleme
    
    def neutral_position(self):
        """Tüm sistemler neutral - su yüzeyinde durma"""
        self.current_position = "neutral"
        
        neutral_pwm = {
            'front_left': self.pwm['neutral'],
            'front_right': self.pwm['neutral'],
            'rear_left': self.pwm['neutral'],
            'rear_right': self.pwm['neutral'],
            'motor': self.pwm['motor_stop']
        }
        
        self.set_all_servos(neutral_pwm)
        print("🏊 Su yüzeyinde durma pozisyonu")
    
    def surface_swim(self):
        """Su yüzeyinde yüzdürme - hafif yukarı açı"""
        self.current_position = "surface_swim"
        
        surface_pwm = {
            'front_left': self.pwm['lift_small'],
            'front_right': self.pwm['lift_small'], 
            'rear_left': self.pwm['lift_small'],
            'rear_right': self.pwm['lift_small'],
            'motor': self.pwm['motor_slow']
        }
        
        self.set_all_servos(surface_pwm)
        print("🏊 Su yüzeyinde yüzdürme - tüm finler hafif yukarı")
    
    def shallow_dive(self):
        """Sığ dalış - 0.5m derinlik"""
        self.current_position = "shallow_dive"
        
        dive_pwm = {
            'front_left': self.pwm['dive_small'],
            'front_right': self.pwm['dive_small'],
            'rear_left': self.pwm['dive_small'], 
            'rear_right': self.pwm['dive_small'],
            'motor': self.pwm['motor_medium']
        }
        
        self.set_all_servos(dive_pwm)
        print("🏊 Sığ dalış - 0.5m derinlik hedefi")
    
    def deep_dive(self):
        """Derin dalış - 1m derinlik (havuz tabanı)"""
        self.current_position = "deep_dive"
        
        deep_pwm = {
            'front_left': self.pwm['dive_medium'],
            'front_right': self.pwm['dive_medium'],
            'rear_left': self.pwm['dive_medium'],
            'rear_right': self.pwm['dive_medium'], 
            'motor': self.pwm['motor_medium']
        }
        
        self.set_all_servos(deep_pwm)
        print("🏊 Derin dalış - 1m derinlik (havuz tabanı)")
    
    def turn_left(self):
        """Sol dönüş - X-wing asimetrik kontrol"""
        self.current_position = "turn_left"
        
        left_turn_pwm = {
            'front_left': self.pwm['dive_small'],    # Sol finler aşağı
            'rear_left': self.pwm['dive_small'],
            'front_right': self.pwm['lift_small'],   # Sağ finler yukarı  
            'rear_right': self.pwm['lift_small'],
            'motor': self.pwm['motor_slow']
        }
        
        self.set_all_servos(left_turn_pwm)
        print("🏊 Sol dönüş - X-wing asimetrik kontrol")
    
    def turn_right(self):
        """Sağ dönüş - X-wing asimetrik kontrol"""
        self.current_position = "turn_right"
        
        right_turn_pwm = {
            'front_left': self.pwm['lift_small'],    # Sol finler yukarı
            'rear_left': self.pwm['lift_small'],
            'front_right': self.pwm['dive_small'],   # Sağ finler aşağı
            'rear_right': self.pwm['dive_small'],
            'motor': self.pwm['motor_slow']
        }
        
        self.set_all_servos(right_turn_pwm)
        print("🏊 Sağ dönüş - X-wing asimetrik kontrol")
    
    def forward_fast(self):
        """Hızlı ileri - düz yüzdürme"""
        self.current_position = "forward_fast"
        
        forward_pwm = {
            'front_left': self.pwm['neutral'],
            'front_right': self.pwm['neutral'],
            'rear_left': self.pwm['neutral'],
            'rear_right': self.pwm['neutral'],
            'motor': self.pwm['motor_fast']
        }
        
        self.set_all_servos(forward_pwm)
        print("🏊 Hızlı ileri - düz yüzdürme")
    
    def reverse(self):
        """Geri hareket"""
        self.current_position = "reverse"
        
        reverse_pwm = {
            'front_left': self.pwm['neutral'],
            'front_right': self.pwm['neutral'],
            'rear_left': self.pwm['neutral'],
            'rear_right': self.pwm['neutral'],
            'motor': self.pwm['motor_reverse']
        }
        
        self.set_all_servos(reverse_pwm)
        print("🏊 Geri hareket")
    
    def emergency_stop(self):
        """Acil durdurma - tüm sistemler neutral"""
        print("\n🚨 ACİL DURDURMA!")
        self.neutral_position()
        self.control_active = False
    
    def run_interactive_control(self):
        """Interaktif kontrol modu"""
        print("\n" + "="*60)
        print("🎮 INTERAKTIF RAW PWM KONTROL")
        print("="*60)
        print("🎯 Komutlar:")
        print("  0 = Neutral (durma)")
        print("  1 = Su yüzeyinde yüzdürme") 
        print("  2 = Sığ dalış (0.5m)")
        print("  3 = Derin dalış (1m)")
        print("  4 = Sol dönüş")
        print("  5 = Sağ dönüş")
        print("  6 = Hızlı ileri")
        print("  7 = Geri hareket")
        print("  s = Acil stop")
        print("  q = Çıkış")
        print("="*60)
        
        self.control_active = True
        
        try:
            while self.control_active:
                command = input("\n🎮 Komut girin: ").strip().lower()
                
                if command == '0':
                    self.neutral_position()
                elif command == '1':
                    self.surface_swim()
                elif command == '2':
                    self.shallow_dive()
                elif command == '3':
                    self.deep_dive()
                elif command == '4':
                    self.turn_left()
                elif command == '5':  
                    self.turn_right()
                elif command == '6':
                    self.forward_fast()
                elif command == '7':
                    self.reverse()
                elif command == 's':
                    self.emergency_stop()
                elif command == 'q':
                    print("👋 Kontrol modundan çıkılıyor...")
                    self.emergency_stop()
                    break
                else:
                    print("❌ Geçersiz komut! (0-7, s, q)")
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n🚨 Ctrl+C ile durduruldu!")
            self.emergency_stop()
        except Exception as e:
            print(f"❌ Kontrol hatası: {e}")
            self.emergency_stop()
    
    def run_automatic_sequence(self):
        """Otomatik test sekansı - 2x2m havuzda"""
        print("\n" + "="*60)
        print("🤖 OTOMATİK TEST SEKASI - 2x2m HAVUZ")
        print("="*60)
        
        sequences = [
            ("Başlangıç - Neutral", self.neutral_position, 3),
            ("Su yüzeyinde yüzdürme", self.surface_swim, 5),
            ("Sığ dalış", self.shallow_dive, 5),
            ("Sol dönüş", self.turn_left, 3),
            ("Düz ileri", self.forward_fast, 4),
            ("Sağ dönüş", self.turn_right, 3),
            ("Derin dalış", self.deep_dive, 5),
            ("Geri hareket", self.reverse, 3),
            ("Yüzeye çıkış", self.surface_swim, 4),
            ("Final - Neutral", self.neutral_position, 2)
        ]
        
        try:
            for i, (description, function, duration) in enumerate(sequences, 1):
                print(f"\n🎯 ADIM {i}/{len(sequences)}: {description}")
                print(f"⏱️ Süre: {duration} saniye")
                
                function()
                
                # Countdown
                for remaining in range(duration, 0, -1):
                    print(f"   ⏰ {remaining} saniye kaldı...", end='\r')
                    time.sleep(1)
                
                print("   ✅ Tamamlandı!        ")
            
            print("\n🎉 Otomatik test sekansı başarıyla tamamlandı!")
            
        except KeyboardInterrupt:
            print("\n🚨 Test sekansı durduruldu!")
            self.emergency_stop()
        except Exception as e:
            print(f"❌ Test sekansı hatası: {e}")
            self.emergency_stop()
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.connected:
            self.emergency_stop()
            self.mavlink.disconnect()
            self.connected = False
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana program"""
    swimmer = RawSwimControl()
    
    # Bağlantı kur
    if not swimmer.connect():
        print("❌ Bağlantı kurulamadı. Çıkılıyor...")
        return
    
    try:
        while True:
            print("\n" + "="*50)
            print("🏊 RAW PWM SWIM CONTROL - MENÜ")
            print("="*50)
            print("1. İnteraktif kontrol")
            print("2. Otomatik test sekansı")
            print("3. Çıkış")
            
            choice = input("\nSeçiminizi yapın (1-3): ").strip()
            
            if choice == '1':
                swimmer.run_interactive_control()
            elif choice == '2':
                swimmer.run_automatic_sequence()
            elif choice == '3':
                break
            else:
                print("❌ Geçersiz seçim!")
                
    except KeyboardInterrupt:
        print("\n👋 Program sonlandırılıyor...")
    finally:
        swimmer.disconnect()

if __name__ == "__main__":
    main() 