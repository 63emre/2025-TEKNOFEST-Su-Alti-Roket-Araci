#!/usr/bin/env python3
"""
GERÇEK Motor Test Script - AUX6 Motor Testi
Pixhawk AUX6 kanalında bağlı motor için kapsamlı test suite'i
"""

import time
import sys
import json
import os
from pymavlink import mavutil

class MotorTest:
    def __init__(self):
        self.master = None
        self.config = self.load_config()
        self.motor_channel = self.config['pixhawk']['motor']  # AUX6 = 6
        self.pwm_min = self.config['pixhawk']['pwm_limits']['motor_min']
        self.pwm_max = self.config['pixhawk']['pwm_limits']['motor_max'] 
        self.pwm_stop = self.config['pixhawk']['pwm_limits']['motor_stop']
        
    def load_config(self):
        """Konfigürasyon dosyasını yükle"""
        try:
            config_path = os.path.join(os.path.dirname(__file__), '..', 'config', 'hardware_config.json')
            with open(config_path, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception as e:
            print(f"❌ Konfigürasyon dosyası yüklenemedi: {e}")
            # Varsayılan değerler
            return {
                'pixhawk': {
                    'motor': 6,
                    'pwm_limits': {
                        'motor_min': 1000,
                        'motor_max': 2000,
                        'motor_stop': 1500
                    }
                },
                'mavlink': {
                    'connection_string': 'tcp:127.0.0.1:5777',
                    'command_timeout': 5
                }
            }
    
    def connect_mavlink(self):
        """MAVLink bağlantısı kur"""
        try:
            connection_string = self.config['mavlink']['connection_string']
            print(f"🔌 MAVLink bağlantısı kuruluyor: {connection_string}")
            
            self.master = mavutil.mavlink_connection(connection_string)
            
            # Heartbeat bekle (max 10 saniye)
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=10)
            
            if heartbeat:
                print(f"✅ Pixhawk bağlantısı kuruldu - System ID: {heartbeat.get_srcSystem()}")
                return True
            else:
                print("❌ Heartbeat alınamadı - Pixhawk bağlı değil")
                return False
                
        except Exception as e:
            print(f"❌ MAVLink bağlantı hatası: {e}")
            return False
    
    def check_arm_status(self):
        """ARM durumunu kontrol et"""
        try:
            # HEARTBEAT mesajını bekle
            msg = self.master.wait_heartbeat(timeout=3)
            if msg:
                armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                if armed:
                    print("✅ Araç ARM edilmiş - Motor testi yapılabilir")
                    return True
                else:
                    print("⚠️  Araç DISARM durumda - Motor testi güvenli değil")
                    return False
            else:
                print("❌ ARM durumu kontrol edilemedi")
                return False
        except Exception as e:
            print(f"❌ ARM durum kontrolü hatası: {e}")
            return False
    
    def send_motor_pwm(self, pwm_value, duration=2):
        """Motor PWM komutu gönder"""
        try:
            print(f"🔧 Motor AUX{self.motor_channel} -> PWM: {pwm_value} ({duration}s)")
            
            # MAV_CMD_DO_SET_SERVO komutu gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                self.motor_channel,  # servo/motor number (AUX6)
                pwm_value,           # PWM value
                0, 0, 0, 0, 0       # unused parameters
            )
            
            # Komut başarı durumunu kontrol et
            timeout_time = time.time() + self.config['mavlink']['command_timeout']
            while time.time() < timeout_time:
                msg = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print(f"✅ PWM komutu kabul edildi")
                        time.sleep(duration)
                        return True
                    else:
                        print(f"❌ PWM komutu reddedildi: {msg.result}")
                        return False
                time.sleep(0.1)
            
            print("⚠️  PWM komutu timeout - durum belirsiz")
            time.sleep(duration)  # Yine de bekle
            return False
            
        except Exception as e:
            print(f"❌ PWM gönderim hatası: {e}")
            return False
    
    def motor_stop(self):
        """Motoru durdur"""
        print("🛑 Motor durduruluyor...")
        return self.send_motor_pwm(self.pwm_stop, 1)
    
    def basic_motor_test(self):
        """Temel motor PWM testi"""
        print("\n🧪 Temel Motor Testi Başlıyor...")
        
        # Stop pozisyonu
        if not self.send_motor_pwm(self.pwm_stop, 2):
            return False
        
        # Düşük hız ileri
        print("📈 Düşük hız ileri...")
        low_speed = self.pwm_stop + 100  # 1600
        if not self.send_motor_pwm(low_speed, 3):
            return False
        
        # Stop
        if not self.motor_stop():
            return False
        
        # Orta hız ileri
        print("📈 Orta hız ileri...")
        med_speed = self.pwm_stop + 200  # 1700
        if not self.send_motor_pwm(med_speed, 3):
            return False
        
        # Stop
        if not self.motor_stop():
            return False
        
        # Yüksek hız ileri
        print("📈 Yüksek hız ileri...")
        high_speed = self.pwm_stop + 300  # 1800
        if not self.send_motor_pwm(high_speed, 2):
            return False
        
        # Final stop
        return self.motor_stop()
    
    def ramp_test(self):
        """Kademeli hız artırma/azaltma testi"""
        print("\n🔄 Ramp Testi Başlıyor...")
        
        # Kademeli artış
        print("📈 Kademeli hız artışı...")
        for pwm in range(self.pwm_stop, self.pwm_stop + 250, 50):
            if not self.send_motor_pwm(pwm, 1.5):
                return False
        
        # Kademeli azalış
        print("📉 Kademeli hız azalışı...")
        for pwm in range(self.pwm_stop + 200, self.pwm_stop - 1, -50):
            if not self.send_motor_pwm(pwm, 1.5):
                return False
        
        return True
    
    def reverse_test(self):
        """Geri yön testi (destekleniyorsa)"""
        print("\n⬅️ Geri Yön Testi...")
        
        # Düşük hız geri
        reverse_speed = self.pwm_stop - 100  # 1400
        if reverse_speed >= self.pwm_min:
            print("📉 Düşük hız geri...")
            if not self.send_motor_pwm(reverse_speed, 3):
                return False
        else:
            print("⚠️  Geri yön PWM limiti aşıldı, test atlanıyor")
        
        # Stop
        return self.motor_stop()
    
    def interactive_test(self):
        """İnteraktif PWM testi"""
        print("\n🎮 İnteraktif Motor Testi")
        print(f"PWM Aralığı: {self.pwm_min} - {self.pwm_max} (Stop: {self.pwm_stop})")
        
        while True:
            try:
                pwm_input = input("\nPWM değeri girin (q=çıkış): ").strip()
                if pwm_input.lower() == 'q':
                    break
                
                pwm_value = int(pwm_input)
                if self.pwm_min <= pwm_value <= self.pwm_max:
                    self.send_motor_pwm(pwm_value, 3)
                else:
                    print(f"⚠️  PWM değeri {self.pwm_min}-{self.pwm_max} aralığında olmalı")
                    
            except ValueError:
                print("❌ Geçersiz PWM değeri")
            except KeyboardInterrupt:
                break
        
        # Çıkışta motoru durdur
        self.motor_stop()
    
    def run_all_tests(self):
        """Tüm testleri çalıştır"""
        print("🚀 MOTOR TEST SİSTEMİ BAŞLATILUYOR")
        print("=" * 50)
        
        # MAVLink bağlantısı
        if not self.connect_mavlink():
            print("\n❌ MOTOR TESTİ BAŞARISIZ - MAVLink bağlantısı kurulamadı")
            return False
        
        # ARM durumu kontrolü
        if not self.check_arm_status():
            print("\n⚠️  ARM edilmemiş araçta motor testi yapılıyor (güvenlik riski)")
            response = input("Devam etmek istiyor musunuz? (y/N): ")
            if response.lower() != 'y':
                print("❌ Motor testi iptal edildi")
                return False
        
        try:
            # Başlangıç güvenlik durduruşu
            print("\n🛡️  Güvenlik: Motor durdurma...")
            if not self.motor_stop():
                print("❌ Motor durdurulamadı!")
                return False
            
            # Temel test
            if not self.basic_motor_test():
                print("❌ Temel motor testi başarısız")
                return False
            
            # Ramp test
            if not self.ramp_test():
                print("❌ Ramp testi başarısız")  
                return False
            
            # Geri yön testi
            if not self.reverse_test():
                print("❌ Geri yön testi başarısız")
                return False
            
            # Final güvenlik durduruşu
            print("\n🛡️  Final güvenlik durduruşu...")
            if not self.motor_stop():
                print("⚠️  Motor durdurulamadı - MANUEL KONTROL GEREKLİ!")
                return False
            
            print("\n✅ TÜM MOTOR TESTLERİ BAŞARILI")
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️  Test kullanıcı tarafından durduruldu")
            self.motor_stop()  # Güvenlik için durdur
            return False
        except Exception as e:
            print(f"\n❌ Motor testi sırasında hata: {e}")
            self.motor_stop()  # Güvenlik için durdur
            return False

def main():
    """Ana fonksiyon"""
    motor_test = MotorTest()
    
    if len(sys.argv) > 1 and sys.argv[1] == '--interactive':
        motor_test.connect_mavlink()
        motor_test.interactive_test()
    else:
        success = motor_test.run_all_tests()
        sys.exit(0 if success else 1)

if __name__ == "__main__":
    main() 