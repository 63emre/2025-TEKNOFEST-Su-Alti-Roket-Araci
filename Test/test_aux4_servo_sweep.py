    #!/usr/bin/env python3
    """
    TEKNOFEST Su Altı Roket Aracı - AUX4 Servo Sweep Test
    Arduino kod mantığını takip eden açı bazlı servo kontrolü
    -90° ile +90° arasında sürekli salınım - 100Hz frekans
    """

    import time
    import math
    from pymavlink import mavutil

    # MAVLink bağlantı adresi
    MAV_ADDRESS = 'tcp:127.0.0.1:5777'

    # AUX4 servo kanal (Pixhawk AUX OUT 4 = Servo channel 12)
    SERVO_CHANNEL = 12

    # Arduino kodundaki ayarlar
    PWM_FREQUENCY = 100  # Hz (Arduino kodundan)
    PERIOD_US = 1000000 // PWM_FREQUENCY  # 10000 µs

    # Açı aralığı ve PWM değerleri
    ANGLE_MIN = -90     # Minimum açı (derece)
    ANGLE_MAX = 90      # Maksimum açı (derece)
    PULSE_MIN = 1000    # Minimum PWM (µs) - Arduino yorumundaki değer
    PULSE_MAX = 2000    # Maksimum PWM (µs) - Arduino yorumundaki değer

    # PWM değer aralıkları (güvenlik için)
    PWM_MIN = 1000    # Minimum PWM (µs)
    PWM_MAX = 2000    # Maksimum PWM (µs)

    class AUX4ServoSweep:
        def __init__(self):
            self.master = None
            self.connected = False
            self.current_angle = 0
            
        def connect_pixhawk(self):
            """Pixhawk bağlantısı"""
            try:
                print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
                self.master.wait_heartbeat(timeout=10)
                
                self.connected = True
                print("✅ MAVLink bağlantısı başarılı!")
                
                # Servo frekansını ayarla
                self.set_servo_frequency(PWM_FREQUENCY)
                
                return True
                
            except Exception as e:
                print(f"❌ Bağlantı hatası: {e}")
                return False
        
        def set_servo_frequency(self, frequency):
            """Servo frekansını ayarla (Hz)"""
            if not self.connected:
                print("❌ MAVLink bağlantısı yok!")
                return False
                
            try:
                print(f"🔧 Servo frekansı ayarlanıyor: {frequency}Hz")
                
                # AUX output frekansını ayarla (PWM_AUX_RATE parametresi)
                self.master.mav.param_set_send(
                    self.master.target_system,
                    self.master.target_component,
                    b'PWM_AUX_RATE',
                    frequency,
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                
                time.sleep(0.5)
                print(f"✅ Servo frekansı {frequency}Hz olarak ayarlandı")
                return True
                
            except Exception as e:
                print(f"❌ Frekans ayarlama hatası: {e}")
                return False
        
        def angle_to_pulse(self, angle):
            """
            Açıyı PWM pulse genişliğine dönüştür (Arduino kodundaki map fonksiyonu)
            -90° → 1000µs, +90° → 2000µs
            """
            # Arduino map(angle, -90, 90, 1000, 2000) eşdeğeri
            pulse = int((angle - ANGLE_MIN) * (PULSE_MAX - PULSE_MIN) / (ANGLE_MAX - ANGLE_MIN) + PULSE_MIN)
            
            # Güvenlik sınırları
            pulse = max(PWM_MIN, min(PWM_MAX, pulse))
            
            return pulse
        
        def set_servo_angle(self, angle):
            """AUX4 servo açı ayarı"""
            if not self.connected:
                print("❌ MAVLink bağlantısı yok!")
                return False
                
            # Açı sınırlarını kontrol et
            angle = max(ANGLE_MIN, min(ANGLE_MAX, angle))
            
            # Açıyı PWM'e dönüştür
            pwm_value = self.angle_to_pulse(angle)
            
            try:
                # MAVLink servo komut gönder
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,
                    SERVO_CHANNEL,  # AUX4 = Servo channel 12
                    pwm_value,      # PWM value
                    0, 0, 0, 0, 0
                )
                
                self.current_angle = angle
                print(f"📍 AUX4 Servo: {angle}° → {pwm_value}µs")
                return True
                
            except Exception as e:
                print(f"❌ Servo kontrol hatası: {e}")
                return False
        
        def arduino_style_sweep(self, cycles=10):
            """Arduino kodundaki mantığı takip eden sweep"""
            print(f"\n🔄 ARDUINO STYLE SWEEP - {cycles} döngü")
            print("=" * 50)
            print(f"📡 Frekans: {PWM_FREQUENCY}Hz")
            print(f"⏱️ Periyot: {PERIOD_US}µs")
            print(f"📐 Açı Aralığı: {ANGLE_MIN}° ↔ {ANGLE_MAX}°")
            print("-" * 50)
            
            for cycle in range(cycles):
                print(f"\n🔄 Döngü {cycle + 1}/{cycles}")
                
                # İlk yön: -90° → +90° (180° adımlarla, yani sadece iki pozisyon)
                for angle in range(ANGLE_MIN, ANGLE_MAX + 1, 180):  # -90, +90
                    print(f"  📍 Açı: {angle}°")
                    self.set_servo_angle(angle)
                    
                    # Arduino kodundaki gibi 2 kez PWM pulse gönder
                    # (Bu MAVLink'te gerekli değil ama timing için delay ekliyoruz)
                    time.sleep(2)  # 20ms (yaklaşık 2 pulse süresi)
                
                # İkinci yön: +90° → -90° 
                for angle in range(ANGLE_MAX, ANGLE_MIN - 1, -180):  # +90, -90
                    print(f"  📍 Açı: {angle}°")
                    self.set_servo_angle(angle)
                    time.sleep(5)  # 20ms
            
            # Orta pozisyona dön
            self.set_servo_angle(0)
            print("\n✅ Arduino style sweep tamamlandı!")
        
        def smooth_sweep(self, duration=30, step_angle=5):
            """Yumuşak sweep hareketi"""
            print(f"\n🌊 YUMUŞAK SWEEP - {duration} saniye")
            print("=" * 50)
            
            start_time = time.time()
            direction = 1  # 1: artıyor, -1: azalıyor
            current_angle = 0
            
            while time.time() - start_time < duration:
                # Açıyı güncelle
                current_angle += step_angle * direction
                
                # Sınırlara ulaşınca yön değiştir
                if current_angle >= ANGLE_MAX:
                    current_angle = ANGLE_MAX
                    direction = -1
                elif current_angle <= ANGLE_MIN:
                    current_angle = ANGLE_MIN
                    direction = 1
                
                # Servoyu hareket ettir
                self.set_servo_angle(current_angle)
                time.sleep(0.1)  # 100ms delay
            
            # Orta pozisyona dön
            self.set_servo_angle(0)
            print("\n✅ Yumuşak sweep tamamlandı!")
        
        def precise_angle_test(self):
            """Hassas açı testi"""
            print("\n📐 HASSİAS AÇI TESTİ")
            print("=" * 50)
            
            test_angles = [-90, -45, -30, -15, 0, 15, 30, 45, 90]
            
            for angle in test_angles:
                pwm = self.angle_to_pulse(angle)
                print(f"📍 Açı: {angle:3d}° → PWM: {pwm}µs")
                self.set_servo_angle(angle)
                time.sleep(2)  # 2 saniye bekle
            
            # Orta pozisyona dön
            self.set_servo_angle(0)
            print("\n✅ Hassas açı testi tamamlandı!")
        
        def interactive_angle_control(self):
            """İnteraktif açı kontrolü"""
            print("\n🎮 İNTERAKTİF AÇI KONTROLÜ")
            print("=" * 50)
            print(f"Açı girin ({ANGLE_MIN}° ile {ANGLE_MAX}° arasında) veya 'q' ile çıkın:")
            print("Özel komutlar:")
            print("  'center' - Orta pozisyon (0°)")
            print("  'min' - Minimum açı (-90°)")
            print("  'max' - Maksimum açı (+90°)")
            print("  'status' - Mevcut durum")
            
            while True:
                try:
                    user_input = input(f"\nAçı ({self.current_angle}°) > ").strip()
                    
                    if user_input.lower() == 'q':
                        break
                    elif user_input.lower() == 'center':
                        self.set_servo_angle(0)
                    elif user_input.lower() == 'min':
                        self.set_servo_angle(ANGLE_MIN)
                    elif user_input.lower() == 'max':
                        self.set_servo_angle(ANGLE_MAX)
                    elif user_input.lower() == 'status':
                        pwm = self.angle_to_pulse(self.current_angle)
                        print(f"📊 Mevcut Durum:")
                        print(f"  📐 Açı: {self.current_angle}°")
                        print(f"  📍 PWM: {pwm}µs")
                        print(f"  🔧 Frekans: {PWM_FREQUENCY}Hz")
                    else:
                        angle = int(user_input)
                        
                        if ANGLE_MIN <= angle <= ANGLE_MAX:
                            self.set_servo_angle(angle)
                            print(f"✅ Açı: {angle}°")
                        else:
                            print(f"⚠️ Açı {ANGLE_MIN}° ile {ANGLE_MAX}° arasında olmalı!")
                            
                except ValueError:
                    print("⚠️ Geçerli bir açı değeri girin!")
                except KeyboardInterrupt:
                    break
            
            # Orta pozisyona dön
            self.set_servo_angle(0)
            print("\n✅ İnteraktif test tamamlandı!")
        
        def run_all_tests(self):
            """Tüm testleri çalıştır"""
            print("🧪 AUX4 SERVO SWEEP TAM TEST PAKETİ")
            print("=" * 70)
            print(f"📡 Arduino Uyumlu - Frekans: {PWM_FREQUENCY}Hz")
            print(f"📍 Servo Kanalı: {SERVO_CHANNEL}")
            print(f"📐 Açı Aralığı: {ANGLE_MIN}° ↔ {ANGLE_MAX}°")
            print("-" * 70)
            
            if not self.connect_pixhawk():
                print("❌ Pixhawk bağlantısı başarısız!")
                return False
            
            try:
                # 1. Arduino style sweep
                self.arduino_style_sweep(5)
                
                input("\n⏸️ Yumuşak sweep için ENTER'a basın...")
                
                # 2. Yumuşak sweep
                self.smooth_sweep(15)
                
                input("\n⏸️ Hassas açı testi için ENTER'a basın...")
                
                # 3. Hassas açı testi
                self.precise_angle_test()
                
                input("\n⏸️ İnteraktif kontrol için ENTER'a basın...")
                
                # 4. İnteraktif kontrol
                self.interactive_angle_control()
                
                print("\n🎉 TÜM AUX4 SERVO SWEEP TESTLERİ BAŞARILI!")
                return True
                
            except KeyboardInterrupt:
                print("\n⚠️ Test kullanıcı tarafından durduruldu")
                return False
            except Exception as e:
                print(f"\n❌ Test hatası: {e}")
                return False
            finally:
                self.cleanup()
        
        def cleanup(self):
            """Temizlik"""
            # Servo orta pozisyon
            if self.connected:
                self.set_servo_angle(0)
            
            if self.master:
                self.master.close()
                print("🔌 MAVLink bağlantısı kapatıldı")

    def main():
        """Ana fonksiyon"""
        servo_sweep = AUX4ServoSweep()
        
        print(f"🚀 TEKNOFEST AUX4 Servo Sweep Test - Arduino Uyumlu")
        print("=" * 70)
        print("AUX4 Servo Sweep Test Menüsü:")
        print("1. Tam test paketi")
        print("2. Arduino style sweep (5 döngü)")
        print("3. Yumuşak sweep (30 saniye)")
        print("4. Hassas açı testi")
        print("5. İnteraktif açı kontrolü")
        
        try:
            choice = input("Seçiminiz (1-5): ").strip()
            
            if not servo_sweep.connect_pixhawk():
                return 1
                
            if choice == '1':
                servo_sweep.run_all_tests()
            elif choice == '2':
                servo_sweep.arduino_style_sweep(5)
            elif choice == '3':
                servo_sweep.smooth_sweep(30)
            elif choice == '4':
                servo_sweep.precise_angle_test()
            elif choice == '5':
                servo_sweep.interactive_angle_control()
            else:
                print("Geçersiz seçim!")
                return 1
                
            return 0
            
        except KeyboardInterrupt:
            print("\n⚠️ Program sonlandırıldı")
            return 1
        finally:
            servo_sweep.cleanup()

    if __name__ == "__main__":
        import sys
        sys.exit(main()) 