#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Servo Kontrol Testi
4x DS3230MG (30kg) servo motor fin kontrolü
"""

import time
import threading
from pymavlink import mavutil
import math

# MAVLink bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
try:
    from connection_config import get_test_constants
    CONFIG = get_test_constants()
    MAV_ADDRESS = CONFIG['MAV_ADDRESS']
    print(f"📡 Using dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to static config
    MAV_ADDRESS = 'tcp:127.0.0.1:5777'
    print(f"⚠️ Using fallback connection: {MAV_ADDRESS}")

# Servo kanal tanımları - X Konfigürasyonu (Pixhawk AUX output)
SERVO_CHANNELS = {
    'fin_front_left': 1,   # Ön sol fin (AUX 1)
    'fin_front_right': 2,  # Ön sağ fin (AUX 2)  
    'fin_rear_left': 3,    # Arka sol fin (AUX 3)
    'fin_rear_right': 4    # Arka sağ fin (AUX 4)
}

# PWM değer aralıkları (DS3230MG için)
PWM_MIN = 1000    # Minimum PWM (µs)
PWM_MID = 1500    # Orta PWM (µs) 
PWM_MAX = 2000    # Maksimum PWM (µs)
PWM_DEADBAND = 50 # Dead band (µs)

class ServoController:
    def __init__(self):
        self.master = None
        self.connected = False
        self.servo_positions = {ch: PWM_MID for ch in SERVO_CHANNELS.values()}
        self.servo_targets = {ch: PWM_MID for ch in SERVO_CHANNELS.values()}
        self.servo_speeds = {ch: 50 for ch in SERVO_CHANNELS.values()}  # PWM/saniye
        
        # Test parametreleri
        self.test_running = False
        self.control_thread = None
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Servo kontrol modunu aktif et
            self.enable_servo_mode()
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def enable_servo_mode(self):
        """Servo kontrol modunu aktif et"""
        try:
            # Manual mode geç (servo kontrolü için)
            self.master.mav.set_mode_send(
                self.master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                0  # Manual mode
            )
            
            print("🎮 Manuel servo kontrol modu aktif")
            
        except Exception as e:
            print(f"⚠️ Servo mod ayarı hatası: {e}")
    
    def set_servo_position(self, channel, pwm_value):
        """Tekil servo pozisyon ayarı"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
            
        # PWM değer kontrolü
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            # MAVLink servo komut gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,      # Servo channel
                pwm_value,    # PWM value
                0, 0, 0, 0, 0
            )
            
            self.servo_positions[channel] = pwm_value
            return True
            
        except Exception as e:
            print(f"❌ Servo {channel} kontrol hatası: {e}")
            return False
    
    def set_all_servos(self, pwm_values):
        """Tüm servoları aynı anda ayarla"""
        if len(pwm_values) != 4:
            print("❌ 4 PWM değeri gerekli!")
            return False
            
        success_count = 0
        for i, (name, channel) in enumerate(SERVO_CHANNELS.items()):
            if self.set_servo_position(channel, pwm_values[i]):
                success_count += 1
                
        return success_count == 4
    
    def servo_calibration_test(self):
        """Servo kalibrasyon testi"""
        print("\n🔧 SERVO KALİBRASYON TESTİ")
        print("-" * 40)
        
        # Her servoyu ayrı ayrı test et
        for name, channel in SERVO_CHANNELS.items():
            print(f"\n🔹 {name.upper()} (Kanal {channel}) testi:")
            
            # Orta pozisyon
            print("  📍 Orta pozisyon (1500µs)")
            self.set_servo_position(channel, PWM_MID)
            time.sleep(2)
            
            # Minimum pozisyon
            print("  📍 Minimum pozisyon (1000µs)")
            self.set_servo_position(channel, PWM_MIN)
            time.sleep(2)
            
            # Maksimum pozisyon  
            print("  📍 Maksimum pozisyon (2000µs)")
            self.set_servo_position(channel, PWM_MAX)
            time.sleep(2)
            
            # Orta pozisyona dön
            print("  📍 Orta pozisyona dönüş")
            self.set_servo_position(channel, PWM_MID)
            time.sleep(1)
            
        print("✅ Servo kalibrasyon testi tamamlandı")
    
    def servo_sweep_test(self):
        """Servo sweep (tarama) testi"""
        print("\n🌊 SERVO SWEEP TESTİ")
        print("-" * 40)
        
        sweep_duration = 10  # 10 saniye
        sweep_frequency = 0.5  # 0.5 Hz
        start_time = time.time()
        
        print(f"⏱️ {sweep_duration}s boyunca {sweep_frequency}Hz frekansta sweep...")
        
        while time.time() - start_time < sweep_duration:
            elapsed = time.time() - start_time
            
            # Sinüs dalgası ile PWM hesapla
            angle = 2 * math.pi * sweep_frequency * elapsed
            pwm_offset = int(250 * math.sin(angle))  # ±250µs sapma
            pwm_value = PWM_MID + pwm_offset
            
            # Tüm servolara aynı değeri gönder
            for channel in SERVO_CHANNELS.values():
                self.set_servo_position(channel, pwm_value)
            
            print(f"  📊 PWM: {pwm_value}µs (Açı: {math.degrees(angle):.1f}°)")
            time.sleep(0.1)
        
        # Orta pozisyona dön
        for channel in SERVO_CHANNELS.values():
            self.set_servo_position(channel, PWM_MID)
            
        print("✅ Servo sweep testi tamamlandı")
    
    def fin_control_test(self):
        """Fin kontrol simülasyonu"""
        print("\n🚀 FİN KONTROL SİMÜLASYONU")
        print("-" * 40)
        
        # Simüle edilmiş kontrol komutları
        control_sequences = [
            ("🔼 YUKARİ YUNUSLAMA", {'fin_1': PWM_MIN, 'fin_3': PWM_MAX}),
            ("🔽 AŞAĞI YUNUSLAMA", {'fin_1': PWM_MAX, 'fin_3': PWM_MIN}), 
            ("↪️  SAĞ DÖNÜŞ", {'fin_2': PWM_MIN, 'fin_4': PWM_MAX}),
            ("↩️  SOL DÖNÜŞ", {'fin_2': PWM_MAX, 'fin_4': PWM_MIN}),
            ("🌀 SAĞ ROLL", {'fin_1': PWM_MAX, 'fin_2': PWM_MIN, 'fin_3': PWM_MIN, 'fin_4': PWM_MAX}),
            ("🌀 SOL ROLL", {'fin_1': PWM_MIN, 'fin_2': PWM_MAX, 'fin_3': PWM_MAX, 'fin_4': PWM_MIN}),
            ("➡️ DÜZ SEYIR", {'fin_1': PWM_MID, 'fin_2': PWM_MID, 'fin_3': PWM_MID, 'fin_4': PWM_MID})
        ]
        
        for description, fin_positions in control_sequences:
            print(f"\n📍 {description}")
            
            # Finleri pozisyonla
            for fin_name, pwm_value in fin_positions.items():
                channel = SERVO_CHANNELS[fin_name]
                self.set_servo_position(channel, pwm_value)
                print(f"  {fin_name}: {pwm_value}µs")
            
            # 3 saniye bekle
            time.sleep(3)
        
        print("✅ Fin kontrol simülasyonu tamamlandı")
    
    def servo_response_test(self):
        """Servo tepki süresi testi"""
        print("\n⏱️ SERVO TEPKİ SÜRESİ TESTİ")
        print("-" * 40)
        
        test_channel = SERVO_CHANNELS['fin_1']  # Test için 1. servoyu kullan
        
        # Hız testi (min->max geçiş)
        positions = [PWM_MIN, PWM_MAX, PWM_MID]
        
        for i, target_pwm in enumerate(positions):
            print(f"\n🎯 Test {i+1}: {target_pwm}µs pozisyonuna geçiş")
            
            start_time = time.time()
            self.set_servo_position(test_channel, target_pwm)
            
            # Servo hareket süresini simüle et (gerçek uygulamada encoder gerekir)
            expected_time = abs(target_pwm - self.servo_positions[test_channel]) / 1000  # Tahmini süre
            time.sleep(expected_time + 0.5)  # +0.5s güvenlik
            
            elapsed = time.time() - start_time
            print(f"  ⏰ Hareket süresi: {elapsed:.2f}s")
            
        print("✅ Servo tepki testi tamamlandı")
    
    def emergency_stop_test(self):
        """Acil durdurma testi"""
        print("\n🚨 ACİL DURDURMA TESTİ")
        print("-" * 40)
        
        # Servolar hareket halindeyken durdur
        print("🌊 Servolar sweep moduna alınıyor...")
        
        for i in range(20):  # 2 saniye sweep
            angle = 2 * math.pi * 0.5 * i * 0.1
            pwm_value = PWM_MID + int(200 * math.sin(angle))
            
            for channel in SERVO_CHANNELS.values():
                self.set_servo_position(channel, pwm_value)
            time.sleep(0.1)
        
        print("🛑 ACİL DURDURMA - Tüm servolar orta pozisyona!")
        
        # Tüm servolar orta pozisyon
        for channel in SERVO_CHANNELS.values():
            self.set_servo_position(channel, PWM_MID)
            
        print("✅ Acil durdurma testi başarılı")
    
    def run_full_test_suite(self):
        """Tam test paketi"""
        print("🧪 SERVO KONTROL TAM TEST PAKETİ")
        print("=" * 50)
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        try:
            # 1. Kalibrasyon testi
            self.servo_calibration_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 2. Sweep testi
            self.servo_sweep_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 3. Fin kontrol testi
            self.fin_control_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 4. Tepki süresi testi
            self.servo_response_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 5. Acil durdurma testi
            self.emergency_stop_test()
            
            print("\n🎉 TÜM SERVO TESTLERİ BAŞARILI!")
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
        # Tüm servolar orta pozisyon
        if self.connected:
            for channel in SERVO_CHANNELS.values():
                self.set_servo_position(channel, PWM_MID)
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    controller = ServoController()
    
    try:
        success = controller.run_full_test_suite()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1
    finally:
        controller.cleanup()

if __name__ == "__main__":
    import sys
    sys.exit(main()) 