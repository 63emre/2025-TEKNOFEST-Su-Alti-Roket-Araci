#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Motor Kontrol Testi  
DEGZ BLU 30A ESC + M5 Su Altı Motor kontrolü
"""

import time
import threading
from pymavlink import mavutil
import math

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

# Motor kanal tanımları - HARDWARE_PIN_MAPPING.md standardı
MOTOR_CHANNEL = 1  # Pixhawk MAIN 1 output (DEGZ M5 Motor + ESC)

# ESC PWM değer aralıkları (BLU 30A ESC için)
ESC_MIN = 1000      # Motor stop
ESC_NEUTRAL = 1500  # Motor neutral (su altı motor için)
ESC_MAX = 2000      # Max forward thrust
ESC_ARM_SIGNAL = 1000  # Arming sinyali

# Motor güvenlik limitleri
MOTOR_SAFE_MIN = 1100   # Güvenli minimum
MOTOR_SAFE_MAX = 1900   # Güvenli maksimum  
MOTOR_RAMP_RATE = 10    # PWM/saniye artış hızı

class MotorController:
    def __init__(self):
        self.master = None
        self.connected = False
        self.motor_armed = False
        self.current_throttle = ESC_NEUTRAL
        self.target_throttle = ESC_NEUTRAL
        self.motor_enabled = False
        
        # Güvenlik
        self.emergency_stop = False
        self.max_test_throttle = 1700  # Test için güvenli maksimum
        
        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def arm_motor(self):
        """Motor arming işlemi"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
            
        print("🔫 Motor arming işlemi başlatılıyor...")
        
        try:
            # ESC arming sequence
            # 1. Minimum throttle gönder (1-2 saniye)
            print("  📡 Arming sinyali gönderiliyor...")
            for _ in range(20):  # 2 saniye
                self.send_motor_command(ESC_ARM_SIGNAL)
                time.sleep(0.1)
            
            # 2. Neutral position
            print("  📡 Neutral pozisyon...")
            for _ in range(10):  # 1 saniye
                self.send_motor_command(ESC_NEUTRAL)
                time.sleep(0.1)
            
            self.motor_armed = True
            self.current_throttle = ESC_NEUTRAL
            print("✅ Motor armed successfully!")
            
            return True
            
        except Exception as e:
            print(f"❌ Motor arming hatası: {e}")
            return False
    
    def send_motor_command(self, throttle_pwm):
        """Motor komut gönder"""
        if not self.connected:
            return False
            
        if self.emergency_stop:
            throttle_pwm = ESC_NEUTRAL
        
        # Güvenlik limitleri
        if throttle_pwm > ESC_NEUTRAL:
            throttle_pwm = min(throttle_pwm, self.max_test_throttle)
        else:
            throttle_pwm = max(throttle_pwm, ESC_MIN)
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL,  # Motor channel
                throttle_pwm,   # PWM value
                0, 0, 0, 0, 0
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Motor komut hatası: {e}")
            return False
    
    def set_throttle_smooth(self, target_pwm):
        """Yumuşak throttle değişimi"""
        if not self.motor_armed:
            print("❌ Motor armed değil!")
            return False
        
        self.target_throttle = target_pwm
        
        # Gradual change thread başlat
        if self.control_thread and self.control_thread.is_alive():
            return True  # Zaten çalışıyor
            
        self.running = True
        self.control_thread = threading.Thread(target=self.throttle_ramp_worker)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        return True
    
    def throttle_ramp_worker(self):
        """Throttle ramping thread"""
        while self.running and self.motor_armed:
            if self.current_throttle != self.target_throttle:
                # Hangi yönde hareket edeceğiz?
                if self.current_throttle < self.target_throttle:
                    self.current_throttle = min(
                        self.current_throttle + MOTOR_RAMP_RATE,
                        self.target_throttle
                    )
                else:
                    self.current_throttle = max(
                        self.current_throttle - MOTOR_RAMP_RATE,
                        self.target_throttle
                    )
                
                # Komutu gönder
                self.send_motor_command(self.current_throttle)
                
            time.sleep(0.1)  # 10Hz kontrol
    
    def motor_arming_test(self):
        """Motor arming testi"""
        print("\n🔫 MOTOR ARMING TESTİ")
        print("-" * 40)
        
        if not self.arm_motor():
            print("❌ Motor arming başarısız!")
            return False
        
        print("✅ Motor arming başarılı!")
        return True
    
    def throttle_response_test(self):
        """Throttle tepki testi"""
        print("\n🚀 THROTTLE TEPKİ TESTİ")
        print("-" * 40)
        
        if not self.motor_armed:
            if not self.arm_motor():
                return False
        
        # Test throttle değerleri
        test_values = [
            (ESC_NEUTRAL, "NEUTRAL"),
            (1550, "DÜŞ%K FORWARD"), 
            (1600, "ORTA FORWARD"),
            (1650, "YÜKSEK FORWARD"),
            (ESC_NEUTRAL, "NEUTRAL"),
            (1450, "DÜŞÜK REVERSE"),
            (1400, "ORTA REVERSE"), 
            (1350, "YÜKSEK REVERSE"),
            (ESC_NEUTRAL, "NEUTRAL STOP")
        ]
        
        for throttle_pwm, description in test_values:
            print(f"\n🎯 {description} ({throttle_pwm}µs)")
            
            # Smooth transition
            self.set_throttle_smooth(throttle_pwm)
            
            # Hedefe ulaşana kadar bekle
            while abs(self.current_throttle - throttle_pwm) > 5:
                print(f"  📊 Current: {self.current_throttle}µs -> Target: {throttle_pwm}µs")
                time.sleep(0.5)
            
            print(f"  ✅ Hedef throttle ulaşıldı: {self.current_throttle}µs")
            time.sleep(2)  # 2 saniye bu throttle'da kal
        
        print("✅ Throttle tepki testi tamamlandı")
        return True
    
    def motor_ramp_test(self):
        """Motor ramping testi"""
        print("\n📈 MOTOR RAMPING TESTİ")
        print("-" * 40)
        
        if not self.motor_armed:
            if not self.arm_motor():
                return False
        
        print("📊 Neutral'den maksimuma yavaş artış...")
        
        # Neutral'den test maksimuma
        ramp_steps = 20
        throttle_range = self.max_test_throttle - ESC_NEUTRAL
        step_size = throttle_range // ramp_steps
        
        for i in range(ramp_steps + 1):
            throttle = ESC_NEUTRAL + (i * step_size)
            print(f"  🎚️ Step {i+1}/{ramp_steps+1}: {throttle}µs")
            
            self.send_motor_command(throttle)
            time.sleep(1)  # Her adımda 1 saniye bekle
        
        # Geri neutral'e in
        print("\n📉 Neutral'e geri dönüş...")
        for i in range(ramp_steps, -1, -1):
            throttle = ESC_NEUTRAL + (i * step_size)
            print(f"  🎚️ Step {ramp_steps-i+1}/{ramp_steps+1}: {throttle}µs")
            
            self.send_motor_command(throttle)
            time.sleep(0.5)
        
        print("✅ Motor ramping testi tamamlandı")
        return True
    
    def motor_oscillation_test(self):
        """Motor oscillation testi"""
        print("\n🌊 MOTOR OSCİLLATION TESTİ")
        print("-" * 40)
        
        if not self.motor_armed:
            if not self.arm_motor():
                return False
        
        test_duration = 20  # 20 saniye
        frequency = 0.2     # 0.2 Hz (5 saniye period)
        amplitude = 100     # ±100µs amplitude
        
        print(f"⏱️ {test_duration}s boyunca {frequency}Hz sinus dalgası...")
        
        start_time = time.time()
        
        while time.time() - start_time < test_duration:
            elapsed = time.time() - start_time
            
            # Sinus dalgası ile throttle hesapla
            angle = 2 * math.pi * frequency * elapsed
            throttle_offset = int(amplitude * math.sin(angle))
            throttle_pwm = ESC_NEUTRAL + throttle_offset
            
            self.send_motor_command(throttle_pwm)
            print(f"  📊 PWM: {throttle_pwm}µs (Offset: {throttle_offset:+d}µs)")
            
            time.sleep(0.2)
        
        # Neutral'e dön
        self.send_motor_command(ESC_NEUTRAL)
        print("✅ Motor oscillation testi tamamlandı")
        return True
    
    def emergency_stop_test(self):
        """Acil durdurma testi"""
        print("\n🚨 ACİL DURDURMA TESTİ")
        print("-" * 40)
        
        if not self.motor_armed:
            if not self.arm_motor():
                return False
        
        # Motoru %50 güce çıkar
        print("🚀 Motor %50 güce çıkarılıyor...")
        test_throttle = ESC_NEUTRAL + 150
        self.send_motor_command(test_throttle)
        time.sleep(3)
        
        print("🚨 ACİL DURDURMA AKTİF!")
        self.emergency_stop = True
        
        # Emergency stop command
        self.send_motor_command(ESC_NEUTRAL)
        
        time.sleep(2)
        
        print("✅ Motor güvenli olarak durduruldu")
        self.emergency_stop = False
        return True
    
    def motor_direction_test(self):
        """Motor yön testi (forward/reverse)"""
        print("\n↔️ MOTOR YÖN TESTİ")
        print("-" * 40)
        
        if not self.motor_armed:
            if not self.arm_motor():
                return False
        
        # Forward test
        print("🔼 FORWARD YÖN TESTİ:")
        forward_values = [1520, 1550, 1600, 1650]
        
        for throttle in forward_values:
            print(f"  ➡️ Forward {throttle}µs")
            self.send_motor_command(throttle)
            time.sleep(3)
        
        # Neutral
        print("  ⏸️ Neutral")
        self.send_motor_command(ESC_NEUTRAL)
        time.sleep(2)
        
        # Reverse test  
        print("\n🔽 REVERSE YÖN TESTİ:")
        reverse_values = [1480, 1450, 1400, 1350]
        
        for throttle in reverse_values:
            print(f"  ⬅️ Reverse {throttle}µs")
            self.send_motor_command(throttle)
            time.sleep(3)
        
        # Neutral'e dön
        self.send_motor_command(ESC_NEUTRAL)
        print("✅ Motor yön testi tamamlandı")
        return True
    
    def run_full_test_suite(self):
        """Tam test paketi"""
        print("🧪 MOTOR KONTROL TAM TEST PAKETİ")
        print("=" * 50)
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("⚠️ GÜVENLIK UYARISI:")
        print("- Motor pervane TAKILI DEĞİL olduğundan emin olun!")
        print("- Test ortamının güvenli olduğunu kontrol edin!")
        print("- Acil durdurma butonuna hazır olun!")
        
        input("\n✅ Güvenlik kontrollerini yaptım, devam et (ENTER):")
        
        try:
            # 1. Motor arming testi
            if not self.motor_arming_test():
                return False
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 2. Throttle tepki testi
            self.throttle_response_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 3. Motor ramping testi
            self.motor_ramp_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 4. Motor yön testi
            self.motor_direction_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 5. Oscillation testi
            self.motor_oscillation_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 6. Acil durdurma testi
            self.emergency_stop_test()
            
            print("\n🎉 TÜM MOTOR TESTLERİ BAŞARILI!")
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            self.emergency_stop = True
            self.send_motor_command(ESC_NEUTRAL)
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik"""
        self.running = False
        self.emergency_stop = True
        
        # Motor stop
        if self.connected:
            self.send_motor_command(ESC_NEUTRAL)
            time.sleep(1)
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    controller = MotorController()
    
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