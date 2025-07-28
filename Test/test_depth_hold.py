#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Derinlik Tutma Sistemi Testi
Basınç sensörü tabanlı derinlik kontrolü + PID
"""

import time
import threading
import math
from pymavlink import mavutil
import numpy as np

# MAVLink Serial bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
import os
try:
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Using dynamic serial connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to serial config with environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Using fallback serial connection: {MAV_ADDRESS}")

# Motor kanal tanımları
MOTOR_CHANNEL = 8  # Ana itki motoru
DEPTH_SERVO_CHANNEL = 5  # Derinlik kontrol servo (elevator)

# Derinlik PID parametreleri
DEPTH_PID_PARAMS = {
    'kp': 150.0,   # Proportional gain
    'ki': 10.0,    # Integral gain  
    'kd': 50.0,    # Derivative gain
    'max_output': 400,  # Max PWM offset
    'deadband': 0.1     # Derinlik deadband (m)
}

# Basınç-derinlik çevirimi (fresh water)
PRESSURE_TO_DEPTH_RATIO = 0.10197  # 1 hPa ≈ 0.10197 m su derinliği
ATMOSPHERIC_PRESSURE = 1013.25     # Deniz seviyesi basıncı (hPa)

# PWM limitleri
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

class DepthController:
    def __init__(self):
        self.master = None
        self.connected = False
        self.depth_hold_active = False
        
        # PID kontrolcü
        self.depth_pid = None
        
        # Sensör verileri
        self.current_pressure = ATMOSPHERIC_PRESSURE  # hPa
        self.current_depth = 0.0  # metre (su yüzeyinden)
        self.surface_pressure = ATMOSPHERIC_PRESSURE  # Kalibre edilmiş yüzey basıncı
        
        # Hedef ve kontrol
        self.target_depth = 0.0  # metre
        self.motor_base_throttle = PWM_NEUTRAL  # Base motor throttle
        self.depth_control_output = 0  # Derinlik kontrolü PWM offset
        
        # Threading
        self.control_thread = None
        self.sensor_thread = None
        self.running = False
        
        # Test verileri
        self.depth_log = []
        self.control_log = []
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor...")
            
            # Handle serial vs TCP connection
            if ',' in MAV_ADDRESS:
                # Serial connection: port,baud
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                # TCP or other connection
                print(f"🌐 TCP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # PID kontrolcü başlat
            self.depth_pid = PIDController(
                DEPTH_PID_PARAMS['kp'],
                DEPTH_PID_PARAMS['ki'], 
                DEPTH_PID_PARAMS['kd'],
                DEPTH_PID_PARAMS['max_output']
            )
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def read_pressure_sensor(self):
        """Basınç sensöründen veri oku"""
        if not self.connected:
            return False
            
        try:
            msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if msg:
                self.current_pressure = msg.press_abs  # Absolüt basınç (hPa)
                
                # Derinlik hesapla (yüzey basıncından fark)
                pressure_diff = self.current_pressure - self.surface_pressure
                self.current_depth = max(0, pressure_diff * PRESSURE_TO_DEPTH_RATIO)
                
                # Log kaydet
                timestamp = time.time()
                self.depth_log.append({
                    'time': timestamp,
                    'pressure': self.current_pressure,
                    'depth': self.current_depth,
                    'target_depth': self.target_depth
                })
                
                return True
        except Exception as e:
            print(f"❌ Basınç sensörü okuma hatası: {e}")
            
        return False
    
    def calibrate_surface_pressure(self, duration=10):
        """Yüzey basıncını kalibre et"""
        print(f"🔧 Yüzey basıncı kalibrasyonu ({duration}s)...")
        
        pressure_samples = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            if self.read_pressure_sensor():
                pressure_samples.append(self.current_pressure)
            time.sleep(0.1)
        
        if pressure_samples:
            self.surface_pressure = np.mean(pressure_samples)
            pressure_std = np.std(pressure_samples)
            
            print(f"✅ Yüzey basıncı: {self.surface_pressure:.2f} ± {pressure_std:.2f} hPa")
            return True
        else:
            print("❌ Basınç verisi alınamadı!")
            return False
    
    def set_motor_throttle(self, throttle_pwm):
        """Motor throttle ayarla"""
        if not self.connected:
            return False
            
        throttle_pwm = max(PWM_MIN, min(PWM_MAX, throttle_pwm))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL, throttle_pwm, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Motor kontrolü hatası: {e}")
            return False
    
    def set_depth_servo(self, servo_pwm):
        """Derinlik kontrol servo ayarla"""
        if not self.connected:
            return False
            
        servo_pwm = max(PWM_MIN, min(PWM_MAX, servo_pwm))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                DEPTH_SERVO_CHANNEL, servo_pwm, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Derinlik servo kontrolü hatası: {e}")
            return False
    
    def depth_control_loop(self):
        """Ana derinlik kontrol döngüsü"""
        print("🔄 Derinlik kontrol döngüsü başladı")
        
        while self.running and self.depth_hold_active:
            
            # 1. Basınç/derinlik verisini oku
            if self.read_pressure_sensor():
                
                # 2. Derinlik hatası kontrolü
                depth_error = abs(self.current_depth - self.target_depth)
                
                if depth_error > DEPTH_PID_PARAMS['deadband']:
                    # 3. PID hesaplama
                    self.depth_control_output = self.depth_pid.update(
                        self.target_depth, self.current_depth
                    )
                    
                    # 4. Motor throttle ayarla (vertical thrust)
                    motor_throttle = self.motor_base_throttle + self.depth_control_output
                    self.set_motor_throttle(motor_throttle)
                    
                    # 5. Depth servo ayarla (elevator control)
                    servo_output = PWM_NEUTRAL - (self.depth_control_output // 2)
                    self.set_depth_servo(servo_output)
                    
                else:
                    # Deadband içinde - motor neutral
                    self.set_motor_throttle(self.motor_base_throttle)
                    self.set_depth_servo(PWM_NEUTRAL)
                    self.depth_control_output = 0
                
                # 6. Log kaydet
                timestamp = time.time()
                self.control_log.append({
                    'time': timestamp,
                    'depth_error': self.current_depth - self.target_depth,
                    'control_output': self.depth_control_output,
                    'motor_throttle': self.motor_base_throttle + self.depth_control_output
                })
                
                # 7. Debug bilgi (her 2 saniyede bir)
                if len(self.depth_log) % 20 == 0:  # 10Hz'de çalışıyorsa 2 saniye
                    print(f"  📊 Depth: {self.current_depth:.2f}m -> Target: {self.target_depth:.2f}m")
                    print(f"      Error: {self.current_depth - self.target_depth:+.2f}m, Output: {self.depth_control_output:+d}")
            
            time.sleep(0.1)  # 10Hz kontrol döngüsü
        
        print("⏹️ Derinlik kontrol döngüsü durdu")
    
    def start_depth_hold(self, target_depth, base_throttle=PWM_NEUTRAL):
        """Derinlik tutmayı başlat"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
        
        if self.depth_hold_active:
            print("⚠️ Derinlik tutma zaten aktif!")
            return True
        
        self.target_depth = target_depth
        self.motor_base_throttle = base_throttle
        
        # PID reset
        self.depth_pid.reset()
        
        # Log temizle
        self.depth_log.clear()
        self.control_log.clear()
        
        self.depth_hold_active = True
        self.running = True
        
        # Kontrol thread başlat
        self.control_thread = threading.Thread(target=self.depth_control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print(f"✅ Derinlik tutma aktif - Hedef: {target_depth:.1f}m")
        return True
    
    def stop_depth_hold(self):
        """Derinlik tutmayı durdur"""
        self.depth_hold_active = False
        self.running = False
        
        # Motorları neutral position
        self.set_motor_throttle(PWM_NEUTRAL)
        self.set_depth_servo(PWM_NEUTRAL)
        
        print("⏹️ Derinlik tutma durduruldu")
    
    def change_target_depth(self, new_target):
        """Hedef derinliği değiştir"""
        old_target = self.target_depth
        self.target_depth = new_target
        print(f"🎯 Hedef derinlik değişti: {old_target:.1f}m -> {new_target:.1f}m")
    
    def surface_calibration_test(self):
        """Yüzey kalibrasyonu testi"""
        print("\n🔧 YÜZEY KALİBRASYONU TESTİ")
        print("-" * 40)
        
        if not self.calibrate_surface_pressure(15):
            return False
        
        print("✅ Yüzey kalibrasyonu başarılı")
        return True
    
    def depth_accuracy_test(self):
        """Derinlik doğruluğu testi"""
        print("\n📏 DERİNLİK DOĞRULUĞU TESTİ")
        print("-" * 40)
        
        # Test derinlikleri
        test_depths = [0.5, 1.0, 1.5, 2.0]
        
        for target_depth in test_depths:
            print(f"\n🎯 Hedef derinlik: {target_depth}m")
            
            if not self.start_depth_hold(target_depth, PWM_NEUTRAL + 50):  # Hafif forward thrust
                continue
            
            # 30 saniye bekle (settling time)
            settle_time = 30
            start_time = time.time()
            
            depth_errors = []
            
            while time.time() - start_time < settle_time:
                if self.depth_log:
                    latest = self.depth_log[-1]
                    error = abs(latest['depth'] - target_depth)
                    depth_errors.append(error)
                    
                    print(f"  📊 Depth: {latest['depth']:.2f}m, Error: {error:.2f}m")
                
                time.sleep(2)
            
            self.stop_depth_hold()
            
            if depth_errors:
                avg_error = np.mean(depth_errors[-10:])  # Son 10 ölçümün ortalaması
                max_error = np.max(depth_errors[-10:])
                
                print(f"  📈 Ortalama hata: {avg_error:.2f}m, Max hata: {max_error:.2f}m")
                
                if avg_error < 0.1:
                    print(f"  ✅ Derinlik doğruluğu: İYİ")
                elif avg_error < 0.2:
                    print(f"  ⚠️ Derinlik doğruluğu: ORTA") 
                else:
                    print(f"  ❌ Derinlik doğruluğu: KÖTÜ")
            
            time.sleep(5)  # Sonraki teste geçmeden önce bekle
        
        print("✅ Derinlik doğruluğu testi tamamlandı")
        return True
    
    def depth_response_test(self):
        """Derinlik tepki testi"""
        print("\n⚡ DERİNLİK TEPKİ TESTİ")
        print("-" * 40)
        
        # Başlangıç derinliği
        initial_depth = 1.0
        print(f"📍 Başlangıç derinliği: {initial_depth}m")
        
        if not self.start_depth_hold(initial_depth, PWM_NEUTRAL + 100):
            return False
        
        # 15 saniye settle
        print("⏱️ Başlangıç derinliğine settle (15s)...")
        time.sleep(15)
        
        # Step inputs
        step_tests = [
            (1.5, "0.5m derinlik artış"),
            (0.5, "1.0m derinlik azalış"), 
            (2.0, "1.5m derinlik artış"),
            (1.0, "1.0m geri dönüş")
        ]
        
        for new_depth, description in step_tests:
            print(f"\n📈 {description}")
            
            step_start_time = time.time()
            self.change_target_depth(new_depth)
            
            # 20 saniye response bekle
            response_time = 20
            max_overshoot = 0
            settling_error = float('inf')
            
            while time.time() - step_start_time < response_time:
                if self.depth_log:
                    latest = self.depth_log[-1]
                    current_error = abs(latest['depth'] - new_depth)
                    
                    # Overshoot kontrolü
                    if latest['depth'] > new_depth:
                        overshoot = latest['depth'] - new_depth
                        max_overshoot = max(max_overshoot, overshoot)
                    
                    # Son 5 saniyede settling error
                    if time.time() - step_start_time > response_time - 5:
                        settling_error = min(settling_error, current_error)
                    
                    print(f"  📊 Depth: {latest['depth']:.2f}m -> Target: {new_depth:.2f}m (Error: {current_error:.2f}m)")
                
                time.sleep(2)
            
            print(f"  📈 Max overshoot: {max_overshoot:.2f}m")
            print(f"  📉 Settling error: {settling_error:.2f}m")
        
        self.stop_depth_hold()
        print("✅ Derinlik tepki testi tamamlandı")
        return True
    
    def depth_disturbance_test(self):
        """Derinlik bozucu etki testi"""
        print("\n🌊 DERİNLİK BOZUCU ETKİ TESTİ")
        print("-" * 40)
        
        # 1.5m sabit derinlik tut
        target_depth = 1.5
        
        if not self.start_depth_hold(target_depth, PWM_NEUTRAL + 80):
            return False
        
        # 10 saniye steady state
        print(f"📐 Steady state {target_depth}m (10s)...")
        time.sleep(10)
        
        # Bozucu etkiler simüle et
        disturbances = [
            ("💨 Yukarı itki bozucu", PWM_NEUTRAL + 200, 8),
            ("⬇️ Aşağı itki bozucu", PWM_NEUTRAL - 100, 8),
            ("🔄 Motor power cycling", None, 10)
        ]
        
        for description, disturbance_throttle, duration in disturbances:
            print(f"\n{description} ({duration}s)")
            
            if disturbance_throttle is not None:
                # Sabit bozucu etki
                disturb_start = time.time()
                while time.time() - disturb_start < duration:
                    # Normal PID'e ek olarak bozucu etki uygula
                    total_throttle = self.motor_base_throttle + self.depth_control_output + (disturbance_throttle - PWM_NEUTRAL)
                    self.set_motor_throttle(total_throttle)
                    time.sleep(0.5)
                    
                    if self.depth_log:
                        latest = self.depth_log[-1]
                        error = abs(latest['depth'] - target_depth)
                        print(f"    📊 Disturbance error: {error:.2f}m")
            
            else:
                # Power cycling bozucu
                for _ in range(duration):
                    # Motor off
                    self.set_motor_throttle(PWM_NEUTRAL)
                    time.sleep(0.5)
                    
                    # Motor on (PID'in hesapladığı değer)
                    normal_throttle = self.motor_base_throttle + self.depth_control_output
                    self.set_motor_throttle(normal_throttle)
                    time.sleep(0.5)
                    
                    if self.depth_log:
                        latest = self.depth_log[-1]
                        error = abs(latest['depth'] - target_depth)
                        print(f"    📊 Cycling error: {error:.2f}m")
            
            print(f"  ↩️ Bozucu etki kaldırıldı, recovery bekle (10s)...")
            
            # Recovery takip et
            recovery_start = time.time()
            while time.time() - recovery_start < 10:
                if self.depth_log:
                    latest = self.depth_log[-1]
                    recovery_error = abs(latest['depth'] - target_depth)
                    print(f"    📊 Recovery error: {recovery_error:.2f}m")
                time.sleep(2)
        
        self.stop_depth_hold()
        print("✅ Derinlik bozucu etki testi tamamlandı")
        return True
    
    def generate_test_report(self):
        """Test raporu oluştur"""
        if not self.depth_log or not self.control_log:
            print("❌ Test verisi bulunamadı!")
            return
            
        print("\n📋 DERİNLİK KONTROL TEST RAPORU")
        print("=" * 50)
        
        # Son 100 sample üzerinden istatistik
        recent_depth_data = self.depth_log[-100:] if len(self.depth_log) > 100 else self.depth_log
        recent_control_data = self.control_log[-100:] if len(self.control_log) > 100 else self.control_log
        
        if recent_depth_data:
            depth_errors = [abs(data['depth'] - data['target_depth']) for data in recent_depth_data]
            depths = [data['depth'] for data in recent_depth_data]
            
            avg_error = np.mean(depth_errors)
            max_error = np.max(depth_errors)
            depth_stability = np.std(depths)
            
            print(f"📊 PERFORMANS METRİKLERİ:")
            print(f"  Ortalama Derinlik Hatası: {avg_error:.3f}m")
            print(f"  Maksimum Derinlik Hatası: {max_error:.3f}m")
            print(f"  Derinlik Stabilitesi (σ): {depth_stability:.3f}m")
        
        if recent_control_data:
            control_outputs = [abs(data['control_output']) for data in recent_control_data]
            avg_control_effort = np.mean(control_outputs)
            
            print(f"  Ortalama Kontrol Çabası: {avg_control_effort:.1f} PWM units")
        
        # Performans değerlendirmesi
        if avg_error < 0.1:
            print(f"✅ DERİNLİK KONTROL PERFORMANSI: MÜKEMİMEL")
        elif avg_error < 0.2:
            print(f"✅ DERİNLİK KONTROL PERFORMANSI: İYİ")
        elif avg_error < 0.3:
            print(f"⚠️ DERİNLİK KONTROL PERFORMANSI: ORTA")
        else:
            print(f"❌ DERİNLİK KONTROL PERFORMANSI: KÖTÜ")
    
    def run_full_test_suite(self):
        """Tam test paketi"""
        print("🧪 DERİNLİK KONTROL TAM TEST PAKETİ")
        print("=" * 50)
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("⚠️ GÜVENLİK UYARISI:")
        print("- Test ortamının güvenli derinlikte olduğunu kontrol edin!")
        print("- Acil yüzey çıkış prosedürünü hazır tutun!")
        print("- Basınç sensörünün çalıştığından emin olun!")
        
        input("\n✅ Güvenlik kontrollerini yaptım, devam et (ENTER):")
        
        try:
            # 1. Yüzey kalibrasyonu
            if not self.surface_calibration_test():
                return False
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 2. Derinlik doğruluğu testi
            self.depth_accuracy_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 3. Derinlik tepki testi
            self.depth_response_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 4. Bozucu etki testi
            self.depth_disturbance_test()
            
            # 5. Test raporu
            self.generate_test_report()
            
            print("\n🎉 TÜM DERİNLİK KONTROL TESTLERİ TAMAMLANDI!")
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
        self.stop_depth_hold()
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

class PIDController:
    """Basit PID kontrolcü sınıfı"""
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01
        
        error = setpoint - measurement
        
        self.integral += error * dt
        integral_limit = self.max_output / self.ki if self.ki > 0 else float('inf')
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        
        derivative = (error - self.previous_error) / dt
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.max_output, min(self.max_output, output))
        
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

def main():
    """Ana fonksiyon"""
    controller = DepthController()
    
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