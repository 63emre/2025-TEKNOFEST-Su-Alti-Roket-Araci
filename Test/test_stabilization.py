#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Stabilizasyon Algoritması Testi
PID kontrolcü + IMU sensör tabanlı stabilizasyon
"""

import time
import threading
import math
from pymavlink import mavutil
import numpy as np

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

# Servo kanalları (X-konfigürasyon fin kontrolü)
SERVO_CHANNELS = {
    'fin_front_left': 1,   # Ön sol fin (AUX 1)
    'fin_front_right': 2,  # Ön sağ fin (AUX 2)  
    'fin_rear_left': 3,    # Arka sol fin (AUX 3)
    'fin_rear_right': 4    # Arka sağ fin (AUX 4)
}

# X-Konfigürasyon Kontrol Matrisi
FIN_MATRIX = {
    'roll_positive': ['fin_front_left', 'fin_rear_left'],    # Sol finler
    'roll_negative': ['fin_front_right', 'fin_rear_right'],  # Sağ finler
    'pitch_positive': ['fin_front_left', 'fin_front_right'], # Ön finler
    'pitch_negative': ['fin_rear_left', 'fin_rear_right'],   # Arka finler
    'yaw_positive': ['fin_front_left', 'fin_rear_right'],    # X-Diagonal 1
    'yaw_negative': ['fin_front_right', 'fin_rear_left']     # X-Diagonal 2
}

# PID kontrolcü parametreleri
PID_PARAMS = {
    'roll': {'kp': 2.0, 'ki': 0.1, 'kd': 0.3, 'max_output': 500},
    'pitch': {'kp': 2.0, 'ki': 0.1, 'kd': 0.3, 'max_output': 500},
    'yaw': {'kp': 1.5, 'ki': 0.05, 'kd': 0.2, 'max_output': 500}
}

# PWM limitleri
PWM_MID = 1500
PWM_MIN = 1000
PWM_MAX = 2000

class PIDController:
    def __init__(self, kp, ki, kd, max_output=500):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        """PID kontrolcü güncelleme"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01  # Minimum dt
        
        # Hata hesaplama
        error = setpoint - measurement
        
        # Integral terimi
        self.integral += error * dt
        
        # Integral windup koruma
        integral_limit = self.max_output / self.ki if self.ki > 0 else float('inf')
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        
        # Derivative terimi  
        derivative = (error - self.previous_error) / dt
        
        # PID çıkışı
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        
        # Çıkış limitleri
        output = max(-self.max_output, min(self.max_output, output))
        
        # Sonraki iterasyon için sakla
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID reset"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

class StabilizationController:
    def __init__(self):
        self.master = None
        self.connected = False
        self.stabilization_active = False
        
        # PID kontrolcüleri
        self.pid_roll = PIDController(**PID_PARAMS['roll'])
        self.pid_pitch = PIDController(**PID_PARAMS['pitch'])
        self.pid_yaw = PIDController(**PID_PARAMS['yaw'])
        
        # Hedef değerler (derece)
        self.target_roll = 0.0
        self.target_pitch = 0.0  
        self.target_yaw = 0.0
        
        # Mevcut attitude (derece)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
        # Threading
        self.control_thread = None
        self.sensor_thread = None
        self.running = False
        
        # Test verileri
        self.attitude_log = []
        self.control_log = []
        
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
    
    def read_attitude(self):
        """IMU'dan attitude oku"""
        if not self.connected:
            return False
            
        try:
            msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if msg:
                # Radyan'dan dereceye çevir
                self.current_roll = math.degrees(msg.roll)
                self.current_pitch = math.degrees(msg.pitch)
                self.current_yaw = math.degrees(msg.yaw)
                
                # Log kaydet
                timestamp = time.time()
                self.attitude_log.append({
                    'time': timestamp,
                    'roll': self.current_roll,
                    'pitch': self.current_pitch,
                    'yaw': self.current_yaw
                })
                
                return True
        except Exception as e:
            print(f"❌ Attitude okuma hatası: {e}")
            
        return False
    
    def set_servo_position(self, channel, pwm_value):
        """Servo pozisyon ayarlama"""
        if not self.connected:
            return False
            
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm_value, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Servo {channel} kontrol hatası: {e}")
            return False
    
    def apply_control_outputs(self, roll_output, pitch_output, yaw_output):
        """Kontrol çıkışlarını X-konfigürasyon servo pozisyonlarına çevir"""
        
        # X-Konfigürasyon Fin Mixing
        # Roll kontrolü: Sol finler vs Sağ finler (çapraz differential)
        # Pitch kontrolü: Ön finler vs Arka finler (ön/arka differential)  
        # Yaw kontrolü: X-Diagonal kontrol (köşegen finler)
        
        fin_outputs = {
            # X-konfigürasyonda her fin, roll, pitch ve yaw'a katkı sağlar
            'fin_front_left':  PWM_MID - roll_output - pitch_output - yaw_output,  # Sol+Ön+X-Diag1
            'fin_front_right': PWM_MID + roll_output - pitch_output + yaw_output,  # Sağ+Ön+X-Diag2  
            'fin_rear_left':   PWM_MID - roll_output + pitch_output + yaw_output,  # Sol+Arka+X-Diag2
            'fin_rear_right':  PWM_MID + roll_output + pitch_output - yaw_output   # Sağ+Arka+X-Diag1
        }
        
        # Servo komutları gönder
        success_count = 0
        for fin_name, pwm_value in fin_outputs.items():
            channel = SERVO_CHANNELS[fin_name]
            if self.set_servo_position(channel, int(pwm_value)):
                success_count += 1
        
        # Log kaydet
        timestamp = time.time()
        self.control_log.append({
            'time': timestamp,
            'roll_output': roll_output,
            'pitch_output': pitch_output, 
            'yaw_output': yaw_output,
            'fin_outputs': fin_outputs.copy()
        })
        
        return success_count == 4
    
    def stabilization_loop(self):
        """Ana stabilizasyon döngüsü"""
        print("🔄 Stabilizasyon döngüsü başladı")
        
        while self.running and self.stabilization_active:
            
            # 1. Sensör verilerini oku
            if self.read_attitude():
                
                # 2. PID hesaplamaları
                roll_output = self.pid_roll.update(self.target_roll, self.current_roll)
                pitch_output = self.pid_pitch.update(self.target_pitch, self.current_pitch)
                yaw_output = self.pid_yaw.update(self.target_yaw, self.current_yaw)
                
                # 3. Kontrol çıkışlarını uygula
                self.apply_control_outputs(roll_output, pitch_output, yaw_output)
                
                # 4. Debug bilgi (her 1 saniyede bir)
                if len(self.attitude_log) % 20 == 0:  # 20Hz'de çalışıyorsa 1 saniye
                    print(f"  📊 Roll: {self.current_roll:+6.1f}° -> {roll_output:+6.1f}")
                    print(f"      Pitch: {self.current_pitch:+6.1f}° -> {pitch_output:+6.1f}")
                    print(f"      Yaw: {self.current_yaw:+6.1f}° -> {yaw_output:+6.1f}")
            
            time.sleep(0.05)  # 20Hz kontrol döngüsü
        
        print("⏹️ Stabilizasyon döngüsü durdu")
    
    def start_stabilization(self):
        """Stabilizasyon başlat"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
        
        if self.stabilization_active:
            print("⚠️ Stabilizasyon zaten aktif!")
            return True
        
        # PID kontrolcüleri reset
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
        
        # Log temizle
        self.attitude_log.clear()
        self.control_log.clear()
        
        self.stabilization_active = True
        self.running = True
        
        # Kontrol thread başlat
        self.control_thread = threading.Thread(target=self.stabilization_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print("✅ Stabilizasyon aktif edildi!")
        return True
    
    def stop_stabilization(self):
        """Stabilizasyon durdur"""
        self.stabilization_active = False
        self.running = False
        
        # Tüm servoları orta pozisyona al
        for channel in SERVO_CHANNELS.values():
            self.set_servo_position(channel, PWM_MID)
        
        print("⏹️ Stabilizasyon durduruldu")
    
    def set_target_attitude(self, roll=None, pitch=None, yaw=None):
        """Hedef attitude ayarla"""
        if roll is not None:
            self.target_roll = roll
        if pitch is not None:
            self.target_pitch = pitch
        if yaw is not None:
            self.target_yaw = yaw
            
        print(f"🎯 Hedef attitude: Roll={self.target_roll:.1f}°, Pitch={self.target_pitch:.1f}°, Yaw={self.target_yaw:.1f}°")
    
    def stability_test(self):
        """Stabilite testi"""
        print("\n⚖️ STABİLİTE TESTİ")
        print("-" * 40)
        
        if not self.start_stabilization():
            return False
        
        # 30 saniye level flight
        print("📐 Level flight (30s) - tüm eksenlerde 0°")
        self.set_target_attitude(0, 0, 0)
        
        start_time = time.time()
        while time.time() - start_time < 30:
            time.sleep(1)
            
            if self.attitude_log:
                latest = self.attitude_log[-1]
                error_roll = abs(latest['roll'] - self.target_roll)
                error_pitch = abs(latest['pitch'] - self.target_pitch)
                
                print(f"  📊 Error - Roll: {error_roll:.2f}°, Pitch: {error_pitch:.2f}°")
        
        self.stop_stabilization()
        print("✅ Stabilite testi tamamlandı")
        return True
    
    def attitude_response_test(self):
        """Attitude response testi"""
        print("\n🎯 ATTITUDE RESPONSE TESTİ")
        print("-" * 40)
        
        if not self.start_stabilization():
            return False
        
        # Test secimi
        test_sequences = [
            (10, 0, 0, "10° ROLL"),
            (-10, 0, 0, "-10° ROLL"),
            (0, 15, 0, "15° PITCH UP"),
            (0, -15, 0, "15° PITCH DOWN"),
            (0, 0, 30, "30° YAW RIGHT"),
            (0, 0, -30, "30° YAW LEFT"),
            (0, 0, 0, "RETURN TO LEVEL")
        ]
        
        for target_roll, target_pitch, target_yaw, description in test_sequences:
            print(f"\n🎯 {description}")
            self.set_target_attitude(target_roll, target_pitch, target_yaw)
            
            # 15 saniye bekle (settle time)
            settling_time = 15
            start_time = time.time()
            
            while time.time() - start_time < settling_time:
                if self.attitude_log:
                    latest = self.attitude_log[-1]
                    error_roll = latest['roll'] - target_roll
                    error_pitch = latest['pitch'] - target_pitch
                    error_yaw = latest['yaw'] - target_yaw
                    
                    total_error = math.sqrt(error_roll**2 + error_pitch**2 + error_yaw**2)
                    
                    print(f"  📊 Total Error: {total_error:.2f}° (R:{error_roll:+.1f}° P:{error_pitch:+.1f}° Y:{error_yaw:+.1f}°)")
                
                time.sleep(2)
        
        self.stop_stabilization()
        print("✅ Attitude response testi tamamlandı")
        return True
    
    def disturbance_rejection_test(self):
        """Bozucu etki reddi testi"""
        print("\n🌊 DISTURBANCE REJECTION TESTİ")
        print("-" * 40)
        
        if not self.start_stabilization():
            return False
        
        # Level flight hedefi
        self.set_target_attitude(0, 0, 0)
        
        print("📐 Level flight hedefinde (10s)...")
        time.sleep(10)
        
        # Simüle edilmiş bozucu etkiler
        disturbances = [
            ("💨 Simüle edilmiş yan rüzgar", {'fin_right': PWM_MID + 200, 'fin_left': PWM_MID - 200}),
            ("🌊 Simüle edilmiş akıntı etkisi", {'fin_top': PWM_MID + 150, 'fin_bottom': PWM_MID - 150}),
            ("🔄 Simüle edilmiş rotasyon etkisi", {'fin_right': PWM_MID + 100, 'fin_left': PWM_MID + 100})
        ]
        
        for description, fin_disturbance in disturbances:
            print(f"\n{description}")
            
            # 5 saniye bozucu etki uygula
            for _ in range(50):  # 0.1s x 50 = 5s
                for fin_name, pwm_offset in fin_disturbance.items():
                    channel = SERVO_CHANNELS[fin_name]
                    self.set_servo_position(channel, pwm_offset)
                time.sleep(0.1)
            
            print("  ↩️ Bozucu etki kaldırıldı, recovery bekle...")
            
            # 10 saniye recovery
            recovery_start = time.time()
            while time.time() - recovery_start < 10:
                if self.attitude_log:
                    latest = self.attitude_log[-1]
                    total_error = math.sqrt(latest['roll']**2 + latest['pitch']**2)
                    print(f"    📊 Recovery error: {total_error:.2f}°")
                time.sleep(2)
        
        self.stop_stabilization()
        print("✅ Disturbance rejection testi tamamlandı")
        return True
    
    def generate_test_report(self):
        """Test raporu oluştur"""
        if not self.attitude_log or not self.control_log:
            print("❌ Test verisi bulunamadı!")
            return
            
        print("\n📋 STABİLİZASYON TEST RAPORU")
        print("=" * 50)
        
        # İstatistiksel analiz
        roll_errors = [abs(data['roll']) for data in self.attitude_log[-100:]]  # Son 100 sample
        pitch_errors = [abs(data['pitch']) for data in self.attitude_log[-100:]]
        
        avg_roll_error = np.mean(roll_errors)
        max_roll_error = np.max(roll_errors)
        avg_pitch_error = np.mean(pitch_errors)
        max_pitch_error = np.max(pitch_errors)
        
        print(f"📊 PERFORMANS METRİKLERİ:")
        print(f"  Roll - Ortalama Hata: {avg_roll_error:.2f}°, Maksimum: {max_roll_error:.2f}°")
        print(f"  Pitch - Ortalama Hata: {avg_pitch_error:.2f}°, Maksimum: {max_pitch_error:.2f}°")
        
        # Kontrol activity
        control_activity = [abs(data['roll_output']) + abs(data['pitch_output']) + abs(data['yaw_output']) for data in self.control_log[-100:]]
        avg_control_activity = np.mean(control_activity)
        
        print(f"  Kontrol Aktivitesi: {avg_control_activity:.1f} PWM units")
        
        # Stabilite değerlendirmesi
        if avg_roll_error < 2.0 and avg_pitch_error < 2.0:
            print(f"✅ STABİLİZASYON PERFORMANSI: İYİ")
        elif avg_roll_error < 5.0 and avg_pitch_error < 5.0:
            print(f"⚠️ STABİLİZASYON PERFORMANSI: ORTA")
        else:
            print(f"❌ STABİLİZASYON PERFORMANSI: KÖTÜ")
    
    def run_full_test_suite(self):
        """Tam test paketi"""
        print("🧪 STABİLİZASYON TAM TEST PAKETİ")
        print("=" * 50)
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        try:
            # 1. Stabilite testi
            self.stability_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 2. Attitude response testi  
            self.attitude_response_test()
            
            input("\n⏸️ Devam etmek için ENTER'a basın...")
            
            # 3. Disturbance rejection testi
            self.disturbance_rejection_test()
            
            # 4. Rapor oluştur
            self.generate_test_report()
            
            print("\n🎉 TÜM STABİLİZASYON TESTLERİ TAMAMLANDI!")
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
        self.stop_stabilization()
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    controller = StabilizationController()
    
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