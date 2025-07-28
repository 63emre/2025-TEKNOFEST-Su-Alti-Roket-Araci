#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - PID Kontrollü Yüzdürme
IMU Tabanlı Otomatik Stabilizasyon ve Pozisyon Kontrolü
HIGHRES_IMU, ATTITUDE, VFR_HUD mesajları kullanılarak gerçek zamanlı kontrol
"""

import time
import json
import threading
import sys
import os
import math
import numpy as np
from collections import deque
from datetime import datetime

# Ana klasörden modülleri import et
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from mavlink_handler import MAVLinkHandler
from pymavlink import mavutil

class PIDController:
    """PID kontrol sınıfı"""
    def __init__(self, kp, ki, kd, output_min=-400, output_max=400):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain  
        self.kd = kd  # Derivative gain
        
        self.output_min = output_min
        self.output_max = output_max
        
        # PID state
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Error history for debugging
        self.error_history = deque(maxlen=100)
        
    def update(self, setpoint, current_value):
        """PID hesaplaması"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.001  # Minimum time step
        
        # Error calculation
        error = setpoint - current_value
        self.error_history.append((current_time, error))
        
        # Proportional term
        proportional = self.kp * error
        
        # Integral term (with windup protection)
        self.integral += error * dt
        # Clamp integral to prevent windup
        integral_limit = 100.0
        self.integral = max(-integral_limit, min(integral_limit, self.integral))
        integral = self.ki * self.integral
        
        # Derivative term
        derivative = self.kd * (error - self.previous_error) / dt
        
        # Calculate output
        output = proportional + integral + derivative
        
        # Clamp output
        output = max(self.output_min, min(self.output_max, output))
        
        # Update for next iteration
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID durumunu sıfırla"""
        self.previous_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.error_history.clear()

class IMUDataProcessor:
    """IMU verilerini işleyen sınıf"""
    def __init__(self):
        # Son IMU verileri
        self.latest_imu = {
            'timestamp': 0,
            'accel': [0, 0, 0],    # m/s²
            'gyro': [0, 0, 0],     # rad/s
            'attitude': [0, 0, 0], # roll, pitch, yaw (rad)
            'groundspeed': 0       # m/s
        }
        
        # IMU filtresi
        self.accel_filter = deque(maxlen=10)
        self.gyro_filter = deque(maxlen=10)
        self.attitude_filter = deque(maxlen=5)
        
        # Thread güvenliği
        self.data_lock = threading.Lock()
        
    def update_imu_data(self, msg_type, msg):
        """IMU verilerini güncelle"""
        with self.data_lock:
            timestamp = time.time()
            
            if msg_type == 'HIGHRES_IMU':
                # Raw sensor data
                accel = [msg.xacc, msg.yacc, msg.zacc]
                gyro = [msg.xgyro, msg.ygyro, msg.zgyro]
                
                self.accel_filter.append(accel)
                self.gyro_filter.append(gyro)
                
                # Filtered values
                if len(self.accel_filter) > 0:
                    self.latest_imu['accel'] = np.mean(self.accel_filter, axis=0).tolist()
                    self.latest_imu['gyro'] = np.mean(self.gyro_filter, axis=0).tolist()
                
            elif msg_type == 'ATTITUDE':
                # Attitude data
                attitude = [msg.roll, msg.pitch, msg.yaw]
                self.attitude_filter.append(attitude)
                
                if len(self.attitude_filter) > 0:
                    self.latest_imu['attitude'] = np.mean(self.attitude_filter, axis=0).tolist()
                    
            elif msg_type == 'VFR_HUD':
                # Speed data
                self.latest_imu['groundspeed'] = msg.groundspeed
            
            self.latest_imu['timestamp'] = timestamp
    
    def get_latest_data(self):
        """En son IMU verilerini al"""
        with self.data_lock:
            return self.latest_imu.copy()
    
    def is_data_fresh(self, max_age=1.0):
        """Veri taze mi kontrol et"""
        with self.data_lock:
            return (time.time() - self.latest_imu['timestamp']) < max_age

class PIDSwimControl:
    def __init__(self):
        """PID kontrollü yüzdürme sistemi"""
        print("🧠 PID KONTROLLÜ YÜZDİRME - IMU TABANII")
        print("=" * 60)
        print("🎯 2x2m Havuz - Otomatik Stabilizasyon")
        print("📊 HIGHRES_IMU + ATTITUDE + VFR_HUD")
        print("🔗 AUX1,3,4,5 (X-Wing) + AUX6 (Motor)")
        print("=" * 60)
        
        # MAVLink bağlantısı
        self.mavlink_handler = MAVLinkHandler()
        self.mavlink_master = None
        self.connected = False
        
        # IMU data processor
        self.imu_processor = IMUDataProcessor()
        
        # X-Wing servo mapping
        self.servos = {
            'front_left': 1,   # AUX1 - Ön Sol Fin
            'front_right': 3,  # AUX3 - Ön Sağ Fin  
            'rear_left': 4,    # AUX4 - Arka Sol Fin
            'rear_right': 5,   # AUX5 - Arka Sağ Fin
            'motor': 6         # AUX6 - Ana Motor
        }
        
        # PID controllers (2x2m havuz için optimize)
        self.pid_controllers = {
            'roll': PIDController(kp=200, ki=20, kd=50),      # Roll stabilizasyonu
            'pitch': PIDController(kp=200, ki=20, kd=50),     # Pitch stabilizasyonu  
            'yaw': PIDController(kp=150, ki=15, kd=30),       # Yaw kontrol
            'depth': PIDController(kp=100, ki=10, kd=25),     # Derinlik kontrol (accel Z)
            'speed': PIDController(kp=80, ki=8, kd=20)        # Hız kontrol
        }
        
        # PWM ayarları
        self.pwm_config = {
            'neutral': 1500,
            'servo_min': 1100,
            'servo_max': 1900,
            'motor_stop': 1500,
            'motor_min': 1000,
            'motor_max': 2000
        }
        
        # Kontrol hedefleri (setpoints)
        self.targets = {
            'roll': 0.0,        # rad (yatay)
            'pitch': 0.0,       # rad (yatay)
            'yaw': 0.0,         # rad (başlangıç yön)
            'depth_accel': -9.8, # m/s² (normal gravity)
            'speed': 0.0        # m/s
        }
        
        # Kontrol durumu
        self.control_active = False
        self.control_mode = "stabilize"  # "stabilize", "navigate", "manual"
        self.control_thread = None
        
        # İstatistikler
        self.control_stats = {
            'loop_count': 0,
            'loop_time_avg': 0.0,
            'last_update': 0
        }
        
    def connect(self):
        """Pixhawk'a bağlan ve IMU stream başlat"""
        print("\n🔌 Pixhawk TCP bağlantısı kuruluyor...")
        
        # MAVLink bağlantısı
        connection_string = "tcp:127.0.0.1:5777"
        try:
            self.mavlink_master = mavutil.mavlink_connection(connection_string)
            self.mavlink_master.wait_heartbeat(timeout=10)
            self.connected = True
            print("✅ MAVLink TCP bağlantısı başarılı!")
            
            # IMU data stream başlat
            self.request_imu_streams()
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def request_imu_streams(self):
        """IMU veri akışlarını başlat"""
        print("📡 IMU veri akışları başlatılıyor...")
        
        # 20 Hz = 50,000 microseconds
        stream_rate = 50000
        
        message_intervals = [
            (mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, stream_rate),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, stream_rate),
            (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, stream_rate),
        ]
        
        for msg_id, interval in message_intervals:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval, 0, 0, 0, 0, 0
            )
        
        print("✅ IMU streams başlatıldı (20 Hz)")
    
    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        try:
            # PWM limitlerini kontrol et
            if channel == self.servos['motor']:
                pwm_value = max(self.pwm_config['motor_min'], 
                              min(self.pwm_config['motor_max'], pwm_value))
            else:
                pwm_value = max(self.pwm_config['servo_min'], 
                              min(self.pwm_config['servo_max'], pwm_value))
            
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,         # Servo channel  
                pwm_value,       # PWM value
                0, 0, 0, 0, 0
            )
            return True
            
        except Exception as e:
            print(f"❌ Servo {channel} PWM hatası: {e}")
            return False
    
    def apply_control_outputs(self, control_outputs):
        """Kontrol çıktılarını servolar üzerinden uygula"""
        
        # X-wing servo mixing
        neutral = self.pwm_config['neutral']
        
        # Control outputs: roll, pitch, yaw, motor
        roll_output = control_outputs.get('roll', 0)
        pitch_output = control_outputs.get('pitch', 0)
        yaw_output = control_outputs.get('yaw', 0)
        motor_output = control_outputs.get('motor', 0)
        
        # X-wing karışım formulü
        front_left_pwm  = neutral + pitch_output + roll_output + yaw_output
        front_right_pwm = neutral + pitch_output - roll_output - yaw_output
        rear_left_pwm   = neutral - pitch_output + roll_output - yaw_output
        rear_right_pwm  = neutral - pitch_output - roll_output + yaw_output
        motor_pwm       = self.pwm_config['motor_stop'] + motor_output
        
        # PWM değerlerini uygula
        self.set_servo_pwm(self.servos['front_left'], int(front_left_pwm))
        self.set_servo_pwm(self.servos['front_right'], int(front_right_pwm))
        self.set_servo_pwm(self.servos['rear_left'], int(rear_left_pwm))
        self.set_servo_pwm(self.servos['rear_right'], int(rear_right_pwm))
        self.set_servo_pwm(self.servos['motor'], int(motor_pwm))
    
    def control_loop(self):
        """Ana PID kontrol döngüsü"""
        print("🔄 PID kontrol döngüsü başlatıldı")
        
        loop_count = 0
        loop_times = deque(maxlen=100)
        
        while self.control_active:
            loop_start = time.time()
            
            try:
                # IMU mesajlarını oku
                msg = self.mavlink_master.recv_match(blocking=False, timeout=0.01)
                if msg:
                    msg_type = msg.get_type()
                    if msg_type in ['HIGHRES_IMU', 'ATTITUDE', 'VFR_HUD']:
                        self.imu_processor.update_imu_data(msg_type, msg)
                
                # IMU verilerini al
                if not self.imu_processor.is_data_fresh(max_age=0.5):
                    print("⚠️ IMU verileri eski - kontrol atlanıyor")
                    time.sleep(0.01)
                    continue
                
                imu_data = self.imu_processor.get_latest_data()
                
                # PID hesaplamaları
                control_outputs = {}
                
                if self.control_mode == "stabilize":
                    # Stabilizasyon modu - sadece attitude kontrolü
                    roll_output = self.pid_controllers['roll'].update(
                        self.targets['roll'], imu_data['attitude'][0])
                    pitch_output = self.pid_controllers['pitch'].update(
                        self.targets['pitch'], imu_data['attitude'][1])
                    yaw_output = self.pid_controllers['yaw'].update(
                        self.targets['yaw'], imu_data['attitude'][2])
                    
                    control_outputs = {
                        'roll': roll_output,
                        'pitch': pitch_output, 
                        'yaw': yaw_output,
                        'motor': 0  # Stabilizasyon modunda motor kapalı
                    }
                    
                elif self.control_mode == "navigate":
                    # Navigasyon modu - position + attitude kontrol
                    roll_output = self.pid_controllers['roll'].update(
                        self.targets['roll'], imu_data['attitude'][0])
                    pitch_output = self.pid_controllers['pitch'].update(
                        self.targets['pitch'], imu_data['attitude'][1])
                    yaw_output = self.pid_controllers['yaw'].update(
                        self.targets['yaw'], imu_data['attitude'][2])
                    
                    # Hız kontrolü motor için
                    speed_output = self.pid_controllers['speed'].update(
                        self.targets['speed'], imu_data['groundspeed'])
                    
                    control_outputs = {
                        'roll': roll_output,
                        'pitch': pitch_output,
                        'yaw': yaw_output,
                        'motor': speed_output
                    }
                
                # Kontrol çıktılarını uygula
                self.apply_control_outputs(control_outputs)
                
                # İstatistikleri güncelle
                loop_count += 1
                loop_time = time.time() - loop_start
                loop_times.append(loop_time)
                
                if loop_count % 100 == 0:  # Her 100 döngüde bir
                    avg_loop_time = np.mean(loop_times)
                    print(f"🔄 Loop #{loop_count}: {avg_loop_time*1000:.1f}ms avg")
                    print(f"   📊 Roll: {imu_data['attitude'][0]:.3f} rad")
                    print(f"   📊 Pitch: {imu_data['attitude'][1]:.3f} rad") 
                    print(f"   📊 Yaw: {imu_data['attitude'][2]:.3f} rad")
                    print(f"   📊 Speed: {imu_data['groundspeed']:.2f} m/s")
                
                # 50Hz kontrol döngüsü (20ms)
                time.sleep(0.02)
                
            except Exception as e:
                print(f"❌ Kontrol döngüsü hatası: {e}")
                time.sleep(0.1)
        
        print("🔄 PID kontrol döngüsü durduruldu")
    
    def start_control(self, mode="stabilize"):
        """PID kontrolünü başlat"""
        if self.control_active:
            print("⚠️ Kontrol zaten aktif!")
            return
        
        self.control_mode = mode
        self.control_active = True
        
        # PID kontrolcülerini sıfırla
        for pid in self.pid_controllers.values():
            pid.reset()
        
        # Kontrol thread'i başlat
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        print(f"✅ PID kontrol başlatıldı - Mod: {mode}")
    
    def stop_control(self):
        """PID kontrolünü durdur"""
        if not self.control_active:
            return
        
        self.control_active = False
        
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        
        # Tüm servoları neutral yap
        self.emergency_stop() 
        
        print("🛑 PID kontrol durduruldu")
    
    def set_target_attitude(self, roll=None, pitch=None, yaw=None):
        """Hedef attitude ayarla"""
        if roll is not None:
            self.targets['roll'] = roll
            print(f"🎯 Hedef Roll: {roll:.3f} rad ({math.degrees(roll):.1f}°)")
        if pitch is not None:
            self.targets['pitch'] = pitch
            print(f"🎯 Hedef Pitch: {pitch:.3f} rad ({math.degrees(pitch):.1f}°)")
        if yaw is not None:
            self.targets['yaw'] = yaw
            print(f"🎯 Hedef Yaw: {yaw:.3f} rad ({math.degrees(yaw):.1f}°)")
    
    def set_target_speed(self, speed):
        """Hedef hız ayarla"""
        self.targets['speed'] = speed
        print(f"🎯 Hedef Hız: {speed:.2f} m/s")
    
    def emergency_stop(self):
        """Acil durdurma"""
        print("\n🚨 PID ACİL DURDURMA!")
        
        neutral = self.pwm_config['neutral']
        motor_stop = self.pwm_config['motor_stop']
        
        # Tüm servoları neutral yap
        for servo_name, channel in self.servos.items():
            if servo_name == 'motor':
                self.set_servo_pwm(channel, motor_stop)
            else:  
                self.set_servo_pwm(channel, neutral)
        
        self.stop_control()
    
    def run_interactive_pid_control(self):
        """İnteraktif PID kontrol"""
        print("\n" + "="*70)
        print("🧠 İNTERAKTİF PID KONTROL")
        print("="*70)
        print("🎯 Komutlar:")
        print("  1 = Stabilizasyon modu başlat")
        print("  2 = Navigasyon modu başlat")  
        print("  3 = Roll hedefi ayarla")
        print("  4 = Pitch hedefi ayarla")
        print("  5 = Yaw hedefi ayarla")
        print("  6 = Hız hedefi ayarla")
        print("  s = Kontrol durdur")
        print("  q = Çıkış")
        print("="*70)
        
        try:
            while True:
                command = input("\n🧠 PID Komut: ").strip().lower()
                
                if command == '1':
                    self.start_control("stabilize")
                elif command == '2':
                    self.start_control("navigate")
                elif command == '3':
                    try:
                        degrees = float(input("Roll hedefi (derece): "))
                        self.set_target_attitude(roll=math.radians(degrees))
                    except ValueError:
                        print("❌ Geçersiz değer!")
                elif command == '4':
                    try:
                        degrees = float(input("Pitch hedefi (derece): "))
                        self.set_target_attitude(pitch=math.radians(degrees))
                    except ValueError:
                        print("❌ Geçersiz değer!")
                elif command == '5':
                    try:
                        degrees = float(input("Yaw hedefi (derece): "))
                        self.set_target_attitude(yaw=math.radians(degrees))
                    except ValueError:
                        print("❌ Geçersiz değer!")
                elif command == '6':
                    try:
                        speed = float(input("Hız hedefi (m/s): "))
                        self.set_target_speed(speed)
                    except ValueError:
                        print("❌ Geçersiz değer!")
                elif command == 's':
                    self.stop_control()
                elif command == 'q':
                    print("👋 PID kontrol modundan çıkılıyor...")
                    self.stop_control()
                    break
                else:
                    print("❌ Geçersiz komut!")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n🚨 Ctrl+C ile durduruldu!")
            self.emergency_stop()
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.connected:
            self.emergency_stop()
            if self.mavlink_master:
                self.mavlink_master.close()
            self.connected = False
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana program"""
    pid_swimmer = PIDSwimControl()
    
    # Bağlantı kur
    if not pid_swimmer.connect():
        print("❌ Bağlantı kurulamadı. Çıkılıyor...")
        return
    
    try:
        while True:
            print("\n" + "="*50)
            print("🧠 PID SWIM CONTROL - MENÜ")
            print("="*50)
            print("1. İnteraktif PID kontrol")
            print("2. Çıkış")
            
            choice = input("\nSeçiminizi yapın (1-2): ").strip()
            
            if choice == '1':
                pid_swimmer.run_interactive_pid_control()
            elif choice == '2':
                break
            else:
                print("❌ Geçersiz seçim!")
                
    except KeyboardInterrupt:
        print("\n👋 Program sonlandırılıyor...")
    finally:
        pid_swimmer.disconnect()

if __name__ == "__main__":
    main() 