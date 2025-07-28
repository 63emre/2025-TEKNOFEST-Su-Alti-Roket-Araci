#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - IMU Dead Reckoning
IMU verilerinden pozisyon tracking sistemi
HIGHRES_IMU + ATTITUDE verilerini kullanarak pozisyon tahmini
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
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation

# Ana klasörden modülleri import et
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from pymavlink import mavutil

class IMUDeadReckoning:
    def __init__(self):
        """IMU tabanlı pozisyon tracking sistemi"""
        print("📍 IMU DEAD RECKONING - POZİSYON TRACKİNG")
        print("=" * 60)
        print("🎯 HIGHRES_IMU + ATTITUDE → Pozisyon Tahmini")
        print("📊 3D Pozisyon, Hız ve Rotasyon Tracking")
        print("🌊 Su altı ortamda GPS olmadan navigasyon")
        print("=" * 60)
        
        # MAVLink bağlantısı
        self.mavlink_master = None
        self.connected = False
        
        # IMU verileri
        self.imu_data = {
            'timestamp': 0,
            'accel': np.array([0.0, 0.0, 0.0]),      # m/s² (body frame)
            'gyro': np.array([0.0, 0.0, 0.0]),       # rad/s (body frame)
            'attitude': np.array([0.0, 0.0, 0.0]),   # roll, pitch, yaw (rad)
            'mag': np.array([0.0, 0.0, 0.0])         # magnetometer (optional)
        }
        
        # Dead reckoning state (NED frame - North East Down)
        self.dr_state = {
            'position': np.array([0.0, 0.0, 0.0]),   # m (North, East, Down)
            'velocity': np.array([0.0, 0.0, 0.0]),   # m/s (North, East, Down)
            'attitude': np.array([0.0, 0.0, 0.0]),   # rad (roll, pitch, yaw)
            'timestamp': time.time()
        }
        
        # Kalman filter için state ve covariance
        self.kf_state = np.zeros(9)  # [pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_bias_x, acc_bias_y, acc_bias_z]
        self.kf_covariance = np.eye(9) * 0.1
        
        # Sensor bias ve kalibrasyon
        self.accel_bias = np.array([0.0, 0.0, 0.0])
        self.gyro_bias = np.array([0.0, 0.0, 0.0])
        self.gravity_vector = np.array([0.0, 0.0, 9.81])  # NED frame
        
        # Trajectory history
        self.position_history = deque(maxlen=1000)
        self.velocity_history = deque(maxlen=1000)
        self.attitude_history = deque(maxlen=1000)
        
        # Threading
        self.tracking_active = False
        self.tracking_thread = None
        self.data_lock = threading.Lock()
        
        # Statistics
        self.stats = {
            'total_distance': 0.0,
            'max_speed': 0.0,
            'tracking_duration': 0.0,
            'update_count': 0
        }
        
        # Koordinat referansı (başlangıç noktası)
        self.reference_point = {
            'position': np.array([0.0, 0.0, 0.0]),
            'attitude': np.array([0.0, 0.0, 0.0]),
            'timestamp': time.time()
        }
        
    def connect(self):
        """Pixhawk'a bağlan ve IMU stream başlat"""
        print("\n🔌 Pixhawk TCP bağlantısı kuruluyor...")
        
        # Serial MAVLink connection with environment variable support
import os
serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
baud_rate = int(os.getenv("MAV_BAUD", "115200"))
connection_string = f"{serial_port},{baud_rate}"
        try:
            self.mavlink_master = mavutil.mavlink_connection(connection_string)
            self.mavlink_master.wait_heartbeat(timeout=10)
            self.connected = True
            print("✅ MAVLink TCP bağlantısı başarılı!")
            
            # IMU stream başlat
            self.request_imu_streams()
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def request_imu_streams(self):
        """IMU veri akışlarını başlat"""
        print("📡 IMU veri akışları başlatılıyor...")
        
        # 50 Hz = 20,000 microseconds
        stream_rate = 20000
        
        message_intervals = [
            (mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, stream_rate),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, stream_rate),
        ]
        
        for msg_id, interval in message_intervals:
            self.mavlink_master.mav.command_long_send(
                self.mavlink_master.target_system,
                self.mavlink_master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, msg_id, interval, 0, 0, 0, 0, 0
            )
        
        print("✅ IMU streams başlatıldı (50 Hz)")
    
    def rotation_matrix_from_euler(self, roll, pitch, yaw):
        """Euler açılarından rotation matrix hesapla"""
        # ZYX rotation sequence (yaw-pitch-roll)
        R = Rotation.from_euler('ZYX', [yaw, pitch, roll])
        return R.as_matrix()
    
    def body_to_ned_transform(self, body_vector, attitude):
        """Body frame'den NED frame'e dönüşüm"""
        roll, pitch, yaw = attitude
        R_body_to_ned = self.rotation_matrix_from_euler(roll, pitch, yaw)
        return R_body_to_ned @ body_vector
    
    def calibrate_sensors(self, duration=10):
        """Sensör kalibrasyonu - statik bias hesaplama"""
        print(f"\n🔧 SENSÖR KALİBRASTONU ({duration}s)")
        print("📍 ROV'u hareketsiz tutun!")
        
        accel_samples = []
        gyro_samples = []
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            msg = self.mavlink_master.recv_match(blocking=True, timeout=0.1)
            if msg and msg.get_type() == 'HIGHRES_IMU':
                accel = np.array([msg.xacc, msg.yacc, msg.zacc])
                gyro = np.array([msg.xgyro, msg.ygyro, msg.zgyro])
                
                accel_samples.append(accel)
                gyro_samples.append(gyro)
                
                # Progress
                elapsed = time.time() - start_time
                print(f"   ⏰ Kalibrasyon: {elapsed:.1f}/{duration}s", end='\r')
        
        if len(accel_samples) > 10:
            # Accelerometer bias (gravity removal)
            mean_accel = np.mean(accel_samples, axis=0)
            # Assuming initial attitude is level, gravity should be [0, 0, -9.81]
            self.accel_bias = mean_accel - np.array([0, 0, -9.81])
            
            # Gyroscope bias
            self.gyro_bias = np.mean(gyro_samples, axis=0)
            
            print(f"\n✅ Kalibrasyon tamamlandı!")
            print(f"   📊 Accel bias: [{self.accel_bias[0]:.3f}, {self.accel_bias[1]:.3f}, {self.accel_bias[2]:.3f}] m/s²")
            print(f"   📊 Gyro bias: [{self.gyro_bias[0]:.4f}, {self.gyro_bias[1]:.4f}, {self.gyro_bias[2]:.4f}] rad/s")
        else:
            print("\n❌ Kalibrasyon başarısız - yeterli veri yok!")
    
    def process_imu_message(self, msg_type, msg):
        """IMU mesajını işle"""
        with self.data_lock:
            timestamp = time.time()
            
            if msg_type == 'HIGHRES_IMU':
                # Raw IMU data
                self.imu_data['accel'] = np.array([msg.xacc, msg.yacc, msg.zacc]) - self.accel_bias
                self.imu_data['gyro'] = np.array([msg.xgyro, msg.ygyro, msg.zgyro]) - self.gyro_bias
                self.imu_data['timestamp'] = timestamp
                
            elif msg_type == 'ATTITUDE':
                # Attitude data
                self.imu_data['attitude'] = np.array([msg.roll, msg.pitch, msg.yaw])
    
    def update_dead_reckoning(self):
        """Dead reckoning hesaplaması"""
        current_time = time.time()
        dt = current_time - self.dr_state['timestamp']
        
        if dt <= 0 or dt > 0.1:  # Skip if dt is too small or too large
            self.dr_state['timestamp'] = current_time
            return
        
        with self.data_lock:
            # Current IMU data
            accel_body = self.imu_data['accel'].copy()
            attitude = self.imu_data['attitude'].copy()
        
        # Transform acceleration from body to NED frame
        accel_ned = self.body_to_ned_transform(accel_body, attitude)
        
        # Remove gravity (in NED frame, gravity is [0, 0, 9.81])
        accel_ned_corrected = accel_ned - self.gravity_vector
        
        # Integrate acceleration to get velocity
        velocity_increment = accel_ned_corrected * dt
        self.dr_state['velocity'] += velocity_increment
        
        # Integrate velocity to get position
        position_increment = self.dr_state['velocity'] * dt + 0.5 * accel_ned_corrected * dt**2
        self.dr_state['position'] += position_increment
        
        # Update attitude
        self.dr_state['attitude'] = attitude.copy()
        
        # Update timestamp
        self.dr_state['timestamp'] = current_time
        
        # Add to history
        self.position_history.append({
            'timestamp': current_time,
            'position': self.dr_state['position'].copy(),
            'velocity': self.dr_state['velocity'].copy(),
            'attitude': self.dr_state['attitude'].copy()
        })
        
        # Update statistics
        speed = np.linalg.norm(self.dr_state['velocity'])
        self.stats['max_speed'] = max(self.stats['max_speed'], speed)
        self.stats['update_count'] += 1
        
        if len(self.position_history) > 1:
            # Calculate distance traveled
            prev_pos = self.position_history[-2]['position']
            curr_pos = self.dr_state['position']
            distance_increment = np.linalg.norm(curr_pos - prev_pos)
            self.stats['total_distance'] += distance_increment
    
    def tracking_loop(self):
        """Ana tracking döngüsü"""
        print("🔄 Dead reckoning tracking başlatıldı")
        
        loop_count = 0
        
        while self.tracking_active:
            try:
                # IMU mesajlarını oku
                msg = self.mavlink_master.recv_match(blocking=False, timeout=0.01)
                if msg:
                    msg_type = msg.get_type()
                    if msg_type in ['HIGHRES_IMU', 'ATTITUDE']:
                        self.process_imu_message(msg_type, msg)
                        
                        # Dead reckoning güncelle (sadece HIGHRES_IMU'da)
                        if msg_type == 'HIGHRES_IMU':
                            self.update_dead_reckoning()
                            loop_count += 1
                
                # Debug output (her 500 güncelleme)
                if loop_count > 0 and loop_count % 500 == 0:
                    pos = self.dr_state['position']
                    vel = self.dr_state['velocity']
                    att = self.dr_state['attitude']
                    
                    print(f"\n📍 Update #{loop_count}")
                    print(f"   Pos: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] m")
                    print(f"   Vel: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}] m/s")
                    print(f"   Att: [{math.degrees(att[0]):.1f}°, {math.degrees(att[1]):.1f}°, {math.degrees(att[2]):.1f}°]")
                    print(f"   Total distance: {self.stats['total_distance']:.1f} m")
                
                time.sleep(0.001)  # 1kHz loop
                
            except Exception as e:
                print(f"❌ Tracking loop hatası: {e}")
                time.sleep(0.1)
        
        print("🔄 Dead reckoning tracking durduruldu")
    
    def start_tracking(self):
        """Tracking başlat"""
        if self.tracking_active:
            print("⚠️ Tracking zaten aktif!")
            return
        
        # Reset state
        self.reset_position()
        
        self.tracking_active = True
        self.tracking_thread = threading.Thread(target=self.tracking_loop)
        self.tracking_thread.daemon = True
        self.tracking_thread.start()
        
        print("✅ Dead reckoning tracking başlatıldı")
    
    def stop_tracking(self):
        """Tracking durdur"""
        if not self.tracking_active:
            return
        
        self.tracking_active = False
        
        if self.tracking_thread:
            self.tracking_thread.join(timeout=2.0)
        
        print("🛑 Dead reckoning tracking durduruldu")
    
    def reset_position(self):
        """Pozisyonu sıfırla - yeni referans noktası"""
        with self.data_lock:
            self.dr_state['position'] = np.array([0.0, 0.0, 0.0])
            self.dr_state['velocity'] = np.array([0.0, 0.0, 0.0])
            self.dr_state['timestamp'] = time.time()
            
            self.reference_point['position'] = self.dr_state['position'].copy()
            self.reference_point['timestamp'] = time.time()
            
            # Clear history
            self.position_history.clear()
            self.velocity_history.clear()
            self.attitude_history.clear()
            
            # Reset stats
            self.stats['total_distance'] = 0.0
            self.stats['max_speed'] = 0.0
            self.stats['update_count'] = 0
            
        print("📍 Pozisyon sıfırlandı - yeni referans noktası")
    
    def get_current_position(self):
        """Mevcut pozisyonu al"""
        with self.data_lock:
            return {
                'position': self.dr_state['position'].copy(),
                'velocity': self.dr_state['velocity'].copy(),
                'attitude': self.dr_state['attitude'].copy(),
                'timestamp': self.dr_state['timestamp'],
                'stats': self.stats.copy()
            }
    
    def save_trajectory(self, filename=None):
        """Trajectory'yi kaydet"""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"trajectory_{timestamp}.json"
        
        trajectory_data = {
            'reference_point': {
                'position': self.reference_point['position'].tolist(),
                'timestamp': self.reference_point['timestamp']
            },
            'trajectory': []
        }
        
        for point in self.position_history:
            trajectory_data['trajectory'].append({
                'timestamp': point['timestamp'],
                'position': point['position'].tolist(),
                'velocity': point['velocity'].tolist(),
                'attitude': point['attitude'].tolist()
            })
        
        with open(filename, 'w') as f:
            json.dump(trajectory_data, f, indent=2)
        
        print(f"💾 Trajectory kaydedildi: {filename}")
        print(f"   📊 {len(self.position_history)} nokta")
        print(f"   📏 Toplam mesafe: {self.stats['total_distance']:.1f} m")
    
    def plot_trajectory_2d(self):
        """2D trajectory çiz"""
        if len(self.position_history) < 2:
            print("❌ Yeterli veri yok!")
            return
        
        # Extract positions
        positions = np.array([point['position'] for point in self.position_history])
        x = positions[:, 0]  # North
        y = positions[:, 1]  # East
        
        plt.figure(figsize=(10, 8))
        plt.plot(y, x, 'b-', linewidth=2, label='Trajectory')
        plt.scatter(y[0], x[0], color='green', s=100, label='Start', zorder=5)
        plt.scatter(y[-1], x[-1], color='red', s=100, label='End', zorder=5)
        
        plt.xlabel('East (m)')
        plt.ylabel('North (m)')
        plt.title('ROV Trajectory - Dead Reckoning')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        # Add statistics
        plt.text(0.02, 0.98, f'Total Distance: {self.stats["total_distance"]:.1f} m\nMax Speed: {self.stats["max_speed"]:.1f} m/s', 
                transform=plt.gca().transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        plt.tight_layout()
        plt.show()
    
    def run_interactive_tracking(self):
        """İnteraktif tracking modu"""
        print("\n" + "="*60)
        print("📍 İNTERAKTİF DEAD RECKONING")
        print("="*60)
        print("🎯 Komutlar:")
        print("  1 = Sensör kalibrasyonu")
        print("  2 = Tracking başlat")
        print("  3 = Tracking durdur")
        print("  4 = Pozisyon sıfırla")
        print("  5 = Mevcut pozisyon göster")
        print("  6 = Trajectory kaydet")
        print("  7 = 2D trajectory çiz")
        print("  q = Çıkış")
        print("="*60)
        
        try:
            while True:
                command = input("\n📍 DR Komut: ").strip().lower()
                
                if command == '1':
                    self.calibrate_sensors()
                elif command == '2':
                    self.start_tracking()
                elif command == '3':
                    self.stop_tracking()
                elif command == '4':
                    self.reset_position()
                elif command == '5':
                    pos_data = self.get_current_position()
                    pos = pos_data['position']
                    vel = pos_data['velocity']
                    att = pos_data['attitude']
                    stats = pos_data['stats']
                    
                    print(f"\n📍 MEVCUT POZİSYON:")
                    print(f"   Position: [{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}] m (N,E,D)")
                    print(f"   Velocity: [{vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}] m/s")
                    print(f"   Attitude: [{math.degrees(att[0]):.1f}°, {math.degrees(att[1]):.1f}°, {math.degrees(att[2]):.1f}°]")
                    print(f"   Distance: {stats['total_distance']:.1f} m")
                    print(f"   Max Speed: {stats['max_speed']:.1f} m/s")
                elif command == '6':
                    self.save_trajectory()
                elif command == '7':
                    self.plot_trajectory_2d()
                elif command == 'q':
                    print("👋 Dead reckoning modundan çıkılıyor...")
                    self.stop_tracking()
                    break
                else:
                    print("❌ Geçersiz komut!")
                
                time.sleep(0.1)
        
        except KeyboardInterrupt:
            print("\n🚨 Ctrl+C ile durduruldu!")
            self.stop_tracking()
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.connected:
            self.stop_tracking()
            if self.mavlink_master:
                self.mavlink_master.close()
            self.connected = False
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana program"""
    dr_system = IMUDeadReckoning()
    
    # Bağlantı kur
    if not dr_system.connect():
        print("❌ Bağlantı kurulamadı. Çıkılıyor...")
        return
    
    try:
        print("\n💡 ÖNERİ: Önce sensör kalibrasyonu yapın (komut: 1)")
        print("💡 Sonra tracking başlatın (komut: 2)")
        
        dr_system.run_interactive_tracking()
        
    except KeyboardInterrupt:
        print("\n👋 Program sonlandırılıyor...")
    finally:
        dr_system.disconnect()

if __name__ == "__main__":
    main() 