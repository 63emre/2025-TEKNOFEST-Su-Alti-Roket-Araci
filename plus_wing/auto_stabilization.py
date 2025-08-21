#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
Plus Wing (+) Otomatik Dengeleme Sistemi
Pixhawk Hareket Ettiğinde Kanatlar Otomatik Döner

Hardware:
- AUX 3: Ön Servo (DS3230MG 30kg)
- AUX 4: Sol Servo (DS3230MG 30kg)  
- AUX 5: Sağ Servo (DS3230MG 30kg)
- AUX 6: Arka Servo (DS3230MG 30kg)
- Pixhawk IMU (Roll, Pitch, Yaw)

Özellikler:
- Gerçek zamanlı IMU okuma
- Otomatik servo dengeleme
- PID kontrol (Roll, Pitch, Yaw)
- Türkçe arayüz ve durum bildirimi
- Acil durum güvenlik sistemi
"""

import os
import sys
import time
import threading
import math
import signal
from datetime import datetime
from collections import deque
from pymavlink import mavutil

# Plus Wing konfigürasyonu import et
try:
    from hardware_config import (
        PLUS_WING_SERVO_CHANNELS,
        PLUS_WING_CONFIG,
        calculate_plus_wing_pwm,
        get_plus_wing_config
    )
except ImportError:
    print("❌ hardware_config.py bulunamadı!")
    sys.exit(1)

# Connection configuration
try:
    import sys
    sys.path.append('../Test')
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Fallback connection: {MAV_ADDRESS}")

class PIDController:
    """Otomatik dengeleme için PID Controller"""
    
    def __init__(self, kp, ki, kd, max_output=400, integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral_limit = integral_limit
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
        # Performans takibi
        self.error_history = deque(maxlen=50)
        
    def update(self, target, current):
        """PID çıkışını güncelle"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            dt = 0.01  # Minimum delta time
            
        # Error hesaplama
        error = target - current
        self.error_history.append(error)
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (windup protection)
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Derivative term
        d_error = (error - self.last_error) / dt
        d_term = self.kd * d_error
        
        # PID output
        output = p_term + i_term + d_term
        output = max(-self.max_output, min(self.max_output, output))
        
        # Update için değerleri sakla
        self.last_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID controller'ı sıfırla"""
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        self.error_history.clear()
    
    def get_rms_error(self):
        """RMS error hesapla"""
        if not self.error_history:
            return 0.0
        errors = list(self.error_history)
        return math.sqrt(sum(e*e for e in errors) / len(errors))

class AutoStabilizationSystem:
    """Otomatik Dengeleme Sistemi - Pixhawk Hareket Ettiğinde Kanatlar Döner"""
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.armed = False
        self.stabilizing = False
        self.emergency_stop = False
        
        # PID Controllers - Plus Wing optimize edilmiş parametreler
        config = get_plus_wing_config()
        pid_params = config['PID_PARAMS']
        
        # Otomatik dengeleme için daha agresif parametreler
        self.pid_roll = PIDController(
            kp=pid_params['roll']['kp'] * 1.2,    # Daha hızlı yanıt
            ki=pid_params['roll']['ki'] * 0.8,    # Daha az integral
            kd=pid_params['roll']['kd'] * 1.5,    # Daha fazla damping
            max_output=300
        )
        
        self.pid_pitch = PIDController(
            kp=pid_params['pitch']['kp'] * 1.2,
            ki=pid_params['pitch']['ki'] * 0.8,
            kd=pid_params['pitch']['kd'] * 1.5,
            max_output=300
        )
        
        self.pid_yaw = PIDController(
            kp=pid_params['yaw']['kp'] * 1.0,     # Yaw daha yumuşak
            ki=pid_params['yaw']['ki'] * 0.6,
            kd=pid_params['yaw']['kd'] * 1.2,
            max_output=200
        )
        
        # IMU data
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'timestamp': time.time()
        }
        
        # Stabilization targets (sıfır pozisyon)
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # Performance metrics
        self.stats = {
            'start_time': None,
            'corrections_made': 0,
            'max_roll_error': 0.0,
            'max_pitch_error': 0.0,
            'max_yaw_error': 0.0,
            'servo_commands_sent': 0
        }
        
        # Threading
        self.sensor_thread = None
        self.control_thread = None
        self.stop_threads = False
        
        # Güvenlik sınırları
        self.SAFETY_LIMITS = {
            'max_roll_angle': 45.0,      # Maksimum roll açısı
            'max_pitch_angle': 30.0,     # Maksimum pitch açısı
            'max_correction': 80.0,      # Maksimum servo korreksiyonu
            'min_update_rate': 0.05      # Minimum 20Hz update
        }
        
        print("🚀 TEKNOFEST Plus Wing (+) Otomatik Dengeleme Sistemi")
        print("=" * 60)
        print(f"📡 MAVLink: {MAV_ADDRESS}")
        print(f"🔧 Servo Kanalları: {PLUS_WING_SERVO_CHANNELS}")
        print("🎯 Hedef: Pixhawk hareket ettiğinde otomatik dengeleme")
        print("⚠️ Güvenlik: Acil durum için Ctrl+C")
    
    def connect(self):
        """MAVLink bağlantısı kur"""
        try:
            print(f"📡 Pixhawk bağlantısı kuruluyor...")
            
            # Parse connection string
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=15)
            
            if heartbeat:
                self.connected = True
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print("✅ MAVLink bağlantısı kuruldu!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Armed Status: {'ARMED' if self.armed else 'DISARMED'}")
                
                return True
            else:
                print("❌ Heartbeat alınamadı!")
                return False
                
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        if not self.connected or self.emergency_stop:
            return False
        
        pwm_value = max(1000, min(2000, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            self.stats['servo_commands_sent'] += 1
            return True
            
        except Exception as e:
            print(f"❌ Servo PWM komutu hatası: {e}")
            return False
    
    def send_stabilization_commands(self, roll_correction, pitch_correction, yaw_correction):
        """Dengeleme servo komutlarını gönder"""
        # Güvenlik kontrolü
        roll_correction = max(-self.SAFETY_LIMITS['max_correction'], 
                            min(self.SAFETY_LIMITS['max_correction'], roll_correction))
        pitch_correction = max(-self.SAFETY_LIMITS['max_correction'], 
                             min(self.SAFETY_LIMITS['max_correction'], pitch_correction))
        yaw_correction = max(-self.SAFETY_LIMITS['max_correction'], 
                           min(self.SAFETY_LIMITS['max_correction'], yaw_correction))
        
        # Plus Wing PWM hesaplama
        pwm_values = calculate_plus_wing_pwm(roll_correction, pitch_correction, yaw_correction)
        
        # Servo komutlarını gönder
        success_count = 0
        for servo_name, pwm_value in pwm_values.items():
            channel = PLUS_WING_SERVO_CHANNELS[servo_name]
            if self.set_servo_pwm(channel, pwm_value):
                success_count += 1
        
        return success_count == len(pwm_values)
    
    def start_sensor_thread(self):
        """Sensor okuma thread'ini başlat"""
        self.stop_threads = False
        self.sensor_thread = threading.Thread(target=self._sensor_reader_thread, daemon=True)
        self.sensor_thread.start()
        print("🔄 IMU sensor okuma thread'i başlatıldı")
    
    def _sensor_reader_thread(self):
        """IMU sensor data okuma thread'i"""
        while not self.stop_threads and self.connected and not self.emergency_stop:
            try:
                # IMU data oku
                msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                if msg:
                    self.imu_data = {
                        'roll': math.degrees(msg.roll),
                        'pitch': math.degrees(msg.pitch),
                        'yaw': math.degrees(msg.yaw),
                        'timestamp': time.time()
                    }
                
                time.sleep(0.02)  # 50Hz sensor okuma
                
            except Exception as e:
                print(f"❌ Sensor okuma hatası: {e}")
                time.sleep(0.1)
    
    def start_stabilization_thread(self):
        """Otomatik dengeleme thread'ini başlat"""
        self.control_thread = threading.Thread(target=self._stabilization_control_thread, daemon=True)
        self.control_thread.start()
        print("🎯 Otomatik dengeleme kontrol thread'i başlatıldı")
    
    def _stabilization_control_thread(self):
        """Otomatik dengeleme kontrol döngüsü"""
        print("🚀 Otomatik dengeleme başlatıldı!")
        print("📊 Gerçek zamanlı durum bilgisi:")
        
        last_status_time = time.time()
        
        while self.stabilizing and not self.stop_threads and not self.emergency_stop:
            try:
                current_time = time.time()
                
                # Mevcut IMU değerleri
                current_roll = self.imu_data['roll']
                current_pitch = self.imu_data['pitch'] 
                current_yaw = self.imu_data['yaw']
                
                # Güvenlik kontrolü - Aşırı açılar
                if (abs(current_roll) > self.SAFETY_LIMITS['max_roll_angle'] or 
                    abs(current_pitch) > self.SAFETY_LIMITS['max_pitch_angle']):
                    print(f"⚠️ GÜVENLİK UYARISI: Aşırı açı tespit edildi!")
                    print(f"   Roll: {current_roll:.1f}°, Pitch: {current_pitch:.1f}°")
                    # Acil durum değil, sadece uyarı
                
                # PID hesaplamaları
                roll_output = self.pid_roll.update(self.target_roll, current_roll)
                pitch_output = self.pid_pitch.update(self.target_pitch, current_pitch)
                yaw_output = self.pid_yaw.update(self.target_yaw, current_yaw)
                
                # Servo komutlarına çevir
                roll_correction = roll_output / 4.0    # Scale down
                pitch_correction = pitch_output / 4.0  
                yaw_correction = yaw_output / 5.0      # Yaw daha yumuşak
                
                # Dengeleme komutlarını gönder
                if self.send_stabilization_commands(roll_correction, pitch_correction, yaw_correction):
                    self.stats['corrections_made'] += 1
                
                # İstatistik güncelleme
                self.stats['max_roll_error'] = max(self.stats['max_roll_error'], abs(current_roll))
                self.stats['max_pitch_error'] = max(self.stats['max_pitch_error'], abs(current_pitch))
                self.stats['max_yaw_error'] = max(self.stats['max_yaw_error'], abs(current_yaw))
                
                # Durum bilgisi (her 2 saniyede bir)
                if current_time - last_status_time >= 2.0:
                    print(f"\n⏱️ {datetime.now().strftime('%H:%M:%S')} - Dengeleme Durumu:")
                    print(f"   Roll: {current_roll:+6.2f}° → Correction: {roll_correction:+5.1f}")
                    print(f"   Pitch: {current_pitch:+6.2f}° → Correction: {pitch_correction:+5.1f}")
                    print(f"   Yaw: {current_yaw:+6.2f}° → Correction: {yaw_correction:+5.1f}")
                    print(f"   Korreksiyonlar: {self.stats['corrections_made']}")
                    print(f"   RMS Error - R:{self.pid_roll.get_rms_error():.2f}° P:{self.pid_pitch.get_rms_error():.2f}° Y:{self.pid_yaw.get_rms_error():.2f}°")
                    last_status_time = current_time
                
                time.sleep(self.SAFETY_LIMITS['min_update_rate'])  # 20Hz kontrol döngüsü
                
            except Exception as e:
                print(f"❌ Stabilization kontrol hatası: {e}")
                time.sleep(0.1)
    
    def start_auto_stabilization(self):
        """Otomatik dengeleme sistemini başlat"""
        if not self.connected:
            print("❌ Pixhawk bağlantısı yok!")
            return False
        
        if not self.armed:
            print("⚠️ Sistem DISARMED. Servo komutları sınırlı olabilir.")
            print("💡 Tam fonksiyonellik için sistemi ARM edin.")
        
        # İstatistikleri sıfırla
        self.stats['start_time'] = datetime.now()
        self.stats['corrections_made'] = 0
        self.stats['max_roll_error'] = 0.0
        self.stats['max_pitch_error'] = 0.0
        self.stats['max_yaw_error'] = 0.0
        self.stats['servo_commands_sent'] = 0
        
        # PID kontrolörleri sıfırla
        self.pid_roll.reset()
        self.pid_pitch.reset()
        self.pid_yaw.reset()
        
        # Thread'leri başlat
        self.start_sensor_thread()
        time.sleep(0.5)  # Sensor data gelmesini bekle
        
        self.stabilizing = True
        self.start_stabilization_thread()
        
        print("\n✅ Otomatik dengeleme sistemi aktif!")
        print("🎯 Pixhawk hareket ettiğinde kanatlar otomatik dengeleme yapacak")
        print("⚠️ Durdurmak için Ctrl+C tuşlayın")
        
        return True
    
    def stop_stabilization(self):
        """Otomatik dengelemeyi durdur"""
        print("\n🛑 Otomatik dengeleme durduruluyor...")
        
        self.stabilizing = False
        self.stop_threads = True
        
        # Thread'leri bekle
        if self.sensor_thread:
            self.sensor_thread.join(timeout=2.0)
        if self.control_thread:
            self.control_thread.join(timeout=2.0)
        
        # Servo'ları neutral'a getir
        if self.connected and not self.emergency_stop:
            print("🔄 Servo'ları neutral pozisyona getiriliyor...")
            self.send_stabilization_commands(0, 0, 0)
            time.sleep(1.0)
        
        self.print_final_stats()
    
    def emergency_stop_all(self):
        """Acil durum - tüm sistemi durdur"""
        print("\n🚨 ACİL DURUM - TÜM SİSTEM DURDURULUYOR!")
        self.emergency_stop = True
        self.stop_stabilization()
        
        # Servo'ları neutral'a getir
        if self.connected:
            for i in range(3):  # 3 kez dene
                self.send_stabilization_commands(0, 0, 0)
                time.sleep(0.2)
        
        print("🛑 Acil durum prosedürü tamamlandı")
    
    def print_final_stats(self):
        """Final istatistikleri yazdır"""
        if self.stats['start_time']:
            duration = (datetime.now() - self.stats['start_time']).total_seconds()
            
            print(f"\n📊 OTOMATIK DENGELEME İSTATİSTİKLERİ")
            print("=" * 50)
            print(f"   Çalışma Süresi: {duration:.1f} saniye")
            print(f"   Toplam Korreksiyonlar: {self.stats['corrections_made']}")
            print(f"   Servo Komutları: {self.stats['servo_commands_sent']}")
            print(f"   Ortalama Korreksiyonlar/s: {self.stats['corrections_made']/duration:.1f}")
            print(f"   Maksimum Hatalar:")
            print(f"     Roll: {self.stats['max_roll_error']:.2f}°")
            print(f"     Pitch: {self.stats['max_pitch_error']:.2f}°")
            print(f"     Yaw: {self.stats['max_yaw_error']:.2f}°")
            
            # PID performansı
            print(f"   Son RMS Hatalar:")
            print(f"     Roll: {self.pid_roll.get_rms_error():.3f}°")
            print(f"     Pitch: {self.pid_pitch.get_rms_error():.3f}°")
            print(f"     Yaw: {self.pid_yaw.get_rms_error():.3f}°")
    
    def disconnect(self):
        """MAVLink bağlantısını kapat"""
        if self.master:
            try:
                self.master.close()
                print("🔌 MAVLink bağlantısı kapatıldı")
            except:
                pass
        self.connected = False

def signal_handler(signum, frame):
    """Signal handler for graceful shutdown"""
    global stabilization_system
    print(f"\n⚠️ Signal {signum} alındı - Güvenli kapatma başlatılıyor...")
    if 'stabilization_system' in globals():
        stabilization_system.emergency_stop_all()
    sys.exit(0)

def main():
    """Ana program"""
    global stabilization_system
    
    # Signal handler'ları kaydet
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    stabilization_system = AutoStabilizationSystem()
    
    try:
        # Bağlantı kur
        if not stabilization_system.connect():
            print("❌ Pixhawk bağlantısı başarısız!")
            sys.exit(1)
        
        # Otomatik dengelemeyi başlat
        if stabilization_system.start_auto_stabilization():
            # Ana döngü - kullanıcı müdahalesi bekle
            print("\n" + "="*60)
            print("🎮 KONTROL KOMUTLARI:")
            print("   Ctrl+C  : Güvenli kapatma")
            print("   Enter   : Anlık durum bilgisi")
            print("="*60)
            
            try:
                while stabilization_system.stabilizing:
                    user_input = input()  # Enter bekle
                    if user_input.lower() in ['q', 'quit', 'exit']:
                        break
                    
                    # Anlık durum bilgisi
                    current_roll = stabilization_system.imu_data['roll']
                    current_pitch = stabilization_system.imu_data['pitch']
                    current_yaw = stabilization_system.imu_data['yaw']
                    
                    print(f"\n📊 Anlık Durum ({datetime.now().strftime('%H:%M:%S')}):")
                    print(f"   Roll: {current_roll:+7.2f}°")
                    print(f"   Pitch: {current_pitch:+7.2f}°")
                    print(f"   Yaw: {current_yaw:+7.2f}°")
                    print(f"   Toplam Korreksiyonlar: {stabilization_system.stats['corrections_made']}")
                    print("   (Enter: Tekrar göster, q+Enter: Çıkış)")
                    
            except EOFError:
                # Pipe veya background çalıştırma durumunda
                while stabilization_system.stabilizing:
                    time.sleep(1.0)
        
        # Güvenli kapatma
        stabilization_system.stop_stabilization()
        stabilization_system.disconnect()
        
        print("\n✅ Otomatik dengeleme sistemi güvenli şekilde kapatıldı")
        
    except KeyboardInterrupt:
        print("\n⚠️ Kullanıcı tarafından durduruldu")
        stabilization_system.emergency_stop_all()
        stabilization_system.disconnect()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Beklenmeyen hata: {e}")
        stabilization_system.emergency_stop_all()
        stabilization_system.disconnect()
        sys.exit(1)

if __name__ == "__main__":
    main()
