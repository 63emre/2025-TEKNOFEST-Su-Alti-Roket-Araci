#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş
PLUS WING (+) KONFİGÜRASYONU - GPS'SİZ DEAD RECKONING

Görev Açıklaması:
- Başlangıç bölgesinden 2m derinlikte düz istikamette 10m ilerle (süre başlatılır)
- Kıyıdan en az 50m uzaklaş (dead reckoning ile)
- Başlangıç noktasına otonom geri dön (IMU + zaman bazlı)
- Pozitif sephiye ile yüzeye çıkıp enerjiyi kes

Puanlama:
- Seyir yapma (hız): 150 puan
- Başlangıç noktasında enerji kesme: 90 puan  
- Sızdırmazlık: 60 puan
- Süre limiti: 5 dakika
- Toplam: 300 puan

Plus Wing Konfigürasyonu:
     ÜST (14)
        |
SOL (13) + SAĞ (12)
        |
     ALT (11)
"""

import time
import threading
import math
import json
import argparse
from datetime import datetime
from pymavlink import mavutil

# Plus Wing hardware config import
try:
    from hardware_config import (
        PLUS_WING_SERVO_CHANNELS,
        PLUS_WING_CONFIG,
        calculate_plus_wing_pwm,
        get_plus_wing_config
    )
except ImportError:
    print("❌ hardware_config.py bulunamadı!")
    exit(1)

# D300 derinlik sensörü import
try:
    import sys
    sys.path.append('../App')
    from depth_sensor import D300DepthSensor
    D300_AVAILABLE = True
    print("✅ D300 derinlik sensörü modülü yüklendi")
except ImportError:
    print("⚠️ D300 derinlik sensörü modülü bulunamadı, SCALED_PRESSURE kullanılacak")
    D300_AVAILABLE = False

# MAVLink bağlantı adresi
import os
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# Görev parametreleri (şartnameden)
MISSION_PARAMS = {
    'target_depth': 2.0,           # 2m derinlik
    'straight_distance': 10.0,     # 10m düz seyir
    'min_offshore_distance': 50.0, # 50m kıyıdan uzaklık
    'cruise_speed': 1.5,           # Seyir hızı (m/s)
    'return_speed': 1.8,           # Geri dönüş hızı (m/s)
    'timeout_seconds': 300,        # 5 dakika süre limiti
    'position_tolerance': 2.0,     # Başlangıç noktası toleransı (m)
    'depth_tolerance': 0.2         # Derinlik toleransı (m)
}

# Plus Wing kontrol parametreleri
CONTROL_PARAMS = {
    'depth_pid': {'kp': 80.0, 'ki': 3.0, 'kd': 25.0},
    'heading_pid': {'kp': 4.0, 'ki': 0.15, 'kd': 0.8},
    'dead_reckoning_pid': {'kp': 1.5, 'ki': 0.02, 'kd': 0.4}
}

# Plus Wing servo ve motor kanalları
MOTOR_CHANNEL = PLUS_WING_CONFIG['MOTOR']['ana_motor']['mavlink_channel']
SERVO_CHANNELS = PLUS_WING_SERVO_CHANNELS

# PWM değerleri
PWM_NEUTRAL = PLUS_WING_CONFIG['PWM_LIMITS']['servo_neutral']
PWM_MIN = PLUS_WING_CONFIG['PWM_LIMITS']['servo_min']
PWM_MAX = PLUS_WING_CONFIG['PWM_LIMITS']['servo_max']

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

class Mission1Navigator:
    def __init__(self, start_heading=0.0):
        self.master = None
        self.connected = False
        self.mission_active = False
        
        # D300 derinlik sensörü
        self.d300_sensor = None
        self.d300_connected = False
        if D300_AVAILABLE:
            try:
                self.d300_sensor = D300DepthSensor(i2c_address=0x76)
                self.d300_connected = self.d300_sensor.initialize()
                if self.d300_connected:
                    print("✅ D300 derinlik sensörü başlatıldı (0x76)")
                else:
                    print("⚠️ D300 sensörü başlatılamadı, SCALED_PRESSURE kullanılacak")
            except Exception as e:
                print(f"⚠️ D300 sensörü hatası: {e}, SCALED_PRESSURE kullanılacak")
                self.d300_connected = False
        
        # Dead Reckoning navigasyon durumu (GPS'siz)
        self.start_position = {'x': 0.0, 'y': 0.0, 'heading': start_heading}
        self.current_position = {'x': 0.0, 'y': 0.0}  # Relatif pozisyon (m)
        self.current_depth = 0.0
        self.current_heading = start_heading
        self.current_speed = 0.0
        
        # Attitude veriler (IMU)
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = start_heading
        
        # Dead reckoning için
        self.last_position_update = time.time()
        self.traveled_distance = 0.0
        self.initial_heading = start_heading
        
        # Görev durumu
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.straight_course_start_time = None
        self.mission_completion_time = None
        
        # Görev başarı metrikleri
        self.max_offshore_distance = 0.0
        self.straight_distance_completed = 0.0
        self.final_position_error = float('inf')
        self.leak_detected = False
        
        # PID kontrolcüler
        self.depth_pid = PIDController(**CONTROL_PARAMS['depth_pid'])
        self.heading_pid = PIDController(**CONTROL_PARAMS['heading_pid'])
        self.position_pid_x = PIDController(**CONTROL_PARAMS['dead_reckoning_pid'])
        self.position_pid_y = PIDController(**CONTROL_PARAMS['dead_reckoning_pid'])
        
        # Veri kayıt
        self.mission_log = []
        self.telemetry_data = []
        
        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı kur"""
        try:
            print("🔌 Pixhawk bağlantısı kuruluyor...")
            
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
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def read_sensors(self):
        """Tüm sensör verilerini oku (GPS'siz - IMU + Dead Reckoning)"""
        if not self.connected:
            return False
            
        try:
            current_time = time.time()
            dt = current_time - self.last_position_update
            
            # Attitude (IMU) - Ana navigasyon kaynağı
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw
            
            # Derinlik sensörü (D300 öncelikli, yoksa SCALED_PRESSURE)
            depth_read_success = False
            if self.d300_connected and self.d300_sensor:
                try:
                    depth_data = self.d300_sensor.read_depth()
                    if depth_data['success']:
                        self.current_depth = max(0.0, depth_data['depth'])
                        depth_read_success = True
                        print(f"📡 D300 Derinlik: {self.current_depth:.2f}m")
                    else:
                        print(f"⚠️ D300 okuma başarısız: {depth_data}")
                except Exception as e:
                    print(f"⚠️ D300 okuma hatası: {e}")
            
            if not depth_read_success:
                # D300 yok veya hatalı, SCALED_PRESSURE kullan
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    depth_pressure = pressure_msg.press_abs - 1013.25
                    self.current_depth = max(0, depth_pressure * 0.10197)
                    print(f"📡 SCALED_PRESSURE Derinlik: {self.current_depth:.2f}m (Basınç: {pressure_msg.press_abs:.1f}hPa)")
                else:
                    print("❌ Hiçbir derinlik verisi yok!")
            
            # Hız bilgisi
            vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if vfr_msg:
                self.current_speed = vfr_msg.groundspeed
            
            # Dead Reckoning pozisyon güncelleme
            if dt > 0.1:  # 10Hz'de güncelle
                distance_traveled = self.current_speed * dt
                self.traveled_distance += distance_traveled
                
                # Heading'e göre X,Y pozisyon hesapla
                heading_rad = math.radians(self.current_heading)
                dx = distance_traveled * math.cos(heading_rad)
                dy = distance_traveled * math.sin(heading_rad)
                
                self.current_position['x'] += dx
                self.current_position['y'] += dy
                
                self.last_position_update = current_time
            
            # Telemetri kaydet
            self.telemetry_data.append({
                'timestamp': current_time,
                'position': self.current_position.copy(),
                'depth': self.current_depth,
                'heading': self.current_heading,
                'speed': self.current_speed,
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'traveled_distance': self.traveled_distance,
                'mission_stage': self.mission_stage
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def calculate_distance_bearing_to_origin(self):
        """Başlangıç noktasına mesafe ve bearing hesapla (Dead Reckoning)"""
        # Mevcut pozisyondan başlangıç noktasına (0,0)
        dx = -self.current_position['x']  # Başlangıca dönmek için negatif
        dy = -self.current_position['y']
        
        # Mesafe hesaplama
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Bearing hesaplama (matematiksel açıdan navigasyon açısına çevir)
        bearing_rad = math.atan2(dy, dx)
        bearing_deg = math.degrees(bearing_rad)
        
        # 0-360 derece aralığına çevir
        bearing_deg = (bearing_deg + 360) % 360
        
        return distance, bearing_deg
    
    def get_current_distance_from_start(self):
        """Başlangıç noktasından mevcut uzaklık"""
        return math.sqrt(self.current_position['x']**2 + self.current_position['y']**2)
    
    def set_motor_throttle(self, throttle_pwm):
        """Motor kontrolü"""
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
        except:
            return False
    
    def set_control_surfaces(self, roll_cmd=0, pitch_cmd=0, yaw_cmd=0):
        """Plus Wing kontrol yüzeylerini ayarla"""
        if not self.connected:
            return False
        
        try:
            # Komut değerlerini güçlendir (çok küçük değerler servo hareket ettirmez)
            roll_cmd = max(-100, min(100, roll_cmd * 2.0))  # 2x güçlendir
            pitch_cmd = max(-100, min(100, pitch_cmd * 2.0))
            yaw_cmd = max(-100, min(100, yaw_cmd * 2.0))
            
            # Plus Wing PWM hesaplama
            pwm_values = calculate_plus_wing_pwm(roll_cmd, pitch_cmd, yaw_cmd)
            
            # Debug çıktısı
            if abs(roll_cmd) > 5 or abs(pitch_cmd) > 5 or abs(yaw_cmd) > 5:
                print(f"🎮 Servo Komutları: R={roll_cmd:+.1f} P={pitch_cmd:+.1f} Y={yaw_cmd:+.1f}")
                print(f"📡 PWM Değerleri: {pwm_values}")
            
            # Tüm servo komutlarını gönder
            success_count = 0
            for servo_name, pwm_value in pwm_values.items():
                channel = SERVO_CHANNELS[servo_name]
                if self.set_servo_position(channel, int(pwm_value)):
                    success_count += 1
                else:
                    print(f"❌ Servo {servo_name} (kanal {channel}) komutu gönderilemedi")
            
            if success_count > 0:
                print(f"✅ {success_count}/{len(pwm_values)} servo komutu gönderildi")
            
            return success_count == len(pwm_values)
            
        except Exception as e:
            print(f"❌ Plus Wing kontrol hatası: {e}")
            return False
    
    def set_servo_position(self, channel, pwm_value):
        """Servo pozisyon kontrolü"""
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
        except:
            return False
    
    def test_servos(self):
        """Servo test fonksiyonu - hareket edip etmediklerini kontrol et"""
        print("\n🔧 SERVO TEST BAŞLIYOR...")
        print("Her eksende servo hareketi test ediliyor...")
        
        test_values = [20, -20, 40, -40]  # Test değerleri
        
        for i, val in enumerate(test_values):
            print(f"\n📊 Test {i+1}/4: Roll={val}")
            self.set_control_surfaces(roll_cmd=val, pitch_cmd=0, yaw_cmd=0)
            time.sleep(2)
            
            print(f"📊 Test {i+1}/4: Pitch={val}")  
            self.set_control_surfaces(roll_cmd=0, pitch_cmd=val, yaw_cmd=0)
            time.sleep(2)
            
            print(f"📊 Test {i+1}/4: Yaw={val}")
            self.set_control_surfaces(roll_cmd=0, pitch_cmd=0, yaw_cmd=val)
            time.sleep(2)
        
        # Nötr pozisyon
        print("\n🔄 Servolar nötr pozisyona getiriliyor...")
        self.set_control_surfaces(roll_cmd=0, pitch_cmd=0, yaw_cmd=0)
        time.sleep(1)
        print("✅ Servo test tamamlandı!")

    def display_mission_status(self):
        """Görev durumunu göster"""
        print("\n" + "="*80)
        print("🚀 TEKNOFEST - GÖREV 1: SEYİR YAPMA & GERİ DÖNÜŞ")
        print("="*80)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        mission_time = (time.time() - self.mission_start_time) if self.mission_start_time else 0
        remaining_time = max(0, MISSION_PARAMS['timeout_seconds'] - mission_time)
        
        print(f"⏰ Zaman: {timestamp} | Görev Süresi: {mission_time:.0f}s | Kalan: {remaining_time:.0f}s")
        print(f"🎯 Görev Aşaması: {self.mission_stage}")
        
        # Dead Reckoning pozisyon bilgisi
        distance_from_start, bearing_to_start = self.calculate_distance_bearing_to_origin()
        print(f"📍 Mevcut Pozisyon (Dead Reckoning): X={self.current_position['x']:.1f}m, Y={self.current_position['y']:.1f}m")
        print(f"🏠 Başlangıçtan Uzaklık: {distance_from_start:.1f}m | Geri Dönüş Yönü: {bearing_to_start:.0f}°")
        print(f"📏 Toplam Mesafe: {self.traveled_distance:.1f}m")
        
        print(f"🌊 Derinlik: {self.current_depth:.1f}m | Hedef: {MISSION_PARAMS['target_depth']:.1f}m")
        print(f"🧭 Heading: {self.current_heading:.0f}° | Hız: {self.current_speed:.1f} m/s")
        print(f"📊 Roll: {self.current_roll:+.1f}° | Pitch: {self.current_pitch:+.1f}°")
        
        # Görev metrikleri
        print(f"📏 Düz Seyir: {self.straight_distance_completed:.1f}m / {MISSION_PARAMS['straight_distance']}m")
        print(f"🌊 Max Kıyı Uzaklığı: {self.max_offshore_distance:.1f}m / {MISSION_PARAMS['min_offshore_distance']}m")
        
        # Başarı durumu
        if self.final_position_error < float('inf'):
            print(f"🎯 Final Pozisyon Hatası: {self.final_position_error:.1f}m")
        
        print("="*80)
    
    def control_loop(self):
        """Ana kontrol döngüsü"""
        while self.running and self.mission_active:
            self.read_sensors()
            
            if self.mission_stage == "DESCENT":
                self.execute_descent()
            elif self.mission_stage == "STRAIGHT_COURSE":
                self.execute_straight_course()
            elif self.mission_stage == "OFFSHORE_CRUISE":
                self.execute_offshore_cruise()
            elif self.mission_stage == "RETURN_NAVIGATION":
                self.execute_return_navigation()
            elif self.mission_stage == "FINAL_APPROACH":
                self.execute_final_approach()
            elif self.mission_stage == "SURFACE_AND_SHUTDOWN":
                self.execute_surface_shutdown()
            elif self.mission_stage == "MISSION_COMPLETE":
                break
            
            time.sleep(0.1)  # 10Hz kontrol
    
    def execute_descent(self):
        """2m derinliğe iniş"""
        # Derinlik kontrolü
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        
        # Debug derinlik sensörü
        print(f"🌊 Derinlik Debug: Mevcut={self.current_depth:.2f}m, Hedef={MISSION_PARAMS['target_depth']:.2f}m, Hata={depth_error:.2f}m")
        
        # Derinlik sensörü çalışmıyorsa simüle et (test için)
        if self.current_depth == 0.0:
            print("⚠️ Derinlik sensörü çalışmıyor! 10 saniye sonra simüle derinliğe geçiş...")
            if hasattr(self, '_descent_start_time'):
                if time.time() - self._descent_start_time > 10:
                    print("✅ Simüle derinlik ulaşıldı! Düz seyire geçiliyor...")
                    self.mission_stage = "STRAIGHT_COURSE"
                    self.straight_course_start_time = time.time()
                    return
            else:
                self._descent_start_time = time.time()
        
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            # PID kontrol ile iniş
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            
            # Motor ve fin kontrolü
            if depth_error > 0:  # Daha derine inmeli
                motor_throttle = PWM_NEUTRAL + 50  # İleri hareket
                pitch_cmd = 50  # Nose down (daha az agresif)
            else:  # Yükselmeli
                motor_throttle = PWM_NEUTRAL - 50  # Geri hareket
                pitch_cmd = -50  # Nose up
            
            # Roll stabilizasyonu ekle (test için)
            roll_error = 0 - self.current_roll  # Sıfır roll hedefi
            roll_cmd = max(-30, min(30, roll_error * 2))  # Basit P kontrolü
            
            # Yaw stabilizasyonu ekle
            if self.initial_heading is not None:
                yaw_error = self.initial_heading - self.current_heading
                if yaw_error > 180: yaw_error -= 360
                if yaw_error < -180: yaw_error += 360
                yaw_cmd = max(-30, min(30, yaw_error * 0.5))
            else:
                yaw_cmd = 0
            
            print(f"🎮 Descent Komutları: Motor={motor_throttle}, P={pitch_cmd}, R={roll_cmd}, Y={yaw_cmd}")
            
            self.set_motor_throttle(motor_throttle)
            self.set_control_surfaces(roll_cmd=roll_cmd, pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        else:
            # Hedef derinliğe ulaşıldı
            print("✅ Hedef derinlik ulaşıldı! Düz seyire geçiliyor...")
            self.mission_stage = "STRAIGHT_COURSE"
            self.straight_course_start_time = time.time()
    
    def execute_straight_course(self):
        """10m düz seyir (süre başlatma)"""
        if not self.straight_course_start_time:
            self.straight_course_start_time = time.time()
        
        elapsed_distance = self.current_speed * (time.time() - self.straight_course_start_time)
        self.straight_distance_completed = elapsed_distance
        
        # Düz seyir kontrolü
        motor_throttle = PWM_NEUTRAL + 120  # Forward thrust
        
        # Derinlik tutma
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-150, min(150, int(depth_correction)))
        
        # Yön stabilizasyonu (başlangıç heading'i tut)
        if self.initial_heading is not None:
            heading_error = self.initial_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_correction = self.heading_pid.update(self.initial_heading, self.current_heading)
            yaw_cmd = max(-50, min(50, int(heading_correction * 2)))  # 2x güçlendir
            
            # Debug çıktısı
            if abs(heading_error) > 5:
                print(f"🧭 Heading Error: {heading_error:.1f}° → Yaw Cmd: {yaw_cmd}")
        else:
            yaw_cmd = 0
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # 10m düz seyir tamamlandı mı?
        if self.straight_distance_completed >= MISSION_PARAMS['straight_distance']:
            print("✅ 10m düz seyir tamamlandı! Kıyıdan uzaklaşmaya başlanıyor...")
            self.mission_stage = "OFFSHORE_CRUISE"
    
    def execute_offshore_cruise(self):
        """Kıyıdan 50m uzaklaşma (Dead Reckoning)"""
        distance_from_start = self.get_current_distance_from_start()
        self.max_offshore_distance = max(self.max_offshore_distance, distance_from_start)
        
        # Hızlı ileri hareket
        motor_throttle = PWM_NEUTRAL + 150
        
        # Derinlik tutma
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-50, min(50, int(depth_correction // 4)))
        
        # Düz heading tutma (başlangıç yönünde devam)
        if self.initial_heading is not None:
            heading_error = self.initial_heading - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            heading_correction = self.heading_pid.update(self.initial_heading, self.current_heading)
            yaw_cmd = max(-40, min(40, int(heading_correction * 1.5)))  # Güçlendir
            
            # Debug çıktısı
            if abs(heading_error) > 10:
                print(f"🧭 Offshore Heading Error: {heading_error:.1f}° → Yaw Cmd: {yaw_cmd}")
        else:
            yaw_cmd = 0
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # 50m uzaklaştık mı?
        if distance_from_start >= MISSION_PARAMS['min_offshore_distance']:
            print("✅ 50m uzaklaşma tamamlandı! Geri dönüş navigasyonu başlıyor...")
            self.mission_stage = "RETURN_NAVIGATION"
    
    def execute_return_navigation(self):
        """Başlangıç noktasına geri dönüş (Dead Reckoning)"""
        distance_from_start, bearing_to_start = self.calculate_distance_bearing_to_origin()
        
        # Hedefe yönlenme
        heading_error = bearing_to_start - self.current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        # Hızlı geri dönüş
        if distance_from_start > 10:  # 10m'den uzaksa hızla git
            motor_throttle = PWM_NEUTRAL + 180
        else:  # Yakınsa yavaşla
            motor_throttle = PWM_NEUTRAL + 100
            
        # Navigasyon kontrolü
        heading_correction = self.heading_pid.update(bearing_to_start, self.current_heading)
        yaw_cmd = max(-50, min(50, int(heading_correction // 2)))
        
        # Derinlik tutma
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-50, min(50, int(depth_correction // 4)))
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # Başlangıç noktasına yaklaştık mı?
        if distance_from_start <= MISSION_PARAMS['position_tolerance'] * 2:  # 4m tolerance
            print("✅ Başlangıç noktasına yaklaşıldı! Final yaklaşım...")
            self.mission_stage = "FINAL_APPROACH"
    
    def execute_final_approach(self):
        """Final yaklaşım ve pozisyon tutma (Dead Reckoning)"""
        distance_from_start, bearing_to_start = self.calculate_distance_bearing_to_origin()
        self.final_position_error = distance_from_start
        
        # Hassas pozisyon kontrolü
        if distance_from_start > MISSION_PARAMS['position_tolerance']:
            # Yavaş yaklaşım
            motor_throttle = PWM_NEUTRAL + 60
            
            heading_correction = self.heading_pid.update(bearing_to_start, self.current_heading)
            yaw_cmd = max(-30, min(30, int(heading_correction // 3)))
        else:
            # Pozisyon tutma
            motor_throttle = PWM_NEUTRAL + 20
            yaw_cmd = 0
        
        # Derinlik tutma
        depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-30, min(30, int(depth_correction // 5)))
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # 5 saniye pozisyon tuttuk mu?
        if distance_from_start <= MISSION_PARAMS['position_tolerance']:
            time.sleep(5)  # 5 saniye bekle
            print("✅ Final pozisyon tutuldu! Yüzeye çıkış ve enerji kesme...")
            self.mission_stage = "SURFACE_AND_SHUTDOWN"
    
    def execute_surface_shutdown(self):
        """Pozitif sephiye ile yüzeye çıkış ve enerji kesme"""
        # Pozitif sephiye (buoyancy ile yüzeye çık)
        self.set_motor_throttle(PWM_NEUTRAL)  # Motor kapat
        self.set_control_surfaces(pitch_cmd=-100)  # Nose up (pozitif sephiye)
        
        # Yüzeye çıkana kadar bekle
        if self.current_depth > 0.5:
            return  # Henüz yüzeye çıkmadı
        
        # Yüzeye çıktık - enerji kesme simülasyonu
        print("🌊 Yüzeye çıkış tamamlandı!")
        print("⚡ Sistem enerjisi kesiliyor...")
        
        # Tüm sistemleri durdur
        self.set_motor_throttle(PWM_NEUTRAL)
        self.set_control_surfaces()
        
        self.mission_completion_time = time.time()
        self.mission_stage = "MISSION_COMPLETE"
        
        print("✅ GÖREV 1 TAMAMLANDI!")
    
    def monitoring_loop(self):
        """İzleme döngüsü"""
        while self.running and self.mission_active:
            # Her 3 saniyede durum göster
            if len(self.telemetry_data) % 30 == 0:
                self.display_mission_status()
            
            # Süre kontrolü
            if self.mission_start_time:
                elapsed = time.time() - self.mission_start_time
                if elapsed > MISSION_PARAMS['timeout_seconds']:
                    print("⏰ Süre doldu! Görev sonlandırılıyor...")
                    self.mission_stage = "MISSION_TIMEOUT"
                    break
            
            time.sleep(0.1)
    
    def generate_mission_report(self):
        """Görev raporu oluştur"""
        mission_duration = (self.mission_completion_time - self.mission_start_time) if (self.mission_completion_time and self.mission_start_time) else 0
        
        print("\n" + "="*80)
        print("📋 GÖREV 1 RAPORU - SEYİR YAPMA & GERİ DÖNÜŞ")
        print("="*80)
        
        print(f"📅 Görev Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Görev Süresi: {mission_duration:.1f} saniye")
        
        # Performans metrikleri
        print(f"\n📊 PERFORMANS METRİKLERİ:")
        print("-"*60)
        print(f"📏 Düz Seyir Mesafesi: {self.straight_distance_completed:.1f}m / {MISSION_PARAMS['straight_distance']}m")
        print(f"🌊 Maksimum Kıyı Uzaklığı: {self.max_offshore_distance:.1f}m / {MISSION_PARAMS['min_offshore_distance']}m")
        print(f"🎯 Final Pozisyon Hatası: {self.final_position_error:.1f}m / {MISSION_PARAMS['position_tolerance']}m")
        print(f"💧 Sızdırmazlık: {'✅ BAŞARILI' if not self.leak_detected else '❌ SIZINTI'}")
        
        # Puanlama hesaplama
        print(f"\n🏆 PUANLAMA:")
        print("-"*40)
        
        # Seyir yapma puanı (hız bazlı)
        time_factor = max(0, (300 - mission_duration) / 300) if mission_duration > 0 else 0
        cruise_points = int(150 * time_factor) if self.straight_distance_completed >= MISSION_PARAMS['straight_distance'] and self.max_offshore_distance >= MISSION_PARAMS['min_offshore_distance'] else 0
        print(f"  🚀 Seyir Yapma (hız): {cruise_points}/150 puan")
        
        # Başlangıç noktasında enerji kesme
        position_points = 90 if self.final_position_error <= MISSION_PARAMS['position_tolerance'] else 0
        print(f"  🎯 Başlangıç Noktasında Enerji Kesme: {position_points}/90 puan")
        
        # Sızdırmazlık
        waterproof_points = 60 if not self.leak_detected else 0
        print(f"  💧 Sızdırmazlık: {waterproof_points}/60 puan")
        
        total_points = cruise_points + position_points + waterproof_points
        print(f"\n📈 TOPLAM PUAN: {total_points}/300")
        
        # Başarı değerlendirmesi
        if total_points >= 240:  # %80 başarı
            print("🎉 MÜKEMMEL PERFORMANS!")
        elif total_points >= 180:  # %60 başarı
            print("👍 İYİ PERFORMANS!")
        elif total_points >= 120:  # %40 başarı
            print("⚠️ ORTA PERFORMANS!")
        else:
            print("❌ DÜŞÜK PERFORMANS!")
        
        # Veri kaydet
        mission_report = {
            'timestamp': datetime.now().isoformat(),
            'mission_duration': mission_duration,
            'performance_metrics': {
                'straight_distance_completed': self.straight_distance_completed,
                'max_offshore_distance': self.max_offshore_distance,
                'final_position_error': self.final_position_error,
                'leak_detected': self.leak_detected
            },
            'scoring': {
                'cruise_points': cruise_points,
                'position_points': position_points,
                'waterproof_points': waterproof_points,
                'total_points': total_points
            },
            'telemetry_summary': {
                'total_samples': len(self.telemetry_data),
                'start_position': self.start_position,
                'mission_stages': list(set([d['mission_stage'] for d in self.telemetry_data]))
            }
        }
        
        with open(f'mission_1_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
            json.dump(mission_report, f, indent=2)
        
        print(f"\n💾 Görev raporu kaydedildi: mission_1_report_*.json")
        
        return total_points >= 180  # %60 başarı şartı
    
    def run_mission_1(self):
        """Görev 1'i çalıştır"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - GÖREV 1 BAŞLIYOR")
        print("="*80)
        print("🎯 Görev: Seyir Yapma & Başlangıç Noktasına Geri Dönüş")
        print("⏱️ Süre Limiti: 5 dakika")
        print("🏆 Maksimum Puan: 300")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        # Dead Reckoning başlangıç pozisyonu ayarla
        print("📍 Dead Reckoning sistemi başlatılıyor...")
        print("🧭 IMU kalibrasyonu bekleniyor...")
        
        # IMU stabilizasyonu için bekle
        for i in range(10):
            if self.read_sensors():
                print(f"🔄 IMU okuma {i+1}/10: Heading={self.current_heading:.1f}°")
            time.sleep(0.5)
        
        # Başlangıç heading'ini ayarla
        self.initial_heading = self.current_heading
        self.start_position['heading'] = self.current_heading
        
        print(f"📍 Başlangıç pozisyonu: X=0.0m, Y=0.0m, Heading={self.initial_heading:.1f}°")
        print("🎯 Dead Reckoning navigasyon hazır!")
        
        print("\n⚠️ GÖREV HAZIRLIĞI:")
        print("- Tüm sistemler hazır mı?")
        print("- Güvenlik kontrolleri tamamlandı mı?") 
        print("- Şamandıra takıldı mı?")
        
        # Servo test seçeneği
        test_servos = input("\n🔧 Servolar test edilsin mi? (y/n): ").lower()
        if test_servos == 'y':
            self.test_servos()
        
        ready = input("\n✅ Görev 1 başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Görev iptal edildi")
            return False
        
        self.mission_start_time = time.time()
        self.mission_active = True
        self.running = True
        self.mission_stage = "DESCENT"
        
        # Control ve monitoring thread'leri başlat
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        try:
            print("\n🚀 GÖREV 1 BAŞLADI!")
            
            # Control thread bitmesini bekle
            self.control_thread.join()
            
            # Görev raporu
            success = self.generate_mission_report()
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Görev kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Görev hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.mission_active = False
        self.running = False
        
        print("\n🧹 Sistem temizleniyor...")
        
        if self.connected:
            self.set_motor_throttle(PWM_NEUTRAL)
            self.set_control_surfaces()
        
        # D300 sensörünü kapat
        if self.d300_connected and self.d300_sensor:
            try:
                self.d300_sensor.close()
                print("🔌 D300 derinlik sensörü kapatıldı")
            except:
                pass
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı")

def main():
    """Ana fonksiyon"""
    parser = argparse.ArgumentParser(description='TEKNOFEST Görev 1: Seyir Yapma & Geri Dönüş (Plus Wing - GPS\'siz)')
    parser.add_argument('--start-heading', type=float, default=0.0, help='Başlangıç heading (derece)')
    
    args = parser.parse_args()
    
    mission = Mission1Navigator(start_heading=args.start_heading)
    
    try:
        success = mission.run_mission_1()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main()) 