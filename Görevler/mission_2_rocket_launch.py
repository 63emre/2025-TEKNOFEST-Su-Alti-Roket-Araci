#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
Görev 2: Roket Ateşleme (Rocket Launch Mission)

Bu script, aracın güvenli atış bölgesine otonom ulaşması, +30° eğim koşuluyla yüzeye çıkış
ve roket taşıma bölmesinin açılarak ayrılmasını gösterir.
Fiziksel roket fırlatılması beklenmez, yalnızca ayrılma mekanizması test edilir.

Puanlama Kriterleri:
- Güvenli Atış Bölgesine Ulaşma: 100 puan
- Su Yüzeyine İstenen Açıyla Güvenli Çıkış: 100 puan  
- Model Roketin Güvenli Ayrılması: 150 puan
- Sızdırmazlık: 50 puan
Maksimum: 400 puan, Süre limiti: 5 dk

Hardware: Raspberry Pi 4B + Pixhawk PX4 PIX 2.4.8 + 4x DS3230MG Servo + DEGZ Motor + ESC
Protocol: MAVLink via tcp:127.0.0.1:5777
"""

from pymavlink import mavutil
import time
import math
import json
import argparse
import numpy as np
from datetime import datetime
import threading
import sys
import os

# MAVLink connection
# Serial MAVLink connection with environment variable support
import os
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# Mission Parameters
MISSION_PARAMS = {
    'operational_depth': 2.0,          # İşletme derinliği (m)
    'launch_zone_radius': 5.0,         # Atış bölgesi toleransı (m)  
    'target_pitch_angle': 30.0,        # Hedef yunuslama açısı (derece)
    'surface_depth_threshold': 0.3,    # Yüzey derinlik eşiği (m)
    'cruise_speed': 1.0,               # Seyir hızı (m/s)
    'timeout_seconds': 300,            # 5 dakika süre limiti
    'position_tolerance': 2.0,         # Pozisyon toleransı (m)
    'angle_tolerance': 5.0,            # Açı toleransı (derece)
    'surface_hold_duration': 10        # Yüzeyde bekleme süresi (s)
}

# Control Parameters  
CONTROL_PARAMS = {
    'depth_pid': {'kp': 120.0, 'ki': 8.0, 'kd': 40.0, 'max_output': 400, 'deadband': 0.1},
    'heading_pid': {'kp': 3.0, 'ki': 0.2, 'kd': 1.0, 'max_output': 300, 'deadband': 2.0},
    'pitch_pid': {'kp': 2.5, 'ki': 0.15, 'kd': 0.8, 'max_output': 400, 'deadband': 1.0}
}

# Hardware Configuration
SERVO_CHANNELS = {'fin_1': 1, 'fin_2': 2, 'fin_3': 3, 'fin_4': 4, 'elevator': 5}
MOTOR_CHANNEL = 8
PAYLOAD_BAY_SERVO = 9        # Payload bay kapağı
SEPARATION_MECHANISM = 10    # Ayrılma mekanizması (servo/solenoid)

PWM_MIN = 1000
PWM_NEUTRAL = 1500  
PWM_MAX = 2000

class PIDController:
    """Gelişmiş PID Controller sınıfı"""
    def __init__(self, kp, ki, kd, max_output=500, deadband=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.deadband = deadband
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, measurement):
        """PID güncelleme fonksiyonu"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0.0:
            return 0.0
            
        error = setpoint - measurement
        
        # Deadband check
        if abs(error) < self.deadband:
            return 0.0
            
        # Proportional term
        proportional = self.kp * error
        
        # Integral term  
        self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.previous_error) / dt
        derivative_term = self.kd * derivative
        
        # Calculate output
        output = proportional + integral_term + derivative_term
        
        # Apply output limits
        output = max(-self.max_output, min(self.max_output, output))
        
        # Update for next iteration
        self.previous_error = error
        self.last_time = current_time
        
        return output
    
    def reset(self):
        """PID değerlerini sıfırla"""
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

class Mission2RocketLaunch:
    """Görev 2: Roket Ateşleme Ana Sınıfı"""
    
    def __init__(self, launch_zone_lat=None, launch_zone_lon=None):
        # MAVLink Connection
        self.master = None
        self.connected = False
        
        # Mission State
        self.mission_active = False
        self.mission_completed = False
        self.emergency_stop = False
        self.start_time = None
        
        # Launch Zone Coordinates
        self.launch_zone = {'lat': launch_zone_lat, 'lon': launch_zone_lon}
        
        # Vehicle State
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_pitch = 0.0
        self.current_roll = 0.0
        self.surface_pressure = 1013.25
        
        # Control Systems
        self.depth_pid = PIDController(**CONTROL_PARAMS['depth_pid'])
        self.heading_pid = PIDController(**CONTROL_PARAMS['heading_pid'])  
        self.pitch_pid = PIDController(**CONTROL_PARAMS['pitch_pid'])
        
        # Mission Stages
        self.mission_stages = {
            'descent': False,
            'navigate_to_launch_zone': False,
            'surface_approach': False,
            'achieve_launch_angle': False,
            'rocket_separation': False,
            'mission_complete': False
        }
        
        # Mission Data
        self.mission_data = {
            'start_time': None,
            'completion_time': None,
            'total_duration': 0.0,
            'max_depth_achieved': 0.0,
            'distance_to_launch_zone': 0.0,
            'final_pitch_angle': 0.0,
            'separation_successful': False,
            'scoring': {
                'launch_zone_reached': False,      # 100 pts
                'safe_surface_angle': False,       # 100 pts  
                'rocket_separation': False,        # 150 pts
                'waterproofing': True              # 50 pts (assumed)
            }
        }
    
    def connect_pixhawk(self):
        """Pixhawk MAVLink bağlantısı"""
        try:
            print("🔌 Pixhawk'a bağlanılıyor...")
            
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
            print(f"❌ MAVLink bağlantı hatası: {e}")
            return False
    
    def disconnect_pixhawk(self):
        """MAVLink bağlantısını kapat"""
        if self.master:
            self.master.close()
            self.connected = False
            print("🔌 MAVLink bağlantısı kapatıldı")
    
    def read_sensors(self):
        """Sensör verilerini oku"""
        try:
            # GPS Position
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
            if gps_msg:
                self.current_position['lat'] = gps_msg.lat / 1e7
                self.current_position['lon'] = gps_msg.lon / 1e7
                self.current_position['alt'] = gps_msg.alt / 1000.0
            
            # Attitude (IMU)
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)  
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_heading = math.degrees(attitude_msg.yaw)
                if self.current_heading < 0:
                    self.current_heading += 360
            
            # Pressure/Depth
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if pressure_msg:
                self.current_depth = max(0, (pressure_msg.press_abs - self.surface_pressure) * 0.10197)
                if self.current_depth > self.mission_data['max_depth_achieved']:
                    self.mission_data['max_depth_achieved'] = self.current_depth
                    
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def calculate_distance_bearing(self, lat1, lon1, lat2, lon2):
        """İki GPS koordinatı arası mesafe ve azimut hesaplama"""
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlat = lat2_rad - lat1_rad
        dlon = lon2_rad - lon1_rad
        
        # Haversine formula for distance
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = 6371000 * c  # Earth radius in meters
        
        # Bearing calculation
        y = math.sin(dlon) * math.cos(lat2_rad)
        x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon)
        bearing = math.degrees(math.atan2(y, x))
        bearing = (bearing + 360) % 360  # Normalize to 0-360
        
        return distance, bearing
    
    def set_servo_position(self, channel, pwm_value):
        """Servo pozisyon kontrolü"""
        if not self.connected or self.emergency_stop:
            return False
            
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, channel, pwm_value, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Servo {channel} kontrol hatası: {e}")
            return False
    
    def set_motor_throttle(self, throttle_pwm):
        """Motor throttle kontrolü"""
        if not self.connected or self.emergency_stop:
            return False
            
        throttle_pwm = max(PWM_MIN, min(PWM_MAX, throttle_pwm))
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, MOTOR_CHANNEL, throttle_pwm, 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Motor kontrol hatası: {e}")
            return False
    
    def execute_descent(self):
        """Aşama 1: İşletme derinliğine (2m) inme"""
        print("🌊 Aşama 1: İşletme derinliğine iniş başladı")
        
        target_depth = MISSION_PARAMS['operational_depth']
        descent_timeout = 60  # 1 dakika
        start_time = time.time()
        
        self.depth_pid.reset()
        
        while time.time() - start_time < descent_timeout:
            if self.emergency_stop:
                return False
                
            self.read_sensors()
            
            depth_error = abs(self.current_depth - target_depth)
            if depth_error < CONTROL_PARAMS['depth_pid']['deadband']:
                print(f"✅ Hedef derinlik ({target_depth}m) başarıyla ulaşıldı!")
                self.mission_stages['descent'] = True
                return True
            
            # PID kontrol
            depth_output = self.depth_pid.update(target_depth, self.current_depth)
            motor_throttle = PWM_NEUTRAL + depth_output
            elevator_output = PWM_NEUTRAL - (depth_output // 2)
            
            self.set_motor_throttle(motor_throttle)
            self.set_servo_position(SERVO_CHANNELS['elevator'], elevator_output)
            
            print(f"📊 Derinlik: {self.current_depth:.2f}m, Hedef: {target_depth}m, Hata: {depth_error:.2f}m")
            time.sleep(0.5)
        
        print("⚠️ Derinlik iniş timeout!")
        return False
    
    def execute_navigate_to_launch_zone(self):
        """Aşama 2: Güvenli atış bölgesine navigasyon"""
        print("🎯 Aşama 2: Atış bölgesine navigasyon başladı")
        
        if not self.launch_zone['lat'] or not self.launch_zone['lon']:
            print("❌ Atış bölgesi koordinatları tanımlanmamış!")
            return False
        
        navigation_timeout = 180  # 3 dakika
        start_time = time.time()
        
        self.heading_pid.reset()
        
        while time.time() - start_time < navigation_timeout:
            if self.emergency_stop:
                return False
                
            self.read_sensors()
            
            # Mevcut pozisyondan atış bölgesine mesafe ve azimut hesapla
            distance, target_bearing = self.calculate_distance_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self.launch_zone['lat'], self.launch_zone['lon']
            )
            
            self.mission_data['distance_to_launch_zone'] = distance
            
            if distance < MISSION_PARAMS['launch_zone_radius']:
                print(f"✅ Atış bölgesine başarıyla ulaşıldı! (Mesafe: {distance:.1f}m)")
                self.mission_data['scoring']['launch_zone_reached'] = True
                self.mission_stages['navigate_to_launch_zone'] = True
                return True
            
            # Heading kontrolü
            heading_error = target_bearing - self.current_heading
            if heading_error > 180:
                heading_error -= 360
            elif heading_error < -180:
                heading_error += 360
            
            if abs(heading_error) > CONTROL_PARAMS['heading_pid']['deadband']:
                heading_output = self.heading_pid.update(target_bearing, self.current_heading)
                
                # Fin mixing for heading control
                left_fins = PWM_NEUTRAL - heading_output
                right_fins = PWM_NEUTRAL + heading_output
                
                self.set_servo_position(SERVO_CHANNELS['fin_1'], right_fins)
                self.set_servo_position(SERVO_CHANNELS['fin_2'], left_fins)
                self.set_servo_position(SERVO_CHANNELS['fin_3'], right_fins)
                self.set_servo_position(SERVO_CHANNELS['fin_4'], left_fins)
            
            # İleri hareket
            forward_throttle = PWM_NEUTRAL + 200  # Moderate forward thrust
            self.set_motor_throttle(forward_throttle)
            
            # Derinliği koruma
            depth_output = self.depth_pid.update(MISSION_PARAMS['operational_depth'], self.current_depth)
            elevator_output = PWM_NEUTRAL - (depth_output // 2)
            self.set_servo_position(SERVO_CHANNELS['elevator'], elevator_output)
            
            print(f"📍 Mesafe: {distance:.1f}m, Azimut: {target_bearing:.1f}°, Başlık: {self.current_heading:.1f}°")
            time.sleep(1.0)
        
        print("⚠️ Atış bölgesi navigasyonu timeout!")
        return False
    
    def execute_surface_approach(self):
        """Aşama 3: Yüzeye kontrollü yaklaşım"""
        print("⬆️ Aşama 3: Yüzeye kontrollü yaklaşım başladı")
        
        surface_timeout = 60  # 1 dakika
        start_time = time.time()
        
        self.depth_pid.reset()
        
        while time.time() - start_time < surface_timeout:
            if self.emergency_stop:
                return False
                
            self.read_sensors()
            
            if self.current_depth <= MISSION_PARAMS['surface_depth_threshold']:
                print(f"✅ Yüzeye başarıyla ulaşıldı! (Derinlik: {self.current_depth:.2f}m)")
                self.mission_stages['surface_approach'] = True
                return True
            
            # Kontrollü çıkış
            depth_output = self.depth_pid.update(0.0, self.current_depth)  # Target: surface (0m)
            motor_throttle = PWM_NEUTRAL + depth_output
            elevator_output = PWM_NEUTRAL - (depth_output // 2)
            
            self.set_motor_throttle(motor_throttle)
            self.set_servo_position(SERVO_CHANNELS['elevator'], elevator_output)
            
            print(f"📊 Derinlik: {self.current_depth:.2f}m, Yüzeye kalan: {self.current_depth:.2f}m")
            time.sleep(0.5)
        
        print("⚠️ Yüzeye çıkış timeout!")
        return False
    
    def execute_achieve_launch_angle(self):
        """Aşama 4: +30° yunuslama açısını elde etme"""
        print("📐 Aşama 4: Atış açısı (+30°) elde etme başladı")
        
        target_pitch = MISSION_PARAMS['target_pitch_angle']
        angle_timeout = 45  # 45 saniye
        start_time = time.time()
        
        self.pitch_pid.reset()
        
        while time.time() - start_time < angle_timeout:
            if self.emergency_stop:
                return False
                
            self.read_sensors()
            
            pitch_error = abs(self.current_pitch - target_pitch)
            if pitch_error < MISSION_PARAMS['angle_tolerance']:
                print(f"✅ Hedef yunuslama açısı ({target_pitch}°) başarıyla elde edildi!")
                self.mission_data['final_pitch_angle'] = self.current_pitch
                self.mission_data['scoring']['safe_surface_angle'] = True
                self.mission_stages['achieve_launch_angle'] = True
                return True
            
            # Pitch kontrolü
            pitch_output = self.pitch_pid.update(target_pitch, self.current_pitch)
            
            # Elevator kontrolü ile pitch ayarı
            elevator_pwm = PWM_NEUTRAL + pitch_output
            self.set_servo_position(SERVO_CHANNELS['elevator'], elevator_pwm)
            
            # Motor ile pozisyonu koruma
            motor_throttle = PWM_NEUTRAL + 100  # Light thrust to maintain position
            self.set_motor_throttle(motor_throttle)
            
            print(f"📐 Pitch: {self.current_pitch:.1f}°, Hedef: {target_pitch}°, Hata: {pitch_error:.1f}°")
            time.sleep(0.5)
        
        print("⚠️ Atış açısı elde etme timeout!")
        return False
    
    def execute_rocket_separation(self):
        """Aşama 5: Roket ayrılma mekanizması aktivasyonu"""
        print("🚀 Aşama 5: Roket ayrılma mekanizması aktivasyonu")
        
        try:
            # 1. Payload bay kapağını aç
            print("📦 Payload bay açılıyor...")
            self.set_servo_position(PAYLOAD_BAY_SERVO, PWM_MAX)  # Open payload bay
            time.sleep(2.0)
            
            # 2. Ayrılma mekanizmasını aktive et
            print("⚡ Ayrılma mekanizması aktive ediliyor...")
            self.set_servo_position(SEPARATION_MECHANISM, PWM_MAX)  # Trigger separation
            time.sleep(3.0)
            
            # 3. Mekanizmayı nötr pozisyona getir
            print("🔄 Mekanizma nötr pozisyona getiriliyor...")
            self.set_servo_position(SEPARATION_MECHANISM, PWM_NEUTRAL)
            time.sleep(1.0)
            
            # 4. Payload bay'i kapat (güvenlik için)
            print("🔒 Payload bay kapatılıyor...")
            self.set_servo_position(PAYLOAD_BAY_SERVO, PWM_MIN)  # Close payload bay
            time.sleep(1.0)
            
            print("✅ Roket ayrılma mekanizması başarıyla çalıştırıldı!")
            self.mission_data['separation_successful'] = True
            self.mission_data['scoring']['rocket_separation'] = True
            self.mission_stages['rocket_separation'] = True
            
            return True
            
        except Exception as e:
            print(f"❌ Roket ayrılma hatası: {e}")
            return False
    
    def emergency_stop_procedure(self):
        """Acil durdurma prosedürü"""
        print("🚨 ACİL DURDURMA AKTİVE!")
        self.emergency_stop = True
        
        try:
            # Motorları durdur
            self.set_motor_throttle(PWM_NEUTRAL)
            
            # Servoları nötr pozisyona getir
            for channel in SERVO_CHANNELS.values():
                self.set_servo_position(channel, PWM_NEUTRAL)
            
            print("✅ Acil durdurma tamamlandı")
        except Exception as e:
            print(f"❌ Acil durdurma hatası: {e}")
    
    def calculate_mission_score(self):
        """Görev puanını hesapla"""
        score = 0
        scoring_details = []
        
        if self.mission_data['scoring']['launch_zone_reached']:
            score += 100
            scoring_details.append("✅ Güvenli Atış Bölgesine Ulaşma: 100 puan")
        else:
            scoring_details.append("❌ Güvenli Atış Bölgesine Ulaşma: 0 puan")
        
        if self.mission_data['scoring']['safe_surface_angle']:
            score += 100
            scoring_details.append("✅ Su Yüzeyine İstenen Açıyla Güvenli Çıkış: 100 puan")
        else:
            scoring_details.append("❌ Su Yüzeyine İstenen Açıyla Güvenli Çıkış: 0 puan")
        
        if self.mission_data['scoring']['rocket_separation']:
            score += 150
            scoring_details.append("✅ Model Roketin Güvenli Ayrılması: 150 puan")
        else:
            scoring_details.append("❌ Model Roketin Güvenli Ayrılması: 0 puan")
        
        if self.mission_data['scoring']['waterproofing']:
            score += 50
            scoring_details.append("✅ Sızdırmazlık: 50 puan")
        else:
            scoring_details.append("❌ Sızdırmazlık: 0 puan")
        
        return score, scoring_details
    
    def generate_mission_report(self):
        """Detaylı görev raporu oluştur"""
        report = {
            'mission_type': 'Görev 2: Roket Ateşleme',
            'timestamp': datetime.now().isoformat(),
            'mission_completed': self.mission_completed,
            'mission_data': self.mission_data,
            'mission_stages': self.mission_stages,
            'vehicle_performance': {
                'max_depth_achieved': self.mission_data['max_depth_achieved'],
                'final_pitch_angle': self.mission_data['final_pitch_angle'],
                'distance_to_launch_zone': self.mission_data['distance_to_launch_zone'],
                'separation_successful': self.mission_data['separation_successful']
            }
        }
        
        score, scoring_details = self.calculate_mission_score()
        report['scoring'] = {
            'total_score': score,
            'max_possible': 400,
            'score_percentage': (score / 400) * 100,
            'details': scoring_details
        }
        
        # Raporu dosyaya kaydet
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"mission_2_report_{timestamp}.json"
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(report, f, ensure_ascii=False, indent=2)
            print(f"📊 Mission raporu kaydedildi: {filename}")
        except Exception as e:
            print(f"❌ Rapor kayıt hatası: {e}")
        
        return report
    
    def run_mission_2(self):
        """Ana Mission 2 execution fonksiyonu"""
        print("🚀 TEKNOFEST 2025 - Görev 2: Roket Ateşleme Başlatılıyor!")
        print("=" * 60)
        
        if not self.launch_zone['lat'] or not self.launch_zone['lon']:
            print("❌ Atış bölgesi koordinatları eksik! Görev durduruldu.")
            return False
        
        # Başlangıç zamanını kaydet
        self.start_time = time.time()
        self.mission_data['start_time'] = datetime.now().isoformat()
        self.mission_active = True
        
        try:
            # Aşama 1: İşletme derinliğine iniş
            if not self.execute_descent():
                print("❌ Görev başarısız - Derinlik inişi tamamlanamadı")
                return False
            
            # Aşama 2: Atış bölgesine navigasyon
            if not self.execute_navigate_to_launch_zone():
                print("❌ Görev başarısız - Atış bölgesine ulaşılamadı")
                return False
            
            # Aşama 3: Yüzeye kontrollü yaklaşım
            if not self.execute_surface_approach():
                print("❌ Görev başarısız - Yüzeye çıkılamadı")
                return False
            
            # Aşama 4: +30° yunuslama açısını elde etme
            if not self.execute_achieve_launch_angle():
                print("❌ Görev başarısız - Atış açısı elde edilemedi")
                return False
            
            # Aşama 5: Roket ayrılma mekanizması
            if not self.execute_rocket_separation():
                print("❌ Görev başarısız - Roket ayrılması başarısız")
                return False
            
            # Görev tamamlandı
            self.mission_completed = True
            self.mission_data['completion_time'] = datetime.now().isoformat()
            self.mission_data['total_duration'] = time.time() - self.start_time
            
            print("🎉 GÖREV 2 BAŞARIYLA TAMAMLANDI!")
            print(f"⏱️ Toplam süre: {self.mission_data['total_duration']:.1f} saniye")
            
            return True
            
        except KeyboardInterrupt:
            print("⚠️ Görev kullanıcı tarafından durduruldu")
            self.emergency_stop_procedure()
            return False
        except Exception as e:
            print(f"❌ Görev hatası: {e}")
            self.emergency_stop_procedure()
            return False
        finally:
            self.mission_active = False
            # Motorları ve servoları güvenli pozisyona getir
            self.set_motor_throttle(PWM_NEUTRAL)
            for channel in SERVO_CHANNELS.values():
                self.set_servo_position(channel, PWM_NEUTRAL)

def main():
    parser = argparse.ArgumentParser(description='TEKNOFEST 2025 - Görev 2: Roket Ateşleme')
    parser.add_argument('--launch-lat', type=float, required=True,
                       help='Atış bölgesi latitude koordinatı (derece)')
    parser.add_argument('--launch-lon', type=float, required=True,
                       help='Atış bölgesi longitude koordinatı (derece)')
    parser.add_argument('--simulation', action='store_true',
                       help='Simülasyon modu (gerçek hardware olmadan test)')
    
    args = parser.parse_args()
    
    # Mission instance oluştur
    mission = Mission2RocketLaunch(
        launch_zone_lat=args.launch_lat,
        launch_zone_lon=args.launch_lon
    )
    
    print(f"🎯 Atış bölgesi: {args.launch_lat:.6f}, {args.launch_lon:.6f}")
    
    if args.simulation:
        print("🎮 Simülasyon modu aktif")
    
    # MAVLink bağlantısı
    if not args.simulation:
        if not mission.connect_pixhawk():
            print("❌ MAVLink bağlantısı başarısız!")
            return 1
    else:
        print("🔌 Simülasyon modunda MAVLink bağlantısı atlanıyor")
        mission.connected = True
    
    try:
        # Surface pressure kalibrasyonu
        if not args.simulation:
            print("🌊 Yüzey basınç kalibrasyonu...")
            mission.read_sensors()
            for _ in range(10):
                pressure_msg = mission.master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=5)
                if pressure_msg:
                    mission.surface_pressure = pressure_msg.press_abs
                time.sleep(0.5)
            print(f"📏 Yüzey basıncı: {mission.surface_pressure:.2f} mbar")
        
        # Mission başlat
        success = mission.run_mission_2()
        
        # Rapor oluştur
        report = mission.generate_mission_report()
        
        print("\n" + "=" * 60)
        print("📊 GÖREV RAPORU")
        print("=" * 60)
        
        score, scoring_details = mission.calculate_mission_score()
        print(f"🏆 Toplam Puan: {score}/400 (%{(score/400)*100:.1f})")
        
        for detail in scoring_details:
            print(detail)
        
        if success:
            print("\n🎉 Mission başarıyla tamamlandı!")
            return 0
        else:
            print("\n❌ Mission başarısız!")
            return 1
        
    except KeyboardInterrupt:
        print("\n⚠️ Program kullanıcı tarafından durduruldu")
        mission.emergency_stop_procedure()
        return 1
    finally:
        if not args.simulation:
            mission.disconnect_pixhawk()

if __name__ == "__main__":
    sys.exit(main()) 