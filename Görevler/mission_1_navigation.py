#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş

Görev Açıklaması:
- Başlangıç bölgesinden 2m derinlikte düz istikamette 10m ilerle (süre başlatılır)
- Kıyıdan en az 50m uzaklaş
- Başlangıç noktasına otonom geri dön
- Pozitif sephiye ile yüzeye çıkıp enerjiyi kes

Puanlama:
- Seyir yapma (hız): 150 puan
- Başlangıç noktasında enerji kesme: 90 puan  
- Sızdırmazlık: 60 puan
- Süre limiti: 5 dakika
- Toplam: 300 puan
"""

import time
import threading
import math
import json
import argparse
from datetime import datetime
from pymavlink import mavutil
import numpy as np

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

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

# Kontrol parametreleri
CONTROL_PARAMS = {
    'depth_pid': {'kp': 100.0, 'ki': 5.0, 'kd': 30.0},
    'heading_pid': {'kp': 3.0, 'ki': 0.1, 'kd': 0.5},
    'position_pid': {'kp': 2.0, 'ki': 0.05, 'kd': 0.3}
}

# Servo ve motor kanalları
MOTOR_CHANNEL = 8
SERVO_CHANNELS = {
    'fin_top': 1,
    'fin_right': 2,
    'fin_bottom': 3,
    'fin_left': 4
}

# PWM değerleri
PWM_NEUTRAL = 1500
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
    def __init__(self, start_lat=None, start_lon=None):
        self.master = None
        self.connected = False
        self.mission_active = False
        
        # Navigasyon durumu
        self.start_position = {'lat': start_lat, 'lon': start_lon, 'alt': 0.0}
        self.current_position = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_speed = 0.0
        
        # Attitude veriler
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0
        
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
        self.position_pid_lat = PIDController(**CONTROL_PARAMS['position_pid'])
        self.position_pid_lon = PIDController(**CONTROL_PARAMS['position_pid'])
        
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
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def read_sensors(self):
        """Tüm sensör verilerini oku"""
        if not self.connected:
            return False
            
        try:
            # GPS pozisyon
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
            if gps_msg and gps_msg.fix_type >= 3:  # 3D fix
                self.current_position['lat'] = gps_msg.lat / 1e7
                self.current_position['lon'] = gps_msg.lon / 1e7
                self.current_position['alt'] = gps_msg.alt / 1e3
            
            # Attitude (IMU)
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw
            
            # Basınç/derinlik
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if pressure_msg:
                depth_pressure = pressure_msg.press_abs - 1013.25
                self.current_depth = max(0, depth_pressure * 0.10197)
            
            # Hız
            vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if vfr_msg:
                self.current_speed = vfr_msg.groundspeed
            
            # Telemetri kaydet
            timestamp = time.time()
            self.telemetry_data.append({
                'timestamp': timestamp,
                'position': self.current_position.copy(),
                'depth': self.current_depth,
                'heading': self.current_heading,
                'speed': self.current_speed,
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'mission_stage': self.mission_stage
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def calculate_distance_bearing(self, lat1, lon1, lat2, lon2):
        """İki nokta arası mesafe ve bearing hesapla"""
        # Haversine formula
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        dlat_rad = math.radians(lat2 - lat1)
        dlon_rad = math.radians(lon2 - lon1)
        
        a = (math.sin(dlat_rad/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon_rad/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        # Bearing hesaplama
        y = math.sin(dlon_rad) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dlon_rad))
        bearing = math.degrees(math.atan2(y, x))
        bearing = (bearing + 360) % 360  # 0-360 range
        
        return distance, bearing
    
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
        """Kontrol yüzeylerini ayarla"""
        fin_commands = {
            'fin_top': PWM_NEUTRAL - pitch_cmd + yaw_cmd,
            'fin_bottom': PWM_NEUTRAL + pitch_cmd + yaw_cmd,
            'fin_right': PWM_NEUTRAL + roll_cmd + yaw_cmd,
            'fin_left': PWM_NEUTRAL - roll_cmd + yaw_cmd
        }
        
        for fin_name, pwm_value in fin_commands.items():
            channel = SERVO_CHANNELS[fin_name]
            self.set_servo_position(channel, int(pwm_value))
    
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
        
        # Pozisyon bilgisi
        if self.start_position['lat']:
            distance_from_start, bearing_to_start = self.calculate_distance_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self.start_position['lat'], self.start_position['lon']
            )
            print(f"📍 Mevcut Pozisyon: {self.current_position['lat']:.6f}, {self.current_position['lon']:.6f}")
            print(f"🏠 Başlangıçtan Uzaklık: {distance_from_start:.1f}m | Yön: {bearing_to_start:.0f}°")
        
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
        
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            # PID kontrol ile iniş
            depth_output = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            
            # Motor ve fin kontrolü
            if depth_error > 0:  # Daha derine inmeli
                motor_throttle = PWM_NEUTRAL - min(100, abs(depth_output) // 3)
                pitch_cmd = min(150, abs(depth_output) // 2)  # Nose down
            else:  # Yükselmeli
                motor_throttle = PWM_NEUTRAL + min(100, abs(depth_output) // 3)
                pitch_cmd = -min(150, abs(depth_output) // 2)  # Nose up
            
            self.set_motor_throttle(motor_throttle)
            self.set_control_surfaces(pitch_cmd=pitch_cmd)
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
        initial_heading = 0  # Bu başlangıçta ayarlanmalı
        heading_error = initial_heading - self.current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        heading_correction = self.heading_pid.update(initial_heading, self.current_heading)
        yaw_cmd = max(-100, min(100, int(heading_correction)))
        
        self.set_motor_throttle(motor_throttle)
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
        
        # 10m düz seyir tamamlandı mı?
        if self.straight_distance_completed >= MISSION_PARAMS['straight_distance']:
            print("✅ 10m düz seyir tamamlandı! Kıyıdan uzaklaşmaya başlanıyor...")
            self.mission_stage = "OFFSHORE_CRUISE"
    
    def execute_offshore_cruise(self):
        """Kıyıdan 50m uzaklaşma"""
        if self.start_position['lat']:
            distance_from_start, _ = self.calculate_distance_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self.start_position['lat'], self.start_position['lon']
            )
            
            self.max_offshore_distance = max(self.max_offshore_distance, distance_from_start)
            
            # Hızlı ileri hareket
            motor_throttle = PWM_NEUTRAL + 150
            
            # Derinlik tutma
            depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = max(-150, min(150, int(depth_correction)))
            
            self.set_motor_throttle(motor_throttle)
            self.set_control_surfaces(pitch_cmd=pitch_cmd)
            
            # 50m uzaklaştık mı?
            if distance_from_start >= MISSION_PARAMS['min_offshore_distance']:
                print("✅ 50m uzaklaşma tamamlandı! Geri dönüş navigasyonu başlıyor...")
                self.mission_stage = "RETURN_NAVIGATION"
    
    def execute_return_navigation(self):
        """Başlangıç noktasına geri dönüş"""
        if self.start_position['lat']:
            distance_from_start, bearing_to_start = self.calculate_distance_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self.start_position['lat'], self.start_position['lon']
            )
            
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
            yaw_cmd = max(-150, min(150, int(heading_correction)))
            
            # Derinlik tutma
            depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = max(-150, min(150, int(depth_correction)))
            
            self.set_motor_throttle(motor_throttle)
            self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)
            
            # Başlangıç noktasına yaklaştık mı?
            if distance_from_start <= MISSION_PARAMS['position_tolerance'] * 2:  # 4m tolerance
                print("✅ Başlangıç noktasına yaklaşıldı! Final yaklaşım...")
                self.mission_stage = "FINAL_APPROACH"
    
    def execute_final_approach(self):
        """Final yaklaşım ve pozisyon tutma"""
        if self.start_position['lat']:
            distance_from_start, bearing_to_start = self.calculate_distance_bearing(
                self.current_position['lat'], self.current_position['lon'],
                self.start_position['lat'], self.start_position['lon']
            )
            
            self.final_position_error = distance_from_start
            
            # Hassas poziyon kontrolü
            if distance_from_start > MISSION_PARAMS['position_tolerance']:
                # Yavaş yaklaşım
                motor_throttle = PWM_NEUTRAL + 60
                
                heading_correction = self.heading_pid.update(bearing_to_start, self.current_heading)
                yaw_cmd = max(-100, min(100, int(heading_correction)))
            else:
                # Pozisyon tutma
                motor_throttle = PWM_NEUTRAL + 30
                yaw_cmd = 0
            
            # Derinlik tutma
            depth_correction = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            pitch_cmd = max(-100, min(100, int(depth_correction)))
            
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
        
        # Başlangıç pozisyonu ayarla
        if not self.start_position['lat']:
            print("📍 Başlangıç pozisyonu GPS'ten alınıyor...")
            for _ in range(30):  # 30 saniye GPS bekle
                if self.read_sensors() and self.current_position['lat']:
                    self.start_position = self.current_position.copy()
                    break
                time.sleep(1)
        
        if not self.start_position['lat']:
            print("❌ GPS sinyali alınamadı!")
            return False
        
        print(f"📍 Başlangıç pozisyonu: {self.start_position['lat']:.6f}, {self.start_position['lon']:.6f}")
        
        print("\n⚠️ GÖREV HAZIRLIĞI:")
        print("- Tüm sistemler hazır mı?")
        print("- Güvenlik kontrolleri tamamlandı mı?") 
        print("- Şamandıra takıldı mı?")
        
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
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı")

def main():
    """Ana fonksiyon"""
    parser = argparse.ArgumentParser(description='TEKNOFEST Görev 1: Seyir Yapma & Geri Dönüş')
    parser.add_argument('--start-lat', type=float, help='Başlangıç latitude')
    parser.add_argument('--start-lon', type=float, help='Başlangıç longitude')
    
    args = parser.parse_args()
    
    mission = Mission1Navigator(start_lat=args.start_lat, start_lon=args.start_lon)
    
    try:
        success = mission.run_mission_1()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main()) 