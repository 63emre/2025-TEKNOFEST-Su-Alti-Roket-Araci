#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Manevrabilite Kabiliyeti Demo
Video çekimi için kontrollü hareket yetenekleri gösterimi
Şartname: En az 1 dakika kontrollü manevralar (seyir, dönüş, yunuslama)
"""

import time
import threading
import math
from datetime import datetime
from pymavlink import mavutil
import json

# MAVLink bağlantı adresi
# Serial MAVLink connection with environment variable support
import os  
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# Test parametreleri
DEMO_DEPTH = 2.0          # Demo derinliği (m)
MIN_MANEUVER_TIME = 60    # Minimum 1 dakika (şartname)
SURFACE_ASCENT_TEST = True # Yüzeye çıkış testi

# Sistem kanalları
MOTOR_CHANNEL = 8
SERVO_CHANNELS = {
    'fin_top': 1,     # Pitch kontrolü
    'fin_right': 2,   # Yaw kontrolü
    'fin_bottom': 3,  # Pitch kontrolü  
    'fin_left': 4     # Yaw kontrolü
}

# PWM değerleri
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

class ManeuverabilityDemo:
    def __init__(self):
        self.master = None
        self.connected = False
        self.demo_active = False
        
        # Navigation veriler
        self.current_depth = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0  
        self.current_yaw = 0.0
        self.current_speed = 0.0
        
        # Position tracking (basit)
        self.position_x = 0.0
        self.position_y = 0.0
        self.start_position = (0.0, 0.0)
        
        # Demo durumu
        self.demo_stage = "PREPARATION"
        self.demo_start_time = None
        self.maneuver_count = 0
        self.total_maneuver_time = 0
        
        # Performance metrikler
        self.max_roll_angle = 0.0
        self.max_pitch_angle = 0.0
        self.max_yaw_rate = 0.0
        self.stability_violations = 0
        
        # Veri kayıt
        self.maneuver_log = []
        self.telemetry_data = []
        
        # Threading
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
        """Sensör verilerini oku"""
        if not self.connected:
            return False
            
        try:
            # Attitude verisi
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                
                # Maksimum açıları kaydet
                self.max_roll_angle = max(self.max_roll_angle, abs(self.current_roll))
                self.max_pitch_angle = max(self.max_pitch_angle, abs(self.current_pitch))
            
            # Derinlik verisi (basınç sensöründen)
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if pressure_msg:
                # Basınç farkından derinlik hesapla
                depth_pressure = pressure_msg.press_abs - 1013.25  # Atmosfer basıncı
                self.current_depth = max(0, depth_pressure * 0.10197)  # hPa to meter
            
            # Hız verisi (VFR_HUD)
            vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if vfr_msg:
                self.current_speed = vfr_msg.groundspeed
            
            # Basit pozisyon tracking (dead reckoning)
            dt = 0.1  # 10Hz update
            self.position_x += self.current_speed * math.cos(math.radians(self.current_yaw)) * dt
            self.position_y += self.current_speed * math.sin(math.radians(self.current_yaw)) * dt
            
            # Telemetri kaydı
            timestamp = time.time()
            self.telemetry_data.append({
                'timestamp': timestamp,
                'depth': self.current_depth,
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'speed': self.current_speed,
                'position_x': self.position_x,
                'position_y': self.position_y,
                'stage': self.demo_stage
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def check_stability(self):
        """Stabilite kontrolü (kontrollü hareket değerlendirmesi için)"""
        # Şartname: "Kontrolsüz müdahale veya kararsız hareket kabul edilmeyecektir"
        
        stability_ok = True
        
        # Aşırı roll kontrolü
        if abs(self.current_roll) > 45:  # 45° limit
            print(f"⚠️ Aşırı roll açısı: {self.current_roll:.1f}°")
            stability_ok = False
            
        # Aşırı pitch kontrolü  
        if abs(self.current_pitch) > 60:  # 60° limit
            print(f"⚠️ Aşırı pitch açısı: {self.current_pitch:.1f}°")
            stability_ok = False
            
        if not stability_ok:
            self.stability_violations += 1
            
        return stability_ok
    
    def display_maneuver_status(self):
        """Manevra durumunu görüntüle"""
        print("\n" + "="*70)
        print(f"🎬 TEKNOFEST - MANEVRABİLİTE DEMOSu - {self.demo_stage}")
        print("="*70)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        demo_time = (time.time() - self.demo_start_time) if self.demo_start_time else 0
        
        print(f"⏰ Zaman: {timestamp} | Demo Süresi: {demo_time:.0f}s")
        print(f"🌊 Derinlik: {self.current_depth:.1f}m | Hız: {self.current_speed:.1f} m/s")
        print(f"🧭 Roll: {self.current_roll:+6.1f}° | Pitch: {self.current_pitch:+6.1f}° | Yaw: {self.current_yaw:06.1f}°")
        print(f"📍 Pozisyon: X={self.position_x:+6.1f}m, Y={self.position_y:+6.1f}m")
        print(f"🎯 Tamamlanan Manevralar: {self.maneuver_count} | Toplam Süre: {self.total_maneuver_time:.0f}s")
        print(f"⚠️ Stabilite İhlalleri: {self.stability_violations}")
        
        # Şartname gereksinimi
        min_time_status = "✅ YETER" if self.total_maneuver_time >= MIN_MANEUVER_TIME else f"❌ EKSİK ({MIN_MANEUVER_TIME}s gerekli)"
        print(f"📏 Minimum Süre Gereksinimi: {min_time_status}")
        
        print("="*70)
    
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
    
    def set_servo_position(self, channel, pwm_value):
        """Servo kontrolü"""
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
    
    def set_control_surfaces(self, roll_cmd=0, pitch_cmd=0, yaw_cmd=0):
        """Kontrol yüzeyleri (finler) ayarla"""
        # Fin mixing algoritması
        fin_commands = {
            'fin_top': PWM_NEUTRAL - pitch_cmd + yaw_cmd,      # Pitch/Yaw
            'fin_bottom': PWM_NEUTRAL + pitch_cmd + yaw_cmd,   # Pitch/Yaw
            'fin_right': PWM_NEUTRAL + roll_cmd + yaw_cmd,     # Roll/Yaw
            'fin_left': PWM_NEUTRAL - roll_cmd + yaw_cmd       # Roll/Yaw
        }
        
        # Servo komutları gönder
        for fin_name, pwm_value in fin_commands.items():
            channel = SERVO_CHANNELS[fin_name]
            self.set_servo_position(channel, int(pwm_value))
            
        return True
    
    def monitoring_loop(self):
        """Sürekli izleme döngüsü"""
        while self.running and self.demo_active:
            self.read_sensors()
            self.check_stability()
            
            # Her 3 saniyede durum göster
            if len(self.telemetry_data) % 30 == 0:  # 10Hz * 3s = 30 samples
                self.display_maneuver_status()
            
            time.sleep(0.1)  # 10Hz
    
    def descend_to_demo_depth(self):
        """Demo derinliğine in"""
        print(f"\n🎯 Demo derinliğine iniş: {DEMO_DEPTH}m")
        
        descend_start = time.time()
        
        while time.time() - descend_start < 90:  # Max 90s iniş süresi
            # Motor ile aşağı in
            self.set_motor_throttle(PWM_NEUTRAL - 80)  # Reverse thrust
            
            # Finlerle pitch kontrolü (aşağı yunuslama)
            self.set_control_surfaces(pitch_cmd=100)  # Nose down
            
            self.read_sensors()
            
            print(f"  📊 İniş: {self.current_depth:.1f}m / {DEMO_DEPTH}m | Pitch: {self.current_pitch:.1f}°")
            
            if self.current_depth >= DEMO_DEPTH:
                break
                
            time.sleep(2)
        
        # Finleri nötr pozisyona al
        self.set_control_surfaces()
        self.set_motor_throttle(PWM_NEUTRAL)
        
        if self.current_depth < DEMO_DEPTH * 0.8:  # En az %80'ine inmeli
            print(f"❌ Yeterli derinliğe inilemedi: {self.current_depth:.1f}m")
            return False
        
        print(f"✅ Demo derinliği: {self.current_depth:.1f}m")
        return True
    
    def maneuver_straight_cruise(self, duration=15):
        """Düz seyir manevası"""
        print(f"\n🚀 1. DÜZ SEYİR MANEVRAsı ({duration}s)")
        
        maneuver_start = time.time()
        initial_yaw = self.current_yaw
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # İleri motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 120)
            
            # Yön stabilizasyonu (heading hold)
            yaw_error = self.current_yaw - initial_yaw
            if yaw_error > 180:
                yaw_error -= 360
            elif yaw_error < -180:
                yaw_error += 360
                
            yaw_correction = int(yaw_error * -3)  # P kontrol
            
            # Pitch stabilizasyonu (level flight) 
            pitch_correction = int(self.current_pitch * -2)
            
            self.set_control_surfaces(yaw_cmd=yaw_correction, pitch_cmd=pitch_correction)
            
            print(f"  📊 Düz seyir: {remaining:.0f}s | Hız: {self.current_speed:.1f} m/s | Yaw error: {yaw_error:.1f}°")
            
            time.sleep(1)
        
        # Motor durdur
        self.set_motor_throttle(PWM_NEUTRAL)
        self.set_control_surfaces()
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        # Manevra kaydı
        self.maneuver_log.append({
            'maneuver': 'STRAIGHT_CRUISE',
            'duration': duration,
            'initial_yaw': initial_yaw,
            'final_yaw': self.current_yaw,
            'max_speed': max([data['speed'] for data in self.telemetry_data[-int(duration*10):]], default=0),
            'success': True
        })
        
        print("✅ Düz seyir tamamlandı!")
        return True
    
    def maneuver_turn_right(self, duration=12, target_angle=90):
        """Sağa dönüş manevası"""
        print(f"\n↪️ 2. SAĞ DÖNÜŞ MANEVRAsı ({duration}s, {target_angle}°)")
        
        maneuver_start = time.time()
        initial_yaw = self.current_yaw
        target_yaw = (initial_yaw + target_angle) % 360
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # Orta hız motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 80)
            
            # Sağ dönüş kontrolü
            yaw_error = target_yaw - self.current_yaw
            if yaw_error > 180:
                yaw_error -= 360
            elif yaw_error < -180:
                yaw_error += 360
                
            # Dönüş kontrolü (sağ finlerin farklı ayarı)
            turn_command = 150 if abs(yaw_error) > 10 else int(yaw_error * 10)
            roll_command = min(100, int(yaw_error * 2))  # Banked turn
            
            self.set_control_surfaces(roll_cmd=roll_command, yaw_cmd=turn_command)
            
            print(f"  📊 Sağ dönüş: {remaining:.0f}s | Yaw: {self.current_yaw:.1f}° -> {target_yaw:.1f}° | Error: {yaw_error:.1f}°")
            
            # Hedefe yakın olduk mu?
            if abs(yaw_error) < 5:
                print("  🎯 Hedef açıya ulaşıldı!")
                break
                
            time.sleep(1)
        
        # Stabilize et
        self.set_control_surfaces()
        self.set_motor_throttle(PWM_NEUTRAL)
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        actual_turn = abs(self.current_yaw - initial_yaw)
        if actual_turn > 180:
            actual_turn = 360 - actual_turn
        
        self.maneuver_log.append({
            'maneuver': 'TURN_RIGHT',
            'duration': duration,
            'target_angle': target_angle,
            'actual_angle': actual_turn,
            'initial_yaw': initial_yaw,
            'final_yaw': self.current_yaw,
            'success': abs(actual_turn - target_angle) < 15
        })
        
        print(f"✅ Sağ dönüş tamamlandı! Gerçek dönüş: {actual_turn:.1f}°")
        return True
    
    def maneuver_turn_left(self, duration=12, target_angle=90):
        """Sola dönüş manevası"""
        print(f"\n↩️ 3. SOL DÖNÜŞ MANEVRAsı ({duration}s, {target_angle}°)")
        
        maneuver_start = time.time()
        initial_yaw = self.current_yaw
        target_yaw = (initial_yaw - target_angle) % 360
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # Orta hız motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 80)
            
            # Sol dönüş kontrolü
            yaw_error = target_yaw - self.current_yaw
            if yaw_error > 180:
                yaw_error -= 360
            elif yaw_error < -180:
                yaw_error += 360
                
            # Sol dönüş komutları
            turn_command = -150 if abs(yaw_error) > 10 else int(yaw_error * 10)
            roll_command = max(-100, int(yaw_error * 2))  # Banked turn (sol)
            
            self.set_control_surfaces(roll_cmd=roll_command, yaw_cmd=turn_command)
            
            print(f"  📊 Sol dönüş: {remaining:.0f}s | Yaw: {self.current_yaw:.1f}° -> {target_yaw:.1f}° | Error: {yaw_error:.1f}°")
            
            if abs(yaw_error) < 5:
                print("  🎯 Hedef açıya ulaşıldı!")
                break
                
            time.sleep(1)
        
        # Stabilize et
        self.set_control_surfaces()
        self.set_motor_throttle(PWM_NEUTRAL)
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        actual_turn = abs(self.current_yaw - initial_yaw)
        if actual_turn > 180:
            actual_turn = 360 - actual_turn
        
        self.maneuver_log.append({
            'maneuver': 'TURN_LEFT',
            'duration': duration,
            'target_angle': target_angle,
            'actual_angle': actual_turn,
            'initial_yaw': initial_yaw,
            'final_yaw': self.current_yaw,
            'success': abs(actual_turn - target_angle) < 15
        })
        
        print(f"✅ Sol dönüş tamamlandı! Gerçek dönüş: {actual_turn:.1f}°")
        return True
    
    def maneuver_pitch_up(self, duration=10, target_angle=20):
        """Yukarı yunuslama manevası"""
        print(f"\n🔼 4. YUKARI YUNUSLAMA ({duration}s, {target_angle}°)")
        
        maneuver_start = time.time()
        initial_pitch = self.current_pitch
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # İleri motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 100)
            
            # Yukarı yunuslama (pitch up)
            pitch_error = target_angle - self.current_pitch
            pitch_command = min(200, max(-200, int(pitch_error * 8)))
            
            # Yön stabilizasyonu
            roll_stabilize = int(self.current_roll * -2)
            
            self.set_control_surfaces(roll_cmd=roll_stabilize, pitch_cmd=-pitch_command)
            
            print(f"  📊 Yunuslama: {remaining:.0f}s | Pitch: {self.current_pitch:.1f}° -> {target_angle}° | Derinlik: {self.current_depth:.1f}m")
            
            time.sleep(1)
        
        # Stabilize et
        self.set_control_surfaces()
        self.set_motor_throttle(PWM_NEUTRAL)
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        max_pitch = max([data['pitch'] for data in self.telemetry_data[-int(duration*10):]], default=0)
        
        self.maneuver_log.append({
            'maneuver': 'PITCH_UP',
            'duration': duration,
            'target_angle': target_angle,
            'max_pitch': max_pitch,
            'initial_pitch': initial_pitch,
            'final_pitch': self.current_pitch,
            'success': max_pitch >= target_angle * 0.7
        })
        
        print(f"✅ Yukarı yunuslama tamamlandı! Max pitch: {max_pitch:.1f}°")
        return True
    
    def maneuver_pitch_down(self, duration=10, target_angle=-20):
        """Aşağı yunuslama manevası"""
        print(f"\n🔽 5. AŞAĞI YUNUSLAMA ({duration}s, {target_angle}°)")
        
        maneuver_start = time.time()
        initial_pitch = self.current_pitch
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # İleri motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 100)
            
            # Aşağı yunuslama (pitch down)
            pitch_error = target_angle - self.current_pitch
            pitch_command = min(200, max(-200, int(pitch_error * 8)))
            
            # Yön stabilizasyonu
            roll_stabilize = int(self.current_roll * -2)
            
            self.set_control_surfaces(roll_cmd=roll_stabilize, pitch_cmd=-pitch_command)
            
            print(f"  📊 Yunuslama: {remaining:.0f}s | Pitch: {self.current_pitch:.1f}° -> {target_angle}° | Derinlik: {self.current_depth:.1f}m")
            
            time.sleep(1)
        
        # Stabilize et
        self.set_control_surfaces()
        self.set_motor_throttle(PWM_NEUTRAL)
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        min_pitch = min([data['pitch'] for data in self.telemetry_data[-int(duration*10):]], default=0)
        
        self.maneuver_log.append({
            'maneuver': 'PITCH_DOWN',
            'duration': duration,
            'target_angle': target_angle,
            'min_pitch': min_pitch,
            'initial_pitch': initial_pitch,
            'final_pitch': self.current_pitch,
            'success': min_pitch <= target_angle * 0.7
        })
        
        print(f"✅ Aşağı yunuslama tamamlandı! Min pitch: {min_pitch:.1f}°")
        return True
    
    def maneuver_surface_ascent(self, duration=20):
        """Yüzeye çıkış manevası (şartname gereği)"""
        print(f"\n🌊 6. YÜZEYE ÇIKIŞ MANEVRAsı ({duration}s)")
        print("📋 Şartname: İkinci görevde burun kapağı açılmadan yüzeye çıkış")
        
        maneuver_start = time.time()
        initial_depth = self.current_depth
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # Yukarı motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 150)
            
            # Yukarı pitch
            self.set_control_surfaces(pitch_cmd=-150)  # Nose up
            
            print(f"  📊 Yüzeye çıkış: {remaining:.0f}s | Derinlik: {self.current_depth:.1f}m | Pitch: {self.current_pitch:.1f}°")
            
            # Yüzeye çıktık mı?
            if self.current_depth < 0.5:  # 0.5m altına çıktık
                print("  🌊 Yüzey seviyesine ulaşıldı!")
                break
                
            time.sleep(1)
        
        # Yüzeyde stabilize
        self.set_motor_throttle(PWM_NEUTRAL)
        self.set_control_surfaces()
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        depth_change = initial_depth - self.current_depth
        
        self.maneuver_log.append({
            'maneuver': 'SURFACE_ASCENT',
            'duration': duration,
            'initial_depth': initial_depth,
            'final_depth': self.current_depth,
            'depth_change': depth_change,
            'success': self.current_depth < 1.0
        })
        
        print(f"✅ Yüzeye çıkış tamamlandı! Derinlik değişimi: {depth_change:.1f}m")
        return True
    
    def return_to_start_position(self, duration=25):
        """Başlangıç noktasına dönüş (bonus manevra)"""
        print(f"\n🏠 7. BAŞLANGIÇ POZİSYONUNA DÖNÜŞ ({duration}s)")
        
        # Başlangıç noktasına olan mesafe
        distance_to_start = math.sqrt(self.position_x**2 + self.position_y**2)
        bearing_to_start = math.degrees(math.atan2(-self.position_y, -self.position_x))
        
        print(f"  📍 Mesafe: {distance_to_start:.1f}m | Yön: {bearing_to_start:.0f}°")
        
        maneuver_start = time.time()
        
        while time.time() - maneuver_start < duration:
            elapsed = time.time() - maneuver_start
            remaining = duration - elapsed
            
            # Başlangıç noktasına yön
            current_distance = math.sqrt(self.position_x**2 + self.position_y**2)
            
            if current_distance > 2.0:  # 2m'den uzaksa
                # Başlangıca doğru git
                target_bearing = math.degrees(math.atan2(-self.position_y, -self.position_x))
                
                bearing_error = target_bearing - self.current_yaw
                if bearing_error > 180:
                    bearing_error -= 360
                elif bearing_error < -180:
                    bearing_error += 360
                
                # Motor ve fin kontrolü
                self.set_motor_throttle(PWM_NEUTRAL + 100)
                yaw_cmd = max(-100, min(100, int(bearing_error * 2)))
                self.set_control_surfaces(yaw_cmd=yaw_cmd)
                
                print(f"  📊 Dönüş: {remaining:.0f}s | Mesafe: {current_distance:.1f}m | Bearing error: {bearing_error:.1f}°")
            else:
                print(f"  🎯 Başlangıç noktasına yakın: {current_distance:.1f}m")
                self.set_motor_throttle(PWM_NEUTRAL)
                self.set_control_surfaces()
                break
                
            time.sleep(1)
        
        final_distance = math.sqrt(self.position_x**2 + self.position_y**2)
        
        self.maneuver_count += 1
        self.total_maneuver_time += duration
        
        self.maneuver_log.append({
            'maneuver': 'RETURN_TO_START',
            'duration': duration,
            'initial_distance': distance_to_start,
            'final_distance': final_distance,
            'success': final_distance < distance_to_start * 0.5
        })
        
        print(f"✅ Başlangıç dönüşü tamamlandı! Final mesafe: {final_distance:.1f}m")
        return True
    
    def generate_demo_report(self):
        """Demo test raporu"""
        print("\n" + "="*70)
        print("📋 MANEVRABİLİTE KABİLİYETİ DEMO RAPORU")
        print("="*70)
        
        demo_duration = time.time() - self.demo_start_time if self.demo_start_time else 0
        
        print(f"📅 Demo Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Toplam Demo Süresi: {demo_duration/60:.1f} dakika")
        print(f"🌊 Maksimum Derinlik: {max([d['depth'] for d in self.telemetry_data], default=0):.1f}m")
        print(f"🚀 Maksimum Hız: {max([d['speed'] for d in self.telemetry_data], default=0):.1f} m/s")
        
        print(f"\n📊 MANEVRABILITE METRİKLERİ:")
        print("-"*50)
        print(f"🎯 Tamamlanan Manevralar: {self.maneuver_count}")
        print(f"⏱️ Toplam Manevrabilite Süresi: {self.total_maneuver_time}s")
        print(f"📐 Maksimum Roll Açısı: {self.max_roll_angle:.1f}°")
        print(f"📐 Maksimum Pitch Açısı: {self.max_pitch_angle:.1f}°")
        print(f"⚠️ Stabilite İhlalleri: {self.stability_violations}")
        
        print(f"\n📋 MANEVRA DETAYLARI:")
        print("-"*50)
        
        total_success = 0
        for i, log_entry in enumerate(self.maneuver_log):
            maneuver = log_entry['maneuver']
            success = log_entry['success']
            duration = log_entry['duration']
            
            status_icon = "✅" if success else "❌"
            maneuver_names = {
                'STRAIGHT_CRUISE': 'Düz Seyir',
                'TURN_RIGHT': 'Sağ Dönüş',
                'TURN_LEFT': 'Sol Dönüş', 
                'PITCH_UP': 'Yukarı Yunuslama',
                'PITCH_DOWN': 'Aşağı Yunuslama',
                'SURFACE_ASCENT': 'Yüzeye Çıkış',
                'RETURN_TO_START': 'Başlangıç Dönüşü'
            }
            
            maneuver_name = maneuver_names.get(maneuver, maneuver)
            print(f"  {status_icon} {i+1}. {maneuver_name}: {duration}s - {'BAŞARILI' if success else 'BAŞARISIZ'}")
            
            if success:
                total_success += 1
        
        # Şartname değerlendirmesi
        print(f"\n🎯 ŞARTNAME GEREKSİNİMLERİ:")
        print("-"*40)
        
        time_ok = self.total_maneuver_time >= MIN_MANEUVER_TIME
        time_icon = "✅" if time_ok else "❌"
        print(f"  {time_icon} Minimum Manevrabilite Süresi (≥{MIN_MANEUVER_TIME}s): {self.total_maneuver_time}s")
        
        stability_ok = self.stability_violations == 0
        stability_icon = "✅" if stability_ok else "❌"
        print(f"  {stability_icon} Kontrollü Hareket: {self.stability_violations} ihlal")
        
        maneuver_variety_ok = len(set([log['maneuver'] for log in self.maneuver_log])) >= 4
        variety_icon = "✅" if maneuver_variety_ok else "❌"
        print(f"  {variety_icon} Manevra Çeşitliliği: {len(set([log['maneuver'] for log in self.maneuver_log]))} farklı tip")
        
        surface_ascent_ok = any(log['maneuver'] == 'SURFACE_ASCENT' and log['success'] for log in self.maneuver_log)
        surface_icon = "✅" if surface_ascent_ok else "❌"
        print(f"  {surface_icon} Yüzeye Çıkış: {'BAŞARILI' if surface_ascent_ok else 'BAŞARISIZ'}")
        
        # Başarı oranı
        success_rate = (total_success / len(self.maneuver_log)) * 100 if self.maneuver_log else 0
        print(f"\n📈 Manevrabilite Başarı Oranı: {success_rate:.1f}% ({total_success}/{len(self.maneuver_log)})")
        
        # Final değerlendirme
        overall_success = time_ok and stability_ok and maneuver_variety_ok and surface_ascent_ok and success_rate >= 70
        
        print(f"\n🏆 GENEL SONUÇ:")
        print("="*30)
        
        if overall_success:
            print("🎉 MANEVRABİLİTE KABİLİYETİ TAM BAŞARI!")
            print("📹 Video çekimi için hazır!")
        else:
            print("❌ MANEVRABİLİTE KABİLİYETİ EKSİKLİKLER VAR!")
            print("🔧 Sistem iyileştirmeleri gerekli!")
        
        # Veri kaydet
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'demo_duration': demo_duration,
            'total_maneuver_time': self.total_maneuver_time,
            'maneuver_count': self.maneuver_count,
            'stability_violations': self.stability_violations,
            'max_roll_angle': self.max_roll_angle,
            'max_pitch_angle': self.max_pitch_angle,
            'maneuver_results': self.maneuver_log,
            'telemetry_summary': {
                'max_depth': max([d['depth'] for d in self.telemetry_data], default=0),
                'max_speed': max([d['speed'] for d in self.telemetry_data], default=0),
                'total_samples': len(self.telemetry_data)
            },
            'overall_success': overall_success
        }
        
        with open(f'maneuver_demo_report_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
            json.dump(report_data, f, indent=2)
        
        print(f"\n💾 Demo raporu kaydedildi: maneuver_demo_report_*.json")
        
        return overall_success
    
    def run_full_maneuverability_demo(self):
        """Tam manevrabilite demo"""
        print("🎬 TEKNOFEST Su Altı Roket Aracı - MANEVRABİLİTE DEMOsu")
        print("="*70)
        print("📹 Video çekimi için manevrabilite yetenekleri gösterimi")
        print("⏱️ Tahmini süre: 6-8 dakika")
        print("🎯 Şartname: ≥1 dakika kontrollü manevralar, yüzeye çıkış")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("\n⚠️ GÜVENLİK UYARISI:")
        print("- Manevrabilite alanı temiz ve güvenli mi?")
        print("- Kameralar tüm açıları kaydediyor mu?")
        print("- Acil müdahale ekibi hazır mı?")
        
        ready = input("\n✅ Manevrabilite demosu başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Demo iptal edildi")
            return False
        
        self.demo_start_time = time.time()
        self.demo_active = True
        self.running = True
        
        # Start position kaydet
        self.start_position = (0.0, 0.0)
        self.position_x = 0.0
        self.position_y = 0.0
        
        # Monitoring thread başlat
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        try:
            print("\n🚀 MANEVRABİLİTE DEMOsu BAŞLADI!")
            
            # 1. Demo derinliğine in
            self.demo_stage = "DESCENT"
            if not self.descend_to_demo_depth():
                print("❌ Demo derinliğine inilemedi!")
                return False
            
            input("\n⏸️ Derinlik tamam! Manevrabilite testlerine başlanacak. ENTER...")
            
            # 2. Manevrabilite testleri
            self.demo_stage = "MANEUVERS"
            
            # Düz seyir
            self.maneuver_straight_cruise(15)
            time.sleep(3)  # Stabilizasyon arası
            
            # Sağ dönüş
            self.maneuver_turn_right(12, 90)
            time.sleep(3)
            
            # Sol dönüş
            self.maneuver_turn_left(12, 90)
            time.sleep(3)
            
            # Yukarı yunuslama
            self.maneuver_pitch_up(10, 20)
            time.sleep(3)
            
            # Aşağı yunuslama
            self.maneuver_pitch_down(10, -20)
            time.sleep(3)
            
            input("\n⏸️ Temel manevralar tamam! Yüzeye çıkış testine geçilsin mi? ENTER...")
            
            # 3. Yüzeye çıkış testi
            self.demo_stage = "SURFACE_ASCENT"
            self.maneuver_surface_ascent(20)
            
            time.sleep(5)  # Yüzeyde bekle
            
            # 4. (Opsiyonel) Başlangıç pozisyonuna dönüş
            if input("\n🏠 Başlangıç pozisyonuna dönülsün mü? (y/n): ").lower() == 'y':
                self.demo_stage = "RETURN_HOME"
                self.return_to_start_position(25)
            
            # 5. Demo raporu
            success = self.generate_demo_report()
            
            if success:
                print("\n🎉 MANEVRABİLİTE DEMOsu BAŞARILI!")
                print("📹 Video montaja hazır!")
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Demo kullanıcı tarafından durduruldu")
            return False
        except Exception as e:
            print(f"\n❌ Demo hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.demo_active = False
        self.running = False
        
        print("\n🧹 Sistem temizleniyor...")
        
        if self.connected:
            # Motorları durdur
            self.set_motor_throttle(PWM_NEUTRAL)
            
            # Finleri nötr pozisyon
            self.set_control_surfaces()
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı")

def main():
    """Ana fonksiyon"""
    demo = ManeuverabilityDemo()
    
    try:
        success = demo.run_full_maneuverability_demo()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main()) 