#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Roket Ayrılma Mekanizması Demo
Video çekimi için roket ateşleme sisteminin gösterimi
Şartname: Su yüzeyinde +30° eğim koşulunda roket taşıma bölmesinin açılması
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
TARGET_SURFACE_ANGLE = 30   # +30° pitch angle (şartname)
SURFACE_DEPTH_THRESHOLD = 1.0  # Yüzey seviyesi (m)
SEPARATION_TIMEOUT = 30     # Ayrılma işlemi için max süre (s)

# Sistem kanalları
MOTOR_CHANNEL = 8
PAYLOAD_BAY_SERVO = 9       # Roket taşıma bölmesi servo
SEPARATION_MECHANISM = 10   # Ayrılma mekanizması servo/motor

# Fin servo kanalları
FIN_CHANNELS = {
    'fin_top': 1,
    'fin_right': 2, 
    'fin_bottom': 3,
    'fin_left': 4
}

# PWM değerleri
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

# Payload bay pozisyonları
PAYLOAD_CLOSED = PWM_MIN     # Kapalı pozisyon
PAYLOAD_OPEN = PWM_MAX       # Açık pozisyon (ayrılma)

class RocketSeparationDemo:
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
        
        # GPS/Position (basit tracking)
        self.current_lat = 0.0
        self.current_lon = 0.0
        
        # Demo durumu
        self.demo_stage = "PREPARATION"
        self.demo_start_time = None
        self.separation_completed = False
        self.surface_achieved = False
        self.target_angle_achieved = False
        
        # Payload bay durumu
        self.payload_bay_status = "CLOSED"
        self.separation_mechanism_status = "ARMED"
        
        # Performance verileri
        self.surface_time = None
        self.max_pitch_achieved = 0.0
        self.separation_time = None
        
        # Test log
        self.separation_log = []
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
                
                # Maximum pitch kaydet
                self.max_pitch_achieved = max(self.max_pitch_achieved, self.current_pitch)
            
            # Derinlik/basınç verisi
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if pressure_msg:
                depth_pressure = pressure_msg.press_abs - 1013.25
                self.current_depth = max(0, depth_pressure * 0.10197)
                
                # Yüzey kontrolü
                if self.current_depth <= SURFACE_DEPTH_THRESHOLD and not self.surface_achieved:
                    self.surface_achieved = True
                    self.surface_time = time.time()
                    print("🌊 YÜZEY SEVİYESİNE ULAŞILDI!")
            
            # Hız verisi
            vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            if vfr_msg:
                self.current_speed = vfr_msg.groundspeed
            
            # GPS verisi (opsiyonel)
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=False)
            if gps_msg:
                self.current_lat = gps_msg.lat / 1e7
                self.current_lon = gps_msg.lon / 1e7
            
            # Telemetri kaydı
            timestamp = time.time()
            self.telemetry_data.append({
                'timestamp': timestamp,
                'depth': self.current_depth,
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'speed': self.current_speed,
                'stage': self.demo_stage,
                'payload_status': self.payload_bay_status,
                'surface_achieved': self.surface_achieved,
                'separation_completed': self.separation_completed
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def display_separation_status(self):
        """Roket ayrılma durumunu göster"""
        print("\n" + "="*70)
        print(f"🚀 TEKNOFEST - ROKET AYRILMA DEMOsu - {self.demo_stage}")
        print("="*70)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        demo_time = (time.time() - self.demo_start_time) if self.demo_start_time else 0
        
        print(f"⏰ Zaman: {timestamp} | Demo Süresi: {demo_time:.0f}s")
        print(f"🌊 Derinlik: {self.current_depth:.1f}m | Hız: {self.current_speed:.1f} m/s")
        print(f"🧭 Roll: {self.current_roll:+6.1f}° | Pitch: {self.current_pitch:+6.1f}° | Yaw: {self.current_yaw:06.1f}°")
        
        # Yüzey durumu
        surface_status = "✅ YÜZEYDE" if self.surface_achieved else f"❌ SU ALTINDA ({self.current_depth:.1f}m)"
        print(f"🌊 Yüzey Durumu: {surface_status}")
        
        # Açı durumu
        angle_status = "✅ YETER" if self.current_pitch >= TARGET_SURFACE_ANGLE else f"❌ EKSİK ({self.current_pitch:.1f}° < {TARGET_SURFACE_ANGLE}°)"
        print(f"📐 Hedef Açı (+{TARGET_SURFACE_ANGLE}°): {angle_status}")
        
        # Payload durumu
        payload_icons = {"CLOSED": "🔒", "OPENING": "🔓", "OPEN": "📂", "SEPARATED": "🚀"}
        payload_icon = payload_icons.get(self.payload_bay_status, "❓")
        print(f"{payload_icon} Payload Bay: {self.payload_bay_status}")
        
        # Ayrılma durumu
        separation_status = "✅ TAMAMLANDI" if self.separation_completed else "⏳ BEKLEMEDE"
        print(f"🎯 Ayrılma Durumu: {separation_status}")
        
        print("="*70)
    
    def set_motor_throttle(self, throttle_pwm):
        """Motor kontrolü"""
        if not self.connected:
            return False
            
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
        """Kontrol yüzeyleri ayarla"""
        fin_commands = {
            'fin_top': PWM_NEUTRAL - pitch_cmd + yaw_cmd,
            'fin_bottom': PWM_NEUTRAL + pitch_cmd + yaw_cmd,
            'fin_right': PWM_NEUTRAL + roll_cmd + yaw_cmd,
            'fin_left': PWM_NEUTRAL - roll_cmd + yaw_cmd
        }
        
        for fin_name, pwm_value in fin_commands.items():
            channel = FIN_CHANNELS[fin_name]
            self.set_servo_position(channel, int(pwm_value))
    
    def control_payload_bay(self, position):
        """Payload bay kontrolü"""
        if position == "OPEN":
            self.set_servo_position(PAYLOAD_BAY_SERVO, PAYLOAD_OPEN)
            self.payload_bay_status = "OPENING"
        elif position == "CLOSE":
            self.set_servo_position(PAYLOAD_BAY_SERVO, PAYLOAD_CLOSED)
            self.payload_bay_status = "CLOSED"
        else:
            # Kısmi açma (pozisyon değeri)
            self.set_servo_position(PAYLOAD_BAY_SERVO, position)
            self.payload_bay_status = f"POSITION_{position}"
        
        return True
    
    def trigger_separation_mechanism(self):
        """Ayrılma mekanizmasını tetikle"""
        print("🚀 ROKET AYRILMA MEKANİZMASI TETİKLENİYOR...")
        
        # Ayrılma sinyali (servo veya solenoid)
        # Bu gerçek uygulamada roket model'in ayrılmasını sağlar
        
        # Ayrılma servo/motor çalıştır
        self.set_servo_position(SEPARATION_MECHANISM, PWM_MAX)
        time.sleep(1)
        
        # Payload bay'i tamamen aç
        self.control_payload_bay("OPEN")
        time.sleep(2)
        
        # Ayrılma mekanizmasını geri al (temizlik)
        self.set_servo_position(SEPARATION_MECHANISM, PWM_NEUTRAL)
        
        self.separation_completed = True
        self.separation_time = time.time()
        self.payload_bay_status = "SEPARATED"
        
        print("✅ ROKET AYRILMA MEKANİZMASI ÇALIŞTIRILDI!")
        return True
    
    def monitoring_loop(self):
        """Sürekli izleme döngüsü"""
        while self.running and self.demo_active:
            self.read_sensors()
            
            # Hedef açı kontrolü
            if (self.surface_achieved and 
                self.current_pitch >= TARGET_SURFACE_ANGLE and 
                not self.target_angle_achieved):
                self.target_angle_achieved = True
                print(f"🎯 HEDEF AÇI ELDE EDİLDİ: {self.current_pitch:.1f}°!")
            
            # Her 2 saniyede durum göster
            if len(self.telemetry_data) % 20 == 0:
                self.display_separation_status()
            
            time.sleep(0.1)  # 10Hz
    
    def ascend_to_surface(self):
        """Yüzeye çıkış"""
        print("\n🌊 1. YÜZEYE ÇIKIŞ AŞAMASI")
        print("-"*40)
        
        self.demo_stage = "SURFACE_ASCENT"
        
        ascent_start = time.time()
        
        while time.time() - ascent_start < 60:  # Max 60s yükselme
            # Yukarı motor gücü
            self.set_motor_throttle(PWM_NEUTRAL + 150)
            
            # Yukarı pitch (gentle ascent)
            self.set_control_surfaces(pitch_cmd=-100)  # Nose up
            
            self.read_sensors()
            
            print(f"  📊 Yükseliş: {self.current_depth:.1f}m | Pitch: {self.current_pitch:.1f}°")
            
            # Yüzeye çıktık mı?
            if self.current_depth <= SURFACE_DEPTH_THRESHOLD:
                print("  🌊 Yüzey seviyesine ulaşıldı!")
                break
                
            time.sleep(2)
        
        # Yüzeyde stabilize
        self.set_motor_throttle(PWM_NEUTRAL + 50)  # Hafif forward thrust
        self.set_control_surfaces()
        
        if not self.surface_achieved:
            print("❌ Yüzeye çıkış başarısız!")
            return False
        
        print("✅ Yüzeye çıkış tamamlandı!")
        return True
    
    def achieve_launch_angle(self):
        """Fırlatma açısına ulaşma (+30°)"""
        print(f"\n📐 2. FIRLATMA AÇISI ELDE ETME (+{TARGET_SURFACE_ANGLE}°)")
        print("-"*40)
        
        self.demo_stage = "LAUNCH_ANGLE"
        
        angle_start = time.time()
        
        while time.time() - angle_start < 45:  # Max 45s açı arama
            # Hedef pitch açısı için kontrol
            pitch_error = TARGET_SURFACE_ANGLE - self.current_pitch
            
            if abs(pitch_error) <= 2:  # 2° tolerance
                print(f"  🎯 Hedef açıya ulaşıldı: {self.current_pitch:.1f}°!")
                break
            
            # Motor ve fin kontrolü
            if pitch_error > 0:
                # Daha fazla pitch up gerekli
                self.set_motor_throttle(PWM_NEUTRAL + 80)
                pitch_cmd = -min(150, int(pitch_error * 8))
            else:
                # Pitch down gerekli
                self.set_motor_throttle(PWM_NEUTRAL + 60)
                pitch_cmd = -max(-150, int(pitch_error * 8))
            
            # Roll ve yaw stabilizasyonu
            roll_stabilize = int(self.current_roll * -2)
            yaw_stabilize = int((self.current_yaw % 360) * -0.5)  # Minimal yaw correction
            
            self.set_control_surfaces(
                roll_cmd=roll_stabilize,
                pitch_cmd=pitch_cmd,
                yaw_cmd=yaw_stabilize
            )
            
            self.read_sensors()
            
            print(f"  📊 Açı ayarı: {self.current_pitch:.1f}° -> {TARGET_SURFACE_ANGLE}° | Error: {pitch_error:.1f}°")
            print(f"      Derinlik: {self.current_depth:.1f}m | Roll: {self.current_roll:.1f}°")
            
            time.sleep(1)
        
        # Stabilize et
        self.set_control_surfaces()
        self.set_motor_throttle(PWM_NEUTRAL + 30)
        
        if not self.target_angle_achieved:
            print(f"❌ Hedef açıya ulaşılamadı! Mevcut: {self.current_pitch:.1f}°")
            return False
        
        print("✅ Fırlatma açısı elde edildi!")
        return True
    
    def perform_rocket_separation(self):
        """Roket ayrılma işlemi"""
        print("\n🚀 3. ROKET AYRILMA İŞLEMİ")
        print("-"*40)
        
        self.demo_stage = "ROCKET_SEPARATION"
        
        # Ayrılma öncesi son kontroller
        print("🔍 Ayrılma öncesi sistem kontrolleri...")
        
        # 1. Yüzey kontrolü
        if not self.surface_achieved:
            print("❌ Araç yüzeyde değil!")
            return False
        
        # 2. Açı kontrolü
        if self.current_pitch < TARGET_SURFACE_ANGLE - 5:  # 5° tolerance
            print(f"❌ Yetersiz pitch açısı: {self.current_pitch:.1f}° < {TARGET_SURFACE_ANGLE}°")
            return False
        
        # 3. Stabilite kontrolü
        if abs(self.current_roll) > 15 or abs(self.current_yaw - self.current_yaw) > 10:
            print("❌ Araç yeterince stabil değil!")
            return False
        
        print("✅ Tüm ayrılma koşulları sağlandı!")
        
        # Ayrılma countdown
        for i in range(5, 0, -1):
            print(f"  🚀 Roket ayrılma: {i}...")
            time.sleep(1)
        
        # AYRILMA!
        separation_success = self.trigger_separation_mechanism()
        
        if separation_success:
            print("🎉 ROKET AYRILMA BAŞARILI!")
            
            # Ayrılma sonrası 10 saniye bekle (video için)
            print("📹 Ayrılma sonrası gözlem (10s)...")
            for i in range(10):
                self.read_sensors()
                print(f"    📊 Ayrılma+{i+1}s: Pitch={self.current_pitch:.1f}° | Payload={self.payload_bay_status}")
                time.sleep(1)
            
            # Log kaydet
            self.separation_log.append({
                'timestamp': time.time(),
                'surface_achieved': self.surface_achieved,
                'launch_angle': self.current_pitch,
                'separation_successful': True,
                'conditions': {
                    'depth': self.current_depth,
                    'roll': self.current_roll,
                    'pitch': self.current_pitch,
                    'yaw': self.current_yaw
                }
            })
            
            return True
        
        print("❌ Roket ayrılma başarısız!")
        return False
    
    def post_separation_recovery(self):
        """Ayrılma sonrası kurtarma"""
        print("\n🔄 4. AYRILMA SONRASI KURTARMA")
        print("-"*40)
        
        self.demo_stage = "POST_SEPARATION"
        
        # Payload bay'i kapat (güvenlik)
        print("🔒 Payload bay kapatılıyor...")
        self.control_payload_bay("CLOSE")
        time.sleep(3)
        
        # Sistem stabilizasyonu
        print("⚖️ Sistem stabilizasyonu...")
        for i in range(10):
            self.set_control_surfaces()
            self.set_motor_throttle(PWM_NEUTRAL)
            
            self.read_sensors()
            
            print(f"  📊 Stabilizasyon: {i+1}/10 | Roll: {self.current_roll:.1f}° | Pitch: {self.current_pitch:.1f}°")
            time.sleep(1)
        
        print("✅ Ayrılma sonrası kurtarma tamamlandı!")
        return True
    
    def generate_separation_report(self):
        """Ayrılma demo raporu"""
        print("\n" + "="*70)
        print("📋 ROKET AYRILMA MEKANİZMASI DEMO RAPORU")
        print("="*70)
        
        demo_duration = time.time() - self.demo_start_time if self.demo_start_time else 0
        
        print(f"📅 Demo Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Toplam Demo Süresi: {demo_duration/60:.1f} dakika")
        
        # Surface phase analizi
        if self.surface_time:
            surface_duration = self.surface_time - self.demo_start_time
            print(f"🌊 Yüzeye çıkış süresi: {surface_duration:.0f} saniye")
        
        # Separation phase analizi
        if self.separation_time:
            separation_duration = self.separation_time - (self.surface_time or self.demo_start_time)
            print(f"🚀 Ayrılma işlem süresi: {separation_duration:.0f} saniye")
        
        print(f"\n📊 PERFORMANS METRİKLERİ:")
        print("-"*50)
        print(f"🌊 Yüzey Başarısı: {'✅ BAŞARILI' if self.surface_achieved else '❌ BAŞARISIZ'}")
        print(f"📐 Maksimum Pitch Açısı: {self.max_pitch_achieved:.1f}°")
        print(f"🎯 Hedef Açı (+{TARGET_SURFACE_ANGLE}°): {'✅ ULAŞILDI' if self.target_angle_achieved else '❌ ULAŞILAMADI'}")
        print(f"🚀 Ayrılma Başarısı: {'✅ BAŞARILI' if self.separation_completed else '❌ BAŞARISIZ'}")
        
        # Şartname değerlendirmesi
        print(f"\n🎯 ŞARTNAME GEREKSİNİMLERİ:")
        print("-"*40)
        
        surface_ok = self.surface_achieved
        surface_icon = "✅" if surface_ok else "❌"
        print(f"  {surface_icon} Su Yüzeyine Çıkış: {'BAŞARILI' if surface_ok else 'BAŞARISIZ'}")
        
        angle_ok = self.max_pitch_achieved >= TARGET_SURFACE_ANGLE
        angle_icon = "✅" if angle_ok else "❌"  
        print(f"  {angle_icon} +{TARGET_SURFACE_ANGLE}° Eğim Koşulu: {self.max_pitch_achieved:.1f}°")
        
        separation_ok = self.separation_completed
        separation_icon = "✅" if separation_ok else "❌"
        print(f"  {separation_icon} Roket Taşıma Bölmesi Ayrılması: {'BAŞARILI' if separation_ok else 'BAŞARISIZ'}")
        
        # Detailed telemetry
        if self.telemetry_data:
            surface_data = [d for d in self.telemetry_data if d.get('surface_achieved', False)]
            if surface_data:
                avg_surface_pitch = sum(d['pitch'] for d in surface_data[-20:]) / min(20, len(surface_data))
                max_surface_roll = max(abs(d['roll']) for d in surface_data[-20:])
                
                print(f"\n📈 YÜZEY PERFORMANSI:")
                print(f"  📐 Ortalama Yüzey Pitch: {avg_surface_pitch:.1f}°")
                print(f"  🔄 Maksimum Yüzey Roll: {max_surface_roll:.1f}°")
        
        # Final başarı değerlendirmesi
        overall_success = surface_ok and angle_ok and separation_ok
        
        print(f"\n🏆 GENEL SONUÇ:")
        print("="*30)
        
        if overall_success:
            print("🎉 ROKET AYRILMA MEKANİZMASI TAM BAŞARI!")
            print("📹 Video çekimi için mükemmel!")
        else:
            print("❌ ROKET AYRILMA MEKANİZMASI EKSİKLİKLER VAR!")
            missing_elements = []
            if not surface_ok:
                missing_elements.append("Yüzey çıkışı")
            if not angle_ok:
                missing_elements.append("Hedef açı")
            if not separation_ok:
                missing_elements.append("Ayrılma mekanizması")
            print(f"🔧 Eksikler: {', '.join(missing_elements)}")
        
        # Veri kaydet
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'demo_duration': demo_duration,
            'surface_achieved': self.surface_achieved,
            'target_angle_achieved': self.target_angle_achieved,
            'separation_completed': self.separation_completed,
            'max_pitch_achieved': self.max_pitch_achieved,
            'separation_events': self.separation_log,
            'telemetry_summary': {
                'total_samples': len(self.telemetry_data),
                'surface_samples': len([d for d in self.telemetry_data if d.get('surface_achieved', False)])
            },
            'overall_success': overall_success
        }
        
        with open(f'rocket_separation_demo_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
            json.dump(report_data, f, indent=2)
        
        print(f"\n💾 Demo raporu kaydedildi: rocket_separation_demo_*.json")
        
        return overall_success
    
    def run_full_separation_demo(self):
        """Tam roket ayrılma demo"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - ROKET AYRILMA DEMOsu")
        print("="*70)
        print("📹 Video çekimi için roket ateşleme sisteminin gösterimi")
        print("⏱️ Tahmini süre: 4-6 dakika")
        print("🎯 Şartname: Su yüzeyinde +30° eğim, roket taşıma bölmesi açılması")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("\n⚠️ GÜVENLİK UYARISI:")
        print("- Roket ayrılma alanı temiz ve güvenli mi?")
        print("- Model roket FİZİKSEL OLARAK FIRLATILMAYACAK!")
        print("- Sadece ayrılma mekanizması gösterilecek!")
        print("- Kameralar yakın çekim hazır mı?")
        
        ready = input("\n✅ Roket ayrılma demosu başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Demo iptal edildi")
            return False
        
        self.demo_start_time = time.time()
        self.demo_active = True
        self.running = True
        
        # Payload bay başlangıçta kapalı
        self.control_payload_bay("CLOSE")
        
        # Monitoring thread başlat
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        try:
            print("\n🚀 ROKET AYRILMA DEMOsu BAŞLADI!")
            
            # 1. Yüzeye çıkış
            if not self.ascend_to_surface():
                print("❌ Yüzeye çıkış başarısız!")
                return False
            
            input("\n⏸️ Yüzey çıkış tamam! Fırlatma açısı ayarına geçilsin mi? ENTER...")
            
            # 2. Fırlatma açısı
            if not self.achieve_launch_angle():
                print("❌ Fırlatma açısı elde edilemedi!")
                return False
            
            input(f"\n⏸️ +{TARGET_SURFACE_ANGLE}° açı hazır! Roket ayrılma işlemi yapılacak. ENTER...")
            
            # 3. Roket ayrılma
            if not self.perform_rocket_separation():
                print("❌ Roket ayrılma başarısız!")
                return False
            
            input("\n⏸️ Roket ayrılma tamam! Kurtarma işlemi yapılacak. ENTER...")
            
            # 4. Ayrılma sonrası kurtarma
            self.post_separation_recovery()
            
            # 5. Demo raporu
            success = self.generate_separation_report()
            
            if success:
                print("\n🎉 ROKET AYRILMA DEMOsu BAŞARILI!")
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
            
            # Kontrol yüzeylerini nötr
            self.set_control_surfaces()
            
            # Payload bay'i kapat
            self.control_payload_bay("CLOSE")
            
            # Separation mechanism nötr
            self.set_servo_position(SEPARATION_MECHANISM, PWM_NEUTRAL)
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı")

def main():
    """Ana fonksiyon"""
    demo = RocketSeparationDemo()
    
    try:
        success = demo.run_full_separation_demo()
        return 0 if success else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main()) 