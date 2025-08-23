#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş (GPS'siz)

GPS KALDIRILDI:
- Konum/bearing/mesafe için GPS kullanılmıyor.
- Mesafe tahmini, verilen hız setpoint'lerinin (cruise/return) zamana göre entegrasyonu ile yapılır.
- Yön (heading) ATTITUDE'tan, derinlik SCALED_PRESSURE'dan alınır.
- "Kıyıdan 50 m uzaklık" koşulu GPS yokken "başlangıçtan toplam 50 m ileri yol" olarak ele alınır.

Aşamalar:
1) 2 m derinliğe in.
2) Düz istikamette ~10 m ilerle (ölü hesap mesafe).
3) Toplam ~50 m ileri mesafe tamamlanana kadar devam et.
4) 180° dön ve aynı mesafeyi geri gel (başlangıca tahmini dönüş).
5) Pozitif sephiye ile yüzeye çık ve enerjiyi kes.

Not: Hız-mesafe tahmini, su koşullarına göre sapma gösterebilir. Kendi aracın için hız↔PWM katsayısını kalibre et.
"""

import time
import threading
import math
import json
import argparse
from datetime import datetime
from pymavlink import mavutil
import os

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

# MAVLink bağlantı adresi (ENV ile özelleştirilebilir)
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0") + "," + str(os.getenv("MAV_BAUD", "115200"))

# Görev parametreleri
MISSION_PARAMS = {
    'target_depth': 2.0,            # m
    'straight_distance': 10.0,      # m (ilk düz hat)
    'min_outbound_distance': 50.0,  # m (toplam ileri mesafe hedefi)
    'cruise_speed': 1.5,            # m/s (ileri)
    'return_speed': 1.8,            # m/s (geri dönüş)
    'timeout_seconds': 300,         # s
    'position_tolerance': 2.0,      # m (tahmini dönüş hatası değerlendirmesi)
    'depth_tolerance': 0.2          # m
}

# Kontrol parametreleri
CONTROL_PARAMS = {
    'depth_pid':   {'kp': 100.0, 'ki': 5.0,  'kd': 30.0},
    'heading_pid': {'kp': 3.0,   'ki': 0.1,  'kd': 0.5}
}

# TEKNOFEST Standart Pin Mapping
MOTOR_CHANNEL = 1  # AUX 1
SERVO_CHANNELS = {
    'fin_1': 3,  # AUX 3 - Ön Sağ
    'fin_2': 4,  # AUX 4 - Arka Sol
    'fin_3': 5,  # AUX 5 - Arka Sağ
    'fin_4': 6   # AUX 6 - Ön Sol
}

# X-Konfigürasyon Kontrol Matrisi
X_WING_MATRIX = {
    'roll_positive': [6, 4],    # Sol finler (AUX6, AUX4)
    'roll_negative': [3, 5],    # Sağ finler (AUX3, AUX5)
    'pitch_positive': [6, 3],   # Ön finler (AUX6, AUX3)
    'pitch_negative': [4, 5],   # Arka finler (AUX4, AUX5)
    'yaw_positive': [6, 5],     # X-Diagonal 1 (AUX6, AUX5)
    'yaw_negative': [3, 4]      # X-Diagonal 2 (AUX3, AUX4)
}

# PWM aralıkları ve güvenlik limitleri
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

# Güvenlik sınırları (mekanik koruma)
PWM_SAFE_MIN = 1300  # Mekanik güvenli minimum
PWM_SAFE_MAX = 1700  # Mekanik güvenli maksimum
SERVO_MAX_DELTA = 300  # Maksimum PWM değişimi (±300µs)
OVERALL_MAX_DELTA_US = 350  # Genel güvenlik sınırı

# Hız→PWM basit modeli (kalibre et!)
# 1.5 m/s ≈ +120 PWM kabul edilirse katsayı ~80 PWM/(m/s)
PWM_PER_MPS = 80.0

# Kalibrasyon dosyası yükle
def load_speed_calibration():
    """PWM→hız kalibrasyon parametrelerini yükle"""
    try:
        with open('config/cal_speed.json', 'r') as f:
            cal_data = json.load(f)
            x_wing_cal = cal_data['pwm_speed_calibration']['x_wing']
            return {
                'a': x_wing_cal['a'],
                'b': x_wing_cal['b'], 
                'neutral_pwm': x_wing_cal['neutral_pwm'],
                'lpf_alpha': cal_data['distance_tracking']['lpf_alpha']
            }
    except Exception as e:
        print(f"⚠️ Kalibrasyon dosyası yüklenemedi: {e}")
        return {
            'a': 0.0125,  # Default: ~80 PWM/(m/s) ters çevrilmiş
            'b': 0.0,
            'neutral_pwm': 1500,
            'lpf_alpha': 0.3
        }

SPEED_CAL = load_speed_calibration()

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
        now = time.time()
        dt = max(0.01, now - self.last_time)

        error = setpoint - measurement
        self.integral += error * dt
        if self.ki > 0:
            integral_limit = self.max_output / self.ki
            self.integral = max(-integral_limit, min(integral_limit, self.integral))

        derivative = (error - self.previous_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-self.max_output, min(self.max_output, output))

        self.previous_error = error
        self.last_time = now
        return output

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.last_time = time.time()

class Mission1Navigator:
    def __init__(self):
        self.master = None
        self.connected = False

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

        # Durum
        self.mission_active = False
        self.mission_stage = "INITIALIZATION"
        self.mission_start_time = None
        self.mission_completion_time = None

        # Ölçümler
        self.current_depth = 0.0
        self.current_heading = 0.0
        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.current_yaw = 0.0

        # Dead-reckoning ve odometri
        self.initial_heading = None
        self.speed_setpoint = 0.0        # m/s (komutlanan)
        self.last_dist_update_t = time.time()
        
        # Net mesafe odometri (PWM tabanlı)
        self.traveled_distance = 0.0     # Toplam gidilen mesafe (odometri)
        self.current_pwm = PWM_NEUTRAL   # Mevcut motor PWM
        self.estimated_speed = 0.0       # PWM'den kestirilen hız
        self.filtered_speed = 0.0        # LPF uygulanmış hız
        
        # Stage referansları
        self.distance_at_straight_start = 0.0
        self.distance_at_offshore_start = 0.0
        self.straight_distance_completed = 0.0   # m (ilk 10 m için)
        self.outbound_distance = 0.0             # m (ileri toplam)
        self.inbound_distance = 0.0              # m (geri toplam)
        self.max_offshore_distance = 0.0

        # Rapor/metrikler
        self.final_position_error_est = float('inf')  # |outbound - inbound|
        self.leak_detected = False
        
        # Sensor zaman damgaları ve kaynak takibi
        self._last_depth_ts = time.time()
        self._last_attitude_ts = time.time()
        self.depth_source = "unknown"  # "d300" veya "scaled_pressure"
        self._latched_fault = None  # Kalıcı hata durumu
        
        # 90 saniye arming interlock sistemi
        self._arming_start_time = None
        self._arming_done = False
        self.ARMING_DURATION = 90.0  # 90 saniye
        self._arming_countdown_displayed = set()  # Gösterilen countdown'ları takip et

        # PID
        self.depth_pid = PIDController(**CONTROL_PARAMS['depth_pid'])
        self.heading_pid = PIDController(**CONTROL_PARAMS['heading_pid'])

        # Threading
        self.control_thread = None
        self.monitoring_thread = None
        self.running = False

        # Telemetri
        self.telemetry_data = []

    # ---------------- MAVLink ----------------
    def connect_pixhawk(self):
        try:
            print("🔌 Pixhawk bağlantısı kuruluyor...")
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                print(f"🌐 TCP/UDP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)

            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            print(f"   System ID: {self.master.target_system}")
            print(f"   Component ID: {self.master.target_component}")
            
            # Stream rate istekleri
            self._request_data_streams()
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def _request_data_streams(self):
        """MAVLink data stream hızlarını ayarla"""
        try:
            # ATTITUDE ≥20 Hz
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE
                20,  # 20 Hz
                1    # Enable
            )
            
            # SCALED_PRESSURE ≥10 Hz  
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,  # SCALED_PRESSURE
                10,  # 10 Hz
                1    # Enable
            )
            
            print("📊 Stream rate istekleri gönderildi: ATTITUDE@20Hz, PRESSURE@10Hz")
        except Exception as e:
            print(f"⚠️ Stream rate ayarlama hatası: {e}")

    def read_sensors(self):
        """GPS YOK: Sadece ATTITUDE ve SCALED_PRESSURE, ayrıca leak için STATUSTEXT dinlenir."""
        if not self.connected:
            return False
        
        # Latched fault kontrolü
        if self._latched_fault:
            return False
            
        try:
            current_time = time.time()
            
            # Attitude
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw
                self._last_attitude_ts = current_time

            # Derinlik sensörü (D300 öncelikli, yoksa SCALED_PRESSURE)
            depth_read_success = False
            if self.d300_connected and self.d300_sensor:
                try:
                    depth_data = self.d300_sensor.read_depth()
                    if depth_data['success']:
                        self.current_depth = max(0.0, depth_data['depth'])
                        self.depth_source = "d300"
                        self._last_depth_ts = current_time
                        depth_read_success = True
                        print(f"📡 D300 Derinlik: {self.current_depth:.2f}m")
                except Exception as e:
                    print(f"⚠️ D300 okuma hatası: {e}")
                    
            if not depth_read_success:
                # D300 yok veya hatalı, SCALED_PRESSURE kullan
                pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                if pressure_msg:
                    depth_pressure = pressure_msg.press_abs - 1013.25
                    self.current_depth = max(0.0, depth_pressure * 0.0102)
                    self.depth_source = "scaled_pressure"
                    self._last_depth_ts = current_time
                    print(f"📡 SCALED_PRESSURE Derinlik: {self.current_depth:.2f}m (Basınç: {pressure_msg.press_abs:.1f}hPa)")
            
            # Watchdog kontrolü - sensör zaman aşımı
            if current_time - self._last_depth_ts > 0.5:
                self._trigger_latched_fault("DEPTH_SENSOR_TIMEOUT")
                return False
            if current_time - self._last_attitude_ts > 0.5:
                self._trigger_latched_fault("ATTITUDE_SENSOR_TIMEOUT")
                return False

            # Leak uyarısı (metin üzerinden basit tespit)
            statustext = self.master.recv_match(type='STATUSTEXT', blocking=False)
            if statustext and statustext.text:
                txt = statustext.text.lower()
                if "leak" in txt or "water" in txt:
                    self.leak_detected = True

            # Telemetri log
            self.telemetry_data.append({
                't': time.time(),
                'depth': self.current_depth,
                'heading': self.current_heading,
                'roll': self.current_roll,
                'pitch': self.current_pitch,
                'yaw': self.current_yaw,
                'stage': self.mission_stage,
                'spd_sp': self.speed_setpoint
            })
            return True
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            self._trigger_latched_fault(f"SENSOR_READ_ERROR: {e}")
            return False
    
    def _trigger_latched_fault(self, fault_reason):
        """Kalıcı hata durumu tetikle"""
        if not self._latched_fault:  # Sadece ilk hata için
            self._latched_fault = fault_reason
            print(f"🚨 LATCHED FAULT: {fault_reason}")
            print("🚨 Sistem güvenli duruma geçiyor...")
            self._emergency_neutral()
            
    def _emergency_neutral(self):
        """Acil durum - motor ve servoları nötr konuma getir"""
        try:
            if self.connected:
                # Motor NEUTRAL
                self.set_motor_throttle_pwm(PWM_NEUTRAL)
                # Servolar nötr
                for channel in SERVO_CHANNELS.values():
                    self._set_servo(channel, PWM_NEUTRAL)
        except Exception as e:
            print(f"❌ Emergency neutral hatası: {e}")
    
    def cleanup(self):
        """Güvenli kapanış sırası"""
        if hasattr(self, '_cleanup_done') and self._cleanup_done:
            return  # Idempotent - birden çok çağrı güvenli
        
        print("🧹 Sistem temizleniyor...")
        
        try:
            # 1. Motor NEUTRAL
            if self.connected:
                self.set_motor_throttle_pwm(PWM_NEUTRAL)
                print("   ✅ Motor nötr")
                
                # 2. Servolar nötr
                for channel in SERVO_CHANNELS.values():
                    self._set_servo(channel, PWM_NEUTRAL)
                print("   ✅ Servolar nötr")
            
            # 3. Mission durumunu sonlandır
            self.running = False
            self.mission_active = False
            
            # 4. Thread'leri güvenli kapatma
            if hasattr(self, 'control_thread') and self.control_thread and self.control_thread.is_alive():
                self.control_thread.join(timeout=2.0)
                print("   ✅ Control thread sonlandırıldı")
            
            if hasattr(self, 'monitoring_thread') and self.monitoring_thread and self.monitoring_thread.is_alive():
                self.monitoring_thread.join(timeout=2.0)
                print("   ✅ Monitoring thread sonlandırıldı")
            
            # 5. Telemetri flush
            if hasattr(self, 'telemetry_data') and len(self.telemetry_data) > 0:
                print(f"   📊 Telemetri kayıtları: {len(self.telemetry_data)} örnek")
            
            # 6. MAVLink bağlantısı kapat
            if self.connected and self.master:
                try:
                    self.master.close()
                    print("   ✅ MAVLink bağlantısı kapatıldı")
                except:
                    pass
                self.connected = False
            
            self._cleanup_done = True
            print("🧹 Sistem temizleme tamamlandı")
            
        except Exception as e:
            print(f"❌ Cleanup hatası: {e}")

    # --------------- Aktüatörler ---------------
    def _check_arming_interlock(self):
        """90 saniye arming interlock kontrolü"""
        if self._arming_start_time is None:
            self._arming_start_time = time.time()
            print("🔒 ARMİNG INTERLOCK başlatıldı - 90 saniye güvenlik süresi")
            return False
        
        elapsed = time.time() - self._arming_start_time
        remaining = self.ARMING_DURATION - elapsed
        
        # Countdown gösterimi (10'ar saniyelik aralıklarla)
        countdown_intervals = [90, 80, 70, 60, 50, 40, 30, 20, 10, 5, 4, 3, 2, 1]
        for interval in countdown_intervals:
            if remaining <= interval and interval not in self._arming_countdown_displayed:
                self._arming_countdown_displayed.add(interval)
                if interval <= 10:
                    print(f"⏰ ARMİNG COUNTDOWN: {interval} saniye")
                else:
                    print(f"🔒 ARMİNG INTERLOCK: {remaining:.0f} saniye kaldı")
        
        if elapsed >= self.ARMING_DURATION:
            if not self._arming_done:
                self._arming_done = True
                print("✅ ARMİNG INTERLOCK tamamlandı - Motor kontrolü serbest!")
            return True
        
        return False

    def set_motor_throttle_pwm(self, pwm):
        if not self.connected:
            return False
            
        # Arming interlock kontrolü
        if not self._check_arming_interlock():
            # Arming süresi dolmadıysa motor NEUTRAL'de tut
            pwm = PWM_NEUTRAL
            
        pwm = max(PWM_MIN, min(PWM_MAX, int(pwm)))
        self.current_pwm = pwm  # PWM tracking için
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL, pwm, 0, 0, 0, 0, 0
            )
            return True
        except:
            return False

    def set_control_surfaces(self, roll_cmd=0, pitch_cmd=0, yaw_cmd=0):
        """X-Wing kontrol yüzeyleri - güvenlik sınırları ile"""
        
        # Komutları sınırla
        roll_cmd = max(-SERVO_MAX_DELTA, min(SERVO_MAX_DELTA, roll_cmd))
        pitch_cmd = max(-SERVO_MAX_DELTA, min(SERVO_MAX_DELTA, pitch_cmd))
        yaw_cmd = max(-SERVO_MAX_DELTA, min(SERVO_MAX_DELTA, yaw_cmd))
        
        # X-Konfigürasyon mixing
        fins = {}
        
        # Roll kontrolü (sol vs sağ finler)
        if roll_cmd != 0:
            for channel in X_WING_MATRIX['roll_positive']:
                fins[channel] = fins.get(channel, 0) + roll_cmd
            for channel in X_WING_MATRIX['roll_negative']:
                fins[channel] = fins.get(channel, 0) - roll_cmd
        
        # Pitch kontrolü (ön vs arka finler)
        if pitch_cmd != 0:
            for channel in X_WING_MATRIX['pitch_positive']:
                fins[channel] = fins.get(channel, 0) - pitch_cmd  # Ön finler nose down için -
            for channel in X_WING_MATRIX['pitch_negative']:
                fins[channel] = fins.get(channel, 0) + pitch_cmd  # Arka finler nose down için +
        
        # Yaw kontrolü (diagonal finler)
        if yaw_cmd != 0:
            for channel in X_WING_MATRIX['yaw_positive']:
                fins[channel] = fins.get(channel, 0) + yaw_cmd
            for channel in X_WING_MATRIX['yaw_negative']:
                fins[channel] = fins.get(channel, 0) - yaw_cmd
        
        # PWM değerlerini hesapla ve güvenlik kontrolü
        for channel, delta in fins.items():
            # Genel sınır kontrolü
            delta = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, delta))
            pwm_value = PWM_NEUTRAL + delta
            
            # Güvenli PWM aralığı
            pwm_value = max(PWM_SAFE_MIN, min(PWM_SAFE_MAX, pwm_value))
            
            self._set_servo(channel, pwm_value)

    def _set_servo(self, channel, pwm):
        if not self.connected:
            return False
        pwm = max(PWM_MIN, min(PWM_MAX, int(pwm)))
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm, 0, 0, 0, 0, 0
            )
            return True
        except:
            return False

    # --------------- Yardımcılar ---------------
    def speed_to_pwm(self, spd_mps):
        """Basit lineer model: PWM = NEUTRAL + k * v"""
        return PWM_NEUTRAL + int(PWM_PER_MPS * spd_mps)

    def set_forward_speed(self, spd_mps):
        """İleri (+) veya geri/çok yavaş (0'a yakın) hız setpoint."""
        self.speed_setpoint = max(0.0, float(spd_mps))  # negatif hız kullanılmıyor
        self.set_motor_throttle_pwm(self.speed_to_pwm(self.speed_setpoint))

    def update_pwm_based_odometry(self):
        """PWM tabanlı hız kestirimi ve mesafe güncelleme"""
        current_time = time.time()
        dt = current_time - self.last_dist_update_t
        
        if dt > 0.05:  # 20 Hz güncelleme
            # PWM'den hız kestir
            pwm_delta = self.current_pwm - SPEED_CAL['neutral_pwm']
            self.estimated_speed = SPEED_CAL['a'] * pwm_delta + SPEED_CAL['b']
            self.estimated_speed = max(0.0, self.estimated_speed)  # Negatif hız yok
            
            # LPF uygula
            alpha = SPEED_CAL['lpf_alpha']
            self.filtered_speed = alpha * self.estimated_speed + (1 - alpha) * self.filtered_speed
            
            # Mesafe entegrasyonu
            distance_increment = self.filtered_speed * dt
            self.traveled_distance += distance_increment
            
            self.last_dist_update_t = current_time
    
    def update_distance_counters(self):
        """Hız setpoint'ini dt ile entegre ederek mesafeleri günceller."""
        now = time.time()
        dt = max(0.0, now - self.last_dist_update_t)
        self.last_dist_update_t = now
        ds = self.speed_setpoint * dt  # m

        if self.mission_stage == "STRAIGHT_COURSE":
            self.straight_distance_completed += ds
            self.outbound_distance += ds
        elif self.mission_stage == "OFFSHORE_CRUISE":
            self.outbound_distance += ds
        elif self.mission_stage in ("RETURN_NAVIGATION", "FINAL_APPROACH"):
            self.inbound_distance += ds
            # tahmini konum hatası: |gidilen ileri - geri|
            self.final_position_error_est = abs(self.outbound_distance - self.inbound_distance)

    def display_status(self):
        print("\n" + "="*80)
        print("🚀 TEKNOFEST - GÖREV 1 (GPS'siz)")
        print("="*80)
        t_now = datetime.now().strftime("%H:%M:%S")
        elapsed = (time.time() - self.mission_start_time) if self.mission_start_time else 0.0
        remaining = max(0, MISSION_PARAMS['timeout_seconds'] - elapsed)
        print(f"⏰ Zaman: {t_now} | Süre: {elapsed:.0f}s | Kalan: {remaining:.0f}s")
        print(f"🎯 Aşama: {self.mission_stage}")
        print(f"🌊 Derinlik: {self.current_depth:.2f} m (hedef {MISSION_PARAMS['target_depth']:.2f} m)")
        print(f"🧭 Heading: {self.current_heading:.1f}° | Hız SP: {self.speed_setpoint:.2f} m/s")
        print(f"📏 Düz Seyir (ilk): {self.straight_distance_completed:.1f} m / {MISSION_PARAMS['straight_distance']} m")
        print(f"➡️  İleri Toplam: {self.outbound_distance:.1f} m / {MISSION_PARAMS['min_outbound_distance']} m")
        if self.inbound_distance > 0:
            print(f"⬅️  Geri Toplam:  {self.inbound_distance:.1f} m | Tahmini Pozisyon Hatası: {self.final_position_error_est:.1f} m")
        print("="*80)

    # --------------- Aşamalar ---------------
    def control_loop(self):
        """Ana kontrol döngüsü - 20-30 Hz efektif kontrol frekansı"""
        loop_start_time = time.time()
        loop_count = 0
        
        while self.running and self.mission_active:
            cycle_start = time.time()
            
            self.read_sensors()
            self.update_pwm_based_odometry()  # PWM tabanlı odometri
            self.update_distance_counters()

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

            # 20-30 Hz kontrol frekansı (0.033-0.05s)
            cycle_time = time.time() - cycle_start
            target_cycle_time = 0.04  # 25 Hz hedef
            sleep_time = max(0.01, target_cycle_time - cycle_time)  # Min 10ms sleep
            time.sleep(sleep_time)
            
            # Performans izleme (her 100 döngüde bir)
            loop_count += 1
            if loop_count % 100 == 0:
                avg_freq = loop_count / (time.time() - loop_start_time)
                if avg_freq < 15:  # 15 Hz altına düşerse uyarı
                    print(f"⚠️ Kontrol frekansı düşük: {avg_freq:.1f} Hz")

    def execute_descent(self):
        depth_error = MISSION_PARAMS['target_depth'] - self.current_depth
        if abs(depth_error) > MISSION_PARAMS['depth_tolerance']:
            out = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
            # Derine inmek için hafif ileri + pitch down
            pitch_cmd = min(150, max(-150, int(out)))
            self.set_forward_speed(0.2)  # az ileri
            self.set_control_surfaces(pitch_cmd=pitch_cmd)
        else:
            print("✅ Hedef derinlik ulaşıldı! Düz seyire geçiliyor...")
            self.mission_stage = "STRAIGHT_COURSE"
            self.initial_heading = self.current_heading  # yön kilitle
            
            # Stage geçişinde PID resetleri
            self.depth_pid.reset()
            self.heading_pid.reset()
            
            # Mesafe referansları set et
            self.distance_at_straight_start = self.traveled_distance
            self.straight_distance_completed = 0.0
            self.outbound_distance = 0.0
            self.last_dist_update_t = time.time()
            
            print(f"📏 STRAIGHT_COURSE başlangıç mesafesi: {self.distance_at_straight_start:.1f}m")

    def execute_straight_course(self):
        # İleri hız
        self.set_forward_speed(MISSION_PARAMS['cruise_speed'])

        # Derinlik tutma
        depth_out = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-150, min(150, int(depth_out)))

        # Başlangıç heading'ini tut
        hdg_correction = self._heading_correction(self.initial_heading, self.current_heading)
        yaw_cmd = max(-100, min(100, int(hdg_correction)))

        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)

        # 10 m tamamlandı mı?
        if self.straight_distance_completed >= MISSION_PARAMS['straight_distance']:
            print("✅ 10 m düz seyir tamamlandı! İleri mesafe hedefi için devam...")
            self.mission_stage = "OFFSHORE_CRUISE"
            
            # Stage geçişinde mesafe referansı
            self.distance_at_offshore_start = self.traveled_distance
            print(f"📏 OFFSHORE_CRUISE başlangıç mesafesi: {self.distance_at_offshore_start:.1f}m")

    def execute_offshore_cruise(self):
        # İleri hız
        self.set_forward_speed(MISSION_PARAMS['cruise_speed'])

        # Derinlik + yön tutma
        depth_out = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-150, min(150, int(depth_out)))

        hdg_correction = self._heading_correction(self.initial_heading, self.current_heading)
        yaw_cmd = max(-100, min(100, int(hdg_correction)))
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)

        # Toplam ileri mesafe 50 m oldu mu?
        if self.outbound_distance >= MISSION_PARAMS['min_outbound_distance']:
            print("✅ İleri toplam ~50 m tamamlandı! Geri dönüş başlıyor...")
            self.mission_stage = "RETURN_NAVIGATION"
            self.inbound_distance = 0.0
            
            # RETURN_NAVIGATION stage'inde PID resetleri
            self.depth_pid.reset()
            self.heading_pid.reset()
            print("🔄 RETURN_NAVIGATION: PID kontrolcüler reset edildi")

    def execute_return_navigation(self):
        # Geri dönüş: 180° yönle (aynı hat üzerinde geri)
        return_heading = (self.initial_heading + 180.0) % 360.0

        # Dönüşte biraz daha hızlı
        self.set_forward_speed(MISSION_PARAMS['return_speed'])

        # Derinlik + yön
        depth_out = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-150, min(150, int(depth_out)))

        hdg_correction = self._heading_correction(return_heading, self.current_heading)
        yaw_cmd = max(-150, min(150, int(hdg_correction)))
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)

        # Geri mesafe, ileri mesafe kadar olduysa final yaklaşım
        if self.inbound_distance >= self.outbound_distance - MISSION_PARAMS['position_tolerance']:
            print("✅ Başlangıca tahmini dönüş tamamlandı! Final yaklaşım/pozisyon tutma...")
            self.mission_stage = "FINAL_APPROACH"
            self._final_hold_start = None
            
            # FINAL_APPROACH stage'inde PID resetleri
            self.depth_pid.reset()
            self.heading_pid.reset()
            print("🔄 FINAL_APPROACH: PID kontrolcüler reset edildi")

    def execute_final_approach(self):
        # Burada hedef: düşük hızda pozisyonu "tahminen" tutmak
        self.set_forward_speed(0.2)  # çok düşük hız

        depth_out = self.depth_pid.update(MISSION_PARAMS['target_depth'], self.current_depth)
        pitch_cmd = max(-100, min(100, int(depth_out)))

        # Yönü sabit tut (orijinal heading)
        hdg_correction = self._heading_correction(self.initial_heading, self.current_heading)
        yaw_cmd = max(-80, min(80, int(hdg_correction)))
        self.set_control_surfaces(pitch_cmd=pitch_cmd, yaw_cmd=yaw_cmd)

        # Tahmini hata küçükse 5 sn bekle ve yüzeye çık
        if self.final_position_error_est <= MISSION_PARAMS['position_tolerance']:
            if self._final_hold_start is None:
                self._final_hold_start = time.time()
            elif (time.time() - self._final_hold_start) >= 5.0:
                print("✅ Final pozisyon tahmini stabil! Yüzeye çıkış ve enerji kesme...")
                self.mission_stage = "SURFACE_AND_SHUTDOWN"
        else:
            self._final_hold_start = None

    def execute_surface_shutdown(self):
        # Motoru nötrle, burnu hafif yukarıya al (pozitif sephiye varsayımı)
        self.set_forward_speed(0.0)
        self.set_control_surfaces(pitch_cmd=-100, yaw_cmd=0)

        # Yüzeye çıkana kadar bekle (yaklaşık derinlik < 0.5 m)
        if self.current_depth > 0.5:
            return

        print("🌊 Yüzeye çıkış tamamlandı!")
        print("⚡ Sistem enerjisi kesiliyor...")

        self.set_motor_throttle_pwm(PWM_NEUTRAL)
        self.set_control_surfaces()

        self.mission_completion_time = time.time()
        self.mission_stage = "MISSION_COMPLETE"
        print("✅ GÖREV 1 TAMAMLANDI!")

    def _heading_correction(self, target_deg, current_deg):
        """PID için hedef-mevcut farkını  -180..+180 aralığına getir."""
        err = target_deg - current_deg
        if err > 180: err -= 360
        if err < -180: err += 360
        return self.heading_pid.update(target_deg, current_deg)

    # --------------- İzleme ---------------
    def monitoring_loop(self):
        while self.running and self.mission_active:
            # Durumu periyodik göster
            if len(self.telemetry_data) % 30 == 0:
                self.display_status()

            # Süre limiti
            if self.mission_start_time and (time.time() - self.mission_start_time) > MISSION_PARAMS['timeout_seconds']:
                print("⏰ Süre doldu! Görev sonlandırılıyor...")
                self.mission_stage = "MISSION_COMPLETE"
                break
            time.sleep(0.1)

    # --------------- Rapor ---------------
    def generate_mission_report(self):
        duration = (self.mission_completion_time - self.mission_start_time) if (self.mission_completion_time and self.mission_start_time) else 0.0

        print("\n" + "="*80)
        print("📋 GÖREV 1 RAPORU (GPS'siz Dead-Reckoning)")
        print("="*80)
        print(f"📅 Tarih: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Süre: {duration:.1f} s")
        print("\n📊 METRİKLER:")
        print("-"*60)
        print(f"📏 Düz Seyir (ilk): {self.straight_distance_completed:.1f} m / {MISSION_PARAMS['straight_distance']} m")
        print(f"➡️  İleri Toplam: {self.outbound_distance:.1f} m / {MISSION_PARAMS['min_outbound_distance']} m")
        print(f"⬅️  Geri Toplam:  {self.inbound_distance:.1f} m")
        print(f"🎯 Tahmini Pozisyon Hatası: {self.final_position_error_est:.1f} m (tolerans {MISSION_PARAMS['position_tolerance']} m)")
        print(f"💧 Sızdırmazlık: {'✅ BAŞARILI' if not self.leak_detected else '❌ SIZINTI'}")

        print("\n🏆 PUANLAMA (yaklaşık):")
        print("-"*40)
        # Hız/seyir puanı (koşullar sağlandıysa süreye göre)
        time_factor = max(0, (300 - duration) / 300) if duration > 0 else 0
        cruise_ok = (self.straight_distance_completed >= MISSION_PARAMS['straight_distance'] and
                     self.outbound_distance >= MISSION_PARAMS['min_outbound_distance'])
        cruise_points = int(150 * time_factor) if cruise_ok else 0
        print(f"  🚀 Seyir Yapma (hız): {cruise_points}/150")

        position_points = 90 if self.final_position_error_est <= MISSION_PARAMS['position_tolerance'] else 0
        print(f"  🎯 Başlangıçta Enerji Kesme: {position_points}/90")

        waterproof_points = 60 if not self.leak_detected else 0
        print(f"  💧 Sızdırmazlık: {waterproof_points}/60")

        total_points = cruise_points + position_points + waterproof_points
        print(f"\n📈 TOPLAM: {total_points}/300")

        report = {
            'timestamp': datetime.now().isoformat(),
            'duration_s': duration,
            'metrics': {
                'straight_distance_completed_m': self.straight_distance_completed,
                'outbound_distance_m': self.outbound_distance,
                'inbound_distance_m': self.inbound_distance,
                'final_position_error_est_m': self.final_position_error_est,
                'leak_detected': self.leak_detected
            },
            'scoring': {
                'cruise_points': cruise_points,
                'position_points': position_points,
                'waterproof_points': waterproof_points,
                'total_points': total_points
            }
        }

        fname = f'mission_1_report_no_gps_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json'
        with open(fname, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"\n💾 Rapor kaydedildi: {fname}")

        return total_points >= 180  # %60 başarı eşiği

    # --------------- Yaşam Döngüsü ---------------
    def run_mission_1(self):
        print("🚀 TEKNOFEST Su Altı Roket Aracı - GÖREV 1 (GPS'siz) BAŞLIYOR")
        print("="*80)
        print("🎯 Görev: Seyir Yapma & Başlangıç Noktasına Tahmini Geri Dönüş")
        print("⏱️ Süre Limiti: 5 dakika")
        print("🏆 Maksimum Puan: 300")

        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False

        print("\n⚠️ GÖREV HAZIRLIĞI:")
        print("- Tüm sistemler hazır mı?")
        print("- Güvenlik kontrolleri tamam mı?")
        print("- Şamandıra takılı mı?")

        try:
            ready = input("\n✅ Görev 1 başlasın mı? (y/n): ").lower()
        except EOFError:
            ready = 'y'  # non-interaktif çalıştırma için otomatik onay
        if ready != 'y':
            print("❌ Görev iptal edildi")
            return False

        self.mission_start_time = time.time()
        self.mission_active = True
        self.running = True
        self.mission_stage = "DESCENT"
        self.last_dist_update_t = time.time()

        # Threadler
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop, daemon=True)
        self.monitoring_thread.start()

        try:
            print("\n🚀 GÖREV 1 BAŞLADI!")
            self.control_thread.join()  # kontrol tamamlanana kadar
            success = self.generate_mission_report()
            return success
        except KeyboardInterrupt:
            print("\n⚠️ Görev kullanıcı tarafından durduruldu")
            self.mission_active = False
            self.running = False
            return False
        except Exception as e:
            print(f"\n❌ Görev hatası: {e}")
            self.mission_active = False
            self.running = False
            return False
        finally:
            # Her durumda güvenli kapanış
            self.cleanup()

    def cleanup(self):
        self.mission_active = False
        self.running = False
        print("\n🧹 Sistem temizleniyor...")

        if self.connected:
            self.set_motor_throttle_pwm(PWM_NEUTRAL)
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

# ------------------ main ------------------
def main():
    parser = argparse.ArgumentParser(description='TEKNOFEST Görev 1 (GPS’siz): Seyir & Tahmini Geri Dönüş')
    args = parser.parse_args()

    mission = Mission1Navigator()
    try:
        ok = mission.run_mission_1()
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1

if __name__ == "__main__":
    import sys
    sys.exit(main())
