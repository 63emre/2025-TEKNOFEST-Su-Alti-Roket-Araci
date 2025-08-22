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

# Kanallar (ArduPilot/servo yerleşimine göre düzenle)
MOTOR_CHANNEL = 8
SERVO_CHANNELS = {
    'fin_top': 1,
    'fin_right': 2,
    'fin_bottom': 3,
    'fin_left': 4
}

# PWM aralıkları
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

# Hız→PWM basit modeli (kalibre et!)
# 1.5 m/s ≈ +120 PWM kabul edilirse katsayı ~80 PWM/(m/s)
PWM_PER_MPS = 80.0

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

        # Dead-reckoning
        self.initial_heading = None
        self.speed_setpoint = 0.0        # m/s (komutlanan)
        self.last_dist_update_t = time.time()
        self.straight_distance_completed = 0.0   # m (ilk 10 m için)
        self.outbound_distance = 0.0             # m (ileri toplam)
        self.inbound_distance = 0.0              # m (geri toplam)

        # Rapor/metrikler
        self.final_position_error_est = float('inf')  # |outbound - inbound|
        self.leak_detected = False

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
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False

    def read_sensors(self):
        """GPS YOK: Sadece ATTITUDE ve SCALED_PRESSURE, ayrıca leak için STATUSTEXT dinlenir."""
        if not self.connected:
            return False
        try:
            # Attitude
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            if attitude_msg:
                self.current_roll = math.degrees(attitude_msg.roll)
                self.current_pitch = math.degrees(attitude_msg.pitch)
                self.current_yaw = math.degrees(attitude_msg.yaw)
                self.current_heading = self.current_yaw

            # Derinlik (basınçtan kaba hesap; kalibrasyon gerekebilir)
            pressure_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
            if pressure_msg:
                # Basınç [hPa]; referans 1013.25 hPa (deniz seviyesi). Tatlı/salt su farkı ve sıcaklık etkisi yok sayıldı.
                depth_pressure = pressure_msg.press_abs - 1013.25
                self.current_depth = max(0.0, depth_pressure * 0.10197)  # ~m su sütunu

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
            return False

    # --------------- Aktüatörler ---------------
    def set_motor_throttle_pwm(self, pwm):
        if not self.connected:
            return False
        pwm = max(PWM_MIN, min(PWM_MAX, int(pwm)))
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
        fins = {
            'fin_top':    PWM_NEUTRAL - pitch_cmd + yaw_cmd,
            'fin_bottom': PWM_NEUTRAL + pitch_cmd + yaw_cmd,
            'fin_right':  PWM_NEUTRAL + roll_cmd + yaw_cmd,
            'fin_left':   PWM_NEUTRAL - roll_cmd + yaw_cmd,
        }
        for name, pwm in fins.items():
            self._set_servo(SERVO_CHANNELS[name], pwm)

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
        while self.running and self.mission_active:
            self.read_sensors()
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

            time.sleep(0.1)  # 10 Hz

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
            self.depth_pid.reset()
            self.straight_distance_completed = 0.0
            self.outbound_distance = 0.0
            self.last_dist_update_t = time.time()

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
            return False
        except Exception as e:
            print(f"\n❌ Görev hatası: {e}")
            return False
        finally:
            self.cleanup()

    def cleanup(self):
        self.mission_active = False
        self.running = False
        print("\n🧹 Sistem temizleniyor...")

        if self.connected:
            self.set_motor_throttle_pwm(PWM_NEUTRAL)
            self.set_control_surfaces()

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
