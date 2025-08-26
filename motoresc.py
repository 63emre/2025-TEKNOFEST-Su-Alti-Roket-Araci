#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Motor Kontrol Testi
DEGZ BLU 30A ESC + M5 Su Altı Motor (Pixhawk/ArduSub, MAVLink üzerinden)

⚠️ Güvenlik:
- Pervaneyi sökün veya aracı emniyete alın.
- Acil durdurma (hard-kill) erişiminizde olsun.
- Testler sırasında çevre güvenliğini sağlayın.
"""

import os
import time
import threading
import math
from pymavlink import mavutil

# ========= Kullanıcı Ayarları (Windows/COM5) =========
# Ortam değişkeni yoksa COM5, 115200 kullan
DEFAULT_PORT = os.getenv("MAV_ADDRESS", "COM5")
DEFAULT_BAUD = int(os.getenv("MAV_BAUD", "115200"))

# Proje standardına göre: Ana motor AUX1 → ArduPilot kanal 9
# (MAIN1-8 = 1-8, AUX1-6 = 9-14)
MOTOR_CHANNEL = 9

# ESC PWM aralıkları (µs)
ESC_MIN = 1000
ESC_NEUTRAL = 1500
ESC_MAX = 2000
ESC_ARM_SIGNAL = 1000

# Güvenlik limitleri
MOTOR_SAFE_MIN = 1100     # ileri/geri dışına çıkma koruması için taban
MOTOR_SAFE_MAX = 1900     # testte ileri için üst sınır
MAX_TEST_FORWARD = 1700   # test güvenliği için sınır
MAX_TEST_REVERSE = 1400   # reverse destekli ESC'lerde ~1400'e kadar

# Ramp parametreleri
MOTOR_RAMP_RATE = 10      # PWM/saniye artış hızı (µs/s), 10 → yumuşak

class MotorController:
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        self.port = port
        self.baud = baud
        self.master = None
        self.connected = False
        self.motor_armed = False

        self.current_throttle = ESC_NEUTRAL
        self.target_throttle = ESC_NEUTRAL
        self.emergency_stop = False

        self.max_test_forward = MAX_TEST_FORWARD
        self.max_test_reverse = MAX_TEST_REVERSE

        self.control_thread = None
        self.running = False

    # ---------- MAVLink / Pixhawk ----------
    def connect_pixhawk(self):
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor… ({self.port} @ {self.baud} baud)")
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud, autoreconnect=True)
            print("💓 Heartbeat bekleniyor…")
            self.master.wait_heartbeat(timeout=20)
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False

    def arm_motor(self):
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False

        print("🔫 Motor arming işlemi başlatılıyor…")
        try:
            # ESC arming sequence: min → neutral
            print("  📡 Min (arming) sinyali…")
            for _ in range(20):  # ~2 sn
                self.send_motor_command(ESC_ARM_SIGNAL, clamp=False)
                time.sleep(0.1)

            print("  📡 Neutral…")
            for _ in range(10):  # ~1 sn
                self.send_motor_command(ESC_NEUTRAL, clamp=False)
                time.sleep(0.1)

            self.motor_armed = True
            self.current_throttle = ESC_NEUTRAL
            print("✅ Motor armed!")
            return True
        except Exception as e:
            print(f"❌ Arming hatası: {e}")
            return False

    # ---------- PWM Komutları ----------
    def send_motor_command(self, throttle_pwm, clamp=True):
        if not self.connected:
            return False

        if self.emergency_stop:
            throttle_pwm = ESC_NEUTRAL

        pwm = int(throttle_pwm)

        if clamp:
            if pwm >= ESC_NEUTRAL:
                pwm = min(pwm, self.max_test_forward, MOTOR_SAFE_MAX)
            else:
                pwm = max(pwm, self.max_test_reverse, ESC_MIN, MOTOR_SAFE_MIN)

        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL,  # ArduPilot çıkış kanalı
                pwm,
                0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"❌ Komut hatası: {e}")
            return False

    # ---------- Yumuşak geçiş ----------
    def set_throttle_smooth(self, target_pwm):
        if not self.motor_armed:
            print("❌ Motor armed değil!")
            return False

        self.target_throttle = int(target_pwm)

        if self.control_thread and self.control_thread.is_alive():
            # Zaten ramp thread çalışıyor; hedef güncellendi
            return True

        self.running = True
        self.control_thread = threading.Thread(target=self._throttle_ramp_worker, daemon=True)
        self.control_thread.start()
        return True

    def _throttle_ramp_worker(self):
        while self.running and self.motor_armed:
            if self.current_throttle != self.target_throttle:
                if self.current_throttle < self.target_throttle:
                    self.current_throttle = min(self.current_throttle + MOTOR_RAMP_RATE, self.target_throttle)
                else:
                    self.current_throttle = max(self.current_throttle - MOTOR_RAMP_RATE, self.target_throttle)
                self.send_motor_command(self.current_throttle)
            time.sleep(0.1)  # 10 Hz

    # ---------- Testler ----------
    def motor_arming_test(self):
        print("\n🔫 MOTOR ARMING TESTİ\n" + "-"*40)
        if not self.arm_motor():
            print("❌ Motor arming başarısız!")
            return False
        print("✅ Motor arming başarılı!")
        return True

    def throttle_response_test(self):
        print("\n🚀 THROTTLE TEPKİ TESTİ\n" + "-"*40)
        if not self.motor_armed and not self.arm_motor():
            return False

        test_values = [
            (ESC_NEUTRAL, "NEUTRAL"),
            (1550, "DÜŞÜK FORWARD"),
            (1600, "ORTA FORWARD"),
            (1650, "YÜKSEK FORWARD"),
            (ESC_NEUTRAL, "NEUTRAL"),
            (1480, "DÜŞÜK REVERSE"),
            (1450, "ORTA REVERSE"),
            (1420, "YÜKSEK REVERSE"),
            (ESC_NEUTRAL, "NEUTRAL STOP"),
        ]

        for pwm, desc in test_values:
            print(f"\n🎯 {desc} ({pwm}µs)")
            self.set_throttle_smooth(pwm)
            while abs(self.current_throttle - pwm) > 5:
                print(f"  📊 {self.current_throttle}µs → hedef {pwm}µs")
                time.sleep(0.4)
            print(f"  ✅ Ulaşıldı: {self.current_throttle}µs")
            time.sleep(1.5)

        print("✅ Throttle tepki testi tamamlandı")
        return True

    def motor_ramp_test(self):
        print("\n📈 MOTOR RAMP TESTİ\n" + "-"*40)
        if not self.motor_armed and not self.arm_motor():
            return False

        steps = 20
        thr_range = self.max_test_forward - ESC_NEUTRAL
        step = max(1, thr_range // steps)

        print("📊 Neutral → max forward (yavaş artış)")
        for i in range(steps + 1):
            pwm = ESC_NEUTRAL + i*step
            print(f"  🎚️ Step {i+1}/{steps+1}: {pwm}µs")
            self.send_motor_command(pwm)
            time.sleep(0.8)

        print("\n📉 Max → Neutral (geri dönüş)")
        for i in range(steps, -1, -1):
            pwm = ESC_NEUTRAL + i*step
            print(f"  🎚️ Step {steps-i+1}/{steps+1}: {pwm}µs")
            self.send_motor_command(pwm)
            time.sleep(0.5)

        print("✅ Ramp testi tamamlandı")
        return True

    def motor_direction_test(self):
        print("\n↔️ MOTOR YÖN TESTİ\n" + "-"*40)
        if not self.motor_armed and not self.arm_motor():
            return False

        print("🔼 FORWARD:")
        for pwm in [1520, 1550, 1600, 1650]:
            print(f"  ➡️ {pwm}µs")
            self.send_motor_command(pwm)
            time.sleep(2)

        print("  ⏸️ Neutral")
        self.send_motor_command(ESC_NEUTRAL)
        time.sleep(1.5)

        print("\n🔽 REVERSE:")
        for pwm in [1480, 1450, 1420]:
            print(f"  ⬅️ {pwm}µs")
            self.send_motor_command(pwm)
            time.sleep(2)

        self.send_motor_command(ESC_NEUTRAL)
        print("✅ Yön testi tamamlandı")
        return True

    def motor_oscillation_test(self):
        print("\n🌊 OSCILLATION (sinüs) TESTİ\n" + "-"*40)
        if not self.motor_armed and not self.arm_motor():
            return False

        duration = 20.0  # sn
        freq = 0.2       # Hz (5 sn periyot)
        amp = 100        # ±100 µs

        t0 = time.time()
        while time.time() - t0 < duration:
            t = time.time() - t0
            offset = int(amp * math.sin(2*math.pi*freq*t))
            pwm = ESC_NEUTRAL + offset
            self.send_motor_command(pwm)
            print(f"  📊 PWM: {pwm}µs (offset {offset:+d})")
            time.sleep(0.2)

        self.send_motor_command(ESC_NEUTRAL)
        print("✅ Oscillation testi tamamlandı")
        return True

    def emergency_stop_test(self):
        print("\n🚨 ACİL DURDURMA TESTİ\n" + "-"*40)
        if not self.motor_armed and not self.arm_motor():
            return False

        print("🚀 %~50 ileri…")
        test_pwm = ESC_NEUTRAL + 150
        self.send_motor_command(test_pwm)
        time.sleep(2.5)

        print("🚨 EMERGENCY STOP!")
        self.emergency_stop = True
        self.send_motor_command(ESC_NEUTRAL)
        time.sleep(1.5)
        self.emergency_stop = False
        print("✅ Motor güvenli şekilde durdu")
        return True

    def run_full_test_suite(self):
        print("🧪 MOTOR KONTROL TAM TEST PAKETİ")
        print("=" * 50)

        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False

        print("\n⚠️ GÜVENLİK KONTROL LİSTESİ")
        print("- Pervane TAKILI DEĞİL")
        print("- Test alanı boş ve güvenli")
        print("- Hard-kill erişimde")
        input("\n✅ Hazırım (ENTER)…")

        try:
            if not self.motor_arming_test():
                return False
            input("\n⏸️ Devam (ENTER)…")

            self.throttle_response_test()
            input("\n⏸️ Devam (ENTER)…")

            self.motor_ramp_test()
            input("\n⏸️ Devam (ENTER)…")

            self.motor_direction_test()
            input("\n⏸️ Devam (ENTER)…")

            self.motor_oscillation_test()
            input("\n⏸️ Devam (ENTER)…")

            self.emergency_stop_test()
            print("\n🎉 TÜM TESTLER BAŞARILI!")
            return True

        except KeyboardInterrupt:
            print("\n⚠️ Kullanıcı tarafından durduruldu")
            self.emergency_stop = True
            self.send_motor_command(ESC_NEUTRAL)
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()

    def cleanup(self):
        self.running = False
        self.emergency_stop = True
        try:
            if self.connected:
                self.send_motor_command(ESC_NEUTRAL, clamp=False)
                time.sleep(0.8)
        finally:
            try:
                if self.master:
                    self.master.close()
                    print("🔌 MAVLink bağlantısı kapatıldı")
            except Exception:
                pass


def main():
    mc = MotorController(port=DEFAULT_PORT, baud=DEFAULT_BAUD)
    try:
        ok = mc.run_full_test_suite()
        return 0 if ok else 1
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1
    finally:
        mc.cleanup()


if __name__ == "__main__":
    import sys
    sys.exit(main())
