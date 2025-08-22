#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Dört servo ile PD tabanlı stabilize: UP, RIGHT, DOWN, LEFT
- Arkadan bakış nedeniyle ROLL/PITCH işaretleri terslenmiştir.
- Servo mekanik yönleri kanal-bazlı ayarlanabilir.
- Yaw hata sarmalama, veri akışı isteği, nötr/failsafe eklendi.
"""

import math
import time
from pymavlink import mavutil

# --------- Bağlantı ----------
PORT = "/dev/ttyACM0"
BAUD = 115200

# --------- Servo Kanal Atamaları ----------
# ArduPilot için: AUX3..AUX6 tipik olarak 11..14'tür (donanıma göre değişebilir)
SERVO_UP    = 14   # AUX6 (UP)
SERVO_RIGHT = 12   # AUX4 (RIGHT)
SERVO_DOWN  = 11   # AUX3 (DOWN)
SERVO_LEFT  = 13   # AUX5 (LEFT)

# --------- PWM Limitleri ----------
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def to_pwm(delta_us):  return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

# --------- PD Kazançları (µs/rad ve µs/(rad/s)) ----------
KP_ROLL,  KD_ROLL  = 500.0, 80.0
KP_PITCH, KD_PITCH = 500.0, 80.0
KP_YAW,   KD_YAW   = 200.0, 50.0

# --------- Setpointler ----------
# YAW_SP=None => yalnızca hız sönümleme (r_dot) yapılır, açı hatası kullanılmaz.
ROLL_SP, PITCH_SP, YAW_SP = 0.0, 0.0, None

# --------- Eksen İşaretleri (arkadan bakış) ----------
# Gözlem: roll/pitch ters; isterseniz +1/-1 ile oynayarak sahada ince ayar yapın.
ROLL_SIGN  = -1.0
PITCH_SIGN = -1.0
YAW_SIGN   = +1.0

# --------- Servo Mekanik Yön Çarpanları ----------
# Mekanik terslik varsa buradan düzeltin (örn. LEFT ters dönüyorsa -1 yapın).
DIR_UP    = +1.0
DIR_DOWN  = +1.0
DIR_RIGHT = +1.0
DIR_LEFT  = +1.0

# --------- Yaw Miksi İşaretleri ----------
# Eski davranış korunarak açıkça tanımlandı:
# Yaw: UP (-), DOWN (+), RIGHT (+), LEFT (-)
YAW_UP_SIGN    = -1.0
YAW_DOWN_SIGN  = +1.0
YAW_RIGHT_SIGN = +1.0
YAW_LEFT_SIGN  = -1.0

def wrap_pi(a):
    """[-pi, pi] aralığına sar."""
    a = (a + math.pi) % (2*math.pi) - math.pi
    return a

def set_servo(m, ch, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(ch), float(pwm), 0, 0, 0, 0, 0
    )

def request_attitude_stream(m, rate_hz):
    # ATTITUDE için mesaj aralığı talebi (ArduPilot destekler)
    try:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            1e6 / rate_hz, # usec
            0, 0, 0, 0, 0
        )
    except Exception:
        pass  # Uygulama desteklemiyorsa sessiz geç

def send_all_neutral(m):
    for ch in (SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT):
        set_servo(m, ch, PWM_NEU)

def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=10):
        print("Heartbeat yok."); return
    print("Bağlı. Stabilizasyon başlıyor… (Ctrl+C)")

    rate_hz = 50.0
    period  = 1.0 / rate_hz
    request_attitude_stream(m, rate_hz)

    last_msg_time = time.perf_counter()
    next_tick = time.perf_counter()

    try:
        while True:
            # Mesajı bekle (timeout ile)
            msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            now = time.perf_counter()

            if msg is None:
                # Uzun süre mesaj yoksa failsafe: nötrle
                if now - last_msg_time > 1.5:
                    send_all_neutral(m)
                # Zamanlamayı koru
                sleep_dt = next_tick - time.perf_counter()
                if sleep_dt > 0: time.sleep(sleep_dt)
                next_tick += period
                continue

            last_msg_time = now

            # Rad cinsinden
            roll   = float(msg.roll)
            pitch  = float(msg.pitch)
            yaw    = float(msg.yaw)
            p_dot  = float(msg.rollspeed)
            q_dot  = float(msg.pitchspeed)
            r_dot  = float(msg.yawspeed)

            # Açı hataları (yaw için sarmalama)
            e_roll  = ROLL_SP  - roll
            e_pitch = PITCH_SP - pitch
            if YAW_SP is None:
                e_yaw = 0.0
            else:
                e_yaw = wrap_pi(YAW_SP - yaw)

            # PD denetçi (arkadan bakış işaretleriyle)
            u_roll  = ROLL_SIGN  * (KP_ROLL  * e_roll  - KD_ROLL  * p_dot)
            u_pitch = PITCH_SIGN * (KP_PITCH * e_pitch - KD_PITCH * q_dot)
            u_yaw   = YAW_SIGN   * (KP_YAW   * e_yaw   - KD_YAW   * r_dot)

            # --- Mikser ---
            # Pitch: UP(+), DOWN(-)  (karşılıklı)
            # Roll : RIGHT(+), LEFT(-) (karşılıklı)
            # Yaw  : UP(-), DOWN(+), RIGHT(+), LEFT(-)
            up_cmd    =  ( +u_pitch ) + (YAW_UP_SIGN    * u_yaw)
            down_cmd  =  ( -u_pitch ) + (YAW_DOWN_SIGN  * u_yaw)
            right_cmd =  ( +u_roll  ) + (YAW_RIGHT_SIGN * u_yaw)
            left_cmd  =  ( -u_roll  ) + (YAW_LEFT_SIGN  * u_yaw)

            # Mekanik servo yön düzeltmeleri
            up_cmd    *= DIR_UP
            down_cmd  *= DIR_DOWN
            right_cmd *= DIR_RIGHT
            left_cmd  *= DIR_LEFT

            # PWM gönder
            set_servo(m, SERVO_UP,    to_pwm(up_cmd))
            set_servo(m, SERVO_DOWN,  to_pwm(down_cmd))
            set_servo(m, SERVO_RIGHT, to_pwm(right_cmd))
            set_servo(m, SERVO_LEFT,  to_pwm(left_cmd))

            # 50 Hz zamanlama
            sleep_dt = next_tick - time.perf_counter()
            if sleep_dt > 0: time.sleep(sleep_dt)
            next_tick += period

    except KeyboardInterrupt:
        print("\nÇıkılıyor (servolar nötr).")
    finally:
        try:
            send_all_neutral(m)
        except Exception:
            pass

if __name__ == "__main__":
    main()
