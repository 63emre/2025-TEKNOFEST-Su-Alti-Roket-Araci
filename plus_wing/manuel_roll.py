#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROLL SAHA TESTİ (yalnızca LEFT & RIGHT)
- Pixhawk'ı elle roll ekseninde çevirerek test.
- Beklenti (arkadan bakış): CCW roll -> SOL ↑, SAĞ ↓
- UP/DOWN nötr.
"""

import time
from pymavlink import mavutil
import math

# ---- Bağlantı ----
PORT = "/dev/ttyACM0"
BAUD  = 115200

# ---- Servo kanalları ----
SERVO_UP    = 14   # nötrde
SERVO_DOWN  = 11   # nötrde
SERVO_RIGHT = 12   # test
SERVO_LEFT  = 13   # test

# ---- PWM ----
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def to_pwm(delta_us):  return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

# ---- Ayarlar ----
# MAVLink roll işaretini sizin "CCW" tanımınıza uydurmak için:
# Eğer CCW çevirdiğinizde beklenenin TERSİ oluyorsa ROLL_SENSE'i -1 yapın.
ROLL_SENSE = +1.0

# Mekanik yön: PWM artınca fiziksel "yukarı" olmuyorsa ilgili DIR'ü -1 yapın.
DIR_LEFT  = +1.0
DIR_RIGHT = -1.0

# Kazanç ve sınırlar
K_ANG_US_PER_RAD = 500.0   # roll (rad) -> mikro-saniye; sahada 300–700 deneyin
DEADBAND_DEG     = 1.0     # küçük açıları bastır
MAX_DELTA_US     = 350.0   # güvenli genlik sınırı

def set_servo(m, ch, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(ch), float(pwm), 0,0,0,0,0
    )

def neutral_all(m):
    for ch in (SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT):
        set_servo(m, ch, PWM_NEU)

def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=10):
        print("Heartbeat yok."); return
    print("Bağlandı. ROLL test başlayacak… (Ctrl+C ile çıkış)")
    # UP/DOWN nötr
    set_servo(m, SERVO_UP,   PWM_NEU)
    set_servo(m, SERVO_DOWN, PWM_NEU)

    last_msg_t = time.perf_counter()

    try:
        while True:
            msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            now = time.perf_counter()

            if msg is None:
                # 1.5 s veri yoksa nötrle
                if now - last_msg_t > 1.5:
                    neutral_all(m)
                continue

            last_msg_t = now

            # Roll (rad)
            roll = float(msg.roll)  # MAVLink rad
            roll_deg = math.degrees(roll)

            # Küçük açıları bastır (titreşim vb.)
            if abs(roll_deg) < DEADBAND_DEG:
                left_cmd_us = 0.0
                right_cmd_us = 0.0
            else:
                # İstenen davranış: CCW roll -> SOL ↑, SAĞ ↓
                # Bunu doğrudan açıyla orantılayalım (P kontrol).
                u = ROLL_SENSE * roll * K_ANG_US_PER_RAD
                # Sol yukarı için +u, sağ aşağı için -u
                left_cmd_us  =  (+u) * DIR_LEFT
                right_cmd_us =  (-u) * DIR_RIGHT

                # Sınırla
                left_cmd_us  = max(-MAX_DELTA_US, min(MAX_DELTA_US, left_cmd_us))
                right_cmd_us = max(-MAX_DELTA_US, min(MAX_DELTA_US, right_cmd_us))

            # Çıkış ver
            set_servo(m, SERVO_LEFT,  to_pwm(left_cmd_us))
            set_servo(m, SERVO_RIGHT, to_pwm(right_cmd_us))

            # Hafif bekleme (yakl. 50 Hz)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nÇıkılıyor, servolar nötrleniyor…")
    finally:
        try: neutral_all(m)
        except Exception: pass

if __name__ == "__main__":
    main()
