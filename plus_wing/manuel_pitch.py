#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PITCH SAHA TESTİ (yalnızca RIGHT & LEFT)
- Pixhawk'ı elle pitch ekseninde (burun yukarı/aşağı) çevirerek test.
- Beklenti (arkadan bakış): +pitch (burun yukarı) -> RIGHT ↑, LEFT ↓
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
# MAVLink pitch işaretini “+pitch = burun yukarı” beklentine uydurmak için.
# Eğer sahada ters davranıyorsa PITCH_SENSE'i -1 yap.
PITCH_SENSE = +1.0

# Mekanik yön: PWM artınca fiziksel "yukarı" olmuyorsa ilgili DIR'ü -1 yap.
DIR_UP   = +1.0
DIR_DOWN = +1.0

# Kazanç ve sınırlar
K_ANG_US_PER_RAD = 500.0   # pitch (rad) -> mikro-saniye; sahada 300–700 deneyin
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
    print("Bağlandı. PITCH test başlayacak… (Ctrl+C ile çıkış)")

    # UP/DOWN nötrde dursun
    set_servo(m, SERVO_UP,    PWM_NEU)
    set_servo(m, SERVO_DOWN,  PWM_NEU)

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

            # Pitch (rad)
            pitch = float(msg.pitch)  # MAVLink rad
            pitch_deg = math.degrees(pitch)

            # Küçük açıları bastır (titreşim vb.)
            if abs(pitch_deg) < DEADBAND_DEG:
                up_cmd_us = 0.0
                down_cmd_us = 0.0
            else:
                # İstenen davranış: +pitch (burun yukarı) -> UP ↑, DOWN ↓
                u = PITCH_SENSE * pitch * K_ANG_US_PER_RAD
                up_cmd_us   = (+u) * DIR_UP
                down_cmd_us = (-u) * DIR_DOWN

                # Sınırla
                up_cmd_us   = max(-MAX_DELTA_US, min(MAX_DELTA_US, up_cmd_us))
                down_cmd_us = max(-MAX_DELTA_US, min(MAX_DELTA_US, down_cmd_us))

            # Çıkış ver
            set_servo(m, SERVO_RIGHT,   to_pwm(up_cmd_us))
            set_servo(m, SERVO_LEFT, to_pwm(down_cmd_us))

            # Hafif bekleme (yakl. 50 Hz)
            # time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nÇıkılıyor, servolar nötrleniyor…")
    finally:
        try: neutral_all(m)
        except Exception: pass

if __name__ == "__main__":
    main()
