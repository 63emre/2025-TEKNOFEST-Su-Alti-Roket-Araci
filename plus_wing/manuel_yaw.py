#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YAW SAHA TESTİ (4 kanat koordineli)
- Pixhawk'ı elle yaw ekseninde (Z ekseni etrafında) çevirerek test.
- Beklenti: CCW yaw -> LEFT-UP ↑ & RIGHT-DOWN ↑ (çapraz koordinasyon)
- 4 kanat koordineli yaw stabilizasyonu.
"""

import time
from pymavlink import mavutil
import math

# ---- Bağlantı ----
PORT = "/dev/ttyACM0"
BAUD  = 115200

# ---- Servo kanalları ----
SERVO_UP    = 14   # ÜST kanat (çapraz grup 1)
SERVO_DOWN  = 11   # ALT kanat (çapraz grup 1)
SERVO_RIGHT = 12   # SAĞ kanat (çapraz grup 2)
SERVO_LEFT  = 13   # SOL kanat (çapraz grup 2)

# ---- PWM ----
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def to_pwm(delta_us):  return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

# ---- Ayarlar ----
# MAVLink yaw işaretini sizin "CCW" tanımınıza uydurmak için:
# Eğer CCW çevirdiğinizde beklenenin TERSİ oluyorsa YAW_SENSE'i -1 yapın.
YAW_SENSE = +1.0

# Mekanik yön: PWM artınca fiziksel "yukarı" olmuyorsa ilgili DIR'ü -1 yapın.
# Yaw için çapraz koordinasyon:
# Grup 1: ÜST & ALT (aynı yön)
# Grup 2: SAĞ & SOL (aynı yön, grup 1'in tersi)
DIR_UP    = +1.0  # SERVO_UP (14)
DIR_DOWN  = +1.0  # SERVO_DOWN (11)
DIR_RIGHT = -1.0  # SERVO_RIGHT (12) 
DIR_LEFT  = -1.0  # SERVO_LEFT (13)

# Kazanç ve sınırlar
K_ANG_US_PER_RAD = 400.0   # yaw (rad) -> mikro-saniye; sahada 200–600 deneyin
DEADBAND_DEG     = 2.0     # küçük açıları bastır (yaw için biraz daha büyük)
MAX_DELTA_US     = 300.0   # güvenli genlik sınırı

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
    print("Bağlandı. YAW test başlayacak… (Ctrl+C ile çıkış)")
    print("CCW yaw -> Çapraz kanatlar koordineli hareket edecek")

    last_msg_t = time.perf_counter()
    yaw_offset = None  # İlk yaw değerini sıfır referans olarak kullan

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

            # Yaw (rad) - sürekli döndürülebilir, bu yüzden offset kullanıyoruz
            raw_yaw = float(msg.yaw)
            
            # İlk okumada offset'i ayarla
            if yaw_offset is None:
                yaw_offset = raw_yaw
                print(f"Yaw referans noktası ayarlandı: {math.degrees(yaw_offset):.1f}°")
                continue
            
            # Relatif yaw hesapla
            yaw = raw_yaw - yaw_offset
            
            # Yaw'ı -π ile +π arasında tut
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi
                
            yaw_deg = math.degrees(yaw)

            # Küçük açıları bastır (titreşim vb.)
            if abs(yaw_deg) < DEADBAND_DEG:
                up_cmd_us    = 0.0
                down_cmd_us  = 0.0
                right_cmd_us = 0.0
                left_cmd_us  = 0.0
            else:
                # İstenen davranış: CCW yaw -> çapraz koordinasyon
                # Grup 1 (ÜST & ALT): +u
                # Grup 2 (SAĞ & SOL): -u
                u = YAW_SENSE * yaw * K_ANG_US_PER_RAD
                
                up_cmd_us    = (+u) * DIR_UP     # SERVO_UP (14)
                down_cmd_us  = (+u) * DIR_DOWN   # SERVO_DOWN (11)
                right_cmd_us = (-u) * DIR_RIGHT  # SERVO_RIGHT (12)
                left_cmd_us  = (-u) * DIR_LEFT   # SERVO_LEFT (13)

                # Sınırla
                up_cmd_us    = max(-MAX_DELTA_US, min(MAX_DELTA_US, up_cmd_us))
                down_cmd_us  = max(-MAX_DELTA_US, min(MAX_DELTA_US, down_cmd_us))
                right_cmd_us = max(-MAX_DELTA_US, min(MAX_DELTA_US, right_cmd_us))
                left_cmd_us  = max(-MAX_DELTA_US, min(MAX_DELTA_US, left_cmd_us))

            # Çıkış ver
            set_servo(m, SERVO_UP,    to_pwm(up_cmd_us))    # 14: ÜST kanat
            set_servo(m, SERVO_DOWN,  to_pwm(down_cmd_us))  # 11: ALT kanat
            set_servo(m, SERVO_RIGHT, to_pwm(right_cmd_us)) # 12: SAĞ kanat
            set_servo(m, SERVO_LEFT,  to_pwm(left_cmd_us))  # 13: SOL kanat

            # Debug çıktısı (her 20 iterasyonda bir)
            if int(time.time() * 5) % 20 == 0:
                print(f"Yaw: {yaw_deg:+6.1f}° | PWM: ÜST={to_pwm(up_cmd_us)} ALT={to_pwm(down_cmd_us)} SAĞ={to_pwm(right_cmd_us)} SOL={to_pwm(left_cmd_us)}")

            # Hafif bekleme (yakl. 50 Hz)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nÇıkılıyor, servolar nötrleniyor…")
    finally:
        try: neutral_all(m)
        except Exception: pass

if __name__ == "__main__":
    main()
