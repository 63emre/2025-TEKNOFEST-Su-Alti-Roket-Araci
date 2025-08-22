#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TAM STABİLİZASYON SİSTEMİ (ROLL + PITCH + YAW)
- 4 kanat koordineli stabilizasyon
- Roll: Sol-sağ kanatlar zıt hareket
- Pitch: Ön-arka kanatlar zıt hareket  
- Yaw: Çapraz kanatlar koordineli hareket
- Tüm eksenler aynı anda aktif
"""

import time
from pymavlink import mavutil
import math

# ---- Bağlantı ----
PORT = "/dev/ttyACM0"
BAUD  = 115200

# ---- Servo kanalları ----
# Kanat pozisyonları (+ şeklinde):
#      ÜST (14)
#         |
# SOL (13) + SAĞ (12)
#         |
#      ALT (11)
#
SERVO_UP    = 14   # ÜST kanat
SERVO_DOWN  = 11   # ALT kanat  
SERVO_RIGHT = 12   # SAĞ kanat
SERVO_LEFT  = 13   # SOL kanat

# ---- PWM ----
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def to_pwm(delta_us):  return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

# ---- Kontrol Ayarları ----
# İşaret yönleri (sahada test ederek ayarlayın)
ROLL_SENSE  = +1.0  # CCW roll -> sol yukarı, sağ aşağı
PITCH_SENSE = +1.0  # +pitch (burun yukarı) -> ön yukarı, arka aşağı
YAW_SENSE   = +1.0  # CCW yaw -> çapraz koordinasyon

# Mekanik yönler (PWM artınca fiziksel yukarı olmuyorsa -1)
DIR_UP    = +1.0  # SERVO_UP (14)
DIR_DOWN  = +1.0  # SERVO_DOWN (11)
DIR_RIGHT = -1.0  # SERVO_RIGHT (12)
DIR_LEFT  = -1.0  # SERVO_LEFT (13)

# Kazanç değerleri (sahada fine-tune edin)
K_ROLL_US_PER_RAD  = 500.0  # Roll duyarlılığı
K_PITCH_US_PER_RAD = 500.0  # Pitch duyarlılığı  
K_YAW_US_PER_RAD   = 400.0  # Yaw duyarlılığı

# Deadband (küçük açıları bastır)
DEADBAND_ROLL_DEG  = 1.0
DEADBAND_PITCH_DEG = 1.0
DEADBAND_YAW_DEG   = 2.0

# Güvenlik sınırları
MAX_DELTA_US = 350.0  # Toplam PWM değişimi sınırı

def set_servo(m, ch, pwm):
    """Servo PWM değeri gönder"""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(ch), float(pwm), 0,0,0,0,0
    )

def neutral_all(m):
    """Tüm servoları nötr konuma getir"""
    for ch in (SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT):
        set_servo(m, ch, PWM_NEU)

def calculate_wing_commands(roll, pitch, yaw):
    """
    Roll, pitch, yaw değerlerinden 4 kanat komutlarını hesapla
    
    Gerçek servo kullanımına göre:
    - ROLL: SERVO_LEFT (13) ve SERVO_RIGHT (12) kullanılır
    - PITCH: SERVO_RIGHT (12) ve SERVO_LEFT (13) kullanılır  
    - YAW: Tüm 4 servo çapraz koordinasyonla kullanılır
    
    Kanat pozisyonları (+ şeklinde):
         ÜST (14)
            |
    SOL (13) + SAĞ (12)
            |
         ALT (11)
    """
    
    # Her kanat için komut başlangıçta sıfır
    up_cmd = 0.0
    down_cmd = 0.0  
    right_cmd = 0.0
    left_cmd = 0.0
    
    # ROLL kontrolü (manuel_roll.py'deki mantık)
    # CCW roll -> SOL ↑, SAĞ ↓ (sadece LEFT ve RIGHT servoları kullanılır)
    # Fiziksel olarak karşılıklı kanatlar, aynı PWM değişimi zıt hareket yaratır
    if abs(roll) > 0:
        roll_signal = ROLL_SENSE * roll * K_ROLL_US_PER_RAD
        left_cmd  += +roll_signal  # Sol kanat
        right_cmd += -roll_signal  # Sağ kanat (zıt yön)
        # UP ve DOWN bu eksende kullanılmaz
    
    # PITCH kontrolü (manuel_pitch.py'deki mantık)  
    # +pitch -> RIGHT ↑, LEFT ↓ (sadece RIGHT ve LEFT servoları kullanılır)
    if abs(pitch) > 0:
        pitch_signal = PITCH_SENSE * pitch * K_PITCH_US_PER_RAD
        right_cmd += -pitch_signal  # Sağ kanat aşağı (burun yukarı için)
        left_cmd  += +pitch_signal  # Sol kanat yukarı (burun yukarı için)
        # UP ve DOWN bu eksende kullanılmaz
    
    # YAW kontrolü (manuel_yaw.py'deki mantık)
    # CCW yaw -> çapraz koordinasyon (tüm 4 servo kullanılır)
    if abs(yaw) > 0:
        yaw_signal = YAW_SENSE * yaw * K_YAW_US_PER_RAD
        # Çapraz grup 1: UP + DOWN (zıt yönler)
        up_cmd   += +yaw_signal
        down_cmd += -yaw_signal  
        # Çapraz grup 2: RIGHT + LEFT (grup 1'in tersi)
        right_cmd += -yaw_signal
        left_cmd  += +yaw_signal
    
    # Mekanik yön düzeltmeleri uygula
    up_cmd    *= DIR_UP
    down_cmd  *= DIR_DOWN
    right_cmd *= DIR_RIGHT
    left_cmd  *= DIR_LEFT
    
    # Güvenlik sınırları uygula
    up_cmd    = max(-MAX_DELTA_US, min(MAX_DELTA_US, up_cmd))
    down_cmd  = max(-MAX_DELTA_US, min(MAX_DELTA_US, down_cmd))
    right_cmd = max(-MAX_DELTA_US, min(MAX_DELTA_US, right_cmd))
    left_cmd  = max(-MAX_DELTA_US, min(MAX_DELTA_US, left_cmd))
    
    return up_cmd, down_cmd, right_cmd, left_cmd

def main():
    """Ana kontrol döngüsü"""
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=10):
        print("Heartbeat alınamadı!"); return
        
    print("🚁 SARA Tam Stabilizasyon Sistemi Başlatıldı")
    print("📡 MAVLink bağlantısı kuruldu")
    print("🎯 Roll + Pitch + Yaw stabilizasyonu aktif")
    print("⏹️  Çıkmak için Ctrl+C")
    print("-" * 50)

    last_msg_t = time.perf_counter()
    yaw_offset = None  # Yaw referans noktası
    loop_count = 0

    try:
        while True:
            msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            now = time.perf_counter()
            loop_count += 1

            if msg is None:
                # Veri gelmiyor, güvenlik için nötrle
                if now - last_msg_t > 1.5:
                    neutral_all(m)
                    print("⚠️  Veri yok, servolar nötrde")
                continue

            last_msg_t = now

            # Attitude verilerini al
            raw_roll  = float(msg.roll)
            raw_pitch = float(msg.pitch)
            raw_yaw   = float(msg.yaw)
            
            # Yaw offset ayarla (ilk okumada)
            if yaw_offset is None:
                yaw_offset = raw_yaw
                print(f"🧭 Yaw referansı: {math.degrees(yaw_offset):.1f}°")
                continue
            
            # Relatif açıları hesapla
            roll = raw_roll
            pitch = raw_pitch
            yaw = raw_yaw - yaw_offset
            
            # Yaw'ı -π ile +π arasında tut
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi

            # Derece cinsinden
            roll_deg  = math.degrees(roll)
            pitch_deg = math.degrees(pitch)
            yaw_deg   = math.degrees(yaw)

            # Deadband uygula
            active_roll  = roll  if abs(roll_deg)  > DEADBAND_ROLL_DEG  else 0.0
            active_pitch = pitch if abs(pitch_deg) > DEADBAND_PITCH_DEG else 0.0
            active_yaw   = yaw   if abs(yaw_deg)   > DEADBAND_YAW_DEG   else 0.0

            # Kanat komutlarını hesapla
            up_cmd, down_cmd, right_cmd, left_cmd = calculate_wing_commands(
                active_roll, active_pitch, active_yaw
            )

            # Servolar gönder
            set_servo(m, SERVO_UP,    to_pwm(up_cmd))     # 14: ÜST kanat
            set_servo(m, SERVO_DOWN,  to_pwm(down_cmd))   # 11: ALT kanat
            set_servo(m, SERVO_RIGHT, to_pwm(right_cmd))  # 12: SAĞ kanat
            set_servo(m, SERVO_LEFT,  to_pwm(left_cmd))   # 13: SOL kanat

            # Periyodik durum raporu (her 2 saniyede bir)
            if loop_count % 100 == 0:  # ~50Hz'de 2 saniye
                print(f"📊 R:{roll_deg:+5.1f}° P:{pitch_deg:+5.1f}° Y:{yaw_deg:+5.1f}° | "
                      f"PWM: {to_pwm(up_cmd):4d} {to_pwm(down_cmd):4d} {to_pwm(right_cmd):4d} {to_pwm(left_cmd):4d}")

            # Kontrol frekansı (~50 Hz)
            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\n🛑 Sistem durduruluyor...")
    except Exception as e:
        print(f"\n❌ Hata: {e}")
    finally:
        try:
            neutral_all(m)
            print("✅ Servolar nötr konuma getirildi")
        except Exception:
            print("⚠️  Servo nötrlemede sorun")

if __name__ == "__main__":
    main()
