#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FULL STABİLİZASYON SİSTEMİ
- Roll, Pitch ve Yaw eksenlerini aynı anda stabilize eder
- Manuel testlerden alınan parametrelerle optimize edilmiştir
- 4 kanat (UP, DOWN, LEFT, RIGHT) koordineli kontrolü
"""

import time
from pymavlink import mavutil
import math

# ---- Bağlantı ----
PORT = "/dev/ttyACM0"
BAUD = 115200

# ---- Servo kanalları ----
SERVO_UP    = 14   # ÜST kanat
SERVO_DOWN  = 11   # ALT kanat  
SERVO_RIGHT = 12   # SAĞ kanat
SERVO_LEFT  = 13   # SOL kanat

# ---- PWM ----
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def to_pwm(delta_us):  
    return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

# ---- Stabilizasyon Ayarları ----
# Manuel testlerden alınan çalışan parametreler

# Roll stabilizasyonu (manuel_roll.py'den)
ROLL_SENSE = +1.0
ROLL_K_ANG_US_PER_RAD = 500.0
ROLL_DEADBAND_DEG = 1.0
ROLL_MAX_DELTA_US = 350.0

# Pitch stabilizasyonu (manuel_pitch.py'den)  
PITCH_SENSE = +1.0
PITCH_K_ANG_US_PER_RAD = 500.0
PITCH_DEADBAND_DEG = 1.0
PITCH_MAX_DELTA_US = 350.0

# Yaw stabilizasyonu (manuel_yaw.py'den)
YAW_SENSE = +1.0
YAW_K_ANG_US_PER_RAD = 400.0
YAW_DEADBAND_DEG = 2.0
YAW_MAX_DELTA_US = 300.0

# ---- Mekanik Yönler ----
# Manuel testlerden alınan çalışan yön ayarları

# Roll için (LEFT & RIGHT kanatlar)
ROLL_DIR_LEFT  = +1.0
ROLL_DIR_RIGHT = +1.0

# Pitch için (UP & DOWN kanatlar)
PITCH_DIR_UP   = +1.0
PITCH_DIR_DOWN = +1.0

# Yaw için (4 kanat koordineli)
YAW_DIR_UP    = +1.0
YAW_DIR_DOWN  = +1.0
YAW_DIR_RIGHT = -1.0
YAW_DIR_LEFT  = -1.0

# ---- Genel Sınırlar ----
OVERALL_MAX_DELTA_US = 400.0  # Tüm eksenlerin toplamı için güvenlik sınırı

def set_servo(m, ch, pwm):
    """Servo pozisyon komutu gönder"""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(ch), float(pwm), 0,0,0,0,0
    )

def neutral_all(m):
    """Tüm servoları nötr pozisyona getir"""
    for ch in (SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT):
        set_servo(m, ch, PWM_NEU)

def calculate_roll_commands(roll):
    """Roll ekseni için servo komutlarını hesapla"""
    roll_deg = math.degrees(roll)
    
    if abs(roll_deg) < ROLL_DEADBAND_DEG:
        return 0.0, 0.0  # left_cmd, right_cmd
    
    # CCW roll -> SOL ↑, SAĞ ↓ (aynı PWM değişimi, fiziksel zıt hareket)
    u = ROLL_SENSE * roll * ROLL_K_ANG_US_PER_RAD
    left_cmd_us  = (+u) * ROLL_DIR_LEFT
    right_cmd_us = (+u) * ROLL_DIR_RIGHT
    
    # Sınırla
    left_cmd_us  = max(-ROLL_MAX_DELTA_US, min(ROLL_MAX_DELTA_US, left_cmd_us))
    right_cmd_us = max(-ROLL_MAX_DELTA_US, min(ROLL_MAX_DELTA_US, right_cmd_us))
    
    return left_cmd_us, right_cmd_us

def calculate_pitch_commands(pitch):
    """Pitch ekseni için servo komutlarını hesapla"""
    pitch_deg = math.degrees(pitch)
    
    if abs(pitch_deg) < PITCH_DEADBAND_DEG:
        return 0.0, 0.0  # right_cmd, left_cmd
    
    # +pitch (burun yukarı) -> RIGHT ↑, LEFT ↓ (manuel_pitch.py'den)
    u = PITCH_SENSE * pitch * PITCH_K_ANG_US_PER_RAD
    right_cmd_us = (+u) * PITCH_DIR_UP    # SERVO_RIGHT için
    left_cmd_us  = (-u) * PITCH_DIR_DOWN  # SERVO_LEFT için
    
    # Sınırla
    right_cmd_us = max(-PITCH_MAX_DELTA_US, min(PITCH_MAX_DELTA_US, right_cmd_us))
    left_cmd_us  = max(-PITCH_MAX_DELTA_US, min(PITCH_MAX_DELTA_US, left_cmd_us))
    
    return right_cmd_us, left_cmd_us

def calculate_yaw_commands(yaw):
    """Yaw ekseni için servo komutlarını hesapla"""
    yaw_deg = math.degrees(yaw)
    
    if abs(yaw_deg) < YAW_DEADBAND_DEG:
        return 0.0, 0.0, 0.0, 0.0  # up_cmd, down_cmd, right_cmd, left_cmd
    
    # CCW yaw -> çapraz koordinasyon
    u = YAW_SENSE * yaw * YAW_K_ANG_US_PER_RAD
    
    up_cmd_us    = (-u) * YAW_DIR_UP     # SERVO_UP (14)
    down_cmd_us  = (+u) * YAW_DIR_DOWN   # SERVO_DOWN (11)
    right_cmd_us = (-u) * YAW_DIR_RIGHT  # SERVO_RIGHT (12)
    left_cmd_us  = (-u) * YAW_DIR_LEFT   # SERVO_LEFT (13)
    
    # Sınırla
    up_cmd_us    = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, up_cmd_us))
    down_cmd_us  = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, down_cmd_us))
    right_cmd_us = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, right_cmd_us))
    left_cmd_us  = max(-YAW_MAX_DELTA_US, min(YAW_MAX_DELTA_US, left_cmd_us))
    
    return up_cmd_us, down_cmd_us, right_cmd_us, left_cmd_us

def combine_commands(roll_left, roll_right, pitch_right, pitch_left, yaw_up, yaw_down, yaw_right, yaw_left):
    """Tüm eksenlerin komutlarını birleştir"""
    # Her kanat için komutları topla
    final_up_cmd    = yaw_up                    # Sadece YAW
    final_down_cmd  = yaw_down                  # Sadece YAW  
    final_right_cmd = roll_right + pitch_right + yaw_right  # ROLL + PITCH + YAW
    final_left_cmd  = roll_left + pitch_left + yaw_left     # ROLL + PITCH + YAW
    
    # Genel güvenlik sınırını uygula
    final_up_cmd    = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_up_cmd))
    final_down_cmd  = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_down_cmd))
    final_right_cmd = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_right_cmd))
    final_left_cmd  = max(-OVERALL_MAX_DELTA_US, min(OVERALL_MAX_DELTA_US, final_left_cmd))
    
    return final_up_cmd, final_down_cmd, final_right_cmd, final_left_cmd

def main():
    """Ana stabilizasyon döngüsü"""
    print("Full Stabilizasyon Sistemi başlatılıyor...")
    
    # MAVLink bağlantısı
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    if not m.wait_heartbeat(timeout=10):
        print("HATA: Heartbeat alınamadı, bağlantıyı kontrol edin.")
        return
    
    print("✓ MAVLink bağlantısı başarılı")
    print("✓ Roll, Pitch, Yaw stabilizasyonu aktif")
    print("✓ Ctrl+C ile güvenli çıkış")
    print("-" * 50)
    
    # Başlangıçta tüm servoları nötrle
    neutral_all(m)
    
    last_msg_t = time.perf_counter()
    yaw_offset = None  # Yaw referans noktası
    loop_count = 0
    
    try:
        while True:
            # ATTITUDE mesajını al
            msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            now = time.perf_counter()
            
            if msg is None:
                # Veri yoksa güvenlik için nötrle
                if now - last_msg_t > 1.5:
                    neutral_all(m)
                    print("UYARI: Veri kesildi, servolar nötrlendi")
                continue
            
            last_msg_t = now
            
            # Attitude verilerini al
            roll = float(msg.roll)
            pitch = float(msg.pitch)
            raw_yaw = float(msg.yaw)
            
            # Yaw offset ayarla (ilk okumada)
            if yaw_offset is None:
                yaw_offset = raw_yaw
                print(f"Yaw referans noktası: {math.degrees(yaw_offset):.1f}°")
                continue
            
            # Relatif yaw hesapla
            yaw = raw_yaw - yaw_offset
            while yaw > math.pi:
                yaw -= 2 * math.pi
            while yaw < -math.pi:
                yaw += 2 * math.pi
            
            # Her eksen için komutları hesapla
            roll_left_cmd, roll_right_cmd = calculate_roll_commands(roll)
            pitch_right_cmd, pitch_left_cmd = calculate_pitch_commands(pitch)
            yaw_up_cmd, yaw_down_cmd, yaw_right_cmd, yaw_left_cmd = calculate_yaw_commands(yaw)
            
            # Komutları birleştir
            final_up, final_down, final_right, final_left = combine_commands(
                roll_left_cmd, roll_right_cmd,
                pitch_right_cmd, pitch_left_cmd,
                yaw_up_cmd, yaw_down_cmd, yaw_right_cmd, yaw_left_cmd
            )
            
            # Servo komutlarını gönder
            set_servo(m, SERVO_UP,    to_pwm(final_up))     # 14
            set_servo(m, SERVO_DOWN,  to_pwm(final_down))   # 11
            set_servo(m, SERVO_RIGHT, to_pwm(final_right))  # 12
            set_servo(m, SERVO_LEFT,  to_pwm(final_left))   # 13
            
            # Periyodik durum raporu (her 2 saniyede bir)
            loop_count += 1
            if loop_count % 100 == 0:  # ~50Hz'de 100 döngü = 2 saniye
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)
                
                print(f"Açılar: R={roll_deg:+6.1f}° P={pitch_deg:+6.1f}° Y={yaw_deg:+6.1f}° | "
                      f"PWM: ÜST={to_pwm(final_up)} ALT={to_pwm(final_down)} "
                      f"SAĞ={to_pwm(final_right)} SOL={to_pwm(final_left)}")
            
            # 50 Hz döngü hızı
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\n" + "="*50)
        print("Kullanıcı tarafından durduruldu")
        
    except Exception as e:
        print(f"\nHATA: {e}")
        
    finally:
        print("Servolar nötrleniyor...")
        try:
            neutral_all(m)
            time.sleep(0.5)  # Nötrlenme için bekle
            print("✓ Güvenli çıkış tamamlandı")
        except Exception as e:
            print(f"Nötrleme hatası: {e}")

if __name__ == "__main__":
    main()
