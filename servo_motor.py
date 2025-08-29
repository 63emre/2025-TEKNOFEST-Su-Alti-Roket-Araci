#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SERVO MOTOR CONTROL - Hem Servo Hem Motor Kontrolü
- Roll, Pitch ve Yaw eksenlerini aynı anda stabilize eder
- Motor hız kontrolü ile birlikte çalışır
- 4 kanat (UP, DOWN, LEFT, RIGHT) koordineli kontrolü
- Manuel testlerden alınan parametrelerle optimize edilmiştir
"""

import time
from pymavlink import mavutil
import math

# ---- Bağlantı ----
PORT = "COM5"  # Windows için COM port
BAUD = 115200

# ---- Servo kanalları ----
SERVO_RIGHT = 11   # AUX3 → Fin Servo 1 (Plus: Sağ Kanat)  
SERVO_DOWN  = 12   # AUX4 → Fin Servo 2 (Plus: Alt Kanat)
SERVO_LEFT  = 13   # AUX5 → Fin Servo 3 (Plus: Sol Kanat)
SERVO_UP    = 14   # AUX6 → Fin Servo 4 (Plus: Üst Kanat)

# ---- Motor kanalı ----
MOTOR_MAIN = 8     # MAIN8 → Ana Motor

# ---- PWM ----
PWM_MIN = 1100
PWM_MAX = 1900
PWM_NEU = 1500

# Motor hız ayarları
MOTOR_STOP = PWM_NEU      # 1500 = dur
MOTOR_SLOW = 1600         # 1600 = yavaş
MOTOR_MEDIUM = 1700      # 1700 = orta
MOTOR_FAST = 1800         # 1800 = hızlı

def clamp(v, lo, hi): 
    return lo if v < lo else hi if v > hi else v

def to_pwm(delta_us):  
    return int(clamp(PWM_NEU + delta_us, PWM_MIN, PWM_MAX))

# ---- Stabilizasyon Ayarları ----
# Manuel testlerden alınan çalışan parametreler

# Roll stabilizasyonu
ROLL_SENSE = +1.0
ROLL_K_ANG_US_PER_RAD = 500.0
ROLL_DEADBAND_DEG = 1.0
ROLL_MAX_DELTA_US = 350.0

# Pitch stabilizasyonu
PITCH_SENSE = +1.0
PITCH_K_ANG_US_PER_RAD = 500.0
PITCH_DEADBAND_DEG = 1.0
PITCH_MAX_DELTA_US = 350.0

# Yaw stabilizasyonu
YAW_SENSE = +1.0
YAW_K_ANG_US_PER_RAD = 400.0
YAW_DEADBAND_DEG = 2.0
YAW_MAX_DELTA_US = 300.0

# ---- Mekanik Yönler ----
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

class ServoMotorController:
    """Servo ve Motor Kontrol Sınıfı"""
    
    def __init__(self, mavlink_connection):
        self.mavlink = mavlink_connection
        self.current_motor_speed = MOTOR_STOP
        self.stabilization_active = True
        self.yaw_offset = None
        
    def set_servo(self, channel, pwm):
        """Servo pozisyon komutu gönder"""
        try:
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system, 
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
                0, float(channel), float(pwm), 0, 0, 0, 0, 0
            )
            return True
        except Exception as e:
            print(f"Servo {channel} hatası: {e}")
            return False
            
    def set_motor(self, pwm):
        """Motor hız komutu gönder"""
        try:
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system, 
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
                0, float(MOTOR_MAIN), float(pwm), 0, 0, 0, 0, 0
            )
            self.current_motor_speed = pwm
            return True
        except Exception as e:
            print(f"Motor hatası: {e}")
            return False
            
    def neutral_all_servos(self):
        """Tüm servoları nötr pozisyona getir"""
        success = True
        for ch in (SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT):
            success &= self.set_servo(ch, PWM_NEU)
        return success
        
    def stop_motor(self):
        """Motoru durdur"""
        return self.set_motor(MOTOR_STOP)
        
    def set_motor_speed(self, speed_name):
        """Motor hızını ayarla"""
        speed_map = {
            'stop': MOTOR_STOP,
            'slow': MOTOR_SLOW,
            'medium': MOTOR_MEDIUM,
            'fast': MOTOR_FAST
        }
        
        if speed_name in speed_map:
            return self.set_motor(speed_map[speed_name])
        else:
            print(f"Geçersiz hız: {speed_name}")
            return False
            
    def calculate_roll_commands(self, roll):
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

    def calculate_pitch_commands(self, pitch):
        """Pitch ekseni için servo komutlarını hesapla"""
        pitch_deg = math.degrees(pitch)
        
        if abs(pitch_deg) < PITCH_DEADBAND_DEG:
            return 0.0, 0.0  # right_cmd, left_cmd
        
        # +pitch (burun yukarı) -> RIGHT ↑, LEFT ↓
        u = PITCH_SENSE * pitch * PITCH_K_ANG_US_PER_RAD
        right_cmd_us = (+u) * PITCH_DIR_UP    # SERVO_RIGHT için
        left_cmd_us  = (-u) * PITCH_DIR_DOWN  # SERVO_LEFT için
        
        # Sınırla
        right_cmd_us = max(-PITCH_MAX_DELTA_US, min(PITCH_MAX_DELTA_US, right_cmd_us))
        left_cmd_us  = max(-PITCH_MAX_DELTA_US, min(PITCH_MAX_DELTA_US, left_cmd_us))
        
        return right_cmd_us, left_cmd_us

    def calculate_yaw_commands(self, yaw):
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

    def combine_commands(self, roll_left, roll_right, pitch_right, pitch_left, yaw_up, yaw_down, yaw_right, yaw_left):
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
        
    def update_stabilization(self, roll, pitch, yaw):
        """Stabilizasyon güncellemesi"""
        if not self.stabilization_active:
            return False
            
        # Her eksen için komutları hesapla
        roll_left_cmd, roll_right_cmd = self.calculate_roll_commands(roll)
        pitch_right_cmd, pitch_left_cmd = self.calculate_pitch_commands(pitch)
        yaw_up_cmd, yaw_down_cmd, yaw_right_cmd, yaw_left_cmd = self.calculate_yaw_commands(yaw)
        
        # Komutları birleştir
        final_up, final_down, final_right, final_left = self.combine_commands(
            roll_left_cmd, roll_right_cmd,
            pitch_right_cmd, pitch_left_cmd,
            yaw_up_cmd, yaw_down_cmd, yaw_right_cmd, yaw_left_cmd
        )
        
        # Servo komutlarını gönder
        success = True
        success &= self.set_servo(SERVO_UP, to_pwm(final_up))     # 14
        success &= self.set_servo(SERVO_DOWN, to_pwm(final_down))   # 11
        success &= self.set_servo(SERVO_RIGHT, to_pwm(final_right))  # 12
        success &= self.set_servo(SERVO_LEFT, to_pwm(final_left))   # 13
        
        return success
        
    def set_yaw_reference(self, yaw_raw):
        """Yaw referans noktasını ayarla"""
        self.yaw_offset = yaw_raw
        print(f"Yaw referans noktası: {math.degrees(yaw_raw):.1f}°")
        
    def get_relative_yaw(self, yaw_raw):
        """Relatif yaw hesapla"""
        if self.yaw_offset is None:
            return 0.0
            
        yaw = yaw_raw - self.yaw_offset
        while yaw > math.pi:
            yaw -= 2 * math.pi
        while yaw < -math.pi:
            yaw += 2 * math.pi
        return yaw

def main():
    """Ana kontrol döngüsü"""
    print("Servo Motor Kontrol Sistemi başlatılıyor...")
    print("Hem stabilizasyon hem motor kontrolü aktif")
    print("-" * 50)
    
    # MAVLink bağlantısı
    try:
        m = mavutil.mavlink_connection(PORT, baud=BAUD)
        if not m.wait_heartbeat(timeout=10):
            print("HATA: Heartbeat alınamadı, bağlantıyı kontrol edin.")
            return
    except Exception as e:
        print(f"Bağlantı hatası: {e}")
        return
    
    print("✓ MAVLink bağlantısı başarılı")
    
    # Kontrolcüyü başlat
    controller = ServoMotorController(m)
    
    # Başlangıçta tüm servoları nötrle
    controller.neutral_all_servos()
    controller.stop_motor()
    
    print("✓ Servolar nötr, motor durduruldu")
    print("✓ Roll, Pitch, Yaw stabilizasyonu aktif")
    print("✓ Motor kontrolü aktif")
    print("✓ Ctrl+C ile güvenli çıkış")
    print("-" * 50)
    
    last_msg_t = time.perf_counter()
    loop_count = 0
    
    try:
        while True:
            # ATTITUDE mesajını al
            msg = m.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
            now = time.perf_counter()
            
            if msg is None:
                # Veri yoksa güvenlik için nötrle
                if now - last_msg_t > 1.5:
                    controller.neutral_all_servos()
                    print("UYARI: Veri kesildi, servolar nötrlendi")
                continue
            
            last_msg_t = now
            
            # Attitude verilerini al
            roll = float(msg.roll)
            pitch = float(msg.pitch)
            raw_yaw = float(msg.yaw)
            
            # Yaw offset ayarla (ilk okumada)
            if controller.yaw_offset is None:
                controller.set_yaw_reference(raw_yaw)
                continue
            
            # Relatif yaw hesapla
            yaw = controller.get_relative_yaw(raw_yaw)
            
            # Stabilizasyonu güncelle
            if not controller.update_stabilization(roll, pitch, yaw):
                print("Stabilizasyon hatası!")
                continue
            
            # Periyodik durum raporu (her 2 saniyede bir)
            loop_count += 1
            if loop_count % 100 == 0:  # ~50Hz'de 100 döngü = 2 saniye
                roll_deg = math.degrees(roll)
                pitch_deg = math.degrees(pitch)
                yaw_deg = math.degrees(yaw)
                motor_pwm = controller.current_motor_speed
                
                print(f"Açılar: R={roll_deg:+6.1f}° P={pitch_deg:+6.1f}° Y={yaw_deg:+6.1f}° | "
                      f"Motor: {motor_pwm} | "
                      f"Stabilizasyon: {'Aktif' if controller.stabilization_active else 'Pasif'}")
            
            # 50 Hz döngü hızı
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        print("\n" + "="*50)
        print("Kullanıcı tarafından durduruldu")
        
    except Exception as e:
        print(f"\nHATA: {e}")
        
    finally:
        print("Güvenli çıkış yapılıyor...")
        try:
            controller.neutral_all_servos()
            controller.stop_motor()
            time.sleep(0.5)  # Nötrlenme için bekle
            print("✓ Güvenli çıkış tamamlandı")
        except Exception as e:
            print(f"Çıkış hatası: {e}")

def test_motor_control():
    """Motor kontrol testi"""
    print("Motor kontrol testi başlatılıyor...")
    
    try:
        m = mavutil.mavlink_connection(PORT, baud=BAUD)
        if not m.wait_heartbeat(timeout=10):
            print("HATA: Heartbeat alınamadı")
            return
            
        controller = ServoMotorController(m)
        
        print("Motor testleri:")
        print("1. Motor durduruluyor...")
        controller.stop_motor()
        time.sleep(2)
        
        print("2. Yavaş hız...")
        controller.set_motor_speed('slow')
        time.sleep(3)
        
        print("3. Orta hız...")
        controller.set_motor_speed('medium')
        time.sleep(3)
        
        print("4. Hızlı...")
        controller.set_motor_speed('fast')
        time.sleep(3)
        
        print("5. Motor durduruluyor...")
        controller.stop_motor()
        
        print("✓ Motor testi tamamlandı")
        
    except Exception as e:
        print(f"Motor test hatası: {e}")

def test_servo_control():
    """Servo kontrol testi"""
    print("Servo kontrol testi başlatılıyor...")
    
    try:
        m = mavutil.mavlink_connection(PORT, baud=BAUD)
        if not m.wait_heartbeat(timeout=10):
            print("HATA: Heartbeat alınamadı")
            return
            
        controller = ServoMotorController(m)
        
        print("Servo testleri:")
        print("1. Tüm servolar nötr...")
        controller.neutral_all_servos()
        time.sleep(2)
        
        print("2. UP servo test...")
        controller.set_servo(SERVO_UP, PWM_MAX)
        time.sleep(1)
        controller.set_servo(SERVO_UP, PWM_NEU)
        time.sleep(1)
        
        print("3. DOWN servo test...")
        controller.set_servo(SERVO_DOWN, PWM_MAX)
        time.sleep(1)
        controller.set_servo(SERVO_DOWN, PWM_NEU)
        time.sleep(1)
        
        print("4. RIGHT servo test...")
        controller.set_servo(SERVO_RIGHT, PWM_MAX)
        time.sleep(1)
        controller.set_servo(SERVO_RIGHT, PWM_NEU)
        time.sleep(1)
        
        print("5. LEFT servo test...")
        controller.set_servo(SERVO_LEFT, PWM_MAX)
        time.sleep(1)
        controller.set_servo(SERVO_LEFT, PWM_NEU)
        time.sleep(1)
        
        print("6. Tüm servolar nötr...")
        controller.neutral_all_servos()
        
        print("✓ Servo testi tamamlandı")
        
    except Exception as e:
        print(f"Servo test hatası: {e}")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        if sys.argv[1] == "motor":
            test_motor_control()
        elif sys.argv[1] == "servo":
            test_servo_control()
        else:
            print("Kullanım: python servo_motor.py [motor|servo]")
            print("motor: Motor kontrol testi")
            print("servo: Servo kontrol testi")
            print("Argümansız: Tam stabilizasyon + motor kontrolü")
    else:
        main()
