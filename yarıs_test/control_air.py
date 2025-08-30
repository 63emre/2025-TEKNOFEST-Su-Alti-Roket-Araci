#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CONTROL AIR - Hava Yarışı için Kontrol Sistemi
D300 derinlik kontrolü KALDIRILDI - Sadece attitude kontrolü
Pluswing/control.py'den uyarlanmıştır
"""

import time
import math
from pymavlink import mavutil
from config_air import *

class Logger:
    """Basit logger sınıfı"""
    def __init__(self):
        pass
    
    def info(self, msg):
        print(f"[INFO] {msg}")
    
    def warning(self, msg):
        print(f"[WARNING] {msg}")
    
    def error(self, msg):
        print(f"[ERROR] {msg}")
    
    def debug(self, msg):
        print(f"[DEBUG] {msg}")

class PIDController:
    """PID Kontrolcü Sınıfı"""
    
    def __init__(self, kp, ki, kd, output_min=-1000, output_max=1000):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        self.previous_error = 0
        self.integral = 0
        self.last_time = None
        
    def reset(self):
        """PID değerlerini sıfırla"""
        self.previous_error = 0
        self.integral = 0
        self.last_time = None
        
    def update(self, setpoint, measured_value):
        """PID hesaplama"""
        if setpoint is None or measured_value is None:
            return 0
            
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            return 0
            
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
            
        # Hata hesapla
        error = setpoint - measured_value
        
        # Integral hesapla
        self.integral += error * dt
        
        # Derivative hesapla
        derivative = (error - self.previous_error) / dt
        
        # PID çıkışını hesapla
        output = (self.kp * error + 
                 self.ki * self.integral + 
                 self.kd * derivative)
        
        # Çıkışı sınırla
        output = max(self.output_min, min(self.output_max, output))
        
        # Bir sonraki iterasyon için değerleri sakla
        self.previous_error = error
        self.last_time = current_time
        
        return output

class ServoController:
    """Servo kontrol sınıfı"""
    
    def __init__(self, mavlink_connection, logger=None):
        self.mavlink = mavlink_connection
        self.logger = logger or Logger()
        
    def set_servo(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        try:
            # Güvenlik kontrolleri
            if channel is None:
                self.logger.error("Servo channel None!")
                return False
            if pwm_value is None:
                self.logger.warning(f"Servo {channel} PWM değeri None, PWM_NEUTRAL kullanılıyor")
                pwm_value = PWM_NEUTRAL
                
            pwm_clamped = clamp(pwm_value, PWM_MIN, PWM_MAX)
            
            self.mavlink.mav.command_long_send(
                self.mavlink.target_system, 
                self.mavlink.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 
                0,
                float(channel), 
                float(pwm_clamped), 
                0, 0, 0, 0, 0
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"Servo {channel} ayarlama hatası: {e}")
            return False
            
    def set_motor(self, pwm_value):
        """Ana motor PWM değeri ayarla"""
        if pwm_value is None:
            self.logger.warning("Motor PWM değeri None, MOTOR_STOP kullanılıyor")
            pwm_value = MOTOR_STOP
        return self.set_servo(MOTOR_MAIN, pwm_value)
        
    def neutral_all_servos(self):
        """Tüm servoları nötr pozisyona getir"""
        success_count = 0
        
        for channel in [SERVO_UP, SERVO_DOWN, SERVO_RIGHT, SERVO_LEFT]:
            if self.set_servo(channel, PWM_NEUTRAL):
                success_count += 1
                
        self.logger.info(f"Servolar nötrlendi: {success_count}/4")
        return success_count == 4
        
    def stop_motor(self):
        """Motoru durdur"""
        return self.set_motor(MOTOR_STOP)

class StabilizationController:
    """Ana stabilizasyon kontrol sistemi - HAVA YARIŞI İÇİN"""
    
    def __init__(self, mavlink_connection, sensor_manager, logger=None):
        self.mavlink = mavlink_connection
        self.sensors = sensor_manager
        self.logger = logger or Logger()
        self.servo_controller = ServoController(mavlink_connection, logger)
        
        # ALTITUDE kontrolü için PID (D300 yerine)
        self.altitude_pid = PIDController(
            kp=ALTITUDE_KP, 
            ki=ALTITUDE_KI, 
            kd=ALTITUDE_KD,
            output_min=-math.radians(ALTITUDE_MAX_PITCH),
            output_max=math.radians(ALTITUDE_MAX_PITCH)
        )
        
        # Hedef değerler
        self.target_altitude = 0.0  # Hava testinde 0 metre
        self.target_yaw = 0.0
        
        # Stabilizasyon aktif durumu
        self.stabilization_active = False
        
        # U DÖNÜŞ MODU
        self.u_turn_mode = False
        self.u_turn_target_yaw = 0.0
        self.u_turn_start_yaw = 0.0
        self.u_turn_progress = 0.0
        
        self.logger.info("Hava yarışı stabilizasyon kontrolcüsü başlatıldı")
        
    def set_target_altitude(self, altitude):
        """Hedef altitude ayarla"""
        self.target_altitude = altitude
        self.logger.info(f"Hedef altitude: {altitude:.2f}m")
        
    def set_target_yaw(self, yaw_radians):
        """Hedef yaw açısı ayarla"""
        self.target_yaw = yaw_radians
        self.logger.info(f"Hedef yaw: {math.degrees(yaw_radians):.1f}°")
        
    def enable_stabilization(self):
        """Stabilizasyonu aktif et"""
        self.stabilization_active = True
        self.altitude_pid.reset()
        self.logger.info("Hava yarışı stabilizasyonu aktif edildi")
        
    def disable_stabilization(self):
        """Stabilizasyonu deaktif et"""
        self.stabilization_active = False
        self.servo_controller.neutral_all_servos()
        self.logger.info("Hava yarışı stabilizasyonu deaktif edildi")
        
    def calculate_roll_commands(self, roll):
        """Roll ekseninde stabilizasyon komutları"""
        if roll is None:
            return 0.0, 0.0
            
        roll_deg = math.degrees(roll)
        
        if abs(roll_deg) < ROLL_DEADBAND_DEG:
            return 0.0, 0.0
            
        # Roll düzeltme sinyali
        u = ROLL_SENSE * roll * ROLL_K_ANG_US_PER_RAD
        
        left_cmd = (+u) * ROLL_DIR_LEFT
        right_cmd = (+u) * ROLL_DIR_RIGHT
        
        # Sınırla
        left_cmd = clamp(left_cmd, -ROLL_MAX_DELTA_US, ROLL_MAX_DELTA_US)
        right_cmd = clamp(right_cmd, -ROLL_MAX_DELTA_US, ROLL_MAX_DELTA_US)
        
        return left_cmd, right_cmd
        
    def calculate_pitch_commands(self, pitch, altitude_correction=0.0):
        """Pitch ekseninde stabilizasyon komutları (altitude düzeltmesi dahil)"""
        if pitch is None:
            return 0.0, 0.0
            
        altitude_correction = altitude_correction if altitude_correction is not None else 0.0
        total_pitch = pitch + altitude_correction
        pitch_deg = math.degrees(total_pitch)
        
        if abs(pitch_deg) < PITCH_DEADBAND_DEG:
            return 0.0, 0.0
            
        # Pitch düzeltme sinyali
        u = PITCH_SENSE * total_pitch * PITCH_K_ANG_US_PER_RAD
        
        up_cmd = (+u) * PITCH_DIR_UP
        down_cmd = (-u) * PITCH_DIR_DOWN
        
        # Sınırla
        up_cmd = clamp(up_cmd, -PITCH_MAX_DELTA_US, PITCH_MAX_DELTA_US)
        down_cmd = clamp(down_cmd, -PITCH_MAX_DELTA_US, PITCH_MAX_DELTA_US)
        
        return up_cmd, down_cmd
        
    def calculate_yaw_commands(self, yaw):
        """Yaw ekseninde stabilizasyon komutları"""
        if yaw is None:
            return 0.0, 0.0, 0.0, 0.0
            
        yaw_deg = math.degrees(yaw)
        
        if abs(yaw_deg) < YAW_DEADBAND_DEG:
            return 0.0, 0.0, 0.0, 0.0
            
        # Yaw düzeltme sinyali
        u = YAW_SENSE * yaw * YAW_K_ANG_US_PER_RAD
        
        up_cmd = (-u) * YAW_DIR_UP
        down_cmd = (+u) * YAW_DIR_DOWN
        right_cmd = (-u) * YAW_DIR_RIGHT
        left_cmd = (-u) * YAW_DIR_LEFT
        
        # Sınırla
        up_cmd = clamp(up_cmd, -YAW_MAX_DELTA_US, YAW_MAX_DELTA_US)
        down_cmd = clamp(down_cmd, -YAW_MAX_DELTA_US, YAW_MAX_DELTA_US)
        right_cmd = clamp(right_cmd, -YAW_MAX_DELTA_US, YAW_MAX_DELTA_US)
        left_cmd = clamp(left_cmd, -YAW_MAX_DELTA_US, YAW_MAX_DELTA_US)
        
        return up_cmd, down_cmd, right_cmd, left_cmd
        
    def calculate_altitude_correction(self, current_altitude):
        """Altitude kontrolü için pitch düzeltmesi"""
        if current_altitude is None:
            return 0.0
            
        if abs(current_altitude - self.target_altitude) < ALTITUDE_DEADBAND:
            return 0.0
            
        # PID ile pitch düzeltmesi hesapla
        pitch_correction = self.altitude_pid.update(self.target_altitude, current_altitude)
        
        return pitch_correction
        
    def combine_commands(self, roll_left, roll_right, pitch_up, pitch_down,
                        yaw_up, yaw_down, yaw_right, yaw_left):
        """Tüm eksen komutlarını birleştir"""
        # None değerleri 0.0 ile değiştir
        roll_left = roll_left if roll_left is not None else 0.0
        roll_right = roll_right if roll_right is not None else 0.0
        pitch_up = pitch_up if pitch_up is not None else 0.0
        pitch_down = pitch_down if pitch_down is not None else 0.0
        yaw_up = yaw_up if yaw_up is not None else 0.0
        yaw_down = yaw_down if yaw_down is not None else 0.0
        yaw_right = yaw_right if yaw_right is not None else 0.0
        yaw_left = yaw_left if yaw_left is not None else 0.0
        
        # Her kanat için komutları topla
        final_up = pitch_up + yaw_up
        final_down = pitch_down + yaw_down  
        final_right = roll_right + yaw_right
        final_left = roll_left + yaw_left
        
        # Genel güvenlik sınırlarını uygula
        final_up = clamp(final_up, -OVERALL_MAX_DELTA_US, OVERALL_MAX_DELTA_US)
        final_down = clamp(final_down, -OVERALL_MAX_DELTA_US, OVERALL_MAX_DELTA_US)
        final_right = clamp(final_right, -OVERALL_MAX_DELTA_US, OVERALL_MAX_DELTA_US)
        final_left = clamp(final_left, -OVERALL_MAX_DELTA_US, OVERALL_MAX_DELTA_US)
        
        return final_up, final_down, final_right, final_left
        
    def update_stabilization(self):
        """Ana stabilizasyon güncellemesi - HAVA YARIŞI"""
        if not self.stabilization_active:
            return False
            
        # Sensör verilerini al
        sensor_data = self.sensors.get_all_sensor_data()
        
        attitude = sensor_data['attitude']
        altitude_data = sensor_data.get('altitude', {})
        
        if not attitude:
            self.logger.warning("Attitude verisi alınamadı")
            return False
            
        # Mevcut değerler
        roll = attitude['roll']
        pitch = attitude['pitch']
        yaw_relative = attitude['yaw_relative']
        current_altitude = altitude_data.get('altitude_m') if altitude_data.get('is_valid') else None
        
        # Altitude düzeltmesi hesapla (D300 yerine)
        altitude_correction = self.calculate_altitude_correction(current_altitude)
        
        # U DÖNÜŞ MODU KONTROLÜ
        if self.u_turn_mode:
            # U dönüş modunda sadece altitude stabilizasyonu aktif
            current_yaw_deg = attitude.get('yaw_relative_deg', 0.0)
            u_up, u_down, u_right, u_left = self.calculate_u_turn_commands(current_yaw_deg)
            
            # Sadece altitude kontrolü için pitch düzeltmesi ekle
            pitch_up_alt, pitch_down_alt = self.calculate_pitch_commands(0, altitude_correction)
            
            # U dönüş komutları + sadece altitude pitch'i
            final_up = u_up + pitch_up_alt
            final_down = u_down + pitch_down_alt
            final_right = u_right
            final_left = u_left
            
            self.logger.debug(f"U-DÖNÜŞ KOMUTLARI: UP={final_up:.1f}, DOWN={final_down:.1f}, "
                            f"RIGHT={final_right:.1f}, LEFT={final_left:.1f}")
        else:
            # Normal stabilizasyon modu
            # Her eksen için komutları hesapla
            roll_left, roll_right = self.calculate_roll_commands(roll)
            pitch_up, pitch_down = self.calculate_pitch_commands(pitch, altitude_correction)
            yaw_up, yaw_down, yaw_right, yaw_left = self.calculate_yaw_commands(yaw_relative)
            
            # Komutları birleştir
            final_up, final_down, final_right, final_left = self.combine_commands(
                roll_left, roll_right, pitch_up, pitch_down,
                yaw_up, yaw_down, yaw_right, yaw_left
            )
        
        # Servo komutlarını gönder
        success = True
        success &= self.servo_controller.set_servo(SERVO_UP, to_pwm(final_up))
        success &= self.servo_controller.set_servo(SERVO_DOWN, to_pwm(final_down))
        success &= self.servo_controller.set_servo(SERVO_RIGHT, to_pwm(final_right))
        success &= self.servo_controller.set_servo(SERVO_LEFT, to_pwm(final_left))
        
        # Debug bilgisi
        if self.logger and hasattr(self.logger, 'debug'):
            roll_deg = math.degrees(roll) if roll is not None else 0.0
            pitch_deg = math.degrees(pitch) if pitch is not None else 0.0
            yaw_deg = math.degrees(yaw_relative) if yaw_relative is not None else 0.0
            altitude_str = f"{current_altitude:.2f}" if current_altitude is not None else "N/A"
            
            self.logger.debug(f"Hava Stabilizasyon - Roll: {roll_deg:.1f}°, "
                            f"Pitch: {pitch_deg:.1f}°, "
                            f"Yaw: {yaw_deg:.1f}° "
                            f"Altitude: {altitude_str}m/{self.target_altitude:.2f}m")
        
        return success
        
    def turn_to_heading(self, target_heading_deg, timeout=30, tolerance=15):
        """Belirtilen heading'e dön"""
        self.logger.info(f"Heading {target_heading_deg:.1f}°'ye dönülüyor...")
        
        start_time = time.time()
        target_yaw_rad = math.radians(target_heading_deg)
        
        # Hedef yaw'ı ayarla
        old_target = self.target_yaw
        self.target_yaw = target_yaw_rad
        
        # Motor yavaş çalıştır
        self.servo_controller.set_motor(SPEED_SLOW)
        
        while time.time() - start_time < timeout:
            # Stabilizasyonu güncelle
            if not self.update_stabilization():
                continue
                
            # Mevcut yaw'ı kontrol et
            sensor_data = self.sensors.get_all_sensor_data()
            attitude = sensor_data['attitude']
            
            if attitude and attitude['yaw_relative'] is not None:
                current_yaw_deg = attitude['yaw_relative_deg']
                yaw_error = abs(target_heading_deg - current_yaw_deg)
                
                # Hedefe ulaştık mı?
                if yaw_error < tolerance:
                    self.logger.info(f"Heading'e ulaşıldı: {current_yaw_deg:.1f}° (hata: {yaw_error:.1f}°)")
                    break
                    
            time.sleep(0.02)  # 50Hz
            
        # Hedefi eski haline getir
        self.target_yaw = old_target
        return True
    
    def start_u_turn_mode(self, current_yaw_deg):
        """Özel U dönüş modunu başlat"""
        self.u_turn_mode = True
        self.u_turn_start_yaw = current_yaw_deg
        self.u_turn_target_yaw = current_yaw_deg + 180
        
        # ±180° aralığında tut
        if self.u_turn_target_yaw > 180:
            self.u_turn_target_yaw -= 360
        elif self.u_turn_target_yaw < -180:
            self.u_turn_target_yaw += 360
            
        self.u_turn_progress = 0.0
        self.logger.info(f"🔄 U DÖNÜŞ MODU: {current_yaw_deg:.1f}° -> {self.u_turn_target_yaw:.1f}°")
    
    def stop_u_turn_mode(self):
        """Özel U dönüş modunu durdur"""
        self.u_turn_mode = False
        self.u_turn_progress = 0.0
        self.logger.info("✅ U dönüş modu tamamlandı - Normal stabilizasyon aktif")
    
    def calculate_u_turn_commands(self, current_yaw_deg):
        """Özel U dönüş servo komutlarını hesapla"""
        if not self.u_turn_mode:
            return 0.0, 0.0, 0.0, 0.0
        
        # İlerleme hesapla
        yaw_diff = abs(current_yaw_deg - self.u_turn_start_yaw)
        if yaw_diff > 180:  # Wrap around durumu
            yaw_diff = 360 - yaw_diff
        self.u_turn_progress = min(1.0, yaw_diff / 180.0)
        
        # Dönüş yönünü belirle (sola dönüş)
        turn_direction = 1.0 if self.u_turn_target_yaw > self.u_turn_start_yaw else -1.0
        if abs(self.u_turn_target_yaw - self.u_turn_start_yaw) > 180:
            turn_direction *= -1
        
        # U dönüş kuvveti (başlangıçta güçlü, sonra azalarak)
        turn_strength = (1.0 - self.u_turn_progress) * 250.0  # Hava testinde daha yumuşak
        
        if turn_direction > 0:  # Sola dönüş
            up_cmd = -turn_strength      
            down_cmd = +turn_strength    
            right_cmd = -turn_strength   
            left_cmd = +turn_strength    
        else:  # Sağa dönüş
            up_cmd = +turn_strength      
            down_cmd = -turn_strength    
            right_cmd = +turn_strength   
            left_cmd = -turn_strength    
        
        # İlerlemeye göre komut gücünü ayarla
        progress_factor = max(0.3, 1.0 - self.u_turn_progress)
        up_cmd *= progress_factor
        down_cmd *= progress_factor
        right_cmd *= progress_factor
        left_cmd *= progress_factor
        
        return up_cmd, down_cmd, right_cmd, left_cmd
        
    def turn_180_degrees(self, timeout=60):
        """180 derece dönüş yap - Hava testi için"""
        self.logger.info("180° dönüş başlatılıyor...")
        
        # Mevcut yaw'ı al
        sensor_data = self.sensors.get_all_sensor_data()
        attitude = sensor_data['attitude']
        
        if not attitude or attitude['yaw_relative'] is None:
            self.logger.error("Yaw verisi alınamadı, dönüş iptal edildi")
            return False
            
        current_yaw_deg = attitude['yaw_relative_deg']
        target_yaw_deg = current_yaw_deg + 180
        
        # ±180° aralığında tut
        if target_yaw_deg > 180:
            target_yaw_deg -= 360
        elif target_yaw_deg < -180:
            target_yaw_deg += 360
            
        return self.turn_to_heading_u_turn_mode(target_yaw_deg, timeout)
    
    def turn_to_heading_u_turn_mode(self, target_heading_deg, timeout=120):
        """Özel U dönüş modu ile heading'e dön"""
        self.logger.info(f"🔄 ÖZEL U DÖNÜŞ: {target_heading_deg:.1f}°'ye dönülüyor...")
        
        start_time = time.time()
        
        # Mevcut yaw'ı al
        sensor_data = self.sensors.get_all_sensor_data()
        attitude = sensor_data['attitude']
        
        if not attitude or attitude['yaw_relative_deg'] is None:
            self.logger.error("Yaw verisi alınamadı, U dönüş iptal edildi")
            return False
        
        current_yaw_deg = attitude['yaw_relative_deg']
        
        # NORMAL STABİLİZASYONU TAMAMEN KAPAT
        old_stabilization_state = self.stabilization_active
        self.disable_stabilization()
        self.logger.info("🚫 Normal stabilizasyon TAMAMEN kapatıldı - U dönüş modu aktif")
        
        # Özel U dönüş modunu başlat
        self.start_u_turn_mode(current_yaw_deg)
        
        # Motor yavaş çalıştır (U dönüş sırasında)
        self.servo_controller.set_motor(SPEED_SLOW)
        
        tolerance = 20.0  # Hava testinde geniş tolerans
        
        while time.time() - start_time < timeout:
            # ÖZEL U DÖNÜŞ KONTROLÜ
            if not self._update_u_turn_only():
                continue
                
            # Mevcut yaw'ı kontrol et
            sensor_data = self.sensors.get_all_sensor_data()
            attitude = sensor_data['attitude']
            
            if attitude and attitude['yaw_relative_deg'] is not None:
                current_yaw_deg = attitude['yaw_relative_deg']
                
                # Yaw farkını hesapla
                yaw_error = abs(target_heading_deg - current_yaw_deg)
                if yaw_error > 180:
                    yaw_error = 360 - yaw_error
                
                # İlerleme raporu
                if time.time() - start_time > 5.0 and (time.time() - start_time) % 10 < 0.1:
                    self.logger.info(f"U Dönüş İlerlemesi: {current_yaw_deg:.1f}° -> {target_heading_deg:.1f}° "
                                   f"(Hata: {yaw_error:.1f}°, İlerleme: %{self.u_turn_progress*100:.1f})")
                
                # Hedefe ulaştık mı?
                if yaw_error < tolerance:
                    self.logger.info(f"✅ U DÖNÜŞ TAMAMLANDI: {current_yaw_deg:.1f}° (hata: {yaw_error:.1f}°)")
                    break
                    
            time.sleep(0.02)  # 50Hz
        
        # U dönüş modunu durdur
        self.stop_u_turn_mode()
        
        # NORMAL STABİLİZASYONU YENİDEN AKTİF ET
        if old_stabilization_state:
            self.enable_stabilization()
            self.logger.info("✅ Normal stabilizasyon yeniden aktif edildi")
        
        return True
    
    def _update_u_turn_only(self):
        """Sadece U dönüş kontrolü - normal stabilizasyon YOK"""
        # Sensör verilerini al
        sensor_data = self.sensors.get_all_sensor_data()
        
        attitude = sensor_data['attitude']
        altitude_data = sensor_data.get('altitude', {})
        
        if not attitude:
            self.logger.warning("Attitude verisi alınamadı")
            return False
        
        current_altitude = altitude_data.get('altitude_m') if altitude_data.get('is_valid') else None
        current_yaw_deg = attitude.get('yaw_relative_deg', 0.0)
        
        # Sadece U dönüş komutları hesapla
        u_up, u_down, u_right, u_left = self.calculate_u_turn_commands(current_yaw_deg)
        
        # Sadece altitude kontrolü (pitch düzeltmesi)
        altitude_correction = self.calculate_altitude_correction(current_altitude)
        pitch_up_alt, pitch_down_alt = self.calculate_pitch_commands(0, altitude_correction)
        
        # Final komutlar: Sadece U dönüş + altitude
        final_up = u_up + pitch_up_alt
        final_down = u_down + pitch_down_alt
        final_right = u_right
        final_left = u_left
        
        # Servo komutlarını gönder
        success = True
        success &= self.servo_controller.set_servo(SERVO_UP, to_pwm(final_up))
        success &= self.servo_controller.set_servo(SERVO_DOWN, to_pwm(final_down))
        success &= self.servo_controller.set_servo(SERVO_RIGHT, to_pwm(final_right))
        success &= self.servo_controller.set_servo(SERVO_LEFT, to_pwm(final_left))
        
        return success
        
    def land_control(self, duration=15):
        """İniş kontrolü - Hava testi için"""
        self.logger.info("İniş kontrolü başlatılıyor...")
        
        # Motoru durdur
        self.servo_controller.stop_motor()
        
        # Servoları iniş pozisyonuna getir
        self.servo_controller.set_servo(SERVO_UP, PWM_MIN)    # Yukarı kanat aşağı
        self.servo_controller.set_servo(SERVO_DOWN, PWM_MAX)  # Aşağı kanat yukarı
        self.servo_controller.set_servo(SERVO_RIGHT, PWM_NEUTRAL)
        self.servo_controller.set_servo(SERVO_LEFT, PWM_NEUTRAL)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Altitude kontrolü
            sensor_data = self.sensors.get_all_sensor_data()
            altitude_data = sensor_data.get('altitude', {})
            
            if altitude_data.get('is_valid') and altitude_data.get('altitude_m', 0) < GROUND_LEVEL_THRESHOLD:
                self.logger.info(f"Yere iniş tamamlandı: {altitude_data['altitude_m']:.2f}m")
                break
                
            time.sleep(0.5)
            
        # Tüm servoları nötrle
        self.servo_controller.neutral_all_servos()
        self.logger.info("İniş kontrolü tamamlandı")
        
        return True
        
    def emergency_stop(self):
        """Acil durum prosedürü"""
        self.logger.error("ACİL DURUM - Tüm kontroller durduruluyor!")
        
        try:
            # Stabilizasyonu deaktif et
            self.disable_stabilization()
            
            # Motoru durdur
            self.servo_controller.stop_motor()
            
            # Tüm servoları nötrle
            self.servo_controller.neutral_all_servos()
            
            self.logger.info("Acil durum prosedürü tamamlandı")
            return True
            
        except Exception as e:
            self.logger.error(f"Acil durum prosedürü hatası: {e}")
            return False

class MotionController:
    """Hareket kontrol sınıfı - Hava yarışı için"""
    
    def __init__(self, stabilization_controller, logger=None):
        self.stabilizer = stabilization_controller
        self.servo_controller = stabilization_controller.servo_controller
        self.logger = logger or Logger()
        
    def forward(self, speed_pwm, duration=None):
        """İleri hareket"""
        self.logger.info(f"İleri hareket başlatıldı: PWM={speed_pwm}")
        self.servo_controller.set_motor(speed_pwm)
        
        if duration:
            time.sleep(duration)
            self.stop()
            
    def reverse(self, speed_pwm, duration=None):
        """Geri hareket"""
        self.logger.info(f"Geri hareket başlatıldı: PWM={speed_pwm}")
        self.servo_controller.set_motor(speed_pwm)
        
        if duration:
            time.sleep(duration)
            self.stop()
            
    def stop(self):
        """Hareketi durdur"""
        self.servo_controller.stop_motor()
        self.logger.info("Hareket durduruldu")
        
    def controlled_climb(self, target_altitude, max_duration=30):
        """Kontrollü tırmanış - Hava yarışı için"""
        self.logger.info(f"Kontrollü tırmanış başlatıldı: {target_altitude}m")
        
        # Hedef altitude'ı ayarla
        self.stabilizer.set_target_altitude(target_altitude)
        
        # Stabilizasyonu aktif et
        self.stabilizer.enable_stabilization()
        
        start_time = time.time()
        
        while time.time() - start_time < max_duration:
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                continue
                
            # Hedefe ulaştık mı kontrol et
            sensor_data = self.stabilizer.sensors.get_all_sensor_data()
            altitude_data = sensor_data.get('altitude', {})
            
            if altitude_data.get('is_valid'):
                current_altitude = altitude_data['altitude_m']
                altitude_error = abs(target_altitude - current_altitude)
                
                if altitude_error < ALTITUDE_DEADBAND:
                    self.logger.info(f"Hedef altitude'a ulaşıldı: {current_altitude:.2f}m")
                    break
                    
            time.sleep(0.02)
            
        return True
