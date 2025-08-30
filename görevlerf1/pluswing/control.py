#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CONTROL - Araç Stabilizasyonu ve Kontrol Sistemi
PID kontrol, servo komutlama ve stabilizasyon fonksiyonları
"""

import time
import math
from pymavlink import mavutil
from config import *
from utils import Logger

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
        
        # 90 SANİYE PWM GÜVENLİK KONTROLLERİ
        self.pwm_allowed = False  # Başlangıçta PWM yollamak yasak
        self.pwm_allowed_start_time = None
        
    def enable_pwm_signals(self):
        """90 saniye sonra PWM sinyallerini etkinleştir"""
        self.pwm_allowed = True
        self.pwm_allowed_start_time = time.time()
        self.logger.info("🚀 PWM sinyalleri etkinleştirildi!")
    
    def disable_pwm_signals(self):
        """PWM sinyallerini devre dışı bırak"""
        self.pwm_allowed = False
        self.pwm_allowed_start_time = None
        self.logger.info("🚫 PWM sinyalleri devre dışı bırakıldı!")
    
    def set_servo(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        try:
            # 90 SANİYE PWM GÜVENLİK KONTROLÜ
            if not self.pwm_allowed:
                self.logger.warning(f"🚫 PWM sinyali engellendi! 90 saniye tamamlanmadan PWM yollanamaz (Kanal:{channel})")
                return False
            
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
    """Ana stabilizasyon kontrol sistemi"""
    
    def __init__(self, mavlink_connection, sensor_manager, logger=None):
        self.mavlink = mavlink_connection
        self.sensors = sensor_manager
        self.logger = logger or Logger()
        self.servo_controller = ServoController(mavlink_connection, logger)
        
        # Derinlik kontrolü için PID
        self.depth_pid = PIDController(
            kp=DEPTH_KP, 
            ki=DEPTH_KI, 
            kd=DEPTH_KD,
            output_min=-math.radians(DEPTH_MAX_PITCH),
            output_max=math.radians(DEPTH_MAX_PITCH)
        )
        
        # Hedef değerler
        self.target_depth = 0.0
        self.target_yaw = 0.0
        
        # Stabilizasyon aktif durumu
        self.stabilization_active = False
        
        # ÖZEL U DÖNÜŞ MODU
        self.u_turn_mode = False
        self.u_turn_target_yaw = 0.0
        self.u_turn_start_yaw = 0.0
        self.u_turn_progress = 0.0  # 0.0 - 1.0 arası ilerleme
        
        self.logger.info("Stabilizasyon kontrolcüsü başlatıldı")
        
    def set_target_depth(self, depth):
        """Hedef derinlik ayarla"""
        self.target_depth = depth
        self.logger.info(f"Hedef derinlik: {depth:.2f}m")
        
    def set_target_yaw(self, yaw_radians):
        """Hedef yaw açısı ayarla"""
        self.target_yaw = yaw_radians
        self.logger.info(f"Hedef yaw: {math.degrees(yaw_radians):.1f}°")
        
    def enable_stabilization(self):
        """Stabilizasyonu aktif et"""
        self.stabilization_active = True
        self.depth_pid.reset()
        self.logger.info("Stabilizasyon aktif edildi")
        
    def disable_stabilization(self):
        """Stabilizasyonu deaktif et"""
        self.stabilization_active = False
        self.servo_controller.neutral_all_servos()
        self.logger.info("Stabilizasyon deaktif edildi")
        
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
        
    def calculate_pitch_commands(self, pitch, depth_correction=0.0):
        """Pitch ekseninde stabilizasyon komutları (derinlik düzeltmesi dahil)"""
        if pitch is None:
            return 0.0, 0.0
            
        depth_correction = depth_correction if depth_correction is not None else 0.0
        total_pitch = pitch + depth_correction
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
        # Relatif yaw kullan
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
        
    def calculate_depth_correction(self, current_depth):
        """Derinlik kontrolü için pitch düzeltmesi"""
        if current_depth is None:
            return 0.0
            
        if abs(current_depth - self.target_depth) < DEPTH_DEADBAND:
            return 0.0
            
        # PID ile pitch düzeltmesi hesapla
        pitch_correction = self.depth_pid.update(self.target_depth, current_depth)
        
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
        """Ana stabilizasyon güncellemesi"""
        if not self.stabilization_active:
            return False
            
        # Sensör verilerini al
        sensor_data = self.sensors.get_all_sensor_data()
        
        attitude = sensor_data['attitude']
        depth_data = sensor_data['depth']
        
        if not attitude:
            self.logger.warning("Attitude verisi alınamadı")
            return False
            
        # Mevcut değerler
        roll = attitude['roll']
        pitch = attitude['pitch']
        yaw_relative = attitude['yaw_relative']
        current_depth = depth_data['depth_m'] if depth_data['is_valid'] else None
        
        # Derinlik düzeltmesi hesapla
        depth_correction = self.calculate_depth_correction(current_depth)
        
        # U DÖNÜŞ MODU KONTROLÜ
        if self.u_turn_mode:
            # U dönüş modunda sadece derinlik stabilizasyonu aktif
            # Roll/Pitch/Yaw stabilizasyonu deaktif
            current_yaw_deg = attitude.get('yaw_relative_deg', 0.0)
            u_up, u_down, u_right, u_left = self.calculate_u_turn_commands(current_yaw_deg)
            
            # Sadece derinlik kontrolü için pitch düzeltmesi ekle
            pitch_up_depth, pitch_down_depth = self.calculate_pitch_commands(0, depth_correction)
            
            # U dönüş komutları + sadece derinlik pitch'i
            final_up = u_up + pitch_up_depth
            final_down = u_down + pitch_down_depth
            final_right = u_right
            final_left = u_left
            
            self.logger.debug(f"U-DÖNÜŞ KOMUTLARI: UP={final_up:.1f}, DOWN={final_down:.1f}, "
                            f"RIGHT={final_right:.1f}, LEFT={final_left:.1f}")
        else:
            # Normal stabilizasyon modu
            # Her eksen için komutları hesapla
            roll_left, roll_right = self.calculate_roll_commands(roll)
            pitch_up, pitch_down = self.calculate_pitch_commands(pitch, depth_correction)
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
        
        # Debug bilgisi (gerektiğinde)
        if self.logger and hasattr(self.logger, 'debug'):
            roll_deg = math.degrees(roll) if roll is not None else 0.0
            pitch_deg = math.degrees(pitch) if pitch is not None else 0.0
            yaw_deg = math.degrees(yaw_relative) if yaw_relative is not None else 0.0
            depth_str = f"{current_depth:.2f}" if current_depth is not None else "N/A"
            
            self.logger.debug(f"Stabilizasyon - Roll: {roll_deg:.1f}°, "
                            f"Pitch: {pitch_deg:.1f}°, "
                            f"Yaw: {yaw_deg:.1f}° "
                            f"Derinlik: {depth_str}m/{self.target_depth:.2f}m")
        
        return success
        
    def turn_to_heading(self, target_heading_deg, timeout=30, tolerance=10):
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
        turn_strength = (1.0 - self.u_turn_progress) * 300.0  # Max 300 microsecond
        
        # SERVO KANAT HAREKETLERİ - SOLA DÖNÜŞ İÇİN
        # Sol kanatlar yukarı, sağ kanatlar aşağı (sola yatırma)
        # Üst ve alt kanatlar yaw için farklı yönde
        
        if turn_direction > 0:  # Sola dönüş
            up_cmd = -turn_strength      # Üst kanat sola yardım
            down_cmd = +turn_strength    # Alt kanat sola yardım  
            right_cmd = -turn_strength   # Sağ kanat aşağı (sola yatırma)
            left_cmd = +turn_strength    # Sol kanat yukarı (sola yatırma)
        else:  # Sağa dönüş
            up_cmd = +turn_strength      # Üst kanat sağa yardım
            down_cmd = -turn_strength    # Alt kanat sağa yardım
            right_cmd = +turn_strength   # Sağ kanat yukarı (sağa yatırma)
            left_cmd = -turn_strength    # Sol kanat aşağı (sağa yatırma)
        
        # İlerlemeye göre komut gücünü ayarla
        progress_factor = max(0.3, 1.0 - self.u_turn_progress)  # Min %30 güç
        up_cmd *= progress_factor
        down_cmd *= progress_factor
        right_cmd *= progress_factor
        left_cmd *= progress_factor
        
        self.logger.debug(f"U-Dönüş: İlerleme={self.u_turn_progress:.2f}, "
                         f"Yön={turn_direction:.1f}, Güç={turn_strength:.1f}")
        
        return up_cmd, down_cmd, right_cmd, left_cmd
        
    def turn_180_degrees(self, timeout=30):
        """180 derece dönüş yap"""
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
    
    def turn_to_heading_u_turn_mode(self, target_heading_deg, timeout=30):
        """Özel U dönüş modu ile heading'e dön - TAMAMEN AYRI STABİLİZASYON"""
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
        
        tolerance = 15.0  # U dönüş için daha geniş tolerans
        
        while time.time() - start_time < timeout:
            # ÖZEL U DÖNÜŞ KONTROLÜ (normal stabilizasyon değil!)
            if not self._update_u_turn_only():
                continue
                
            # Mevcut yaw'ı kontrol et
            sensor_data = self.sensors.get_all_sensor_data()
            attitude = sensor_data['attitude']
            
            if attitude and attitude['yaw_relative_deg'] is not None:
                current_yaw_deg = attitude['yaw_relative_deg']
                
                # Yaw farkını hesapla (wrap around dikkate al)
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
        
        # Son kontrol
        final_sensor_data = self.sensors.get_all_sensor_data()
        final_attitude = final_sensor_data['attitude']
        if final_attitude and final_attitude['yaw_relative_deg'] is not None:
            final_yaw = final_attitude['yaw_relative_deg']
            final_error = abs(target_heading_deg - final_yaw)
            if final_error > 180:
                final_error = 360 - final_error
            
            if final_error < 20.0:  # 20° tolerans ile başarılı
                self.logger.info(f"🎯 U DÖNÜŞ BAŞARILI: Final yaw={final_yaw:.1f}°, hata={final_error:.1f}°")
                return True
            else:
                self.logger.warning(f"⚠️ U DÖNÜŞ KISMEN BAŞARILI: Final yaw={final_yaw:.1f}°, hata={final_error:.1f}°")
                return True  # Kısmen başarılı da kabul et
        
        return True
    
    def _update_u_turn_only(self):
        """Sadece U dönüş kontrolü - normal stabilizasyon YOK"""
        # Sensör verilerini al
        sensor_data = self.sensors.get_all_sensor_data()
        
        attitude = sensor_data['attitude']
        depth_data = sensor_data['depth']
        
        if not attitude:
            self.logger.warning("Attitude verisi alınamadı")
            return False
        
        current_depth = depth_data['depth_m'] if depth_data['is_valid'] else None
        current_yaw_deg = attitude.get('yaw_relative_deg', 0.0)
        
        # Sadece U dönüş komutları hesapla
        u_up, u_down, u_right, u_left = self.calculate_u_turn_commands(current_yaw_deg)
        
        # Sadece derinlik kontrolü (pitch düzeltmesi)
        depth_correction = self.calculate_depth_correction(current_depth)
        pitch_up_depth, pitch_down_depth = self.calculate_pitch_commands(0, depth_correction)
        
        # Final komutlar: Sadece U dönüş + derinlik
        final_up = u_up + pitch_up_depth
        final_down = u_down + pitch_down_depth
        final_right = u_right
        final_left = u_left
        
        # Servo komutlarını gönder
        success = True
        success &= self.servo_controller.set_servo(SERVO_UP, to_pwm(final_up))
        success &= self.servo_controller.set_servo(SERVO_DOWN, to_pwm(final_down))
        success &= self.servo_controller.set_servo(SERVO_RIGHT, to_pwm(final_right))
        success &= self.servo_controller.set_servo(SERVO_LEFT, to_pwm(final_left))
        
        return success
        
    def surface_control(self, duration=10):
        """Yüzeye çıkış kontrolü"""
        self.logger.info("Yüzeye çıkış kontrolü başlatılıyor...")
        
        # Motoru durdur
        self.servo_controller.stop_motor()
        
        # Servoları yüzeye çıkış pozisyonuna getir
        self.servo_controller.set_servo(SERVO_UP, PWM_MAX)
        self.servo_controller.set_servo(SERVO_DOWN, PWM_MIN)
        self.servo_controller.set_servo(SERVO_RIGHT, PWM_NEUTRAL)
        self.servo_controller.set_servo(SERVO_LEFT, PWM_NEUTRAL)
        
        start_time = time.time()
        
        while time.time() - start_time < duration:
            # Derinlik kontrolü
            sensor_data = self.sensors.get_all_sensor_data()
            depth_data = sensor_data['depth']
            
            if depth_data['is_valid'] and depth_data['depth_m'] < SURFACE_DEPTH_THRESHOLD:
                self.logger.info(f"Yüzeye ulaşıldı: {depth_data['depth_m']:.2f}m")
                break
                
            time.sleep(0.5)
            
        # Tüm servoları nötrle
        self.servo_controller.neutral_all_servos()
        self.logger.info("Yüzeye çıkış tamamlandı")
        
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
    """Hareket kontrol sınıfı"""
    
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
        
    def controlled_dive(self, target_depth, max_duration=60):
        """Kontrollü dalış"""
        self.logger.info(f"Kontrollü dalış başlatıldı: {target_depth}m")
        
        # Hedef derinliği ayarla
        self.stabilizer.set_target_depth(target_depth)
        
        # Stabilizasyonu aktif et
        self.stabilizer.enable_stabilization()
        
        start_time = time.time()
        
        while time.time() - start_time < max_duration:
            # Stabilizasyonu güncelle
            if not self.stabilizer.update_stabilization():
                continue
                
            # Hedefe ulaştık mı kontrol et
            sensor_data = self.stabilizer.sensors.get_all_sensor_data()
            depth_data = sensor_data['depth']
            
            if depth_data['is_valid']:
                current_depth = depth_data['depth_m']
                depth_error = abs(target_depth - current_depth)
                
                if depth_error < DEPTH_DEADBAND:
                    self.logger.info(f"Hedef derinliğe ulaşıldı: {current_depth:.2f}m")
                    break
                    
            time.sleep(0.02)
            
        return True
