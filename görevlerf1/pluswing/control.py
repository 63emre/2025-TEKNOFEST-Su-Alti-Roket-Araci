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
    """Gelişmiş PID Kontrolcü Sınıfı - Deniz Koşulları İçin Optimize Edilmiş
    
    Özellikler:
    - 5Hz Low-pass sinyal filtresi
    - Integral windup koruması
    - Fail-safe mekanizması
    - Deniz koşullarında kararlı çalışma
    """
    
    def __init__(self, kp, ki, kd, output_min=-1000, output_max=1000, 
                 integral_clamp=None, filter_cutoff=None, logger=None):
        # Temel PID parametreleri
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        # Gelişmiş parametreler (config'den alınır)
        self.integral_clamp = integral_clamp or DEPTH_INTEGRAL_CLAMP
        self.filter_cutoff = filter_cutoff or DEPTH_FILTER_CUTOFF
        self.failsafe_timeout = DEPTH_FAILSAFE_TIMEOUT
        self.max_output_threshold = DEPTH_MAX_OUTPUT_THRESHOLD
        
        # PID durumu
        self.previous_error = 0
        self.integral = 0
        self.last_time = None
        
        # Sinyal filtresi durumu
        self.filtered_error = 0.0
        self.filtered_derivative = 0.0
        
        # Fail-safe durumu
        self.max_output_start_time = None
        self.consecutive_max_outputs = 0
        self.logger = logger
        
        if self.logger:
            self.logger.info(f"Gelişmiş PID başlatıldı - Kp:{kp}, Ki:{ki}, Kd:{kd}")
            self.logger.info(f"Filtre: {self.filter_cutoff}Hz, Clamp: ±{self.integral_clamp}")
        
    def reset(self):
        """PID değerlerini sıfırla"""
        self.previous_error = 0
        self.integral = 0
        self.last_time = None
        self.filtered_error = 0.0
        self.filtered_derivative = 0.0
        self.max_output_start_time = None
        self.consecutive_max_outputs = 0
        
        if self.logger:
            self.logger.info("PID reset edildi")
        
    def _low_pass_filter(self, new_value, old_filtered_value, dt):
        """5Hz Low-pass sinyal filtresi
        
        Args:
            new_value: Yeni ham değer
            old_filtered_value: Önceki filtrelenmiş değer
            dt: Zaman adımı
            
        Returns:
            Filtrelenmiş değer
        """
        if dt <= 0:
            return old_filtered_value
            
        # RC = 1 / (2 * π * fc) formülü ile kesim frekansı
        rc = 1.0 / (2.0 * math.pi * self.filter_cutoff)
        alpha = dt / (rc + dt)
        
        return alpha * new_value + (1.0 - alpha) * old_filtered_value
        
    def _apply_integral_clamp(self):
        """Integral windup koruması uygula"""
        if abs(self.integral) > self.integral_clamp:
            # Integral sınırını aş
            clamped_integral = math.copysign(self.integral_clamp, self.integral)
            
            if self.logger and abs(self.integral - clamped_integral) > 0.01:
                self.logger.debug(f"Integral clamp: {self.integral:.3f} → {clamped_integral:.3f}")
                
            self.integral = clamped_integral
            
    def _check_failsafe(self, output):
        """Fail-safe mekanizması kontrolü
        
        Args:
            output: PID çıkışı
            
        Returns:
            Güvenli çıkış değeri
        """
        output_ratio = abs(output) / max(abs(self.output_max), abs(self.output_min))
        
        # Max output eşiğini aştık mı?
        if output_ratio > self.max_output_threshold:
            current_time = time.time()
            
            if self.max_output_start_time is None:
                self.max_output_start_time = current_time
                self.consecutive_max_outputs = 1
            else:
                self.consecutive_max_outputs += 1
                duration = current_time - self.max_output_start_time
                
                # Fail-safe süresi aşıldı mı?
                if duration > self.failsafe_timeout:
                    if self.logger:
                        self.logger.error(f"🚨 PID FAIL-SAFE TETİKLENDİ! {duration:.1f}s boyunca max output")
                        self.logger.error(f"Output: {output:.1f}, Eşik: {self.max_output_threshold*100:.0f}%")
                        self.logger.error("Sistem resetleniyor...")
                    
                    # PID'i resetle ve güvenli çıkış ver
                    self.reset()
                    return 0.0
                    
        else:
            # Normal output - fail-safe sayacını resetle
            self.max_output_start_time = None
            self.consecutive_max_outputs = 0
            
        return output
        
    def update(self, setpoint, measured_value):
        """Gelişmiş PID hesaplama
        
        Args:
            setpoint: Hedef değer
            measured_value: Ölçülen değer
            
        Returns:
            PID çıkışı (filtrelenmiş ve korumalı)
        """
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            return 0
            
        dt = current_time - self.last_time
        if dt <= 0:
            return 0
            
        # 1. Ham hata hesapla
        raw_error = setpoint - measured_value
        
        # 2. Hata sinyalini filtrele (5Hz low-pass)
        self.filtered_error = self._low_pass_filter(raw_error, self.filtered_error, dt)
        
        # 3. Integral hesapla ve clamp uygula
        self.integral += self.filtered_error * dt
        self._apply_integral_clamp()
        
        # 4. Derivative hesapla ve filtrele
        if dt > 0:
            raw_derivative = (self.filtered_error - self.previous_error) / dt
            self.filtered_derivative = self._low_pass_filter(raw_derivative, self.filtered_derivative, dt)
        else:
            self.filtered_derivative = 0
            
        # 5. PID çıkışını hesapla
        output = (self.kp * self.filtered_error + 
                 self.ki * self.integral + 
                 self.kd * self.filtered_derivative)
        
        # 6. Çıkışı sınırla
        output = max(self.output_min, min(self.output_max, output))
        
        # 7. Fail-safe kontrolü uygula
        output = self._check_failsafe(output)
        
        # 8. Bir sonraki iterasyon için değerleri sakla
        self.previous_error = self.filtered_error
        self.last_time = current_time
        
        return output
        
    def get_debug_info(self):
        """Debug bilgilerini döndür"""
        return {
            'filtered_error': self.filtered_error,
            'integral': self.integral,
            'filtered_derivative': self.filtered_derivative,
            'integral_clamped': abs(self.integral) >= self.integral_clamp * 0.95,
            'consecutive_max_outputs': self.consecutive_max_outputs,
            'failsafe_active': self.max_output_start_time is not None
        }

class ServoController:
    """Servo kontrol sınıfı"""
    
    def __init__(self, mavlink_connection, logger=None):
        self.mavlink = mavlink_connection
        self.logger = logger or Logger()
        
    def set_servo(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        try:
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
        
        # Derinlik kontrolü için gelişmiş PID
        self.depth_pid = PIDController(
            kp=DEPTH_KP, 
            ki=DEPTH_KI, 
            kd=DEPTH_KD,
            output_min=-math.radians(DEPTH_MAX_PITCH),
            output_max=math.radians(DEPTH_MAX_PITCH),
            logger=logger
        )
        
        # Hedef değerler
        self.target_depth = 0.0
        self.target_yaw = 0.0
        
        # Stabilizasyon aktif durumu
        self.stabilization_active = False
        
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
            # D300 yok - manuel derinlik kontrolü
            # Hedef derinliğe göre sabit pitch ver
            if self.target_depth > 1.0:  # 1m'den derin hedefse
                return math.radians(-5.0)  # 5° burun aşağı (dalış)
            else:
                return 0.0  # Yüzeye yakınsa nötr
            
        if abs(current_depth - self.target_depth) < DEPTH_DEADBAND:
            return 0.0
            
        # PID ile pitch düzeltmesi hesapla
        pitch_correction = self.depth_pid.update(self.target_depth, current_depth)
        
        return pitch_correction
        
    def combine_commands(self, roll_left, roll_right, pitch_up, pitch_down,
                        yaw_up, yaw_down, yaw_right, yaw_left):
        """Tüm eksen komutlarını birleştir"""
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
        
        # Debug bilgisi (gelişmiş PID dahil)
        if self.logger and hasattr(self.logger, 'debug'):
            pid_debug = self.depth_pid.get_debug_info()
            
            self.logger.debug(f"Stabilizasyon - Roll: {math.degrees(roll):.1f}°, "
                            f"Pitch: {math.degrees(pitch):.1f}°, "
                            f"Yaw: {math.degrees(yaw_relative):.1f}° "
                            f"Derinlik: {current_depth:.2f}m/{self.target_depth:.2f}m")
                            
            # PID debug bilgileri
            if pid_debug['integral_clamped']:
                self.logger.debug(f"🔒 Integral clamp aktif: {pid_debug['integral']:.3f}")
            if pid_debug['failsafe_active']:
                self.logger.debug(f"⚠️ PID fail-safe aktif: {pid_debug['consecutive_max_outputs']} max output")
        
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
            
        return self.turn_to_heading(target_yaw_deg, timeout)
        
    def emergency_180_turn(self, timeout=30):
        """Acil durum 180 derece dönüş (turn_180_degrees ile aynı)"""
        return self.turn_180_degrees(timeout)
        
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
