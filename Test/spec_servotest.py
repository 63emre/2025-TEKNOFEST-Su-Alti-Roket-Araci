#!/usr/bin/env python3
"""
Teknofest Su Altı Roket Aracı - X Kanat Servo Kontrol Sistemi
IMU Filtreli Servo Kontrol Sistemi - Makine Mühendisliği Prensipleri

Bu sistem aşağıdaki mühendislik prensiplerini kullanır:
- Komplemanter Filtre (IMU veri füzyonu)
- PID Kontrol (Servo pozisyon kontrolü)
- Rate Limiting (Aşırı hareket önleme)
- Dead Zone (Mikro titreşim önleme)
- Anti-Windup (Integral toplama sınırı)
- Kalman Filtresi benzeri tahmin
"""

from pymavlink import mavutil
import atexit, signal, time, sys
import math
import numpy as np
from collections import deque
from threading import Thread, Lock
import logging

# Servo ve Sistem Parametreleri
SERVO_CH = 9        # AUX1 - X Kanat Servo Kanalı
NEUTRAL = 1500      # Nötr PWM değeri (µs)
MIN_PWM = 1100      # Minimum PWM değeri
MAX_PWM = 1900      # Maksimum PWM değeri
SERVO_RANGE = 90    # Servo hareket aralığı (derece)

# Filtre Parametreleri (Makine Mühendisliği Optimizasyonu)
class FilterConfig:
    # Komplemanter Filtre Katsayıları
    GYRO_WEIGHT = 0.98      # Gyro ağırlığı (yüksek frekans)
    ACCEL_WEIGHT = 0.02     # Accelerometer ağırlığı (düşük frekans)
    
    # Kalman Filtre Benzeri Parametreler
    PROCESS_NOISE = 0.01    # Sistem gürültüsü
    MEASUREMENT_NOISE = 0.1 # Ölçüm gürültüsü
    
    # Servo Dinamik Parametreleri (ULTRA Anti-Vibration)
    MAX_RATE = 30.0         # Maksimum açısal hız (derece/saniye) - DAHA DA AZALTILDI
    DEAD_ZONE = 5.0         # Ölü bölge (derece) - DAHA DA ARTTIRILDI
    
    # PID Kontrol Parametreleri (ULTRA Smooth)
    KP = 0.8                # Proportional gain - daha yumuşak
    KI = 0.02               # Integral gain - minimum
    KD = 0.12               # Derivative gain - maksimum damping
    
    # Anti-Windup
    MAX_INTEGRAL = 50.0     # Maksimum integral birikimi

class IMUFilter:
    """
    Gelişmiş IMU Filtresi - Makine Mühendisliği Prensipleri
    - Komplemanter filtre
    - Outlier tespiti ve reddi
    - Adaptive filtering
    - Vibration damping
    """
    
    def __init__(self):
        self.angle_filtered = 0.0
        self.gyro_bias = 0.0
        self.last_time = time.time()
        
        # Outlier Detection
        self.accel_buffer = deque(maxlen=10)
        self.gyro_buffer = deque(maxlen=10)
        
        # Vibration Filter (2nd Order Butterworth)
        self.vibration_filter_x = [0.0, 0.0]
        self.vibration_filter_y = [0.0, 0.0]
        
        # Adaptive parameters
        self.confidence = 1.0
        self.error_accumulator = 0.0
        
    def is_outlier(self, value, buffer, threshold=3.0):
        """Statistical outlier detection using Z-score"""
        if len(buffer) < 5:
            return False
        
        mean = np.mean(buffer)
        std = np.std(buffer)
        
        if std < 0.01:  # Avoid division by zero
            return False
            
        z_score = abs((value - mean) / std)
        return z_score > threshold
    
    def butterworth_filter(self, input_val, filter_state, cutoff=1.5, dt=0.02):
        """2nd Order Butterworth Low-Pass Filter for vibration damping"""
        # Normalized cutoff frequency
        wc = 2 * math.pi * cutoff * dt
        k1 = math.sqrt(2) * wc
        k2 = wc * wc
        
        # Filter coefficients
        a = k2 + k1 + 1
        b1 = 2 * (k2 - 1) / a
        b2 = (k2 - k1 + 1) / a
        
        # Apply filter
        output = (k2 * input_val + 2 * k2 * filter_state[0] + k2 * filter_state[1]) / a
        output -= b1 * filter_state[0] + b2 * filter_state[1]
        
        # Update state
        filter_state[1] = filter_state[0]
        filter_state[0] = output
        
        return output
    
    def update(self, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z):
        """
        Ana filtre fonksiyonu - Komplemanter + Outlier Rejection + Vibration Damping
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt > 0.1 or dt <= 0:  # Sanity check
            dt = 0.02
        
        # Roll açısını accelerometer'dan hesapla
        accel_roll = math.atan2(accel_y, math.sqrt(accel_x*accel_x + accel_z*accel_z))
        accel_roll_deg = math.degrees(accel_roll)
        
        # Outlier detection
        self.accel_buffer.append(accel_roll_deg)
        self.gyro_buffer.append(gyro_x)
        
        # Outlier rejection
        if self.is_outlier(accel_roll_deg, self.accel_buffer):
            accel_roll_deg = np.mean(list(self.accel_buffer)[-5:])
            self.confidence *= 0.95
        else:
            self.confidence = min(1.0, self.confidence * 1.01)
            
        if self.is_outlier(gyro_x, self.gyro_buffer):
            gyro_x = np.mean(list(self.gyro_buffer)[-5:])
        
        # Vibration filtering
        accel_roll_filtered = self.butterworth_filter(accel_roll_deg, self.vibration_filter_x)
        gyro_filtered = self.butterworth_filter(gyro_x, self.vibration_filter_y)
        
        # Gyro bias estimation (slow adaptation)
        if abs(gyro_filtered) < 0.5:  # Stationary condition
            self.gyro_bias = 0.99 * self.gyro_bias + 0.01 * gyro_filtered
        
        # Bias corrected gyro
        gyro_corrected = gyro_filtered - self.gyro_bias
        
        # Adaptive complementary filter weights
        accel_weight = FilterConfig.ACCEL_WEIGHT * self.confidence
        gyro_weight = 1.0 - accel_weight
        
        # Complementary filter
        gyro_angle = self.angle_filtered + gyro_corrected * dt
        self.angle_filtered = gyro_weight * gyro_angle + accel_weight * accel_roll_filtered
        
        return self.angle_filtered, self.confidence

class PIDController:
    """
    PID Controller with Anti-Windup and Rate Limiting
    Makine Mühendisliği optimal değerleri
    """
    
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.last_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()
        
    def update(self, setpoint, current_value):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            dt = 0.02
            
        error = setpoint - current_value
        
        # Dead zone implementation
        if abs(error) < FilterConfig.DEAD_ZONE:
            error = 0.0
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term with anti-windup
        if abs(self.integral) < FilterConfig.MAX_INTEGRAL:
            self.integral += error * dt
        
        i_term = self.ki * self.integral
        
        # Derivative term
        d_term = self.kd * (error - self.last_error) / dt
        self.last_error = error
        
        # PID output
        output = p_term + i_term + d_term
        
        return output

class ServoController:
    """
    X-Kanat Servo Kontrol Sistemi
    Rate limiting, smooth movement ve safety features ile
    """
    
    def __init__(self, master):
        self.master = master
        self.current_position = 0.0  # Derece cinsinden
        self.target_position = 0.0
        self.last_update = time.time()
        self.lock = Lock()
        
    def angle_to_pwm(self, angle):
        """Açıyı PWM değerine çevir"""
        # -45 ile +45 derece arası -> 1100-1900 PWM
        angle = max(-45, min(45, angle))
        pwm = NEUTRAL + (angle / 45.0) * (MAX_PWM - NEUTRAL) if angle > 0 else NEUTRAL + (angle / 45.0) * (NEUTRAL - MIN_PWM)
        return int(max(MIN_PWM, min(MAX_PWM, pwm)))
    
    def set_target(self, angle):
        """Hedef açı belirle (rate limiting ile)"""
        with self.lock:
            current_time = time.time()
            dt = current_time - self.last_update
            self.last_update = current_time
            
            if dt <= 0:
                dt = 0.02
            
            # Rate limiting
            max_change = FilterConfig.MAX_RATE * dt
            error = angle - self.current_position
            
            if abs(error) > max_change:
                self.target_position = self.current_position + (max_change if error > 0 else -max_change)
            else:
                self.target_position = angle
                
            self.current_position = self.target_position
            
    def send_servo_command(self):
        """Servo komutunu gönder"""
        with self.lock:
            pwm = self.angle_to_pwm(self.current_position)
            
            try:
                self.master.mav.command_long_send(
                    self.master.target_system, 
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, SERVO_CH, pwm, 0, 0, 0, 0, 0
                )
                return True
            except Exception as e:
                logging.error(f"Servo command error: {e}")
                return False

class XWingController:
    """
    Ana X-Kanat Kontrol Sistemi
    IMU Filtresi + PID + Servo Control
    """
    
    def __init__(self, connection_string='/dev/ttyACM0,115200'):
        # MAVLink bağlantısı - Serial connection support
        try:
            if ',' in connection_string:
                # Serial connection: port,baud
                port, baud = connection_string.split(',')
                print(f"🔌 Serial bağlantısı: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                # TCP or other connection
                print(f"🌐 TCP bağlantısı: {connection_string}")
                self.master = mavutil.mavlink_connection(connection_string)
            
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            print("✅ MAVLink bağlantısı kuruldu")
        except Exception as e:
            print(f"❌ MAVLink bağlantı hatası: {e}")
            print("💡 Pixhawk bağlantısını ve port ayarlarını kontrol edin")
            raise
        
        # Kontrol bileşenleri
        self.imu_filter = IMUFilter()
        self.pid_controller = PIDController(FilterConfig.KP, FilterConfig.KI, FilterConfig.KD)
        self.servo_controller = ServoController(self.master)
        
        # Sistem durumu
        self.running = False
        self.target_angle = 0.0
        
        # Cleanup kurulumu
        self.install_cleanup()
        
        # Logging
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        
        # Servo yumuşak başlatma
        self.smooth_startup()
    
    def smooth_startup(self):
        """Servo yumuşak başlatma - titreşimsiz başlangıç"""
        try:
            print("🔄 Servo yumuşak başlatma...")
            
            # Mevcut pozisyonu tahmin et (başlangıçta neutral kabul et)
            start_pos = 0.0
            target_pos = 0.0
            
            # 5 adımda yavaşça neutral'a git
            for i in range(5):
                current_pos = start_pos + ((target_pos - start_pos) * (i + 1) / 5)
                pwm = self.servo_controller.angle_to_pwm(current_pos)
                
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, SERVO_CH, pwm, 0, 0, 0, 0, 0
                )
                time.sleep(0.2)  # Yavaş geçiş
            
            # Neutral pozisyonu güçlendir
            for _ in range(3):
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, SERVO_CH, NEUTRAL, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
            
            print("✅ Servo hazır - neutral pozisyonda")
            
        except Exception as e:
            print(f"⚠️  Startup warning: {e}")
        
    def neutral_and_release(self):
        """Sistem kapatma - AGRESIF servo kapatma"""
        try:
            print("🛑 SERVO TAMAMEN KAPATILIYOR...")
            
            # 1. KONTROL DÖNGÜSÜNÜ DURDUR
            self.running = False
            time.sleep(0.5)  # Döngünün durmasını bekle
            
            # 2. RC OVERRIDE İLE SERVO KANAL KONTROLÜ AL
            print("🎛️  RC Override aktif...")
            channels = [65535] * 18  # Tüm kanalları boş bırak
            channels[SERVO_CH - 1] = NEUTRAL  # Sadece servo kanalını kontrol et
            
            # RC override ile servo'yu zorla neutral tut
            for _ in range(20):  # 2 saniye boyunca neutral zorla
                self.master.mav.rc_channels_override_send(
                    self.master.target_system, self.master.target_component,
                    *channels
                )
                time.sleep(0.1)
            
            print("📍 Neutral pozisyon zorlandı")
            
            # 3. SERVO KANALINI TAMAMEN KES - FARKLI YÖNTEMLER DENEYECEĞİZ
            print("⚡ PWM Sinyali kesiliyor - Yöntem 1 (RC Override=0)")
            channels[SERVO_CH - 1] = 0  # PWM'i sıfırla
            for _ in range(10):
                self.master.mav.rc_channels_override_send(
                    self.master.target_system, self.master.target_component,
                    *channels
                )
                time.sleep(0.05)
            
            print("⚡ PWM Sinyali kesiliyor - Yöntem 2 (Servo Disable)")
            # Servo disable komutu
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0, SERVO_CH, 0, 0, 0, 0, 0, 0  # PWM=0
            )
            time.sleep(0.3)
            
            print("⚡ PWM Sinyali kesiliyor - Yöntem 3 (Servo Function Disable)")
            # SERVO_FUNCTION parametresini geçici olarak disable etmeyi dene
            try:
                # SERVOx_FUNCTION parametresini 0 (Disabled) yap
                param_name = f"SERVO{SERVO_CH}_FUNCTION"
                self.master.mav.param_set_send(
                    self.master.target_system, self.master.target_component,
                    param_name.encode('utf-8'), 0, mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                time.sleep(0.2)
                print(f"📋 {param_name} = 0 (Disabled)")
            except:
                print("⚠️  Parametre değiştirilemedi")
            
            print("⚡ PWM Sinyali kesiliyor - Yöntem 4 (Complete RC Override Clear)")
            # Tüm RC override'ları tamamen temizle - birkaç kez
            for _ in range(15):
                self.master.mav.rc_channels_override_send(
                    self.master.target_system, self.master.target_component,
                    *([65535]*18)  # Tüm kanalları serbest bırak
                )
                time.sleep(0.05)
            
            print("⚡ PWM Sinyali kesiliyor - Yöntem 5 (Manual PWM Control)")
            # Manuel PWM kontrolü - minimum değer gönder
            for _ in range(10):
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, SERVO_CH, 900, 0, 0, 0, 0, 0  # Çok düşük PWM
                )
                time.sleep(0.05)
            
            # Son olarak tamamen kes
            for _ in range(5):
                self.master.mav.command_long_send(
                    self.master.target_system, self.master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0, SERVO_CH, 0, 0, 0, 0, 0, 0
                )
                time.sleep(0.1)
            
            # 6. DISARM - SERVO ÇIKIŞLARINI TAMAMEN DURDUR
            print("🔒 Sistem disarm ediliyor...")
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0, 0, 0, 0, 0, 0, 0, 0  # Disarm
            )
            time.sleep(0.5)
            
            print("✅ TÜM SERVO KAPATMA YÖNTEMLERİ UYGULANDI")
            print("🔇 Servo artık sessiz olmalı - eğer hala titreşiyorsa fiziksel problem var")
            
        except Exception as e:
            print(f"❌ Cleanup error: {e}")
        finally:
            try:
                self.master.close()
            except:
                pass
    
    def install_cleanup(self):
        """Cleanup handler kurulumu"""
        def _cleanup(*_):
            self.running = False
            self.neutral_and_release()
        
        atexit.register(_cleanup)
        signal.signal(signal.SIGINT, _cleanup)
        signal.signal(signal.SIGTERM, _cleanup)
    
    def get_imu_data(self):
        """IMU verilerini al"""
        try:
            msg = self.master.recv_match(type='RAW_IMU', blocking=True, timeout=0.1)
            if msg:
                # Convert to m/s² and rad/s
                accel_x = msg.xacc / 1000.0  # mg to m/s²
                accel_y = msg.yacc / 1000.0
                accel_z = msg.zacc / 1000.0
                gyro_x = math.radians(msg.xgyro / 1000.0)  # mrad/s to rad/s
                gyro_y = math.radians(msg.ygyro / 1000.0)
                gyro_z = math.radians(msg.zgyro / 1000.0)
                
                return accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
        except:
            pass
        return None
    
    def set_target_angle(self, angle):
        """Hedef açı belirle"""
        self.target_angle = max(-45, min(45, angle))
        
    def control_loop(self):
        """Ana kontrol döngüsü - Anti-Vibration Optimized"""
        self.running = True
        loop_rate = 20  # Hz - DÜŞÜRÜLDÜ (50->20)
        dt = 1.0 / loop_rate
        
        # PWM Hysteresis - aynı PWM'i tekrar göndermemek için
        last_pwm_sent = None
        pwm_hysteresis = 3  # µs
        
        # Status display timing
        last_status_time = 0
        status_interval = 0.5  # 2 Hz yerine 2 saniyede bir
        
        print(f"X-Kanat kontrol döngüsü başladı ({loop_rate} Hz - Anti-Vibration)")
        
        while self.running:
            start_time = time.time()
            
            # IMU verilerini al
            imu_data = self.get_imu_data()
            if imu_data:
                accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = imu_data
                
                # IMU filtresi uygula
                filtered_angle, confidence = self.imu_filter.update(
                    accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z
                )
                
                # PID kontrolü
                pid_output = self.pid_controller.update(self.target_angle, filtered_angle)
                
                # Servo komutunu ayarla
                self.servo_controller.set_target(pid_output)
                
                # PWM hesapla
                target_pwm = self.servo_controller.angle_to_pwm(self.servo_controller.current_position)
                
                # PWM HYSTERESIS - sadece anlamlı değişikliklerde gönder
                should_send = False
                if last_pwm_sent is None:
                    should_send = True
                elif abs(target_pwm - last_pwm_sent) >= pwm_hysteresis:
                    should_send = True
                
                # Servo komutunu gönder (sadece gerekirse)
                if should_send:
                    if self.servo_controller.send_servo_command():
                        last_pwm_sent = target_pwm
                
                # Status bilgisi (daha seyrek)
                current_time = time.time()
                if current_time - last_status_time >= status_interval:
                    pwm = self.servo_controller.angle_to_pwm(self.servo_controller.current_position)
                    print(f"\r[SERVO] Target: {self.target_angle:+6.1f}° | Current: {filtered_angle:+6.1f}° | "
                          f"PID: {pid_output:+6.1f}° | PWM: {pwm:4d}µs | Conf: {confidence:.2f}", end="")
                    last_status_time = current_time
            
            # Timing control
            elapsed = time.time() - start_time
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)
    
    def run_manual_test(self):
        """Manuel test modu"""
        print("\n" + "="*60)
        print("    🚁 X-KANAT SERVO KONTROL SİSTEMİ - TEKNOFEST 2025")
        print("="*60)
        print(f"📡 MAVLink Bağlantı: BAŞARILI")
        print(f"🎛️  Servo Kanalı: AUX{SERVO_CH-8} (Kanal {SERVO_CH})")
        print(f"⚙️  Kontrol Frekansı: 20 Hz (Anti-Vibration)")
        print(f"📐 Açı Aralığı: -45° ile +45°")
        print(f"🔇 Dead Zone: ±{FilterConfig.DEAD_ZONE:.1f}° (Titreşim Önleme)")
        print(f"⚡ PWM Hysteresis: 3µs (Gereksiz Sinyal Önleme)")
        print("="*60)
        print("\n📝 KOMUTLAR:")
        print("  • 'q' veya 'quit' - Programdan çıkış")
        print("  • sayı (-45 ile +45) - Hedef açı ayarla")
        print("  • 'auto' - Otomatik test sekansi")
        print("  • 'status' - Detaylı sistem durumu")
        print("  • 'zero' - Sıfır pozisyona dön")
        print("-"*60)
        
        # Kontrol thread'ini başlat
        control_thread = Thread(target=self.control_loop, daemon=True)
        control_thread.start()
        
        print("🔄 Sistem hazır - komut bekleniyor...")
        print()
        
        try:
            while self.running:
                try:
                    user_input = input("\n[KOMUT] ➤ ").strip().lower()
                    
                    if user_input in ['q', 'quit', 'exit']:
                        print("🛑 Sistem kapatılıyor...")
                        break
                    elif user_input == 'auto':
                        self.run_auto_test()
                    elif user_input == 'zero':
                        self.set_target_angle(0.0)
                        print("📍 Hedef açı: 0.0° (sıfır pozisyon)")
                    elif user_input == 'status':
                        self.print_system_status()
                    else:
                        try:
                            angle = float(user_input)
                            if -45 <= angle <= 45:
                                self.set_target_angle(angle)
                                print(f"🎯 Hedef açı: {angle:+.1f}°")
                            else:
                                print("❌ Hata: Açı -45° ile +45° arasında olmalı!")
                        except ValueError:
                            print("❌ Geçersiz komut! Sayı girin veya 'help' yazın.")
                            
                except EOFError:
                    break
                except KeyboardInterrupt:
                    print("\n🛑 Klavye kesintisi - sistem kapatılıyor...")
                    break
                    
        except Exception as e:
            print(f"❌ Sistem hatası: {e}")
        finally:
            self.running = False
            print("\n✅ Güvenli kapatma tamamlandı.")
    
    def print_system_status(self):
        """Sistem durumu yazdır"""
        print("\n" + "="*50)
        print("📊 SİSTEM DURUMU")
        print("="*50)
        print(f"🎯 Hedef Açı: {self.target_angle:+6.1f}°")
        print(f"📍 Servo Pozisyon: {self.servo_controller.current_position:+6.1f}°")
        pwm = self.servo_controller.angle_to_pwm(self.servo_controller.current_position)
        print(f"⚡ PWM Değeri: {pwm:4d} µs")
        print(f"🔧 PID Parametreleri: Kp={FilterConfig.KP}, Ki={FilterConfig.KI}, Kd={FilterConfig.KD}")
        print(f"⏱️  Max Hız: {FilterConfig.MAX_RATE:.1f}°/s")
        print(f"🎛️  Ölü Bölge: ±{FilterConfig.DEAD_ZONE:.1f}°")
        print("="*50)
    
    def run_auto_test(self):
        """Otomatik test sekansi"""
        print("\n🔄 Otomatik test başlıyor...")
        test_angles = [0, 15, -15, 30, -30, 45, -45, 0]
        
        for i, angle in enumerate(test_angles, 1):
            if not self.running:
                print("❌ Test iptal edildi!")
                break
            print(f"📐 [{i}/{len(test_angles)}] Test açısı: {angle:+3.0f}°")
            self.set_target_angle(angle)
            time.sleep(2.5)  # Test aralığı
        
        print("✅ Otomatik test tamamlandı - sıfır pozisyona dönülüyor")
        self.set_target_angle(0)

# Ana program
if __name__ == "__main__":
    try:
        # X-Kanat kontrol sistemi oluştur
        x_wing = XWingController()
        
        # Test modunu çalıştır
        x_wing.run_manual_test()
        
    except Exception as e:
        print(f"Sistem hatası: {e}")
        sys.exit(1)
    
    print("Program sonlandırıldı.")
