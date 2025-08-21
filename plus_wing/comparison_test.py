#!/usr/bin/env python3
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
X Wing vs Plus Wing (+) Karşılaştırmalı Test Sistemi
Gerçek Zamanlı Performans Analizi ve Hassas Ölçümler

GÜNCEL CHANNEL MAPPING:
X Wing:
- AUX 1 → Ana Motor (MAVLink Channel 9)
- AUX 3 → Ön Sağ Fin (MAVLink Channel 11)
- AUX 4 → Arka Sol Fin (MAVLink Channel 12)
- AUX 5 → Arka Sağ Fin (MAVLink Channel 13)
- AUX 6 → (MAVLink Channel 14)

Plus Wing:
- AUX 1 → Ana Motor (MAVLink Channel 9)
- AUX 3 → Ön Servo (MAVLink Channel 11)
- AUX 4 → Sol Servo (MAVLink Channel 12)
- AUX 5 → Sağ Servo (MAVLink Channel 13)
- AUX 6 → Arka Servo (MAVLink Channel 14)

Test Özellikleri:
- Gerçek zamanlı IMU feedback
- PID kontrol döngüsü karşılaştırması
- Precision measurement analizi
- Performance metrics comparison
- Türkçe raporlama sistemi
"""

import os
import sys
import time
import math
import json
import threading
from datetime import datetime
from collections import deque
from pymavlink import mavutil

# Plus Wing konfigürasyonu import et
try:
    from hardware_config import (
        PLUS_WING_SERVO_CHANNELS,
        PLUS_WING_CONFIG,
        calculate_plus_wing_pwm,
        get_plus_wing_config
    )
except ImportError:
    print("❌ Plus Wing hardware_config.py bulunamadı!")
    sys.exit(1)

# Connection configuration
try:
    import sys
    sys.path.append('../Test')
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Fallback connection: {MAV_ADDRESS}")

# X Wing konfigürasyonu - Güncel channel mapping
X_WING_SERVO_CHANNELS = {
    'aux1': 9,   # Ana Motor (MAVLink Channel 9)
    'aux3': 11,  # Ön Sağ Fin (MAVLink Channel 11)
    'aux4': 12,  # Arka Sol Fin (MAVLink Channel 12)
    'aux5': 13,  # Arka Sağ Fin (MAVLink Channel 13)
    'aux6': 14   # (MAVLink Channel 14)
}

# X Wing Control Matrix - Güncel mapping
X_WING_CONFIG = {
    'SERVOS': {
        'front_right': {'aux': 3, 'mavlink_channel': 11, 'name': 'Ön Sağ Fin'},
        'rear_left': {'aux': 4, 'mavlink_channel': 12, 'name': 'Arka Sol Fin'},
        'rear_right': {'aux': 5, 'mavlink_channel': 13, 'name': 'Arka Sağ Fin'},
        'extra': {'aux': 6, 'mavlink_channel': 14, 'name': 'Ekstra Fin'}
    },
    'MOTOR': {
        'main_motor': {'aux': 1, 'mavlink_channel': 9, 'name': 'Ana Motor'}
    },
    'CONTROL_GAINS': {
        'pitch_gain': 8,
        'roll_gain': 10,
        'yaw_gain': 6,
        'motor_gain': 1.0
    },
    'PID_PARAMS': {
        'roll': {'kp': 0.80, 'ki': 0.10, 'kd': 0.05, 'max_output': 400},
        'pitch': {'kp': 0.90, 'ki': 0.08, 'kd': 0.06, 'max_output': 400},
        'yaw': {'kp': 0.75, 'ki': 0.12, 'kd': 0.06, 'max_output': 350},
        'depth': {'kp': 1.15, 'ki': 0.12, 'kd': 0.08, 'max_output': 500}
    }
}

def calculate_x_wing_pwm(roll, pitch, yaw):
    """
    X Wing konfigürasyonu için servo PWM değerlerini hesapla
    Güncel channel mapping ile
    """
    neutral = 1500
    
    # X Wing control gains
    pitch_gain = X_WING_CONFIG['CONTROL_GAINS']['pitch_gain']
    roll_gain = X_WING_CONFIG['CONTROL_GAINS']['roll_gain']
    yaw_gain = X_WING_CONFIG['CONTROL_GAINS']['yaw_gain']
    
    # X Wing Mixing Matrix - 3 servo ile (front_right, rear_left, rear_right)
    pwm_values = {
        'front_right': neutral + int((pitch * pitch_gain) - (roll * roll_gain) - (yaw * yaw_gain)),  # AUX3
        'rear_left': neutral + int((-pitch * pitch_gain) + (roll * roll_gain) - (yaw * yaw_gain)),   # AUX4
        'rear_right': neutral + int((-pitch * pitch_gain) - (roll * roll_gain) + (yaw * yaw_gain)),  # AUX5
        'extra': neutral  # AUX6 - Ekstra servo neutral'da kalsın
    }
    
    # PWM limit kontrolü
    for servo in pwm_values:
        pwm_values[servo] = max(1000, min(2000, pwm_values[servo]))
    
    return pwm_values

class ConfigurationComparator:
    """X Wing vs Plus Wing Konfigürasyon Karşılaştırıcı"""
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.armed = False
        
        # Test data storage
        self.test_results = {
            'x_wing': {
                'response_times': [],
                'precision_errors': [],
                'overshoot_events': 0,
                'servo_commands_sent': 0,
                'stability_score': 0.0
            },
            'plus_wing': {
                'response_times': [],
                'precision_errors': [],
                'overshoot_events': 0,
                'servo_commands_sent': 0,
                'stability_score': 0.0
            }
        }
        
        # IMU data
        self.imu_data = {
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0,
            'timestamp': time.time()
        }
        
        # Threading
        self.sensor_thread = None
        self.stop_threads = False
        
        print("🚀 TEKNOFEST X Wing vs Plus Wing Karşılaştırma Sistemi")
        print("=" * 60)
        print(f"📡 MAVLink: {MAV_ADDRESS}")
        print("\n📍 Channel Mapping:")
        print("X Wing Channels:")
        for name, channel in X_WING_SERVO_CHANNELS.items():
            print(f"   {name}: Channel {channel}")
        print("Plus Wing Channels:")
        for name, channel in PLUS_WING_SERVO_CHANNELS.items():
            print(f"   {name}: Channel {channel}")
    
    def connect(self):
        """MAVLink bağlantısı kur"""
        try:
            print(f"📡 Pixhawk bağlantısı kuruluyor...")
            
            # Parse connection string
            if ',' in MAV_ADDRESS:
                port, baud = MAV_ADDRESS.split(',')
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            heartbeat = self.master.wait_heartbeat(timeout=15)
            
            if heartbeat:
                self.connected = True
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print("✅ MAVLink bağlantısı kuruldu!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Armed Status: {'ARMED' if self.armed else 'DISARMED'}")
                
                # Start sensor thread
                self.start_sensor_thread()
                return True
            else:
                print("❌ Heartbeat alınamadı!")
                return False
                
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def start_sensor_thread(self):
        """Sensor okuma thread'ini başlat"""
        self.stop_threads = False
        self.sensor_thread = threading.Thread(target=self._sensor_reader_thread, daemon=True)
        self.sensor_thread.start()
        print("🔄 Sensor okuma thread'i başlatıldı")
    
    def _sensor_reader_thread(self):
        """Sensor data okuma thread'i"""
        while not self.stop_threads and self.connected:
            try:
                # IMU data oku
                msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                if msg:
                    self.imu_data = {
                        'roll': math.degrees(msg.roll),
                        'pitch': math.degrees(msg.pitch),
                        'yaw': math.degrees(msg.yaw),
                        'timestamp': time.time()
                    }
                
                time.sleep(0.02)  # 50Hz sensor okuma
                
            except Exception as e:
                print(f"❌ Sensor okuma hatası: {e}")
                time.sleep(0.1)
    
    def set_servo_pwm(self, channel, pwm_value):
        """Servo PWM değeri ayarla"""
        if not self.connected:
            return False
        
        pwm_value = max(1000, min(2000, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            return True
            
        except Exception as e:
            print(f"❌ Servo PWM komutu hatası: {e}")
            return False
    
    def send_x_wing_commands(self, roll_cmd, pitch_cmd, yaw_cmd):
        """X Wing servo komutlarını gönder"""
        pwm_values = calculate_x_wing_pwm(roll_cmd, pitch_cmd, yaw_cmd)
        
        # Servo komutlarını gönder
        results = []
        servo_channels = {
            'front_right': 11,  # AUX3
            'rear_left': 12,    # AUX4
            'rear_right': 13,   # AUX5
            'extra': 14         # AUX6
        }
        
        for servo_name, pwm_value in pwm_values.items():
            channel = servo_channels[servo_name]
            success = self.set_servo_pwm(channel, pwm_value)
            results.append(success)
            if success:
                self.test_results['x_wing']['servo_commands_sent'] += 1
        
        return all(results)
    
    def send_plus_wing_commands(self, roll_cmd, pitch_cmd, yaw_cmd):
        """Plus Wing servo komutlarını gönder"""
        pwm_values = calculate_plus_wing_pwm(roll_cmd, pitch_cmd, yaw_cmd)
        
        # Servo komutlarını gönder
        results = []
        for servo_name, pwm_value in pwm_values.items():
            channel = PLUS_WING_SERVO_CHANNELS[servo_name]
            success = self.set_servo_pwm(channel, pwm_value)
            results.append(success)
            if success:
                self.test_results['plus_wing']['servo_commands_sent'] += 1
        
        return all(results)
    
    def test_response_time_comparison(self):
        """Response time karşılaştırma testi"""
        print("\n🎮 Response Time Karşılaştırma Testi")
        print("-" * 40)
        
        test_movements = [
            (30, 0, 0, "Roll +30°"),
            (-30, 0, 0, "Roll -30°"),
            (0, 30, 0, "Pitch +30°"),
            (0, -30, 0, "Pitch -30°"),
            (0, 0, 30, "Yaw +30°"),
            (0, 0, -30, "Yaw -30°")
        ]
        
        for config_name in ['x_wing', 'plus_wing']:
            print(f"\n🔧 {config_name.upper()} Response Time Testi:")
            
            for roll, pitch, yaw, description in test_movements:
                # Başlangıç pozisyonu kaydet
                start_roll = self.imu_data['roll']
                start_pitch = self.imu_data['pitch']
                start_yaw = self.imu_data['yaw']
                start_time = time.time()
                
                # Komutu gönder
                if config_name == 'x_wing':
                    self.send_x_wing_commands(roll, pitch, yaw)
                else:
                    self.send_plus_wing_commands(roll, pitch, yaw)
                
                # Response time ölç
                target_reached = False
                timeout = 3.0  # 3 saniye timeout
                
                while (time.time() - start_time) < timeout and not target_reached:
                    current_roll = self.imu_data['roll']
                    current_pitch = self.imu_data['pitch']
                    current_yaw = self.imu_data['yaw']
                    
                    # Target'a ulaşma kontrolü (tolerance: 5°)
                    roll_error = abs((current_roll - start_roll) - roll)
                    pitch_error = abs((current_pitch - start_pitch) - pitch)
                    yaw_error = abs((current_yaw - start_yaw) - yaw)
                    
                    if roll_error < 5.0 and pitch_error < 5.0 and yaw_error < 5.0:
                        response_time = time.time() - start_time
                        self.test_results[config_name]['response_times'].append(response_time)
                        target_reached = True
                        print(f"   {description}: {response_time:.3f}s")
                    
                    time.sleep(0.05)  # 20Hz kontrol
                
                if not target_reached:
                    print(f"   {description}: TIMEOUT (>3s)")
                    self.test_results[config_name]['response_times'].append(3.0)
                
                # Neutral'a dön
                if config_name == 'x_wing':
                    self.send_x_wing_commands(0, 0, 0)
                else:
                    self.send_plus_wing_commands(0, 0, 0)
                
                time.sleep(1.0)  # Stabilizasyon bekle
        
        print("✅ Response time testi tamamlandı")
    
    def test_precision_comparison(self):
        """Precision karşılaştırma testi"""
        print("\n🎮 Precision Karşılaştırma Testi")
        print("-" * 40)
        
        precision_tests = [
            (5, 0, 0, "Küçük Roll (+5°)"),
            (0, 5, 0, "Küçük Pitch (+5°)"),
            (0, 0, 5, "Küçük Yaw (+5°)"),
            (2, 2, 2, "Mikro Kombine Hareket")
        ]
        
        for config_name in ['x_wing', 'plus_wing']:
            print(f"\n🔧 {config_name.upper()} Precision Testi:")
            
            for roll_target, pitch_target, yaw_target, description in precision_tests:
                # Başlangıç pozisyonu kaydet
                start_roll = self.imu_data['roll']
                start_pitch = self.imu_data['pitch']
                start_yaw = self.imu_data['yaw']
                
                # Komutu gönder
                if config_name == 'x_wing':
                    self.send_x_wing_commands(roll_target * 10, pitch_target * 10, yaw_target * 10)
                else:
                    self.send_plus_wing_commands(roll_target * 10, pitch_target * 10, yaw_target * 10)
                
                # Stabilizasyon bekle
                time.sleep(2.5)
                
                # Son pozisyonu ölç
                end_roll = self.imu_data['roll']
                end_pitch = self.imu_data['pitch']
                end_yaw = self.imu_data['yaw']
                
                # Precision error hesapla
                roll_error = abs((end_roll - start_roll) - roll_target)
                pitch_error = abs((end_pitch - start_pitch) - pitch_target)
                yaw_error = abs((end_yaw - start_yaw) - yaw_target)
                total_error = roll_error + pitch_error + yaw_error
                
                self.test_results[config_name]['precision_errors'].append(total_error)
                
                print(f"   {description}:")
                print(f"     Roll Error: {roll_error:.3f}°")
                print(f"     Pitch Error: {pitch_error:.3f}°")
                print(f"     Yaw Error: {yaw_error:.3f}°")
                print(f"     Total Error: {total_error:.3f}°")
                
                # Neutral'a dön
                if config_name == 'x_wing':
                    self.send_x_wing_commands(0, 0, 0)
                else:
                    self.send_plus_wing_commands(0, 0, 0)
                
                time.sleep(1.0)
        
        print("✅ Precision testi tamamlandı")
    
    def test_stability_comparison(self):
        """Stability karşılaştırma testi"""
        print("\n🎮 Stability Karşılaştırma Testi")
        print("-" * 40)
        
        test_duration = 15  # 15 saniye
        
        for config_name in ['x_wing', 'plus_wing']:
            print(f"\n🔧 {config_name.upper()} Stability Testi ({test_duration}s):")
            
            # Hedef pozisyon: Hafif eğimli
            target_roll = 10.0
            target_pitch = 5.0
            target_yaw = 0.0
            
            start_time = time.time()
            stability_errors = []
            
            # Sürekli kontrol döngüsü
            while (time.time() - start_time) < test_duration:
                current_roll = self.imu_data['roll']
                current_pitch = self.imu_data['pitch']
                current_yaw = self.imu_data['yaw']
                
                # Error hesapla
                roll_error = target_roll - current_roll
                pitch_error = target_pitch - current_pitch
                yaw_error = target_yaw - current_yaw
                
                # Basit P controller
                roll_cmd = roll_error * 2.0
                pitch_cmd = pitch_error * 2.0
                yaw_cmd = yaw_error * 1.5
                
                # Komutları sınırla
                roll_cmd = max(-50, min(50, roll_cmd))
                pitch_cmd = max(-50, min(50, pitch_cmd))
                yaw_cmd = max(-50, min(50, yaw_cmd))
                
                # Servo komutlarını gönder
                if config_name == 'x_wing':
                    self.send_x_wing_commands(roll_cmd, pitch_cmd, yaw_cmd)
                else:
                    self.send_plus_wing_commands(roll_cmd, pitch_cmd, yaw_cmd)
                
                # Stability error kaydet
                total_error = abs(roll_error) + abs(pitch_error) + abs(yaw_error)
                stability_errors.append(total_error)
                
                # Progress göster
                elapsed = time.time() - start_time
                if int(elapsed) % 3 == 0 and int(elapsed * 10) % 10 == 0:
                    print(f"   {elapsed:.1f}s - Error: {total_error:.2f}°")
                
                time.sleep(0.1)  # 10Hz kontrol
            
            # Stability score hesapla
            avg_error = sum(stability_errors) / len(stability_errors)
            max_error = max(stability_errors)
            stability_score = max(0, 100 - (avg_error * 10) - (max_error * 5))
            
            self.test_results[config_name]['stability_score'] = stability_score
            
            print(f"   Ortalama Error: {avg_error:.3f}°")
            print(f"   Maksimum Error: {max_error:.3f}°")
            print(f"   Stability Score: {stability_score:.1f}/100")
            
            # Neutral'a dön
            if config_name == 'x_wing':
                self.send_x_wing_commands(0, 0, 0)
            else:
                self.send_plus_wing_commands(0, 0, 0)
            
            time.sleep(2.0)
        
        print("✅ Stability testi tamamlandı")
    
    def generate_comparison_report(self):
        """Karşılaştırma raporu oluştur"""
        print("\n📊 X WING vs PLUS WING KARŞILAŞTIRMA RAPORU")
        print("=" * 60)
        
        # Response Time Analizi
        x_wing_avg_response = sum(self.test_results['x_wing']['response_times']) / len(self.test_results['x_wing']['response_times']) if self.test_results['x_wing']['response_times'] else 0
        plus_wing_avg_response = sum(self.test_results['plus_wing']['response_times']) / len(self.test_results['plus_wing']['response_times']) if self.test_results['plus_wing']['response_times'] else 0
        
        print(f"\n🚀 Response Time Karşılaştırması:")
        print(f"   X Wing Ortalama: {x_wing_avg_response:.3f}s")
        print(f"   Plus Wing Ortalama: {plus_wing_avg_response:.3f}s")
        
        if plus_wing_avg_response > 0 and x_wing_avg_response > 0:
            response_improvement = ((x_wing_avg_response - plus_wing_avg_response) / x_wing_avg_response) * 100
            winner = "Plus Wing" if response_improvement > 0 else "X Wing"
            print(f"   Kazanan: {winner} ({abs(response_improvement):.1f}% daha hızlı)")
        
        # Precision Analizi
        x_wing_avg_precision = sum(self.test_results['x_wing']['precision_errors']) / len(self.test_results['x_wing']['precision_errors']) if self.test_results['x_wing']['precision_errors'] else 0
        plus_wing_avg_precision = sum(self.test_results['plus_wing']['precision_errors']) / len(self.test_results['plus_wing']['precision_errors']) if self.test_results['plus_wing']['precision_errors'] else 0
        
        print(f"\n🎯 Precision Karşılaştırması:")
        print(f"   X Wing Ortalama Error: {x_wing_avg_precision:.3f}°")
        print(f"   Plus Wing Ortalama Error: {plus_wing_avg_precision:.3f}°")
        
        if plus_wing_avg_precision > 0 and x_wing_avg_precision > 0:
            precision_improvement = ((x_wing_avg_precision - plus_wing_avg_precision) / x_wing_avg_precision) * 100
            winner = "Plus Wing" if precision_improvement > 0 else "X Wing"
            print(f"   Kazanan: {winner} ({abs(precision_improvement):.1f}% daha hassas)")
        
        # Stability Analizi
        x_wing_stability = self.test_results['x_wing']['stability_score']
        plus_wing_stability = self.test_results['plus_wing']['stability_score']
        
        print(f"\n⚖️ Stability Karşılaştırması:")
        print(f"   X Wing Stability Score: {x_wing_stability:.1f}/100")
        print(f"   Plus Wing Stability Score: {plus_wing_stability:.1f}/100")
        
        stability_winner = "Plus Wing" if plus_wing_stability > x_wing_stability else "X Wing"
        stability_diff = abs(plus_wing_stability - x_wing_stability)
        print(f"   Kazanan: {stability_winner} (+{stability_diff:.1f} puan)")
        
        # Genel Değerlendirme
        print(f"\n🏆 GENEL DEĞERLENDİRME:")
        
        scores = {
            'x_wing': {
                'response': 100 - (x_wing_avg_response * 20) if x_wing_avg_response > 0 else 0,
                'precision': max(0, 100 - (x_wing_avg_precision * 5)),
                'stability': x_wing_stability
            },
            'plus_wing': {
                'response': 100 - (plus_wing_avg_response * 20) if plus_wing_avg_response > 0 else 0,
                'precision': max(0, 100 - (plus_wing_avg_precision * 5)),
                'stability': plus_wing_stability
            }
        }
        
        x_wing_total = (scores['x_wing']['response'] + scores['x_wing']['precision'] + scores['x_wing']['stability']) / 3
        plus_wing_total = (scores['plus_wing']['response'] + scores['plus_wing']['precision'] + scores['plus_wing']['stability']) / 3
        
        print(f"   X Wing Toplam Skor: {x_wing_total:.1f}/100")
        print(f"   Plus Wing Toplam Skor: {plus_wing_total:.1f}/100")
        
        overall_winner = "Plus Wing" if plus_wing_total > x_wing_total else "X Wing"
        score_diff = abs(plus_wing_total - x_wing_total)
        print(f"   🥇 GENEL KAZANAN: {overall_winner} (+{score_diff:.1f} puan)")
        
        # Öneriler
        print(f"\n💡 ÖNERİLER:")
        if plus_wing_total > x_wing_total:
            print("   ✅ Plus Wing konfigürasyonu bu test koşullarında daha iyi performans gösterdi")
            print("   ✅ Basit kontrol mantığı ve az coupling etkisi avantaj sağladı")
            print("   💡 Başlangıç seviyesi operatörler için Plus Wing önerilir")
        else:
            print("   ✅ X Wing konfigürasyonu bu test koşullarında daha iyi performans gösterdi")
            print("   ✅ Karmaşık hareket kabiliyeti avantaj sağladı")
            print("   💡 İleri seviye operasyonlar için X Wing önerilir")
        
        return {
            'x_wing_total_score': x_wing_total,
            'plus_wing_total_score': plus_wing_total,
            'winner': overall_winner,
            'detailed_scores': scores,
            'recommendations': self._generate_detailed_recommendations(scores)
        }
    
    def _generate_detailed_recommendations(self, scores):
        """Detaylı öneriler oluştur"""
        recommendations = []
        
        # Response time önerileri
        if scores['plus_wing']['response'] > scores['x_wing']['response']:
            recommendations.append("Plus Wing daha hızlı response time gösterdi - basit manevralarda tercih edilebilir")
        else:
            recommendations.append("X Wing daha hızlı response time gösterdi - dinamik manevralarda avantajlı")
        
        # Precision önerileri
        if scores['plus_wing']['precision'] > scores['x_wing']['precision']:
            recommendations.append("Plus Wing daha hassas kontrol sağladı - precision görevlerde tercih edilebilir")
        else:
            recommendations.append("X Wing daha hassas kontrol sağladı - yüksek precision gerektiren görevlerde avantajlı")
        
        # Stability önerileri
        if scores['plus_wing']['stability'] > scores['x_wing']['stability']:
            recommendations.append("Plus Wing daha kararlı performans gösterdi - uzun süreli görevlerde tercih edilebilir")
        else:
            recommendations.append("X Wing daha kararlı performans gösterdi - karmaşık görevlerde avantajlı")
        
        return recommendations
    
    def save_comparison_report(self, filename=None):
        """Karşılaştırma raporunu kaydet"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"x_wing_vs_plus_wing_comparison_{timestamp}.json"
        
        report_data = {
            'test_info': {
                'timestamp': datetime.now().isoformat(),
                'mav_address': MAV_ADDRESS,
                'armed_status': self.armed
            },
            'channel_mapping': {
                'x_wing': X_WING_SERVO_CHANNELS,
                'plus_wing': PLUS_WING_SERVO_CHANNELS
            },
            'test_results': self.test_results,
            'comparison_analysis': self.generate_comparison_report()
        }
        
        try:
            with open(filename, 'w', encoding='utf-8') as f:
                json.dump(report_data, f, indent=2, ensure_ascii=False)
            print(f"\n💾 Karşılaştırma raporu kaydedildi: {filename}")
            return filename
        except Exception as e:
            print(f"❌ Rapor kaydetme hatası: {e}")
            return ""
    
    def run_full_comparison(self):
        """Tam karşılaştırma testi çalıştır"""
        print("🚀 TEKNOFEST X Wing vs Plus Wing Tam Karşılaştırma Testi")
        print("=" * 60)
        
        # Bağlantı testi
        if not self.connect():
            print("❌ Bağlantı testi başarısız!")
            return False
        
        if not self.armed:
            print("⚠️ Sistem DISARMED. Bazı testler sınırlı olabilir.")
        
        # Test suite
        tests = [
            ("Response Time Karşılaştırması", self.test_response_time_comparison),
            ("Precision Karşılaştırması", self.test_precision_comparison),
            ("Stability Karşılaştırması", self.test_stability_comparison)
        ]
        
        # Testleri çalıştır
        for test_name, test_func in tests:
            try:
                print(f"\n{'='*20} {test_name} {'='*20}")
                test_func()
                print(f"✅ {test_name}: TAMAMLANDI")
            except Exception as e:
                print(f"❌ {test_name}: HATA - {e}")
        
        # Karşılaştırma raporu
        comparison_result = self.generate_comparison_report()
        
        # Raporu kaydet
        report_file = self.save_comparison_report()
        
        return True
    
    def disconnect(self):
        """Bağlantıyı kapat ve cleanup"""
        # Thread'i durdur
        self.stop_threads = True
        if self.sensor_thread:
            self.sensor_thread.join(timeout=2.0)
        
        # Servo'ları neutral'a getir
        if self.connected:
            print("🔄 Tüm servo'ları neutral pozisyona getiriliyor...")
            self.send_x_wing_commands(0, 0, 0)
            self.send_plus_wing_commands(0, 0, 0)
            time.sleep(1.0)
        
        # MAVLink bağlantısını kapat
        if self.master:
            try:
                self.master.close()
                print("🔌 MAVLink bağlantısı kapatıldı")
            except:
                pass
        
        self.connected = False

def main():
    """Ana test fonksiyonu"""
    comparator = ConfigurationComparator()
    
    try:
        # Tam karşılaştırma testi çalıştır
        success = comparator.run_full_comparison()
        
        # Cleanup
        comparator.disconnect()
        
        # Exit code
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
        comparator.disconnect()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Karşılaştırma test hatası: {e}")
        comparator.disconnect()
        sys.exit(1)

if __name__ == "__main__":
    main()
