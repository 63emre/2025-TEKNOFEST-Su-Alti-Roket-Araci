#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - GERÇEK HARDWARE VIDEO TEST
X-Wing (AUX1,3,4,5) + Motor (AUX6) - Kapsamlı Real-time Test

GERÇEK HARDWARE MAPPING:
   Ön Sol (AUX 1) ────────────── Ön Sağ (AUX 3)
       │   \                 /   │
       │    \               /    │
       │     \             /     │
       │      \           /      │
       │       \         /       │
       │        \       /        │
       │         \     /         │
       │          \ X /          │
       │          / X \          │
       │         /     \         │
       │        /       \        │
       │       /         \       │
       │      /           \      │
       │     /             \     │
       │    /               \    │
       │   /                 \   │
  Arka Sol (AUX 4) ────────────── Arka Sağ (AUX 5)

AUX6 MOTOR 
"""

import time
import threading
import sys
import termios
import tty
import select
from datetime import datetime

# MAVLink Serial bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
import os
try:
    from connection_config import get_primary_connection
    MAV_ADDRESS = get_primary_connection()
    print(f"📡 Using dynamic serial connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to serial config with environment variables
    serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    MAV_ADDRESS = f"{serial_port},{baud_rate}"
    print(f"⚠️ Using fallback serial connection: {MAV_ADDRESS}")

try:
    from hardware_mapping import (
        HARDWARE_CONFIG, calculate_x_mixing, fins_to_pwm, 
        log_control_state, get_hardware_status
    )
    HAS_HARDWARE_MAPPING = True
except ImportError:
    HAS_HARDWARE_MAPPING = False
    print("⚠️ hardware_mapping.py bulunamadı - basit test modu")

from pymavlink import mavutil
import math

# GERÇEK HARDWARE CHANNELS
REAL_CHANNELS = {
    'aux1_front_left': 9,   # Ön Sol Fin
    'aux3_front_right': 11, # Ön Sağ Fin  
    'aux4_rear_left': 12,   # Arka Sol Fin
    'aux5_rear_right': 13,  # Arka Sağ Fin
    'aux6_motor': 14        # Ana Motor
}

# PWM Values
PWM_MIN = 1000
PWM_MID = 1500
PWM_MAX = 2000
PWM_STEP = 50

class RealHardwareVideoTest:
    """Gerçek Hardware Video Test Sınıfı"""
    
    def __init__(self):
        self.master = None
        self.connected = False
        self.running = False
        
        # Real-time control state
        self.current_pwm = {
            'aux1': PWM_MID, 'aux3': PWM_MID, 'aux4': PWM_MID, 
            'aux5': PWM_MID, 'aux6': PWM_MID
        }
        
        # Video test logging
        self.test_start_time = time.time()
        self.command_log = []
        
        # Terminal settings
        self.old_settings = None
        
    def connect_hardware(self):
        """Gerçek hardware'a bağlan"""
        try:
            print(f"🔌 GERÇEK HARDWARE bağlantısı...")
            
            # Handle serial vs TCP connection
            if ',' in MAV_ADDRESS:
                # Serial connection: port,baud
                port, baud = MAV_ADDRESS.split(',')
                print(f"📡 Serial: {port} @ {baud} baud")
                self.master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
            else:
                # TCP or other connection
                print(f"🌐 TCP: {MAV_ADDRESS}")
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=15)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Hardware durumunu göster
            if HAS_HARDWARE_MAPPING:
                status = get_hardware_status()
                print(f"📋 Hardware Config: {status['configuration']}")
                print("🎯 Real Channels:")
                for aux, channel in REAL_CHANNELS.items():
                    print(f"   {aux.upper()}: MAVLink Channel {channel}")
                print(f"✅ Video test için hazır: {status['ready_for_video']}")
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def send_real_pwm(self, aux_name, pwm_value):
        """Gerçek hardware'a PWM gönder"""
        if not self.connected:
            return False
        
        try:
            # PWM limits
            pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
            
            # Get real MAVLink channel
            channel = REAL_CHANNELS.get(aux_name)
            if not channel:
                print(f"❌ Bilinmeyen AUX: {aux_name}")
                return False
            
            # Send MAVLink command
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                channel,  # servo number (MAVLink channel)
                pwm_value,  # PWM value
                0, 0, 0, 0, 0  # unused parameters
            )
            
            # Update state
            aux_key = aux_name.split('_')[0]  # aux1, aux3, aux4, aux5, aux6
            self.current_pwm[aux_key] = pwm_value
            
            # Video test logging
            timestamp = time.time() - self.test_start_time
            log_entry = {
                'time': timestamp,
                'aux': aux_name,
                'channel': channel, 
                'pwm': pwm_value,
                'action': self.get_action_description(aux_name, pwm_value)
            }
            self.command_log.append(log_entry)
            
            return True
            
        except Exception as e:
            print(f"❌ PWM gönderme hatası ({aux_name}): {e}")
            return False
    
    def get_action_description(self, aux_name, pwm_value):
        """Hareket açıklaması - video için"""
        if 'motor' in aux_name:
            if pwm_value > PWM_MID + 100:
                return "MOTOR İLERİ"
            elif pwm_value < PWM_MID - 100:
                return "MOTOR GERİ"
            else:
                return "MOTOR NÖTR"
        else:
            if pwm_value > PWM_MID + 50:
                return "FIN YUKARI"
            elif pwm_value < PWM_MID - 50:
                return "FIN AŞAĞI"
            else:
                return "FIN NÖTR"
    
    def test_individual_servos(self):
        """Bireysel servo testi - video demo için"""
        print("\n🎬 SERVO TESTİ - VİDEO DEMO")
        print("=" * 50)
        
        test_sequence = [
            ('aux1_front_left', 'Ön Sol Fin'),
            ('aux3_front_right', 'Ön Sağ Fin'),
            ('aux4_rear_left', 'Arka Sol Fin'),
            ('aux5_rear_right', 'Arka Sağ Fin'),
            ('aux6_motor', 'Ana Motor')
        ]
        
        for aux_name, description in test_sequence:
            print(f"\n🎯 {description} Test Başlıyor...")
            
            # Neutral position
            self.send_real_pwm(aux_name, PWM_MID)
            print(f"   📍 {description}: NÖTR pozisyon ({PWM_MID}µs)")
            time.sleep(1)
            
            # Positive direction
            test_pwm = PWM_MID + 300
            self.send_real_pwm(aux_name, test_pwm)
            print(f"   ⬆️ {description}: POZİTİF hareket ({test_pwm}µs)")
            time.sleep(2)
            
            # Negative direction  
            test_pwm = PWM_MID - 300
            self.send_real_pwm(aux_name, test_pwm)
            print(f"   ⬇️ {description}: NEGATİF hareket ({test_pwm}µs)")
            time.sleep(2)
            
            # Return to neutral
            self.send_real_pwm(aux_name, PWM_MID)
            print(f"   🏁 {description}: NÖTR'e dönüldü")
            time.sleep(1)
    
    def test_x_wing_movements(self):
        """X-Wing hareket testi - gerçek X-mixing"""
        print("\n🎬 X-WİNG HAREKET TESTİ")
        print("=" * 50)
        
        movements = [
            {'name': 'ROLL SOL', 'roll': -20, 'pitch': 0, 'yaw': 0},
            {'name': 'ROLL SAĞ', 'roll': 20, 'pitch': 0, 'yaw': 0},
            {'name': 'PITCH YUKARI', 'roll': 0, 'pitch': 20, 'yaw': 0},
            {'name': 'PITCH AŞAĞI', 'roll': 0, 'pitch': -20, 'yaw': 0},
            {'name': 'YAW SOL', 'roll': 0, 'pitch': 0, 'yaw': -20},
            {'name': 'YAW SAĞ', 'roll': 0, 'pitch': 0, 'yaw': 20},
            {'name': 'KOMBİNE 1', 'roll': 15, 'pitch': 10, 'yaw': -10},
            {'name': 'KOMBİNE 2', 'roll': -10, 'pitch': -15, 'yaw': 15}
        ]
        
        for movement in movements:
            print(f"\n🎯 {movement['name']} hareketi başlıyor...")
            
            if HAS_HARDWARE_MAPPING:
                # Advanced X-mixing with hardware mapping
                fins = calculate_x_mixing(movement['roll'], movement['pitch'], movement['yaw'])
                pwm_values = fins_to_pwm(fins)
                
                # Send to real hardware
                self.send_real_pwm('aux1_front_left', pwm_values['front_left'])
                self.send_real_pwm('aux3_front_right', pwm_values['front_right'])
                self.send_real_pwm('aux4_rear_left', pwm_values['rear_left'])
                self.send_real_pwm('aux5_rear_right', pwm_values['rear_right'])
                
                # Log with hardware mapping
                log_control_state(movement['roll'], movement['pitch'], movement['yaw'], 0, fins, pwm_values)
                
            else:
                # Basit X-mixing
                self.simple_x_mixing(movement['roll'], movement['pitch'], movement['yaw'])
            
            time.sleep(3)  # Her hareketi 3 saniye tut
            
            # Neutral'a dön
            self.return_to_neutral()
            time.sleep(1)
    
    def simple_x_mixing(self, roll, pitch, yaw):
        """Basit X-mixing algoritması"""
        # X-mixing formülü
        front_left = PWM_MID + (pitch * 8) + (roll * 10) + (yaw * 6)
        front_right = PWM_MID + (pitch * 8) - (roll * 10) - (yaw * 6)
        rear_left = PWM_MID - (pitch * 8) + (roll * 10) - (yaw * 6)
        rear_right = PWM_MID - (pitch * 8) - (roll * 10) + (yaw * 6)
        
        # PWM limits
        front_left = max(PWM_MIN, min(PWM_MAX, int(front_left)))
        front_right = max(PWM_MIN, min(PWM_MAX, int(front_right)))
        rear_left = max(PWM_MIN, min(PWM_MAX, int(rear_left)))
        rear_right = max(PWM_MIN, min(PWM_MAX, int(rear_right)))
        
        # Send to hardware
        self.send_real_pwm('aux1_front_left', front_left)
        self.send_real_pwm('aux3_front_right', front_right)
        self.send_real_pwm('aux4_rear_left', rear_left)
        self.send_real_pwm('aux5_rear_right', rear_right)
        
        print(f"🎮 X-Mixing: R={roll}° P={pitch}° Y={yaw}°")
        print(f"   PWM → AUX1:{front_left} AUX3:{front_right} AUX4:{rear_left} AUX5:{rear_right}")
    
    def test_motor_control(self):
        """Motor kontrolü testi"""
        print("\n🎬 MOTOR KONTROL TESTİ")
        print("=" * 50)
        
        motor_tests = [
            {'name': 'YAVAS İLERİ', 'pwm': PWM_MID + 100, 'duration': 3},
            {'name': 'ORTA İLERİ', 'pwm': PWM_MID + 200, 'duration': 2},
            {'name': 'HIZLI İLERİ', 'pwm': PWM_MID + 300, 'duration': 2},
            {'name': 'NÖTR', 'pwm': PWM_MID, 'duration': 2},
            {'name': 'YAVAS GERİ', 'pwm': PWM_MID - 100, 'duration': 3},
            {'name': 'ORTA GERİ', 'pwm': PWM_MID - 200, 'duration': 2},
            {'name': 'HIZLI GERİ', 'pwm': PWM_MID - 300, 'duration': 2}
        ]
        
        for test in motor_tests:
            print(f"\n🚁 MOTOR {test['name']}: PWM {test['pwm']}µs")
            self.send_real_pwm('aux6_motor', test['pwm'])
            
            for i in range(test['duration']):
                print(f"   ⏱️ {i+1}/{test['duration']} saniye...")
                time.sleep(1)
        
        # Motor'u durdur
        print(f"\n🏁 Motor NÖTR pozisyona getiriliyor...")
        self.send_real_pwm('aux6_motor', PWM_MID)
    
    def return_to_neutral(self):
        """Tüm servo ve motor'u neutral pozisyona getir"""
        for aux_name in REAL_CHANNELS.keys():
            self.send_real_pwm(aux_name, PWM_MID)
        print("🏁 Tüm sistemler NÖTR pozisyonda")
    
    def interactive_control(self):
        """Interaktif gerçek zamanlı kontrol - video demo için"""
        print("\n🎮 İNTERAKTİF GERÇEK ZAMANLI KONTROL")
        print("=" * 50)
        print("KONTROLLER:")
        print("  W/S: Pitch ±     A/D: Roll ±      Q/E: Yaw ±")
        print("  O/L: Motor ±     R: Reset All     ESC: Çıkış")
        print("=" * 50)
        
        # Terminal'i raw moda al
        self.setup_raw_terminal()
        
        try:
            self.running = True
            
            while self.running:
                if self.has_input():
                    key = sys.stdin.read(1).lower()
                    
                    if key == '\x1b':  # ESC
                        break
                    elif key == 'w':
                        self.handle_pitch(1)
                    elif key == 's':
                        self.handle_pitch(-1)
                    elif key == 'a':
                        self.handle_roll(-1)
                    elif key == 'd':
                        self.handle_roll(1)
                    elif key == 'q':
                        self.handle_yaw(-1)
                    elif key == 'e':
                        self.handle_yaw(1)
                    elif key == 'o':
                        self.handle_motor(1)
                    elif key == 'l':
                        self.handle_motor(-1)
                    elif key == 'r':
                        self.return_to_neutral()
                
                time.sleep(0.02)  # 50Hz control loop
                
        finally:
            self.restore_terminal()
            self.return_to_neutral()
    
    def handle_pitch(self, direction):
        """Pitch kontrolü"""
        if HAS_HARDWARE_MAPPING:
            fins = calculate_x_mixing(0, direction * 20, 0)
            pwm_values = fins_to_pwm(fins)
            
            self.send_real_pwm('aux1_front_left', pwm_values['front_left'])
            self.send_real_pwm('aux3_front_right', pwm_values['front_right'])
            self.send_real_pwm('aux4_rear_left', pwm_values['rear_left'])
            self.send_real_pwm('aux5_rear_right', pwm_values['rear_right'])
        else:
            self.simple_x_mixing(0, direction * 20, 0)
        
        print(f"🎮 PITCH {'YUKARI' if direction > 0 else 'AŞAĞI'}")
    
    def handle_roll(self, direction):
        """Roll kontrolü"""
        if HAS_HARDWARE_MAPPING:
            fins = calculate_x_mixing(direction * 20, 0, 0)
            pwm_values = fins_to_pwm(fins)
            
            self.send_real_pwm('aux1_front_left', pwm_values['front_left'])
            self.send_real_pwm('aux3_front_right', pwm_values['front_right'])
            self.send_real_pwm('aux4_rear_left', pwm_values['rear_left'])
            self.send_real_pwm('aux5_rear_right', pwm_values['rear_right'])
        else:
            self.simple_x_mixing(direction * 20, 0, 0)
        
        print(f"🎮 ROLL {'SAĞ' if direction > 0 else 'SOL'}")
    
    def handle_yaw(self, direction):
        """Yaw kontrolü"""
        if HAS_HARDWARE_MAPPING:
            fins = calculate_x_mixing(0, 0, direction * 20)
            pwm_values = fins_to_pwm(fins)
            
            self.send_real_pwm('aux1_front_left', pwm_values['front_left'])
            self.send_real_pwm('aux3_front_right', pwm_values['front_right'])
            self.send_real_pwm('aux4_rear_left', pwm_values['rear_left'])
            self.send_real_pwm('aux5_rear_right', pwm_values['rear_right'])
        else:
            self.simple_x_mixing(0, 0, direction * 20)
        
        print(f"🎯 YAW {'SAĞ' if direction > 0 else 'SOL'}")
    
    def handle_motor(self, direction):
        """Motor kontrolü"""
        current_motor = self.current_pwm['aux6']
        new_motor = current_motor + (direction * PWM_STEP)
        new_motor = max(PWM_MIN, min(PWM_MAX, new_motor))
        
        self.send_real_pwm('aux6_motor', new_motor)
        print(f"🚁 MOTOR {'İLERİ' if direction > 0 else 'GERİ'}: {new_motor}µs")
    
    def setup_raw_terminal(self):
        """Terminal'i raw moda al"""
        try:
            self.old_settings = termios.tcgetattr(sys.stdin)
            tty.cbreak(sys.stdin.fileno())
        except:
            pass
    
    def restore_terminal(self):
        """Terminal ayarlarını geri yükle"""
        try:
            if self.old_settings:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        except:
            pass
    
    def has_input(self):
        """Input var mı kontrol et"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
    
    def generate_test_report(self):
        """Test raporu oluştur - video için"""
        print("\n📊 VİDEO TEST RAPORU")
        print("=" * 50)
        
        total_time = time.time() - self.test_start_time
        total_commands = len(self.command_log)
        
        print(f"⏱️ Toplam Test Süresi: {total_time:.1f} saniye")
        print(f"📡 Toplam Komut Sayısı: {total_commands}")
        
        if total_commands > 0:
            print(f"📊 Ortalama Komut Frekansı: {total_commands/total_time:.1f} komut/saniye")
            
            # AUX usage breakdown
            aux_usage = {}
            for log_entry in self.command_log:
                aux = log_entry['aux']
                if aux not in aux_usage:
                    aux_usage[aux] = 0
                aux_usage[aux] += 1
            
            print(f"\n📋 AUX Kullanım Dağılımı:")
            for aux, count in aux_usage.items():
                percentage = (count * 100) // total_commands
                print(f"   {aux.upper()}: {count} komut ({percentage}%)")
        
        print(f"\n✅ GERÇEK HARDWARE TEST TAMAMLANDI!")
        print(f"🎬 Video çekimi için tüm testler başarıyla çalıştırıldı!")
    
    def disconnect(self):
        """Bağlantıyı kapat ve temizlik yap"""
        if self.connected:
            print("\n🔄 Hardware bağlantısı kapatılıyor...")
            self.return_to_neutral()
            time.sleep(1)
            
            if self.master:
                self.master.close()
            
            self.connected = False
            print("✅ Hardware bağlantısı kapatıldı")

def main():
    """Ana fonksiyon - Video test menüsü"""
    print("🚀 TEKNOFEST - GERÇEK HARDWARE VİDEO TEST")
    print("=" * 60)
    print("GERÇEK HARDWARE MAPPING:")
    print("   AUX1: Ön Sol Fin      AUX3: Ön Sağ Fin")
    print("   AUX4: Arka Sol Fin    AUX5: Arka Sağ Fin")
    print("   AUX6: Ana Motor")
    print("=" * 60)
    
    # Test sistemi oluştur
    test_system = RealHardwareVideoTest()
    
    # Hardware'a bağlan
    if not test_system.connect_hardware():
        print("❌ Hardware bağlantısı başarısız!")
        return 1
    
    try:
        while True:
            print(f"\n🎬 VİDEO TEST MENÜSÜ:")
            print("1: Bireysel Servo Testi (Demo)")
            print("2: X-Wing Hareket Testi") 
            print("3: Motor Kontrol Testi")
            print("4: İnteraktif Real-time Kontrol")
            print("5: Tam Test Paketi (Video İçin)")
            print("6: Test Raporu")
            print("Q: Çıkış")
            
            choice = input("\nSeçiminiz: ").strip().upper()
            
            if choice == '1':
                test_system.test_individual_servos()
            elif choice == '2':
                test_system.test_x_wing_movements()
            elif choice == '3':
                test_system.test_motor_control()
            elif choice == '4':
                test_system.interactive_control()
            elif choice == '5':
                print("🎬 TAM TEST PAKETİ BAŞLATILUYOR...")
                test_system.test_individual_servos()
                input("⏸️ X-Wing test için ENTER'a basın...")
                test_system.test_x_wing_movements()
                input("⏸️ Motor test için ENTER'a basın...")
                test_system.test_motor_control()
                print("🎉 TAM TEST PAKETİ TAMAMLANDI!")
            elif choice == '6':
                test_system.generate_test_report()
            elif choice == 'Q':
                break
            else:
                print("⚠️ Geçersiz seçim!")
                
    except KeyboardInterrupt:
        print("\n⏹️ Test kullanıcı tarafından durduruldu!")
    finally:
        test_system.disconnect()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 