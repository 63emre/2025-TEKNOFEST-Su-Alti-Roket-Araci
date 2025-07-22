#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - AUX4 Servo Attitude Control
Pixhawk'tan roll/pitch verilerini alarak servo kontrolü
El hareketi ile servo kontrolü - Real-time attitude feedback
"""

import time
import math
import threading
from pymavlink import mavutil

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

# AUX4 servo kanal (Pixhawk AUX OUT 4 = Servo channel 12)
SERVO_CHANNEL = 12

# Servo ayarları
SERVO_FREQUENCY = 100  # Hz
PWM_MIN = 1000    # Minimum PWM (µs)
PWM_MID = 1500    # Orta PWM (µs) 
PWM_MAX = 2000    # Maksimum PWM (µs)

# Attitude → Servo mapping ayarları
ROLL_MAX_ANGLE = 45.0   # Maksimum roll açısı (derece)
PITCH_MAX_ANGLE = 45.0  # Maksimum pitch açısı (derece)

# Servo hareket modu
class ServoMode:
    ROLL_ONLY = "roll"      # Sadece roll (yatay hareket)
    PITCH_ONLY = "pitch"    # Sadece pitch (dikey hareket)  
    COMBINED = "combined"   # Roll + pitch birleşik

class AUX4ServoAttitude:
    def __init__(self):
        self.master = None
        self.connected = False
        self.running = False
        
        # Attitude verileri
        self.current_roll = 0.0      # Radyan
        self.current_pitch = 0.0     # Radyan
        self.current_yaw = 0.0       # Radyan
        
        # Servo durumu
        self.current_servo_pwm = PWM_MID
        self.servo_mode = ServoMode.COMBINED
        
        # Threading
        self.attitude_thread = None
        self.servo_thread = None
        self.stop_event = threading.Event()
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Servo frekansını ayarla
            self.set_servo_frequency(SERVO_FREQUENCY)
            
            # Attitude stream'ini etkinleştir
            self.request_attitude_stream()
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def set_servo_frequency(self, frequency):
        """Servo frekansını ayarla (Hz)"""
        if not self.connected:
            return False
            
        try:
            print(f"🔧 Servo frekansı ayarlanıyor: {frequency}Hz")
            
            self.master.mav.param_set_send(
                self.master.target_system,
                self.master.target_component,
                b'PWM_AUX_RATE',
                frequency,
                mavutil.mavlink.MAV_PARAM_TYPE_INT32
            )
            
            time.sleep(0.5)
            print(f"✅ Servo frekansı {frequency}Hz olarak ayarlandı")
            return True
            
        except Exception as e:
            print(f"❌ Frekans ayarlama hatası: {e}")
            return False
    
    def request_attitude_stream(self):
        """Attitude veri akışını talep et"""
        if not self.connected:
            return False
        
        try:
            # ATTITUDE mesajlarını 20Hz'de talep et
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,  # ATTITUDE stream
                20,  # 20Hz
                1    # Başlat
            )
            
            print("📡 Attitude veri akışı etkinleştirildi (20Hz)")
            return True
            
        except Exception as e:
            print(f"❌ Attitude stream hatası: {e}")
            return False
    
    def read_attitude_data(self):
        """Attitude verilerini sürekli oku"""
        while not self.stop_event.is_set() and self.connected:
            try:
                # ATTITUDE mesajını bekle
                msg = self.master.recv_match(type='ATTITUDE', timeout=0.1)
                
                if msg:
                    # Radyan cinsinden attitude verileri
                    self.current_roll = msg.roll      # Yatay (sola/sağa)
                    self.current_pitch = msg.pitch    # Dikey (öne/arkaya)  
                    self.current_yaw = msg.yaw        # Dönme
                    
            except Exception as e:
                print(f"❌ Attitude okuma hatası: {e}")
                time.sleep(0.1)
        
        print("🔄 Attitude okuma thread'i durdu")
    
    def radians_to_degrees(self, radians):
        """Radyan'ı dereceye çevir"""
        return radians * 180.0 / math.pi
    
    def map_attitude_to_servo(self):
        """Attitude verilerini servo PWM'ine çevir"""
        if self.servo_mode == ServoMode.ROLL_ONLY:
            # Sadece roll kullan (yatay hareket)
            roll_deg = self.radians_to_degrees(self.current_roll)
            normalized = max(-1.0, min(1.0, roll_deg / ROLL_MAX_ANGLE))
            
        elif self.servo_mode == ServoMode.PITCH_ONLY:
            # Sadece pitch kullan (dikey hareket)
            pitch_deg = self.radians_to_degrees(self.current_pitch)
            normalized = max(-1.0, min(1.0, pitch_deg / PITCH_MAX_ANGLE))
            
        else:  # ServoMode.COMBINED
            # Roll ve pitch'i birleştir
            roll_deg = self.radians_to_degrees(self.current_roll)
            pitch_deg = self.radians_to_degrees(self.current_pitch)
            
            # Vektör büyüklüğü hesapla
            magnitude = math.sqrt(roll_deg**2 + pitch_deg**2)
            max_magnitude = math.sqrt(ROLL_MAX_ANGLE**2 + PITCH_MAX_ANGLE**2)
            
            # Yön belirle (roll ağırlıklı)
            if roll_deg != 0:
                sign = 1 if roll_deg > 0 else -1
            else:
                sign = 1 if pitch_deg > 0 else -1
            
            normalized = max(-1.0, min(1.0, (magnitude / max_magnitude) * sign))
        
        # Normalized değeri PWM'e çevir (-1.0 = PWM_MIN, +1.0 = PWM_MAX)
        servo_pwm = int(PWM_MID + normalized * (PWM_MAX - PWM_MID) / 2)
        servo_pwm = max(PWM_MIN, min(PWM_MAX, servo_pwm))
        
        return servo_pwm
    
    def set_servo_pwm(self, pwm_value):
        """AUX4 servo PWM ayarı"""
        if not self.connected:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                SERVO_CHANNEL,
                pwm_value,
                0, 0, 0, 0, 0
            )
            
            self.current_servo_pwm = pwm_value
            return True
            
        except Exception as e:
            print(f"❌ Servo kontrol hatası: {e}")
            return False
    
    def servo_control_loop(self):
        """Servo kontrol döngüsü"""
        last_pwm = PWM_MID
        
        while not self.stop_event.is_set() and self.connected:
            try:
                # Attitude verilerini servo PWM'ine çevir
                target_pwm = self.map_attitude_to_servo()
                
                # PWM değişikliği varsa servo güncelle
                if abs(target_pwm - last_pwm) > 5:  # 5µs threshold
                    self.set_servo_pwm(target_pwm)
                    last_pwm = target_pwm
                
                time.sleep(0.05)  # 20Hz servo update
                
            except Exception as e:
                print(f"❌ Servo kontrol hatası: {e}")
                time.sleep(0.1)
        
        # Servo orta pozisyona getir
        self.set_servo_pwm(PWM_MID)
        print("🔄 Servo kontrol thread'i durdu")
    
    def start_realtime_control(self, mode=ServoMode.COMBINED):
        """Gerçek zamanlı kontrol başlat"""
        self.servo_mode = mode
        self.running = True
        self.stop_event.clear()
        
        print(f"\n🎯 GERÇEK ZAMANLI SERVO KONTROLÜ BAŞLADI")
        print(f"📍 Mod: {mode.upper()}")
        print(f"🎮 Pixhawk'ı hareket ettirin, servo takip edecek!")
        print(f"❌ Durdurmak için Ctrl+C basın")
        print("=" * 60)
        
        # Attitude okuma thread'i başlat
        self.attitude_thread = threading.Thread(target=self.read_attitude_data)
        self.attitude_thread.daemon = True
        self.attitude_thread.start()
        
        # Servo kontrol thread'i başlat  
        self.servo_thread = threading.Thread(target=self.servo_control_loop)
        self.servo_thread.daemon = True
        self.servo_thread.start()
        
        try:
            # Ana döngü - durum göster
            while self.running:
                # Attitude verilerini derece cinsine çevir
                roll_deg = self.radians_to_degrees(self.current_roll)
                pitch_deg = self.radians_to_degrees(self.current_pitch)
                yaw_deg = self.radians_to_degrees(self.current_yaw)
                
                # Servo PWM değerini hesapla
                target_pwm = self.map_attitude_to_servo()
                
                # Durum raporu (her saniye)
                print(f"\r📊 Roll: {roll_deg:6.1f}° | Pitch: {pitch_deg:6.1f}° | Yaw: {yaw_deg:6.1f}° | Servo: {target_pwm:4d}µs", end='', flush=True)
                
                time.sleep(0.2)  # 5Hz status update
                
        except KeyboardInterrupt:
            print("\n\n⚠️ Kullanıcı tarafından durduruldu")
        
        finally:
            self.stop_realtime_control()
    
    def stop_realtime_control(self):
        """Gerçek zamanlı kontrolü durdur"""
        print("\n🔄 Kontrol durduruluyor...")
        self.running = False
        self.stop_event.set()
        
        # Thread'lerin bitmesini bekle
        if self.attitude_thread and self.attitude_thread.is_alive():
            self.attitude_thread.join(timeout=2)
        
        if self.servo_thread and self.servo_thread.is_alive():
            self.servo_thread.join(timeout=2)
        
        # Servo orta pozisyona getir
        self.set_servo_pwm(PWM_MID)
        print("✅ Kontrol durduruldu, servo orta pozisyonda")
    
    def calibration_test(self):
        """Kalibrasyon testi"""
        print("\n🔧 KALİBRASYON TESTİ")
        print("=" * 50)
        print("Pixhawk'ı farklı pozisyonlara tutup servo tepkisini gözlemleyin:")
        print("1. Düz tutun (0° roll, 0° pitch)")
        print("2. Sola eğin (negatif roll)")  
        print("3. Sağa eğin (pozitif roll)")
        print("4. Öne eğin (pozitif pitch)")
        print("5. Arkaya eğin (negatif pitch)")
        
        input("\nHazır olduğunuzda ENTER'a basın...")
        
        # 30 saniye kalibrasyon
        self.start_realtime_control(ServoMode.COMBINED)
    
    def cleanup(self):
        """Temizlik"""
        self.stop_realtime_control()
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    servo_attitude = AUX4ServoAttitude()
    
    print(f"🚀 TEKNOFEST AUX4 Servo Attitude Control")
    print("=" * 70)
    print("Pixhawk Hareket → Servo Kontrol Modları:")
    print("1. Gerçek zamanlı kontrol (Roll + Pitch birleşik)")
    print("2. Sadece yatay hareket (Roll only)")
    print("3. Sadece dikey hareket (Pitch only)")  
    print("4. Kalibrasyon testi")
    print("5. Çıkış")
    
    try:
        choice = input("Seçiminiz (1-5): ").strip()
        
        if choice == '5':
            print("👋 Çıkış yapılıyor...")
            return 0
        
        if not servo_attitude.connect_pixhawk():
            return 1
        
        if choice == '1':
            servo_attitude.start_realtime_control(ServoMode.COMBINED)
        elif choice == '2':
            servo_attitude.start_realtime_control(ServoMode.ROLL_ONLY)
        elif choice == '3':
            servo_attitude.start_realtime_control(ServoMode.PITCH_ONLY)
        elif choice == '4':
            servo_attitude.calibration_test()
        else:
            print("Geçersiz seçim!")
            return 1
            
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1
    finally:
        servo_attitude.cleanup()

if __name__ == "__main__":
    import sys
    sys.exit(main()) 