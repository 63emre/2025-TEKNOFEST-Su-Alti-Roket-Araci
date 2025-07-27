#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - AUX1234 Toplam Servo Test
AUX 1, 2, 3, 4 servo motorlarının aynı anda test edilmesi
Çoklu servo kontrolü ve senkronize hareket testleri - 330Hz
"""

import time
import threading
import queue
from pymavlink import mavutil
from concurrent.futures import ThreadPoolExecutor, as_completed

# MAVLink bağlantı adresi - DYNAMIC CONFIGURATION SYSTEM
try:
    from connection_config import get_test_constants
    CONFIG = get_test_constants()
    MAV_ADDRESS = CONFIG['MAV_ADDRESS']
    print(f"📡 Using dynamic connection: {MAV_ADDRESS}")
except ImportError:
    # Fallback to static config
    MAV_ADDRESS = 'tcp:127.0.0.1:5777'
    print(f"⚠️ Using fallback connection: {MAV_ADDRESS}")

# Servo kanalları (Pixhawk AUX OUT 1-4 = Servo channels 9-12)
SERVO_CHANNELS = {
    'AUX1': 9,
    'AUX2': 10,
    'AUX3': 11,
    'AUX4': 12
}

# Servo frekansı (Hz)
SERVO_FREQUENCY = 330

# PWM değer aralıkları
PWM_MIN = 1000    # Minimum PWM (µs)
PWM_MID = 1500    # Orta PWM (µs) 
PWM_MAX = 2000    # Maksimum PWM (µs)

class MultiServoTest:
    def __init__(self):
        self.master = None
        self.connected = False
        self.test_running = False
        self.stop_event = threading.Event()
        self.servo_status = {}
        
        # Her servo için ayrı thread lock
        self.servo_locks = {name: threading.Lock() for name in SERVO_CHANNELS.keys()}
        
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
            
            # Frekans ayarını doğrula
            current_freq = self.get_servo_frequency()
            if current_freq != SERVO_FREQUENCY:
                print(f"⚠️ Frekans doğrulaması başarısız: Hedef {SERVO_FREQUENCY}Hz, Mevcut {current_freq}Hz")
            
            # Servo durumlarını başlat
            for servo_name in SERVO_CHANNELS.keys():
                self.servo_status[servo_name] = {"pwm": PWM_MID, "active": False}
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def set_servo_frequency(self, frequency):
        """Servo frekansını ayarla (Hz)"""
        if not self.connected:
            print("❌ MAVLink bağlantısı yok!")
            return False
            
        try:
            print(f"🔧 Servo frekansı ayarlanıyor: {frequency}Hz")
            
            # AUX output frekansını ayarla
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
    
    def get_servo_frequency(self):
        """Mevcut servo frekansını oku"""
        if not self.connected:
            return None
            
        try:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                b'PWM_AUX_RATE',
                -1
            )
            
            msg = self.master.recv_match(type='PARAM_VALUE', timeout=5)
            if msg:
                return int(msg.param_value)
            return None
                
        except Exception as e:
            return None
    
    def set_servo_pwm(self, servo_name, pwm_value):
        """Tek servo PWM ayarı"""
        if not self.connected:
            return False
            
        # PWM değer kontrolü
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
        try:
            channel = SERVO_CHANNELS[servo_name]
            
            # MAVLink servo komut gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel,
                pwm_value,
                0, 0, 0, 0, 0
            )
            
            # Durumu güncelle
            self.servo_status[servo_name]["pwm"] = pwm_value
            print(f"📍 {servo_name}: {pwm_value}µs")
            return True
            
        except Exception as e:
            print(f"❌ {servo_name} servo kontrol hatası: {e}")
            return False
    
    def set_all_servos_pwm(self, pwm_values):
        """Tüm servoları aynı anda ayarla"""
        if not self.connected:
            return False
        
        success = True
        for servo_name, pwm_value in pwm_values.items():
            if servo_name in SERVO_CHANNELS:
                if not self.set_servo_pwm(servo_name, pwm_value):
                    success = False
        
        return success
    
    def test_individual_servos(self):
        """Her servo için ayrı pozisyon testi"""
        print("\n🔧 BİREYSEL SERVO POZİSYON TESTLERİ")
        print("=" * 60)
        
        def test_single_servo(servo_name):
            print(f"\n🔹 {servo_name} Servo Test Başlıyor...")
            positions = [PWM_MID, PWM_MIN, PWM_MAX, PWM_MID]
            descriptions = ["Orta", "Min", "Max", "Orta"]
            
            for pwm, desc in zip(positions, descriptions):
                if self.stop_event.is_set():
                    break
                print(f"  📍 {servo_name} -> {desc}: {pwm}µs")
                self.set_servo_pwm(servo_name, pwm)
                time.sleep(2)
            
            print(f"✅ {servo_name} test tamamlandı!")
        
        # Paralel test - her servo için ayrı thread
        with ThreadPoolExecutor(max_workers=4) as executor:
            futures = [executor.submit(test_single_servo, name) 
                      for name in SERVO_CHANNELS.keys()]
            
            for future in as_completed(futures):
                try:
                    future.result()
                except Exception as e:
                    print(f"❌ Servo test hatası: {e}")
        
        print("\n✅ Tüm bireysel testler tamamlandı!")
    
    def test_synchronized_movement(self):
        """Senkronize hareket testi"""
        print("\n🤝 SENKRONİZE HAREKET TESTİ")
        print("=" * 60)
        
        sync_sequences = [
            {"desc": "Tümü Orta", "values": {name: PWM_MID for name in SERVO_CHANNELS}},
            {"desc": "Tümü Minimum", "values": {name: PWM_MIN for name in SERVO_CHANNELS}},
            {"desc": "Tümü Maksimum", "values": {name: PWM_MAX for name in SERVO_CHANNELS}},
            {"desc": "Alternatif 1", "values": {"AUX1": PWM_MIN, "AUX2": PWM_MAX, "AUX3": PWM_MIN, "AUX4": PWM_MAX}},
            {"desc": "Alternatif 2", "values": {"AUX1": PWM_MAX, "AUX2": PWM_MIN, "AUX3": PWM_MAX, "AUX4": PWM_MIN}},
            {"desc": "Tümü Orta", "values": {name: PWM_MID for name in SERVO_CHANNELS}}
        ]
        
        for sequence in sync_sequences:
            if self.stop_event.is_set():
                break
            
            print(f"\n🎯 {sequence['desc']}")
            for servo_name, pwm_value in sequence['values'].items():
                print(f"  📍 {servo_name}: {pwm_value}µs")
            
            self.set_all_servos_pwm(sequence['values'])
            time.sleep(3)
        
        print("\n✅ Senkronize hareket testi tamamlandı!")
    
    def test_wave_motion(self):
        """Dalga hareketi testi"""
        print("\n🌊 DALGA HAREKETİ TESTİ")
        print("=" * 60)
        
        print("🔄 10 saniye boyunca dalga hareketi...")
        
        start_time = time.time()
        phase_offset = 0
        
        while time.time() - start_time < 10 and not self.stop_event.is_set():
            for i, servo_name in enumerate(SERVO_CHANNELS.keys()):
                # Sinüs dalgası ile PWM hesapla
                phase = (time.time() * 2 + i * 1.5 + phase_offset) % (2 * 3.14159)
                pwm_range = (PWM_MAX - PWM_MIN) // 2
                pwm_center = PWM_MIN + pwm_range
                pwm_value = int(pwm_center + (pwm_range * 0.8) * 
                               (1 + __import__('math').sin(phase)) / 2)
                
                self.set_servo_pwm(servo_name, pwm_value)
            
            time.sleep(0.1)
        
        # Tümünü orta pozisyona getir
        self.set_all_servos_pwm({name: PWM_MID for name in SERVO_CHANNELS})
        print("\n✅ Dalga hareketi testi tamamlandı!")
    
    def test_sequence_pattern(self):
        """Sıralı hareket deseni testi"""
        print("\n🎪 SIRALI HAREKET DESENİ TESTİ")
        print("=" * 60)
        
        print("🔄 Servo sıralaması: AUX1 → AUX2 → AUX3 → AUX4 → tekrar...")
        
        for cycle in range(3):  # 3 döngü
            if self.stop_event.is_set():
                break
                
            print(f"\n🔄 Döngü {cycle + 1}/3")
            
            # Tümünü orta pozisyona getir
            self.set_all_servos_pwm({name: PWM_MID for name in SERVO_CHANNELS})
            time.sleep(0.5)
            
            # Sırayla her servoyu hareket ettir
            for servo_name in SERVO_CHANNELS.keys():
                if self.stop_event.is_set():
                    break
                
                print(f"  ⚡ {servo_name} aktif...")
                
                # Mevcut servoyu max'a götür
                self.set_servo_pwm(servo_name, PWM_MAX)
                time.sleep(0.8)
                
                # Orta pozisyona dön
                self.set_servo_pwm(servo_name, PWM_MID)
                time.sleep(0.3)
        
        print("\n✅ Sıralı hareket deseni testi tamamlandı!")
    
    def interactive_multi_servo_test(self):
        """Çoklu servo interaktif testi"""
        print("\n🎮 ÇOK SERVO İNTERAKTİF TEST")
        print("=" * 60)
        print("Komutlar:")
        print("  'servo_name PWM_value' (örn: AUX1 1200)")
        print("  'all PWM_value' (tümü için)")
        print("  'reset' (tümü orta pozisyon)")
        print("  'status' (durum göster)")
        print("  'q' (çıkış)")
        
        while not self.stop_event.is_set():
            try:
                user_input = input("\nKomut > ").strip()
                
                if user_input.lower() == 'q':
                    break
                elif user_input.lower() == 'reset':
                    self.set_all_servos_pwm({name: PWM_MID for name in SERVO_CHANNELS})
                    print("✅ Tüm servolar orta pozisyona getirildi")
                elif user_input.lower() == 'status':
                    print("\n📊 SERVO DURUMLARI:")
                    for name, status in self.servo_status.items():
                        print(f"  {name}: {status['pwm']}µs")
                elif user_input.startswith('all '):
                    try:
                        pwm_value = int(user_input.split()[1])
                        if PWM_MIN <= pwm_value <= PWM_MAX:
                            self.set_all_servos_pwm({name: pwm_value for name in SERVO_CHANNELS})
                            print(f"✅ Tüm servolar {pwm_value}µs olarak ayarlandı")
                        else:
                            print(f"⚠️ PWM değeri {PWM_MIN}-{PWM_MAX} arasında olmalı!")
                    except ValueError:
                        print("⚠️ Geçerli PWM değeri girin!")
                else:
                    parts = user_input.split()
                    if len(parts) == 2:
                        servo_name, pwm_str = parts
                        servo_name = servo_name.upper()
                        
                        if servo_name in SERVO_CHANNELS:
                            try:
                                pwm_value = int(pwm_str)
                                if PWM_MIN <= pwm_value <= PWM_MAX:
                                    self.set_servo_pwm(servo_name, pwm_value)
                                    print(f"✅ {servo_name}: {pwm_value}µs")
                                else:
                                    print(f"⚠️ PWM değeri {PWM_MIN}-{PWM_MAX} arasında olmalı!")
                            except ValueError:
                                print("⚠️ Geçerli PWM değeri girin!")
                        else:
                            print(f"⚠️ Geçersiz servo adı! Kullanılabilir: {', '.join(SERVO_CHANNELS.keys())}")
                    else:
                        print("⚠️ Format: 'servo_name PWM_value' veya 'all PWM_value'")
                        
            except KeyboardInterrupt:
                break
        
        # Çıkışta tümünü orta pozisyona getir
        self.set_all_servos_pwm({name: PWM_MID for name in SERVO_CHANNELS})
        print("\n✅ İnteraktif test tamamlandı!")
    
    def run_all_tests(self):
        """Tüm testleri sırayla çalıştır"""
        print("🧪 AUX1234 SERVO TOPLAM TEST PAKETİ")
        print("=" * 70)
        print(f"📡 Servo Frekansı: {SERVO_FREQUENCY}Hz")
        print(f"📍 Servo Kanalları: {SERVO_CHANNELS}")
        print("-" * 70)
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        try:
            self.test_running = True
            
            # 1. Bireysel servo testleri
            self.test_individual_servos()
            
            if not self.stop_event.is_set():
                input("\n⏸️ Senkronize test için ENTER'a basın...")
                
                # 2. Senkronize hareket testi
                self.test_synchronized_movement()
            
            if not self.stop_event.is_set():
                input("\n⏸️ Dalga hareketi için ENTER'a basın...")
                
                # 3. Dalga hareketi testi
                self.test_wave_motion()
            
            if not self.stop_event.is_set():
                input("\n⏸️ Sıralı hareket için ENTER'a basın...")
                
                # 4. Sıralı hareket testi
                self.test_sequence_pattern()
            
            if not self.stop_event.is_set():
                input("\n⏸️ İnteraktif test için ENTER'a basın...")
                
                # 5. İnteraktif test
                self.interactive_multi_servo_test()
            
            print("\n🎉 TÜM AUX1234 SERVO TESTLERİ BAŞARILI!")
            return True
            
        except KeyboardInterrupt:
            print("\n⚠️ Test kullanıcı tarafından durduruldu")
            self.stop_event.set()
            return False
        except Exception as e:
            print(f"\n❌ Test hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik"""
        self.test_running = False
        self.stop_event.set()
        
        # Tüm servoları orta pozisyona getir
        if self.connected:
            try:
                print("\n🔄 Servolar orta pozisyona getiriliyor...")
                self.set_all_servos_pwm({name: PWM_MID for name in SERVO_CHANNELS})
                time.sleep(1)
            except:
                pass
        
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    multi_servo_test = MultiServoTest()
    
    print(f"🚀 TEKNOFEST AUX1234 Çoklu Servo Test - Frekans: {SERVO_FREQUENCY}Hz")
    print("=" * 80)
    print("AUX1234 Servo Test Menüsü:")
    print("1. Tam test paketi (Tüm testler)")
    print("2. Sadece bireysel servo testleri")
    print("3. Sadece senkronize hareket testi")
    print("4. Sadece dalga hareketi testi")
    print("5. Sadece sıralı hareket testi")
    print("6. İnteraktif çoklu servo test")
    
    try:
        choice = input("Seçiminiz (1-6): ").strip()
        
        if not multi_servo_test.connect_pixhawk():
            return 1
            
        if choice == '1':
            multi_servo_test.run_all_tests()
        elif choice == '2':
            multi_servo_test.test_individual_servos()
        elif choice == '3':
            multi_servo_test.test_synchronized_movement()
        elif choice == '4':
            multi_servo_test.test_wave_motion()
        elif choice == '5':
            multi_servo_test.test_sequence_pattern()
        elif choice == '6':
            multi_servo_test.interactive_multi_servo_test()
        else:
            print("Geçersiz seçim!")
            return 1
            
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1
    finally:
        multi_servo_test.cleanup()

if __name__ == "__main__":
    import sys
    sys.exit(main()) 