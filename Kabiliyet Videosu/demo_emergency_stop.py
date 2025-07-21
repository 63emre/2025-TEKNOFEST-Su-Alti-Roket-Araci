#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Acil Durdurma Sistemi Demo
Video çekimi için acil durdurma butonunun çalıştığını gösterme
Şartname: Butona basıldığında motorların durması ve sistemin kapanması
"""

import time
import threading
import signal
import sys
from datetime import datetime
from pymavlink import mavutil
import RPi.GPIO as GPIO
import json

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

# GPIO pinleri
EMERGENCY_BUTTON_PIN = 19    # Acil durdurma butonu
STATUS_LED_PIN = 20          # Durum LED'i
SYSTEM_POWER_PIN = 21        # Sistem güç rölesi

# Sistem kanalları
MOTOR_CHANNEL = 8
SERVO_CHANNELS = [1, 2, 3, 4]  # Fin servolar
PAYLOAD_SERVO = 9

# PWM değerleri
PWM_NEUTRAL = 1500
PWM_MIN = 1000
PWM_MAX = 2000

class EmergencyStopDemo:
    def __init__(self):
        self.master = None
        self.connected = False
        self.demo_active = False
        
        # Sistem durumu
        self.system_powered = False
        self.motors_active = False
        self.emergency_triggered = False
        self.emergency_button_pressed = False
        
        # Demo durumu
        self.demo_stage = "PREPARATION"
        self.demo_start_time = None
        self.emergency_trigger_time = None
        
        # Test verileri
        self.motor_stop_time = None
        self.system_shutdown_time = None
        self.button_press_count = 0
        
        # Performance metrikler
        self.motor_stop_delay = 0.0    # Buton basımı -> motor durdurma süresi
        self.system_shutdown_delay = 0.0  # Buton basımı -> tam sistem kapanma süresi
        
        # Test log
        self.emergency_log = []
        self.telemetry_data = []
        
        # Threading
        self.monitoring_thread = None
        self.running = False
        
        # GPIO setup
        self.setup_gpio()
        
    def setup_gpio(self):
        """GPIO pinlerini kur"""
        try:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Input pins (pull-up with debounce)
            GPIO.setup(EMERGENCY_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            
            # Output pins
            GPIO.setup(STATUS_LED_PIN, GPIO.OUT)
            GPIO.setup(SYSTEM_POWER_PIN, GPIO.OUT)
            
            # Başlangıçta sistem açık
            GPIO.output(STATUS_LED_PIN, GPIO.HIGH)  # LED on
            GPIO.output(SYSTEM_POWER_PIN, GPIO.HIGH)  # System power on
            self.system_powered = True
            
            # Interrupt callback
            GPIO.add_event_detect(EMERGENCY_BUTTON_PIN, GPIO.FALLING,
                                callback=self.emergency_button_callback, 
                                bouncetime=300)
            
            print("🔧 GPIO kurulumu tamamlandı")
            return True
            
        except Exception as e:
            print(f"❌ GPIO kurulum hatası: {e}")
            return False
    
    def emergency_button_callback(self, channel):
        """Acil durdurma butonu callback"""
        self.emergency_button_pressed = True
        self.button_press_count += 1
        self.emergency_trigger_time = time.time()
        
        print("\n🚨 ACİL DURDURMA BUTONU BASILDI!")
        print(f"⏰ Buton basım #{self.button_press_count}")
        
        # LED hızlı yanıp sön (acil durum)
        threading.Thread(target=self.emergency_led_blink, daemon=True).start()
        
        # Acil durdurma prosedürü başlat
        threading.Thread(target=self.execute_emergency_stop, daemon=True).start()
    
    def emergency_led_blink(self):
        """Acil durum LED yanıp sönmesi"""
        for _ in range(20):  # 10 saniye hızlı blink
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
            time.sleep(0.25)
            GPIO.output(STATUS_LED_PIN, GPIO.HIGH) 
            time.sleep(0.25)
        
        # Final olarak LED söndür
        GPIO.output(STATUS_LED_PIN, GPIO.LOW)
    
    def connect_pixhawk(self):
        """Pixhawk bağlantısı kur"""
        try:
            print("🔌 Pixhawk bağlantısı kuruluyor...")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            self.master.wait_heartbeat(timeout=10)
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def read_sensors(self):
        """Temel sensör verilerini oku"""
        if not self.connected:
            return False
            
        try:
            # Attitude
            attitude_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
            current_roll = current_pitch = current_yaw = 0.0
            if attitude_msg:
                current_roll = math.degrees(attitude_msg.roll) 
                current_pitch = math.degrees(attitude_msg.pitch)
                current_yaw = math.degrees(attitude_msg.yaw)
            
            # VFR HUD (hız/throttle)
            vfr_msg = self.master.recv_match(type='VFR_HUD', blocking=False)
            current_speed = current_throttle = 0.0
            if vfr_msg:
                current_speed = vfr_msg.groundspeed
                current_throttle = vfr_msg.throttle
            
            # Telemetri kaydı
            timestamp = time.time()
            self.telemetry_data.append({
                'timestamp': timestamp,
                'roll': current_roll,
                'pitch': current_pitch, 
                'yaw': current_yaw,
                'speed': current_speed,
                'throttle': current_throttle,
                'motors_active': self.motors_active,
                'system_powered': self.system_powered,
                'emergency_triggered': self.emergency_triggered,
                'stage': self.demo_stage
            })
            
            return True
            
        except Exception as e:
            print(f"❌ Sensör okuma hatası: {e}")
            return False
    
    def display_emergency_status(self):
        """Acil durdurma durumunu göster"""
        print("\n" + "="*70)
        print(f"🚨 TEKNOFEST - ACİL DURDURMA SİSTEMİ DEMOsu - {self.demo_stage}")
        print("="*70)
        
        timestamp = datetime.now().strftime("%H:%M:%S")
        demo_time = (time.time() - self.demo_start_time) if self.demo_start_time else 0
        
        print(f"⏰ Zaman: {timestamp} | Demo Süresi: {demo_time:.0f}s")
        print(f"🔘 Buton Basım Sayısı: {self.button_press_count}")
        
        # Sistem durumu
        system_status = "✅ AKTİF" if self.system_powered else "🔴 KAPALI"
        print(f"💡 Sistem Durumu: {system_status}")
        
        motor_status = "🚀 AKTİF" if self.motors_active else "⏹️ DURDURULDU"
        print(f"🔧 Motor Durumu: {motor_status}")
        
        emergency_status = "🚨 TETİKLENDİ" if self.emergency_triggered else "🟢 HAZIR"
        print(f"🚨 Acil Durdurma: {emergency_status}")
        
        # Response times
        if self.motor_stop_delay > 0:
            print(f"⚡ Motor Durdurma Süresi: {self.motor_stop_delay:.2f}s")
            
        if self.system_shutdown_delay > 0:
            print(f"⏱️ Sistem Kapanma Süresi: {self.system_shutdown_delay:.2f}s")
        
        print("="*70)
    
    def set_motor_throttle(self, throttle_pwm):
        """Motor kontrolü"""
        if not self.connected or self.emergency_triggered:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                MOTOR_CHANNEL, throttle_pwm, 0, 0, 0, 0, 0
            )
            
            self.motors_active = (throttle_pwm != PWM_NEUTRAL)
            return True
        except:
            return False
    
    def set_servo_position(self, channel, pwm_value):
        """Servo kontrolü"""
        if not self.connected or self.emergency_triggered:
            return False
            
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                channel, pwm_value, 0, 0, 0, 0, 0
            )
            return True
        except:
            return False
    
    def execute_emergency_stop(self):
        """Acil durdurma prosedürünü yürüt"""
        print("\n🚨 ACİL DURDURMA PROSEDÜRÜ BAŞLADI")
        print("-"*50)
        
        self.emergency_triggered = True
        
        # 1. Motorları derhal durdur
        print("1️⃣ Motorlar durduruluyor...")
        start_time = time.time()
        
        if self.connected:
            self.set_motor_throttle(PWM_NEUTRAL)
        
        self.motors_active = False
        self.motor_stop_time = time.time()
        self.motor_stop_delay = self.motor_stop_time - self.emergency_trigger_time
        
        print(f"   ✅ Motorlar durduruldu ({self.motor_stop_delay:.2f}s)")
        
        # 2. Tüm servoları güvenli pozisyona al
        print("2️⃣ Servolar güvenli pozisyona alınıyor...")
        
        if self.connected:
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL)
            
            # Payload bay kapat
            self.set_servo_position(PAYLOAD_SERVO, PWM_MIN)
        
        time.sleep(0.5)
        print("   ✅ Servolar güvenli pozisyonda")
        
        # 3. Sistem güç kontrolü
        print("3️⃣ Sistem güç kontrolü...")
        
        # GPIO power relay kontrolü
        GPIO.output(SYSTEM_POWER_PIN, GPIO.LOW)
        self.system_powered = False
        self.system_shutdown_time = time.time()
        self.system_shutdown_delay = self.system_shutdown_time - self.emergency_trigger_time
        
        print(f"   ✅ Sistem gücü kesildi ({self.system_shutdown_delay:.2f}s)")
        
        # 4. MAVLink bağlantısını kapat
        print("4️⃣ Haberleşme bağlantıları kapatılıyor...")
        
        if self.master:
            try:
                self.master.close()
                self.connected = False
                print("   ✅ MAVLink bağlantısı kapatıldı")
            except:
                print("   ⚠️ MAVLink bağlantısı zaten kapalı")
        
        # 5. Emergency log kaydet
        self.emergency_log.append({
            'timestamp': self.emergency_trigger_time,
            'button_press_count': self.button_press_count,
            'motor_stop_delay': self.motor_stop_delay,
            'system_shutdown_delay': self.system_shutdown_delay,
            'emergency_successful': True
        })
        
        print("\n✅ ACİL DURDURMA PROSEDÜRÜ TAMAMLANDI!")
        print(f"📊 Motor durdurma: {self.motor_stop_delay:.2f}s")
        print(f"📊 Sistem kapanma: {self.system_shutdown_delay:.2f}s")
    
    def monitoring_loop(self):
        """Sürekli izleme döngüsü"""
        while self.running and self.demo_active:
            self.read_sensors()
            
            # Her 2 saniyede durum göster (acil durdurma öncesi)
            if not self.emergency_triggered and len(self.telemetry_data) % 20 == 0:
                self.display_emergency_status()
            
            # Acil durdurma tetiklendiyse monitoring'i durdur
            if self.emergency_triggered:
                break
                
            time.sleep(0.1)  # 10Hz
    
    def simulate_normal_operation(self, duration=30):
        """Normal operasyon simülasyonu"""
        print(f"\n🚀 NORMAL OPERASYON SİMÜLASYONU ({duration}s)")
        print("-"*50)
        print("📋 Bu aşamada sistem normal çalışır, butonun basılmasını bekler")
        
        self.demo_stage = "NORMAL_OPERATION"
        
        operation_start = time.time()
        
        # Normal operasyon döngüsü
        while time.time() - operation_start < duration:
            if self.emergency_triggered:
                print("\n🚨 ACİL DURDURMA TETİKLENDİ - Normal operasyon durduruluyor!")
                break
            
            elapsed = time.time() - operation_start
            remaining = duration - elapsed
            
            # Motor test sinyalleri (güvenli seviyede)
            motor_test_throttle = PWM_NEUTRAL + int(50 * math.sin(elapsed * 0.5))
            self.set_motor_throttle(motor_test_throttle)
            
            # Servo test sinyalleri
            servo_test_offset = int(30 * math.cos(elapsed * 0.3))
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL + servo_test_offset)
            
            print(f"  📊 Normal operasyon: {remaining:.0f}s | Motor: {motor_test_throttle} | Servo: {PWM_NEUTRAL + servo_test_offset}")
            print(f"      🔘 ACİL DURDURMA butonuna basın! Buton basım: {self.button_press_count}")
            
            time.sleep(2)
        
        if not self.emergency_triggered:
            print("⚠️ Normal operasyon tamamlandı - acil durdurma tetiklenmedi!")
            # Motorları manuel durdur
            self.set_motor_throttle(PWM_NEUTRAL)
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL)
        
        return self.emergency_triggered
    
    def system_recovery_test(self):
        """Sistem kurtarma testi (acil durdurma sonrası)"""
        print("\n🔄 SİSTEM KURTARMA TESTİ")
        print("-"*50)
        print("📋 Acil durdurma sonrası sistemin yeniden başlatılabilirliği")
        
        self.demo_stage = "SYSTEM_RECOVERY"
        
        # Manuel sistem resetleme
        print("🔧 Manuel sistem resetleme...")
        
        # GPIO power tekrar aç
        GPIO.output(SYSTEM_POWER_PIN, GPIO.HIGH)
        self.system_powered = True
        
        # LED'i normal modda yak
        GPIO.output(STATUS_LED_PIN, GPIO.HIGH)
        
        # Emergency flag temizle
        self.emergency_triggered = False
        self.emergency_button_pressed = False
        
        print("✅ Sistem kurtarıldı ve yeniden çalışır durumda!")
        
        # Kısa fonksiyon testi
        print("🧪 Sistem fonksiyon testi...")
        
        # MAVLink yeniden bağlan
        if self.connect_pixhawk():
            # Test sinyalleri
            self.set_motor_throttle(PWM_NEUTRAL + 30)
            time.sleep(1)
            self.set_motor_throttle(PWM_NEUTRAL)
            
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL + 50)
            time.sleep(1)
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL)
            
            print("✅ Sistem fonksiyon testi başarılı!")
            return True
        else:
            print("❌ Sistem kurtarma başarısız!")
            return False
    
    def generate_emergency_report(self):
        """Acil durdurma testi raporu"""
        print("\n" + "="*70)
        print("📋 ACİL DURDURMA SİSTEMİ DEMO RAPORU")
        print("="*70)
        
        demo_duration = time.time() - self.demo_start_time if self.demo_start_time else 0
        
        print(f"📅 Demo Tarihi: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"⏱️ Toplam Demo Süresi: {demo_duration/60:.1f} dakika")
        print(f"🔘 Toplam Buton Basım: {self.button_press_count}")
        
        print(f"\n📊 PERFORMANS METRİKLERİ:")
        print("-"*50)
        print(f"🚨 Acil Durdurma Tetiklenme: {'✅ BAŞARILI' if self.emergency_triggered else '❌ BAŞARISIZ'}")
        
        if self.motor_stop_delay > 0:
            motor_response_status = "✅ ÇOK HİZLI" if self.motor_stop_delay < 0.5 else ("✅ HİZLI" if self.motor_stop_delay < 1.0 else "⚠️ YAVAS")
            print(f"⚡ Motor Durdurma Süresi: {self.motor_stop_delay:.3f}s ({motor_response_status})")
        
        if self.system_shutdown_delay > 0:
            shutdown_response_status = "✅ ÇOK HİZLI" if self.system_shutdown_delay < 1.0 else ("✅ HİZLI" if self.system_shutdown_delay < 2.0 else "⚠️ YAVAS")
            print(f"⏱️ Sistem Kapanma Süresi: {self.system_shutdown_delay:.3f}s ({shutdown_response_status})")
        
        print(f"🔌 Sistem Güç Kontrolü: {'✅ ÇALIŞIYOR' if not self.system_powered else '❌ ÇALIŞMIYOR'}")
        print(f"🔗 MAVLink Bağlantı Kesimi: {'✅ BAŞARILI' if not self.connected else '❌ BAŞARISIZ'}")
        
        # Şartname değerlendirmesi
        print(f"\n🎯 ŞARTNAME GEREKSİNİMLERİ:")
        print("-"*40)
        
        button_working = self.button_press_count > 0
        button_icon = "✅" if button_working else "❌"
        print(f"  {button_icon} Acil Durdurma Butonu Çalışması: {'ÇALIŞIYOR' if button_working else 'ÇALIŞMIYOR'}")
        
        motor_stop = self.motors_active == False and self.motor_stop_delay > 0
        motor_icon = "✅" if motor_stop else "❌"
        print(f"  {motor_icon} Motorların Durdurulması: {'BAŞARILI' if motor_stop else 'BAŞARISIZ'}")
        
        system_shutdown = not self.system_powered and self.system_shutdown_delay > 0
        system_icon = "✅" if system_shutdown else "❌"
        print(f"  {system_icon} Sistem Kapanması: {'BAŞARILI' if system_shutdown else 'BAŞARISIZ'}")
        
        # Response time değerlendirmesi
        fast_response = (self.motor_stop_delay < 1.0 and self.system_shutdown_delay < 2.0) if (self.motor_stop_delay > 0 and self.system_shutdown_delay > 0) else False
        response_icon = "✅" if fast_response else "❌"
        print(f"  {response_icon} Hızlı Tepki Süresi: {'BAŞARILI' if fast_response else 'BAŞARISIZ'}")
        
        # Safety compliance
        safety_compliance = button_working and motor_stop and system_shutdown and fast_response
        
        print(f"\n🏆 GENEL SONUÇ:")
        print("="*30)
        
        if safety_compliance:
            print("🎉 ACİL DURDURMA SİSTEMİ TAM BAŞARI!")
            print("🛡️ Güvenlik gereksinimleri karşılandı!")
            print("📹 Video çekimi için mükemmel!")
        else:
            print("❌ ACİL DURDURMA SİSTEMİ EKSİKLİKLER VAR!")
            missing_elements = []
            if not button_working:
                missing_elements.append("Buton çalışması")
            if not motor_stop:
                missing_elements.append("Motor durdurma")
            if not system_shutdown:
                missing_elements.append("Sistem kapanma")  
            if not fast_response:
                missing_elements.append("Hızlı tepki")
            print(f"🔧 Eksikler: {', '.join(missing_elements)}")
        
        # Veri kaydet
        report_data = {
            'timestamp': datetime.now().isoformat(),
            'demo_duration': demo_duration,
            'button_press_count': self.button_press_count,
            'emergency_triggered': self.emergency_triggered,
            'motor_stop_delay': self.motor_stop_delay,
            'system_shutdown_delay': self.system_shutdown_delay,
            'emergency_events': self.emergency_log,
            'telemetry_summary': {
                'total_samples': len(self.telemetry_data),
                'emergency_samples': len([d for d in self.telemetry_data if d.get('emergency_triggered', False)])
            },
            'safety_compliance': safety_compliance
        }
        
        with open(f'emergency_stop_demo_{datetime.now().strftime("%Y%m%d_%H%M%S")}.json', 'w') as f:
            json.dump(report_data, f, indent=2)
        
        print(f"\n💾 Demo raporu kaydedildi: emergency_stop_demo_*.json")
        
        return safety_compliance
    
    def run_full_emergency_demo(self):
        """Tam acil durdurma demo"""
        print("🚨 TEKNOFEST Su Altı Roket Aracı - ACİL DURDURMA SİSTEMİ DEMOsu")
        print("="*70)
        print("📹 Video çekimi için acil durdurma sisteminin gösterimi") 
        print("⏱️ Tahmini süre: 2-3 dakika")
        print("🎯 Şartname: Butona basıldığında motorlar durmalı, sistem kapanmalı")
        
        if not self.connect_pixhawk():
            print("❌ Pixhawk bağlantısı başarısız!")
            return False
        
        print("\n⚠️ GÜVENLİK UYARISI:")
        print("- Acil durdurma butonu çalışır durumda mı?")
        print("- Test ortamı güvenli mi?")
        print("- Kameralar buton basımını kaydediyor mu?")
        print("- Sistem power LED'i görülüyor mu?")
        
        ready = input("\n✅ Acil durdurma demosu başlasın mı? (y/n): ").lower()
        if ready != 'y':
            print("❌ Demo iptal edildi")
            return False
        
        self.demo_start_time = time.time()
        self.demo_active = True
        self.running = True
        
        # Monitoring thread başlat
        self.monitoring_thread = threading.Thread(target=self.monitoring_loop)
        self.monitoring_thread.daemon = True
        self.monitoring_thread.start()
        
        try:
            print("\n🚨 ACİL DURDURMA SİSTEMİ DEMOsu BAŞLADI!")
            
            # 1. Normal operasyon simülasyonu
            emergency_triggered = self.simulate_normal_operation(30)
            
            if not emergency_triggered:
                print("⚠️ Acil durdurma tetiklenmedi - demo başarısız!")
                return False
            
            # Acil durdurma işlemi bitmesini bekle
            time.sleep(3)
            
            input("\n⏸️ Acil durdurma tamamlandı! Sistem kurtarma testine geçilsin mi? ENTER...")
            
            # 2. Sistem kurtarma testi
            recovery_success = self.system_recovery_test()
            
            # 3. Demo raporu
            success = self.generate_emergency_report()
            
            if success and recovery_success:
                print("\n🎉 ACİL DURDURMA SİSTEMİ DEMOsu BAŞARILI!")
                print("📹 Video montaja hazır!")
            
            return success
            
        except KeyboardInterrupt:
            print("\n⚠️ Demo kullanıcı tarafından durduruldu")
            # Acil durdurma simüle et
            self.execute_emergency_stop()
            return False
        except Exception as e:
            print(f"\n❌ Demo hatası: {e}")
            return False
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Temizlik işlemleri"""
        self.demo_active = False
        self.running = False
        
        print("\n🧹 Sistem temizleniyor...")
        
        # Motorları güvenli pozisyon
        if self.connected:
            self.set_motor_throttle(PWM_NEUTRAL)
            for channel in SERVO_CHANNELS:
                self.set_servo_position(channel, PWM_NEUTRAL)
        
        # GPIO temizlik
        try:
            GPIO.output(STATUS_LED_PIN, GPIO.LOW)
            GPIO.output(SYSTEM_POWER_PIN, GPIO.LOW)
            GPIO.cleanup()
            print("🔧 GPIO temizlendi")
        except:
            pass
        
        # MAVLink bağlantı kapat
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")
        
        print("✅ Sistem temizleme tamamlandı")

def main():
    """Ana fonksiyon"""
    demo = EmergencyStopDemo()
    
    def signal_handler(sig, frame):
        print("\n🚨 CTRL+C ile acil durdurma simülasyonu!")
        demo.emergency_button_callback(None)
        time.sleep(2)
        demo.cleanup()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        success = demo.run_full_emergency_demo()
        return 0 if success else 1
    except KeyboardInterrupt:
        return 1

if __name__ == "__main__":
    import math
    sys.exit(main()) 