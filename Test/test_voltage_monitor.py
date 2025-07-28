#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - Voltaj Monitör
AUX/Main channel voltaj ölçümü ve arm durumu kontrolü
"""

import time
import csv
from datetime import datetime
from pymavlink import mavutil
import threading

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

class VoltageMonitor:
    def __init__(self):
        self.master = None
        self.connected = False
        self.monitoring = False
        self.log_file = None
        
        # Voltage readings
        self.battery_voltage = 0.0
        self.servo_voltage = 0.0
        self.system_voltage = 0.0
        self.is_armed = False
        self.flight_mode = "UNKNOWN"
        
    def connect_pixhawk(self):
        """Pixhawk bağlantısı"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor...")
            
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
            
            # Request data streams
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,  # 10 Hz
                1    # Enable
            )
            
            self.connected = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def read_voltage_data(self):
        """Voltaj verilerini oku"""
        if not self.connected:
            return False
            
        try:
            # MAVLink mesajlarını oku
            msg = self.master.recv_match(blocking=False)
            
            if msg is not None:
                msg_type = msg.get_type()
                
                # Battery voltage
                if msg_type == 'SYS_STATUS':
                    self.battery_voltage = msg.voltage_battery / 1000.0  # mV to V
                    self.system_voltage = msg.voltage_battery / 1000.0
                    
                # Power status
                elif msg_type == 'POWER_STATUS':
                    self.servo_voltage = msg.Vservo / 1000.0  # mV to V
                    
                # Heartbeat - arm durumu
                elif msg_type == 'HEARTBEAT':
                    self.is_armed = (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
                    self.flight_mode = mavutil.mode_string_v10(msg)
                    
                # Alternative voltage reading
                elif msg_type == 'BATTERY_STATUS':
                    if len(msg.voltages) > 0:
                        self.battery_voltage = msg.voltages[0] / 1000.0  # mV to V
                        
            return True
            
        except Exception as e:
            print(f"⚠️ Veri okuma hatası: {e}")
            return False
    
    def arm_vehicle(self):
        """Vehicle'ı arm et"""
        try:
            print("🔓 Vehicle arm ediliyor...")
            
            # Arm komutu gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,  # Arm = 1, Disarm = 0
                0, 0, 0, 0, 0, 0
            )
            
            # Response bekle
            response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if response:
                if response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("✅ Vehicle başarıyla arm edildi!")
                    return True
                else:
                    print(f"❌ Arm başarısız: {response.result}")
            else:
                print("❌ Arm komutu timeout!")
                
        except Exception as e:
            print(f"❌ Arm hatası: {e}")
            
        return False
    
    def disarm_vehicle(self):
        """Vehicle'ı disarm et"""
        try:
            print("🔒 Vehicle disarm ediliyor...")
            
            # Disarm komutu gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,  # Arm = 1, Disarm = 0
                0, 0, 0, 0, 0, 0
            )
            
            # Response bekle
            response = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if response:
                if response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("✅ Vehicle başarıyla disarm edildi!")
                    return True
                else:
                    print(f"❌ Disarm başarısız: {response.result}")
            else:
                print("❌ Disarm komutu timeout!")
                
        except Exception as e:
            print(f"❌ Disarm hatası: {e}")
            
        return False
    
    def start_logging(self, filename=None):
        """Voltaj loglamayı başlat"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"voltage_log_{timestamp}.csv"
            
        try:
            self.log_file = open(filename, 'w', newline='')
            writer = csv.writer(self.log_file)
            
            # CSV header
            writer.writerow([
                'Timestamp',
                'Battery_Voltage_V', 
                'Servo_Voltage_V',
                'System_Voltage_V',
                'Is_Armed',
                'Flight_Mode'
            ])
            
            print(f"📝 Loglama başlatıldı: {filename}")
            return True
            
        except Exception as e:
            print(f"❌ Log dosyası açma hatası: {e}")
            return False
    
    def log_voltage_data(self):
        """Voltaj verilerini log dosyasına yaz"""
        if self.log_file:
            try:
                writer = csv.writer(self.log_file)
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                
                writer.writerow([
                    timestamp,
                    f"{self.battery_voltage:.3f}",
                    f"{self.servo_voltage:.3f}", 
                    f"{self.system_voltage:.3f}",
                    "YES" if self.is_armed else "NO",
                    self.flight_mode
                ])
                
                self.log_file.flush()  # Immediately write to file
                return True
                
            except Exception as e:
                print(f"❌ Log yazma hatası: {e}")
                return False
        return False
    
    def print_voltage_status(self):
        """Voltaj durumunu ekrana yazdır"""
        arm_status = "🔓 ARMED" if self.is_armed else "🔒 DISARMED"
        
        print(f"\r🔋 Battery: {self.battery_voltage:.2f}V | "
              f"⚡ Servo: {self.servo_voltage:.2f}V | "
              f"💻 System: {self.system_voltage:.2f}V | "
              f"{arm_status} | Mode: {self.flight_mode}", end="")
    
    def monitor_voltages(self, duration=60, log_enable=True):
        """Voltajları monitor et"""
        print(f"\n📊 VOLTAJ MONİTÖRÜ - {duration} saniye")
        print("-" * 60)
        
        if log_enable:
            if not self.start_logging():
                print("⚠️ Loglama başlatılamadı, sadece ekran çıkışı")
                log_enable = False
        
        self.monitoring = True
        start_time = time.time()
        
        try:
            while self.monitoring and (time.time() - start_time) < duration:
                # Veri oku
                self.read_voltage_data()
                
                # Ekrana yazdır
                self.print_voltage_status()
                
                # Log dosyasına yaz
                if log_enable:
                    self.log_voltage_data()
                
                time.sleep(0.5)  # 2Hz update
                
        except KeyboardInterrupt:
            print("\n⚠️ Monitoring kullanıcı tarafından durduruldu")
            
        finally:
            self.monitoring = False
            if self.log_file:
                self.log_file.close()
                print(f"\n📝 Log dosyası kapatıldı")
    
    def voltage_test_with_arm(self):
        """Arm öncesi/sonrası voltaj karşılaştırması"""
        print("\n🧪 ARM DURUMU VOLTAJ TESTİ")
        print("=" * 50)
        
        # Başlangıç readings
        print("📊 Başlangıç voltaj ölçümleri (DISARMED):")
        for i in range(5):
            self.read_voltage_data()
            self.print_voltage_status()
            time.sleep(1)
        
        disarmed_battery = self.battery_voltage
        disarmed_servo = self.servo_voltage
        
        print(f"\n📋 DISARMED Ortalama:")
        print(f"   Battery: {disarmed_battery:.2f}V")
        print(f"   Servo:   {disarmed_servo:.2f}V")
        
        # Arm et
        input("\n⏸️ ARM etmek için ENTER'a basın...")
        
        if self.arm_vehicle():
            time.sleep(2)  # Arm işleminin tamamlanması için bekle
            
            print("\n📊 ARM durumu voltaj ölçümleri:")
            for i in range(5):
                self.read_voltage_data()
                self.print_voltage_status()
                time.sleep(1)
            
            armed_battery = self.battery_voltage
            armed_servo = self.servo_voltage
            
            print(f"\n📋 ARMED Ortalama:")
            print(f"   Battery: {armed_battery:.2f}V")
            print(f"   Servo:   {armed_servo:.2f}V")
            
            print(f"\n📊 VOLTAJ FARKI:")
            print(f"   Battery: {armed_battery - disarmed_battery:+.2f}V")
            print(f"   Servo:   {armed_servo - disarmed_servo:+.2f}V")
            
            # Disarm et
            input("\n⏸️ DISARM etmek için ENTER'a basın...")
            self.disarm_vehicle()
        
        print("✅ ARM durumu voltaj testi tamamlandı!")
    
    def cleanup(self):
        """Temizlik"""
        self.monitoring = False
        
        if self.log_file:
            self.log_file.close()
            
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    monitor = VoltageMonitor()
    
    if not monitor.connect_pixhawk():
        return 1
    
    print("\nVoltaj Monitor Menüsü:")
    print("1. Sürekli voltaj monitoring (60s)")
    print("2. ARM durumu voltaj testi")
    print("3. Manuel ARM/DISARM")
    print("4. Anlık voltaj okuma")
    
    try:
        choice = input("Seçiminiz (1-4): ").strip()
        
        if choice == '1':
            duration = input("Monitoring süresi (saniye, default 60): ").strip()
            duration = int(duration) if duration.isdigit() else 60
            monitor.monitor_voltages(duration)
            
        elif choice == '2':
            monitor.voltage_test_with_arm()
            
        elif choice == '3':
            action = input("ARM için 'a', DISARM için 'd': ").strip().lower()
            if action == 'a':
                monitor.arm_vehicle()
            elif action == 'd':
                monitor.disarm_vehicle()
            else:
                print("Geçersiz seçim!")
                
        elif choice == '4':
            for i in range(10):
                monitor.read_voltage_data()
                monitor.print_voltage_status()
                time.sleep(1)
            print()
            
        else:
            print("Geçersiz seçim!")
            return 1
            
        return 0
        
    except KeyboardInterrupt:
        print("\n⚠️ Program sonlandırıldı")
        return 1
    finally:
        monitor.cleanup()

if __name__ == "__main__":
    import sys
    sys.exit(main()) 