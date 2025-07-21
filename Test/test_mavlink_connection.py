#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - MAVLink Bağlantı Testi
Bu script Pixhawk ile MAVLink bağlantısını test eder
"""

from pymavlink import mavutil
import time
import sys

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

class MAVLinkTester:
    def __init__(self):
        self.master = None
        self.connection_status = False
        
    def connect_pixhawk(self):
        """Pixhawk ile bağlantı kur"""
        try:
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            
            # İlk heartbeat bekle (timeout 10sn)
            print("💓 Heartbeat bekleniyor...")
            self.master.wait_heartbeat(timeout=10)
            
            self.connection_status = True
            print("✅ MAVLink bağlantısı başarılı!")
            
            # Sistem bilgilerini al
            self.get_system_info()
            
            return True
            
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            return False
    
    def get_system_info(self):
        """Sistem bilgilerini görüntüle"""
        if not self.connection_status:
            print("❌ Bağlantı yok!")
            return
            
        print("\n📊 SİSTEM BİLGİLERİ:")
        print(f"Sistem ID: {self.master.target_system}")
        print(f"Komponent ID: {self.master.target_component}")
        print(f"MAVLink Versiyonu: {self.master.WIRE_PROTOCOL_VERSION}")
        
        # Autopilot bilgisi al
        try:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                print(f"Autopilot Tipi: {msg.autopilot}")
                print(f"Araç Tipi: {msg.type}")
                print(f"Sistem Durumu: {msg.system_status}")
                print(f"Custom Mode: {msg.custom_mode}")
        except:
            print("⚠️ Heartbeat alınamadı")
    
    def test_parameter_read(self):
        """Parametreleri okuma testi"""
        print("\n🔧 PARAMETRE OKUMA TESTİ:")
        
        try:
            # Parametre listesi iste
            self.master.mav.param_request_list_send(
                self.master.target_system,
                self.master.target_component
            )
            
            param_count = 0
            start_time = time.time()
            
            while time.time() - start_time < 10:  # 10sn timeout
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=False)
                if msg:
                    param_count += 1
                    if param_count <= 5:  # İlk 5 parametreyi göster
                        print(f"  {msg.param_id}: {msg.param_value}")
                
                if param_count > 0 and param_count == msg.param_count:
                    break
                    
                time.sleep(0.1)
            
            print(f"✅ Toplam {param_count} parametre okundu")
            
        except Exception as e:
            print(f"❌ Parametre okuma hatası: {e}")
    
    def test_sensor_data(self):
        """Sensör verilerini okuma testi"""
        print("\n📡 SENSÖR VERİLERİ TESTİ:")
        
        sensor_types = ['ATTITUDE', 'VFR_HUD', 'GPS_RAW_INT', 'SCALED_PRESSURE']
        sensor_data = {}
        
        start_time = time.time()
        
        while time.time() - start_time < 10:  # 10sn veri topla
            for sensor_type in sensor_types:
                msg = self.master.recv_match(type=sensor_type, blocking=False)
                if msg and sensor_type not in sensor_data:
                    sensor_data[sensor_type] = msg
            
            # Tüm sensörlerden veri geldi mi?
            if len(sensor_data) == len(sensor_types):
                break
                
            time.sleep(0.1)
        
        # Sonuçları göster
        for sensor_type, data in sensor_data.items():
            if sensor_type == 'ATTITUDE':
                print(f"  🧭 Attitude - Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
            elif sensor_type == 'VFR_HUD':
                print(f"  🚀 VFR HUD - Speed: {data.groundspeed:.1f} m/s, Alt: {data.alt:.1f}m")
            elif sensor_type == 'GPS_RAW_INT':
                print(f"  🛰️ GPS - Lat: {data.lat/1e7:.6f}, Lon: {data.lon/1e7:.6f}, Fix: {data.fix_type}")
            elif sensor_type == 'SCALED_PRESSURE':
                print(f"  🌡️ Pressure - Press: {data.press_abs:.1f} hPa, Temp: {data.temperature/100:.1f}°C")
        
        missing_sensors = set(sensor_types) - set(sensor_data.keys())
        if missing_sensors:
            print(f"⚠️ Eksik sensörler: {', '.join(missing_sensors)}")
    
    def test_command_send(self):
        """Komut gönderme testi"""
        print("\n📤 KOMUT GÖNDERME TESTİ:")
        
        try:
            # System ID request gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            
            # Response bekle
            msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            if msg:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("✅ Komut başarıyla gönderildi ve kabul edildi")
                else:
                    print(f"⚠️ Komut gönderildi ama sonuç: {msg.result}")
            else:
                print("❌ Komut response timeout")
                
        except Exception as e:
            print(f"❌ Komut gönderme hatası: {e}")
    
    def run_full_test(self):
        """Tam test süitini çalıştır"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - MAVLink Test Başlıyor")
        print("=" * 60)
        
        # 1. Bağlantı testi
        if not self.connect_pixhawk():
            print("❌ Bağlantı başarısız - Test sonlandırıldı")
            return False
        
        # 2. Parametre testi
        self.test_parameter_read()
        
        # 3. Sensör testi
        self.test_sensor_data()
        
        # 4. Komut testi
        self.test_command_send()
        
        print("\n" + "=" * 60)
        print("✅ MAVLink test tamamlandı!")
        
        return True
    
    def close_connection(self):
        """Bağlantıyı kapat"""
        if self.master:
            self.master.close()
            print("🔌 MAVLink bağlantısı kapatıldı")

def main():
    """Ana fonksiyon"""
    tester = MAVLinkTester()
    
    try:
        success = tester.run_full_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"\n❌ Beklenmeyen hata: {e}")
    finally:
        tester.close_connection()

if __name__ == "__main__":
    main() 