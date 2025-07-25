#!/usr/bin/env python3
"""
TEKNOFEST Su Altı Roket Aracı - MAVLink Bağlantı Testi
Bu script Pixhawk ile MAVLink bağlantısını test eder
"""

from pymavlink import mavutil
import time
import sys
import socket
import threading

# MAVLink bağlantı adresi
MAV_ADDRESS = 'tcp:127.0.0.1:5777'

class MAVLinkTester:
    def __init__(self):
        self.master = None
        self.connection_status = False
        self.connection_timeout = 5  # 5 saniye timeout
        
    def check_connection_available(self):
        """Bağlantının mevcut olup olmadığını kontrol et"""
        try:
            if MAV_ADDRESS.startswith('tcp:'):
                # TCP bağlantısı için socket kontrolü
                parts = MAV_ADDRESS.replace('tcp:', '').split(':')
                host = parts[0]
                port = int(parts[1])
                
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(2)  # 2 saniye timeout
                result = sock.connect_ex((host, port))
                sock.close()
                
                if result == 0:
                    print(f"✅ TCP bağlantısı mevcut: {host}:{port}")
                    return True
                else:
                    print(f"❌ TCP bağlantısı mevcut değil: {host}:{port}")
                    return False
            else:
                # Serial port için farklı kontrol yapılabilir
                return True
                
        except Exception as e:
            print(f"⚠️ Bağlantı kontrolü hatası: {e}")
            return False
        
    def connect_pixhawk(self):
        """Pixhawk ile bağlantı kur"""
        try:
            # Önce bağlantının mevcut olup olmadığını kontrol et
            if not self.check_connection_available():
                print("❌ MAVLink hedefi ulaşılabilir değil!")
                return False
            
            print(f"🔌 Pixhawk'a bağlanılıyor: {MAV_ADDRESS}")
            
            # Timeout ile bağlantı oluştur
            try:
                self.master = mavutil.mavlink_connection(MAV_ADDRESS)
            except Exception as conn_error:
                print(f"❌ MAVLink bağlantısı oluşturulamadı: {conn_error}")
                return False
            
            # İlk heartbeat bekle (kısa timeout)
            print("💓 Heartbeat bekleniyor...")
            
            # Heartbeat'i thread ile bekle timeout için
            heartbeat_received = threading.Event()
            heartbeat_error = None
            
            def wait_for_heartbeat():
                nonlocal heartbeat_error
                try:
                    self.master.wait_heartbeat(timeout=self.connection_timeout)
                    heartbeat_received.set()
                except Exception as e:
                    heartbeat_error = e
                    heartbeat_received.set()
            
            heartbeat_thread = threading.Thread(target=wait_for_heartbeat)
            heartbeat_thread.daemon = True
            heartbeat_thread.start()
            
            # Timeout ile bekle
            if heartbeat_received.wait(timeout=self.connection_timeout + 1):
                if heartbeat_error:
                    print(f"❌ Heartbeat hatası: {heartbeat_error}")
                    self.master.close()
                    return False
                
                self.connection_status = True
                print("✅ MAVLink bağlantısı başarılı!")
                
                # Sistem bilgilerini al
                try:
                    self.get_system_info()
                except Exception as info_error:
                    print(f"⚠️ Sistem bilgisi alma hatası: {info_error}")
                
                return True
            else:
                print("❌ Heartbeat timeout - Pixhawk yanıt vermiyor")
                if self.master:
                    self.master.close()
                return False
            
        except Exception as e:
            print(f"❌ Genel bağlantı hatası: {e}")
            if self.master:
                try:
                    self.master.close()
                except:
                    pass
            return False
    
    def get_system_info(self):
        """Sistem bilgilerini görüntüle"""
        if not self.connection_status:
            print("❌ Bağlantı yok!")
            return
            
        try:
            print("\n📊 SİSTEM BİLGİLERİ:")
            print(f"Sistem ID: {self.master.target_system}")
            print(f"Komponent ID: {self.master.target_component}")
            print(f"MAVLink Versiyonu: {self.master.WIRE_PROTOCOL_VERSION}")
            
            # Autopilot bilgisi al
            try:
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
                if msg:
                    print(f"Autopilot Tipi: {msg.autopilot}")
                    print(f"Araç Tipi: {msg.type}")
                    print(f"Sistem Durumu: {msg.system_status}")
                    print(f"Custom Mode: {msg.custom_mode}")
                else:
                    print("⚠️ Heartbeat mesajı alınamadı")
            except Exception as hb_error:
                print(f"⚠️ Heartbeat alma hatası: {hb_error}")
        except Exception as e:
            print(f"⚠️ Sistem bilgisi hatası: {e}")
    
    def test_parameter_read(self):
        """Parametreleri okuma testi"""
        print("\n🔧 PARAMETRE OKUMA TESTİ:")
        
        if not self.connection_status:
            print("❌ Bağlantı yok!")
            return
        
        try:
            # Parametre listesi iste
            self.master.mav.param_request_list_send(
                self.master.target_system,
                self.master.target_component
            )
            
            param_count = 0
            start_time = time.time()
            timeout = 10
            
            while time.time() - start_time < timeout:
                try:
                    msg = self.master.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.1)
                    if msg:
                        param_count += 1
                        if param_count <= 5:  # İlk 5 parametreyi göster
                            print(f"  {msg.param_id}: {msg.param_value}")
                    
                    if param_count > 0 and hasattr(msg, 'param_count') and param_count == msg.param_count:
                        break
                        
                except Exception as recv_error:
                    print(f"⚠️ Parametre alma hatası: {recv_error}")
                    break
                    
                time.sleep(0.1)
            
            if param_count > 0:
                print(f"✅ Toplam {param_count} parametre okundu")
            else:
                print("⚠️ Hiç parametre okunamadı")
            
        except Exception as e:
            print(f"❌ Parametre okuma hatası: {e}")
    
    def test_sensor_data(self):
        """Sensör verilerini okuma testi"""
        print("\n📡 SENSÖR VERİLERİ TESTİ:")
        
        if not self.connection_status:
            print("❌ Bağlantı yok!")
            return
        
        sensor_types = ['ATTITUDE', 'VFR_HUD', 'GPS_RAW_INT', 'SCALED_PRESSURE']
        sensor_data = {}
        
        start_time = time.time()
        timeout = 10
        
        try:
            while time.time() - start_time < timeout:
                for sensor_type in sensor_types:
                    try:
                        msg = self.master.recv_match(type=sensor_type, blocking=False, timeout=0.1)
                        if msg and sensor_type not in sensor_data:
                            sensor_data[sensor_type] = msg
                    except Exception as sensor_error:
                        print(f"⚠️ {sensor_type} sensör hatası: {sensor_error}")
                
                # Tüm sensörlerden veri geldi mi?
                if len(sensor_data) == len(sensor_types):
                    break
                    
                time.sleep(0.1)
            
            # Sonuçları göster
            for sensor_type, data in sensor_data.items():
                try:
                    if sensor_type == 'ATTITUDE':
                        print(f"  🧭 Attitude - Roll: {data.roll:.2f}, Pitch: {data.pitch:.2f}, Yaw: {data.yaw:.2f}")
                    elif sensor_type == 'VFR_HUD':
                        print(f"  🚀 VFR HUD - Speed: {data.groundspeed:.1f} m/s, Alt: {data.alt:.1f}m")
                    elif sensor_type == 'GPS_RAW_INT':
                        print(f"  🛰️ GPS - Lat: {data.lat/1e7:.6f}, Lon: {data.lon/1e7:.6f}, Fix: {data.fix_type}")
                    elif sensor_type == 'SCALED_PRESSURE':
                        print(f"  🌡️ Pressure - Press: {data.press_abs:.1f} hPa, Temp: {data.temperature/100:.1f}°C")
                except Exception as display_error:
                    print(f"  ⚠️ {sensor_type} - Veri görüntüleme hatası: {display_error}")
            
            missing_sensors = set(sensor_types) - set(sensor_data.keys())
            if missing_sensors:
                print(f"⚠️ Eksik sensörler: {', '.join(missing_sensors)}")
            
        except Exception as e:
            print(f"❌ Sensör testi hatası: {e}")
    
    def test_command_send(self):
        """Komut gönderme testi"""
        print("\n📤 KOMUT GÖNDERME TESTİ:")
        
        if not self.connection_status:
            print("❌ Bağlantı yok!")
            return
        
        try:
            # System ID request gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            
            # Response bekle
            try:
                msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
                if msg:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        print("✅ Komut başarıyla gönderildi ve kabul edildi")
                    else:
                        print(f"⚠️ Komut gönderildi ama sonuç: {msg.result}")
                else:
                    print("❌ Komut response timeout")
            except Exception as response_error:
                print(f"❌ Komut response hatası: {response_error}")
                
        except Exception as e:
            print(f"❌ Komut gönderme hatası: {e}")
    
    def run_full_test(self):
        """Tam test süitini çalıştır"""
        print("🚀 TEKNOFEST Su Altı Roket Aracı - MAVLink Test Başlıyor")
        print("=" * 60)
        
        # 1. Bağlantı testi
        if not self.connect_pixhawk():
            print("❌ Bağlantı başarısız - Test sonlandırıldı")
            print("\n🔧 ÇÖZÜM ÖNERİLERİ:")
            print("   1. SITL/Simulator çalışıyor mu? (Mission Planner, SITL vb.)")
            print("   2. MAVProxy çalışıyor mu? mavproxy.py --master=/dev/ttyUSB0 --out=tcpin:0.0.0.0:5777")
            print("   3. Bağlantı adresi doğru mu? (şu an: " + MAV_ADDRESS + ")")
            print("   4. Firewall MAVLink portunu engelliyor mu?")
            return False
        
        try:
            # 2. Parametre testi
            self.test_parameter_read()
            
            # 3. Sensör testi  
            self.test_sensor_data()
            
            # 4. Komut testi
            self.test_command_send()
            
        except Exception as test_error:
            print(f"⚠️ Test sırasında hata: {test_error}")
        
        print("\n" + "=" * 60)
        print("✅ MAVLink test tamamlandı!")
        
        return True
    
    def close_connection(self):
        """Bağlantıyı kapat"""
        try:
            if self.master:
                self.master.close()
                print("🔌 MAVLink bağlantısı kapatıldı")
        except Exception as e:
            print(f"⚠️ Bağlantı kapatma hatası: {e}")

def main():
    """Ana fonksiyon"""
    tester = MAVLinkTester()
    
    try:
        success = tester.run_full_test()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⚠️ Test kullanıcı tarafından durduruldu (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ Beklenmeyen hata: {e}")
        import traceback
        traceback.print_exc()
    finally:
        tester.close_connection()

if __name__ == "__main__":
    main() 