#!/usr/bin/env python3
"""
TCP 5777 MAVLink Bağlantı Testi
Socat köprüsünün çalışıp çalışmadığını test et
"""

import socket
import time
import sys

def test_tcp_port():
    """TCP 5777 portunu test et"""
    print("🔌 TCP Port 5777 testi...")
    
    try:
        # Socket oluştur
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        
        # Bağlantı dene
        result = sock.connect_ex(('127.0.0.1', 5777))
        
        if result == 0:
            print("✅ TCP 5777 portu açık ve dinliyor")
            
            # Basit veri gönder/al testi
            try:
                sock.send(b'ping')
                sock.settimeout(2)
                data = sock.recv(1024)
                print(f"📡 Veri alındı: {len(data)} byte")
                return True
            except socket.timeout:
                print("⚠️ Port açık ama veri alınamadı (normal olabilir)")
                return True
            except Exception as e:
                print(f"⚠️ Veri test hatası: {e}")
                return True
                
        else:
            print("❌ TCP 5777 portu kapalı veya erişilemiyor")
            return False
            
    except Exception as e:
        print(f"❌ TCP test hatası: {e}")
        return False
    finally:
        sock.close()

def test_socat_service():
    """Socat service durumunu kontrol et"""
    print("\n🔧 Socat service testi...")
    
    import subprocess
    try:
        # Systemctl status
        result = subprocess.run(['systemctl', 'is-active', 'mavtcp'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0 and 'active' in result.stdout:
            print("✅ mavtcp service aktif")
        else:
            print("❌ mavtcp service aktif değil")
            print("💡 Çözüm: sudo systemctl restart mavtcp")
            
        # Port dinleme kontrolü
        result = subprocess.run(['ss', '-tlnp'], capture_output=True, text=True)
        if ':5777' in result.stdout:
            print("✅ Port 5777 dinleniyor")
            # Hangi process dinliyor
            for line in result.stdout.split('\n'):
                if ':5777' in line:
                    print(f"📡 Process: {line}")
        else:
            print("❌ Port 5777 dinlenmiyor")
            
    except Exception as e:
        print(f"⚠️ Service kontrol hatası: {e}")

def test_mavlink_connection():
    """MAVLink bağlantısını test et"""
    print("\n📡 MAVLink connection testi...")
    
    try:
        from pymavlink import mavutil
        
        # MAVLink bağlantısı dene
        print("🔌 MAVLink bağlantısı kuruluyor...")
        master = mavutil.mavlink_connection('tcp:127.0.0.1:5777', timeout=10)
        
        print("⏳ Heartbeat bekleniyor...")
        master.wait_heartbeat(timeout=15)
        
        print("✅ MAVLink heartbeat alındı!")
        print(f"📡 System ID: {master.target_system}")
        print(f"📡 Component ID: {master.target_component}")
        
        # IMU verisi test et
        print("\n📊 IMU veri testi (5 saniye)...")
        for i in range(5):
            msg = master.recv_match(type='RAW_IMU', blocking=True, timeout=2)
            if msg:
                print(f"   IMU {i+1}: xacc={msg.xacc}, yacc={msg.yacc}, zacc={msg.zacc}")
            else:
                print(f"   IMU {i+1}: Veri yok")
            time.sleep(1)
            
        master.close()
        return True
        
    except ImportError:
        print("❌ pymavlink modülü yok - venv aktif mi?")
        print("💡 Çözüm: source venv/bin/activate")
        return False
    except Exception as e:
        print(f"❌ MAVLink test hatası: {e}")
        print("💡 Pixhawk bağlı mı? Socat çalışıyor mu?")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🧪 TCP MAVLink Köprü Testi\n")
    
    results = []
    
    # TCP port testi
    results.append(("TCP Port", test_tcp_port()))
    
    # Socat service testi  
    test_socat_service()
    
    # MAVLink testi
    results.append(("MAVLink", test_mavlink_connection()))
    
    # Sonuç özeti
    print("\n" + "="*50)
    print("📊 TEST SONUÇLARI:")
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"  {test_name:15} : {status}")
    
    if all(result for _, result in results):
        print("\n🚀 Sistem hazır! Terminal GUI çalıştırılabilir")
    else:
        print("\n⚠️ Sorunlar var. Aşağıdaki adımları dene:")
        print("  1. sudo systemctl restart mavtcp")
        print("  2. bash pixhawk_port_check.sh  # Doğru port kontrolü")
        print("  3. source venv/bin/activate    # Python venv")

if __name__ == "__main__":
    main() 