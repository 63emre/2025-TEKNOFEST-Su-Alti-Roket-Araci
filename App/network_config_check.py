#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Network Configuration Check
Raspberry Pi IP tespit ve MAVProxy TCP bağlantı kontrolü
"""

import subprocess
import socket
import time
import json
from pymavlink import mavutil

def get_local_ip():
    """Raspberry Pi'nin güncel IP adresini al"""
    try:
        # En güvenilir yöntem: external connection simülasyonu
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            local_ip = s.getsockname()[0]
        return local_ip
    except Exception as e:
        print(f"❌ IP tespit hatası (method 1): {e}")
        
    # Alternatif yöntem: ifconfig parsing
    try:
        result = subprocess.run(['hostname', '-I'], capture_output=True, text=True)
        ips = result.stdout.strip().split()
        # 192.168.x.x range'ini tercih et
        for ip in ips:
            if ip.startswith('192.168.'):
                return ip
        # Hiç yoksa ilk IP'yi döndür
        return ips[0] if ips else None
    except Exception as e:
        print(f"❌ IP tespit hatası (method 2): {e}")
        return None

def check_tcp_port(ip, port):
    """TCP port'un açık olup olmadığını kontrol et"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(3)
            result = s.connect_ex((ip, port))
            return result == 0  # 0 = başarılı bağlantı
    except Exception as e:
        print(f"❌ TCP port kontrol hatası: {e}")
        return False

def test_mavlink_connection(connection_string):
    """MAVLink TCP bağlantısını test et"""
    try:
        print(f"🔌 MAVLink test: {connection_string}")
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat(timeout=10)
        print("✅ MAVLink heartbeat alındı!")
        
        # Sistem bilgilerini al
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg:
            print(f"📊 Sistem durumu: {msg}")
        
        master.close()
        return True
        
    except Exception as e:
        print(f"❌ MAVLink test hatası: {e}")
        return False

def update_config_file(new_ip):
    """Config dosyasını yeni IP ile güncelle"""
    config_path = "config/hardware_config.json"
    
    try:
        with open(config_path, 'r') as f:
            config = json.load(f)
        
        # Hem localhost hem de yeni IP'yi test etmek için seçenekler
        localhost_connection = "tcp:127.0.0.1:5777"
        network_connection = f"tcp:{new_ip}:5777"
        
        print(f"\n🔧 Test edilen bağlantı seçenekleri:")
        print(f"1. Localhost: {localhost_connection}")
        print(f"2. Network: {network_connection}")
        
        # Her iki seçeneği de test et
        localhost_works = test_mavlink_connection(localhost_connection)
        network_works = test_mavlink_connection(network_connection)
        
        if localhost_works:
            print("✅ Localhost (127.0.0.1:5777) çalışıyor - config değişmiyor")
            chosen_connection = localhost_connection
        elif network_works:
            print(f"✅ Network ({new_ip}:5777) çalışıyor - config güncelleniyor")
            config["mavlink"]["connection_string"] = network_connection
            chosen_connection = network_connection
            
            # Config dosyasını güncelle
            with open(config_path, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"💾 Config dosyası güncellendi: {config_path}")
        else:
            print("❌ Her iki bağlantı da çalışmıyor!")
            print("💡 MAVProxy'nin çalıştığından emin olun")
            return None
        
        return chosen_connection
        
    except Exception as e:
        print(f"❌ Config güncelleme hatası: {e}")
        return None

def main():
    """Ana fonksiyon"""
    print("🚀 TEKNOFEST ROV - Network Configuration Check")
    print("=" * 50)
    
    # 1. Raspberry Pi IP'sini tespit et
    print("\n📡 1. IP Address Detection:")
    local_ip = get_local_ip()
    if local_ip:
        print(f"✅ Raspberry Pi IP: {local_ip}")
    else:
        print("❌ IP tespit edilemedi!")
        return
    
    # 2. TCP port kontrolü
    print("\n🔌 2. TCP Port Check:")
    localhost_port_open = check_tcp_port("127.0.0.1", 5777)
    network_port_open = check_tcp_port(local_ip, 5777)
    
    print(f"Localhost (127.0.0.1:5777): {'✅ AÇIK' if localhost_port_open else '❌ KAPALI'}")
    print(f"Network ({local_ip}:5777): {'✅ AÇIK' if network_port_open else '❌ KAPALI'}")
    
    # 3. MAVLink connection testi ve config güncellemesi
    print("\n🚁 3. MAVLink Connection Test & Config Update:")
    working_connection = update_config_file(local_ip)
    
    if working_connection:
        print(f"\n🎯 SONUÇ: Kullanılacak bağlantı: {working_connection}")
        print("✅ Network configuration hazır!")
        
        # Memory update için IP'yi kaydet
        with open("network_status.json", "w") as f:
            json.dump({
                "raspberry_pi_ip": local_ip,
                "working_connection": working_connection,
                "timestamp": time.time(),
                "localhost_works": localhost_port_open,
                "network_works": network_port_open
            }, f, indent=2)
        
    else:
        print("\n❌ HATA: Çalışan bağlantı bulunamadı!")
        print("💡 Çözüm adımları:")
        print("1. sudo systemctl status mavproxy")
        print("2. ps aux | grep mavproxy")
        print("3. netstat -tlnp | grep 5777")

if __name__ == "__main__":
    main() 