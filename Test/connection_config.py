#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Test Scripts Connection Config Helper
Tüm test scriptleri için ortak network bağlantı ayarları
"""

import json
import os
import sys
import socket
from pymavlink import mavutil

def get_working_connection():
    """Çalışan MAVLink bağlantısını tespit et"""
    
    # 1. Network status dosyasından kontrol et
    network_status_file = "../App/network_status.json"
    if os.path.exists(network_status_file):
        try:
            with open(network_status_file, 'r') as f:
                status = json.load(f)
            working_connection = status.get('working_connection')
            if working_connection:
                print(f"📡 Kaydedilmiş bağlantı bulundu: {working_connection}")
                return working_connection
        except Exception as e:
            print(f"⚠️ Network status dosyası okunamadı: {e}")
    
    # 2. Config dosyasından kontrol et
    config_file = "../App/config/hardware_config.json"
    if os.path.exists(config_file):
        try:
            with open(config_file, 'r') as f:
                config = json.load(f)
            config_connection = config.get('mavlink', {}).get('connection_string')
            if config_connection:
                print(f"📡 Config bağlantısı bulundu: {config_connection}")
                return config_connection
        except Exception as e:
            print(f"⚠️ Config dosyası okunamadı: {e}")
    
    # 3. Default connections'ları test et
    default_connections = [
        "tcp:127.0.0.1:5777",
        "tcp:192.168.137.96:5777",  # Pi IP from memory
        "tcp:192.168.1.100:5777",  # Alternative Pi IP
        "tcp:192.168.0.100:5777"   # Alternative Pi IP
    ]
    
    print("🔍 Default bağlantıları test ediliyor...")
    for connection in default_connections:
        if test_mavlink_connection(connection, timeout=5):
            print(f"✅ Çalışan bağlantı bulundu: {connection}")
            return connection
    
    # 4. Hiçbiri çalışmıyorsa localhost'u döndür
    print("⚠️ Çalışan bağlantı bulunamadı, localhost varsayılan olarak kullanılacak")
    return "tcp:127.0.0.1:5777"

def test_mavlink_connection(connection_string, timeout=5):
    """MAVLink bağlantısını hızlı test et"""
    try:
        master = mavutil.mavlink_connection(connection_string)
        master.wait_heartbeat(timeout=timeout)
        master.close()
        return True
    except:
        return False

def get_tcp_config():
    """Test scriptleri için TCP konfigürasyonu"""
    working_connection = get_working_connection()
    
    # Connection string'den IP ve port'u ayır
    if "tcp:" in working_connection:
        tcp_part = working_connection.replace("tcp:", "")
        if ":" in tcp_part:
            ip, port = tcp_part.split(":")
            return {
                'connection_string': working_connection,
                'ip': ip,
                'port': int(port),
                'address': working_connection
            }
    
    # Fallback
    return {
        'connection_string': 'tcp:127.0.0.1:5777',
        'ip': '127.0.0.1',
        'port': 5777,
        'address': 'tcp:127.0.0.1:5777'
    }

def test_tcp_connection():
    """Mevcut TCP konfigürasyonunu test et"""
    config = get_tcp_config()
    
    print(f"🧪 TCP Test: {config['connection_string']}")
    
    # Port kontrolü
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(3)
            result = s.connect_ex((config['ip'], config['port']))
            port_open = (result == 0)
    except:
        port_open = False
    
    print(f"🔌 Port Status: {'✅ OPEN' if port_open else '❌ CLOSED'}")
    
    # MAVLink testi
    mavlink_works = test_mavlink_connection(config['connection_string'], timeout=8)
    print(f"🚁 MAVLink Status: {'✅ CONNECTED' if mavlink_works else '❌ FAILED'}")
    
    return {
        'config': config,
        'port_open': port_open,
        'mavlink_works': mavlink_works,
        'overall_status': port_open and mavlink_works
    }

# Test scriptleri için sabitler
def get_test_constants():
    """Test scriptleri için ortak sabitler"""
    config = get_tcp_config()
    
    return {
        # Connection
        'MAV_ADDRESS': config['connection_string'],
        'TCP_IP': config['ip'],
        'TCP_PORT': config['port'],
        
        # Servo Channels (GERÇEK X-Configuration)
        'SERVO_CHANNELS': {
            'fin_front_left': 1,   # AUX 1 → MAVLink 9
            'fin_front_right': 3,  # AUX 3 → MAVLink 11  
            'fin_rear_left': 4,    # AUX 4 → MAVLink 12
            'fin_rear_right': 5    # AUX 5 → MAVLink 13
        },
        
        # PWM Values
        'PWM_MIN': 1000,
        'PWM_MID': 1500,
        'PWM_MAX': 2000,
        'PWM_DEADBAND': 50,
        
        # X-Wing Configuration
        'X_WING_CONFIG': {
            'front_left': 'aux1',   # Ön Sol
            'rear_left': 'aux3',    # Arka Sol
            'rear_right': 'aux4',   # Arka Sağ
            'extra': 'aux5'         # Ekstra kontrol
        },
        
        # Servo Mapping (GERÇEK AUX to MAVLink channel)
        'AUX_TO_MAVLINK': {
            'aux1': 9,   # AUX 1 = Ön Sol Fin (MAVLink 9)
            'aux3': 11,  # AUX 3 = Ön Sağ Fin (MAVLink 11)
            'aux4': 12,  # AUX 4 = Arka Sol Fin (MAVLink 12)
            'aux5': 13,  # AUX 5 = Arka Sağ Fin (MAVLink 13)
            'aux6': 14   # AUX 6 = Ana Motor (MAVLink 14)
        },
        
        # GERÇEK X-Wing Hardware Layout
        'REAL_HARDWARE': {
            'front_left_aux1': 9,   # Ön Sol
            'front_right_aux3': 11, # Ön Sağ
            'rear_left_aux4': 12,   # Arka Sol
            'rear_right_aux5': 13,  # Arka Sağ
            'motor_aux6': 14        # Ana Motor
        }
    }

if __name__ == "__main__":
    """Connection test script"""
    print("🚀 TEKNOFEST ROV - Connection Configuration Test")
    print("=" * 50)
    
    # Test konfigürasyonu
    test_result = test_tcp_connection()
    
    if test_result['overall_status']:
        print("\n✅ BİR SONUÇ: Connection hazır, test scriptleri çalıştırılabilir!")
        print(f"📡 Address: {test_result['config']['connection_string']}")
    else:
        print("\n❌ PROBLEM: Connection çalışmıyor!")
        print("💡 Çözüm adımları:")
        print("1. cd ../App && python network_config_check.py")
        print("2. sudo systemctl status mavproxy")
        print("3. MAVProxy TCP server'ın çalıştığını kontrol edin")
    
    # Test sabitleri
    print(f"\n📋 Test Constants:")
    constants = get_test_constants()
    print(f"MAV_ADDRESS: {constants['MAV_ADDRESS']}")
    print(f"SERVO_CHANNELS: {constants['SERVO_CHANNELS']}")
    print(f"PWM_RANGE: {constants['PWM_MIN']}-{constants['PWM_MAX']}") 