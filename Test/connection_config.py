#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Connection Configuration
Serial MAVLink Connection Helper
Environment Variable Support: MAV_ADDRESS, MAV_BAUD
"""

import os
import time
import platform
from pymavlink import mavutil

def get_default_connections():
    """Varsayılan bağlantı seçenekleri - Environment variables ile"""
    # Platform-based default port detection
    if platform.system() == "Windows":
        default_port = "COM17"
        alt_ports = ["COM17,115200", "COM19,115200", "COM20,115200", "COM21,115200"]
    else:
        # Linux/Unix systems
        default_port = "/dev/ttyACM0"
        alt_ports = ["/dev/ttyUSB0,115200", "/dev/ttyUSB1,115200", "/dev/ttyAMA0,115200"]
    
    # Environment variables'dan değerler al
    serial_port = os.getenv("MAV_ADDRESS", default_port)
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    
    connections = [
        f"{serial_port},{baud_rate}",  # Primary serial connection
    ] + alt_ports
    
    print(f"🔧 Default connections configured:")
    print(f"   Primary: {serial_port} @ {baud_rate} baud")
    print(f"   Environment: MAV_ADDRESS={serial_port}, MAV_BAUD={baud_rate}")
    
    return connections

def get_primary_connection():
    """Birincil bağlantı string'i döndür"""
    # Platform-based default port detection
    if platform.system() == "Windows":
        default_port = "COM17"
    else:
        # Linux/Unix systems
        default_port = "/dev/ttyACM0"
    
    serial_port = os.getenv("MAV_ADDRESS", default_port)
    baud_rate = int(os.getenv("MAV_BAUD", "115200"))
    return f"{serial_port},{baud_rate}"

def get_test_constants():
    """Test scriptleri için eski uyumluluk - serial connection döndür"""
    return {
        'MAV_ADDRESS': get_primary_connection()
    }

def create_mavlink_connection(connection_string=None, timeout=15):
    """Standart MAVLink bağlantısı oluştur - serial veya TCP destekli"""
    if connection_string is None:
        connection_string = get_primary_connection()
    
    try:
        print(f"🔌 MAVLink bağlantısı kuruluyor...")
        
        if ',' in connection_string:
            # Serial connection: port,baud
            port, baud = connection_string.split(',')
            print(f"📡 Serial: {port} @ {baud} baud")
            master = mavutil.mavlink_connection(port, baud=int(baud), autoreconnect=True)
        else:
            # TCP or other connection
            print(f"🌐 TCP: {connection_string}")
            master = mavutil.mavlink_connection(connection_string)
        
        print("💓 Heartbeat bekleniyor...")
        master.wait_heartbeat(timeout=timeout)
        print("✅ MAVLink bağlantısı başarılı!")
        
        return master
        
    except Exception as e:
        print(f"❌ MAVLink bağlantı hatası: {e}")
        print("💡 Kontrol listesi:")
        print("   • Pixhawk cihazının bağlı olduğunu kontrol edin")
        print("   • Serial port ve baud rate ayarlarını kontrol edin")
        print("   • ArduSub firmware'inin çalıştığını kontrol edin")
        raise

def test_connection(connection_string, timeout=5):
    """Bağlantıyı test et"""
    try:
        print(f"📡 Testing connection: {connection_string}")
        
        # Parse connection string
        if ',' in connection_string:
            # Serial connection
            port, baud = connection_string.split(',')
            master = mavutil.mavlink_connection(port, baud=int(baud))
        else:
            # TCP or other connection
            master = mavutil.mavlink_connection(connection_string)
        
        # Wait for heartbeat
        print(f"💓 Waiting for heartbeat (timeout: {timeout}s)...")
        heartbeat = master.wait_heartbeat(timeout=timeout)
        
        if heartbeat:
            print(f"✅ Connection successful!")
            print(f"   System ID: {master.target_system}")
            print(f"   Component ID: {master.target_component}")
            print(f"   Vehicle Type: {heartbeat.type}")
            master.close()
            return True
        else:
            print(f"❌ No heartbeat received")
            master.close()
            return False
            
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

def find_working_connection():
    """Çalışan bağlantıyı bul"""
    connections = get_default_connections()
    
    print("🔍 Searching for working MAVLink connection...")
    
    for connection in connections:
        if test_connection(connection, timeout=8):
            print(f"🎯 Found working connection: {connection}")
            return connection
    
    print("❌ No working connection found!")
    return None

def get_connection_config():
    """Connection config döndür"""
    working_connection = find_working_connection()
    
    if working_connection:
        config = {
            'connection_string': working_connection,
            'working': True,
            'address': working_connection
        }
        print(f"✅ Connection config: {config}")
        return config
    else:
        # Fallback to primary connection
        primary = get_primary_connection()
        config = {
            'connection_string': primary,
            'working': False,
            'address': primary
        }
        print(f"⚠️ Fallback config: {config}")
        return config

def main():
    """Test connection configuration"""
    print("🚀 TEKNOFEST ROV - Connection Configuration Test")
    print("=" * 50)
    
    # Environment variables
    print(f"🔧 Environment Variables:")
    
    # Platform-based default display
    if platform.system() == "Windows":
        default_display = "COM17"
    else:
        default_display = "/dev/ttyACM0"
    
    print(f"   MAV_ADDRESS = {os.getenv('MAV_ADDRESS', f'Not set (default: {default_display})')}")
    print(f"   MAV_BAUD = {os.getenv('MAV_BAUD', 'Not set (default: 115200)')}")
    print(f"   Platform: {platform.system()}")
    
    # Test primary connection
    primary = get_primary_connection()
    print(f"\n📡 Testing primary connection: {primary}")
    
    if test_connection(primary):
        print("✅ Primary connection working!")
    else:
        print("❌ Primary connection failed, searching alternatives...")
        working = find_working_connection()
        if working:
            print(f"✅ Alternative found: {working}")
        else:
            print("❌ No working connection found!")
    
    # Connection config summary
    config = get_connection_config()
    print(f"\n📋 Final configuration:")
    print(f"   Connection: {config['connection_string']}")
    print(f"   Status: {'Working' if config['working'] else 'Untested'}")

if __name__ == "__main__":
    main() 