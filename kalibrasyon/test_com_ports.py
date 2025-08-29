#!/usr/bin/env python3
"""
COM Port Test Utility
Mevcut seri portları test eder ve erişim sorunlarını tespit eder
"""

import serial
import serial.tools.list_ports
import sys
import time

def list_available_ports():
    """Mevcut tüm seri portları listele"""
    print("🔍 Mevcut Seri Portlar:")
    print("-" * 50)
    
    ports = serial.tools.list_ports.comports()
    available_ports = []
    
    for port in ports:
        print(f"Port: {port.device}")
        print(f"  Açıklama: {port.description}")
        print(f"  Üretici: {port.manufacturer}")
        print(f"  VID:PID: {port.vid}:{port.pid}")
        print(f"  Seri No: {port.serial_number}")
        print()
        available_ports.append(port.device)
    
    return available_ports

def test_port_access(port_name, baud_rate=115200):
    """Belirli bir porta erişim testi yap"""
    print(f"🔧 {port_name} portuna erişim testi...")
    
    try:
        # Portu aç
        ser = serial.Serial(
            port=port_name,
            baudrate=baud_rate,
            timeout=1,
            write_timeout=1
        )
        
        print(f"✅ {port_name} portu başarıyla açıldı!")
        print(f"   Baud Rate: {ser.baudrate}")
        print(f"   Timeout: {ser.timeout}")
        
        # Kısa test
        time.sleep(0.1)
        
        # Portu kapat
        ser.close()
        print(f"✅ {port_name} portu başarıyla kapatıldı!")
        return True
        
    except serial.SerialException as e:
        if "Access is denied" in str(e) or "PermissionError" in str(e):
            print(f"❌ {port_name} - Erişim izni hatası!")
            print("   Çözüm önerileri:")
            print("   1. Diğer programları kapatın (Mission Planner, QGroundControl)")
            print("   2. Yönetici olarak çalıştırın")
            print("   3. USB kablosunu çıkarıp takın")
        elif "device reports readiness to read but returned no data" in str(e):
            print(f"⚠️ {port_name} - Port açık ama veri yok (normal olabilir)")
            return True
        else:
            print(f"❌ {port_name} - Seri port hatası: {e}")
        return False
        
    except Exception as e:
        print(f"❌ {port_name} - Beklenmeyen hata: {e}")
        return False

def test_mavlink_connection(port_name):
    """MAVLink bağlantısı testi"""
    print(f"🚁 {port_name} MAVLink bağlantı testi...")
    
    try:
        from pymavlink import mavutil
        
        # MAVLink bağlantısı kur
        mavlink = mavutil.mavlink_connection(port_name, baud=115200)
        
        print("Heartbeat bekleniyor... (5 saniye)")
        if mavlink.wait_heartbeat(timeout=5):
            print(f"✅ MAVLink heartbeat alındı!")
            print(f"   Sistem ID: {mavlink.target_system}")
            print(f"   Komponent ID: {mavlink.target_component}")
            return True
        else:
            print("⚠️ Heartbeat alınamadı (cihaz MAVLink göndermeyebilir)")
            return False
            
    except ImportError:
        print("❌ pymavlink modülü bulunamadı!")
        print("   Kurulum: pip install pymavlink")
        return False
    except Exception as e:
        print(f"❌ MAVLink bağlantı hatası: {e}")
        return False

def main():
    """Ana fonksiyon"""
    print("=" * 60)
    print("COM PORT TEST UTILITY")
    print("Seri Port Erişim ve MAVLink Testi")
    print("=" * 60)
    
    # Mevcut portları listele
    available_ports = list_available_ports()
    
    if not available_ports:
        print("❌ Hiç seri port bulunamadı!")
        return
    
    # Her portu test et
    working_ports = []
    for port in available_ports:
        print("\n" + "=" * 40)
        if test_port_access(port):
            working_ports.append(port)
            
            # MAVLink testi
            test_mavlink_connection(port)
    
    # Özet
    print("\n" + "=" * 60)
    print("TEST SONUÇLARI")
    print("=" * 60)
    print(f"Toplam Port: {len(available_ports)}")
    print(f"Erişilebilir: {len(working_ports)}")
    
    if working_ports:
        print("✅ Çalışan Portlar:")
        for port in working_ports:
            print(f"   - {port}")
    else:
        print("❌ Hiçbir port erişilebilir değil!")
        
    print("\n💡 COM17 erişim sorunları için:")
    print("1. Mission Planner, QGroundControl gibi programları kapatın")
    print("2. Terminal'i yönetici olarak çalıştırın")
    print("3. USB kablosunu çıkarıp tekrar takın")
    print("4. Windows Device Manager'da port ayarlarını kontrol edin")

if __name__ == "__main__":
    main()

