#!/usr/bin/env python3
"""
Pixhawk Direct Serial Test
/dev/ttyACM0 üzerinden direkt veri kontrol et
"""

import serial
import time
import sys

def test_serial_connection():
    """Pixhawk serial bağlantısını test et"""
    print("🔌 Pixhawk Direct Serial Test...")
    
    # Farklı baud rate'leri test et
    baud_rates = [115200, 57600, 38400, 19200]
    
    for baud in baud_rates:
        print(f"\n📡 Baud rate {baud} test ediliyor...")
        
        try:
            # Serial port aç
            ser = serial.Serial('/dev/ttyACM0', baud, timeout=3)
            print(f"✅ Port açıldı: {baud} baud")
            
            # 5 saniye veri dinle
            print("👂 5 saniye veri dinleniyor...")
            data_received = False
            
            for i in range(50):  # 50 x 0.1s = 5s
                if ser.in_waiting > 0:
                    data = ser.read(ser.in_waiting)
                    print(f"📡 Veri alındı ({baud}): {len(data)} byte")
                    print(f"   Raw: {data[:50]}...")  # İlk 50 byte göster
                    
                    # MAVLink magic bytes kontrol et
                    if b'\xfe' in data or b'\xfd' in data:
                        print("🎯 MAVLink paketi tespit edildi!")
                    
                    data_received = True
                    break
                
                time.sleep(0.1)
            
            ser.close()
            
            if data_received:
                print(f"✅ {baud} baud'da veri var!")
                return baud
            else:
                print(f"❌ {baud} baud'da veri yok")
                
        except Exception as e:
            print(f"❌ {baud} baud hatası: {e}")
    
    return None

def test_pixhawk_status():
    """Pixhawk durumunu kontrol et"""
    print("\n🔍 Pixhawk Durum Kontrolü...")
    
    try:
        # USB device durumu
        import subprocess
        result = subprocess.run(['lsusb', '-v', '-d', '1209:5741'], 
                              capture_output=True, text=True, timeout=10)
        
        if 'Pixhawk1-BL' in result.stdout:
            print("⚠️ Pixhawk BOOTLOADER modunda!")
            print("💡 Firmware yüklenmesi gerekiyor")
            return "bootloader"
        elif 'Pixhawk1' in result.stdout:
            print("✅ Pixhawk normal modda")
            return "normal"
        else:
            print("❓ Pixhawk durumu belirsiz")
            return "unknown"
            
    except Exception as e:
        print(f"⚠️ Durum kontrolü hatası: {e}")
        return "unknown"

def send_heartbeat_request():
    """Pixhawk'a heartbeat request gönder"""
    print("\n💓 Heartbeat Request Gönderiliyor...")
    
    try:
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=2)
        
        # MAVLink Heartbeat Request (basit)
        # Bu ArduSub'ı uyandırabilir
        heartbeat_req = b'\xfe\x09\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
        
        ser.write(heartbeat_req)
        print("📤 Heartbeat request gönderildi")
        
        # Yanıt bekle
        time.sleep(2)
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"📥 Yanıt alındı: {len(response)} byte")
            return True
        else:
            print("❌ Yanıt yok")
            return False
            
        ser.close()
        
    except Exception as e:
        print(f"❌ Heartbeat gönderme hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🧪 Pixhawk Serial Tanı Testi")
    print("=" * 50)
    
    # Pixhawk durumu kontrol et
    status = test_pixhawk_status()
    
    if status == "bootloader":
        print("\n🚨 SORUN: Pixhawk bootloader modunda!")
        print("🔧 Çözüm: ArduSub firmware yükle")
        print("💡 QGroundControl ile firmware yükleyebilirsin")
        return False
    
    # Serial veri testi
    working_baud = test_serial_connection()
    
    if working_baud:
        print(f"\n🎯 Çalışan baud rate: {working_baud}")
        
        # Heartbeat test
        heartbeat_success = send_heartbeat_request()
        
        if heartbeat_success:
            print("\n✅ SERİAL ÇALIŞIYOR!")
            print(f"💡 Socat config'ini güncelle: baud={working_baud}")
        else:
            print("\n⚠️ Serial port açık ama MAVLink yanıt yok")
            print("💡 ArduSub firmware kontrol et")
        
        return True
    else:
        print("\n❌ HİÇBİR BAUD RATE'DE VERİ YOK!")
        print("🔧 Muhtemel sebepler:")
        print("  - Pixhawk power yok")
        print("  - USB kablosu veri taşımıyor")  
        print("  - Firmware yüklü değil")
        print("  - Pixhawk bootloop'ta")
        
        return False

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n👋 Test kullanıcı tarafından durduruldu")
        sys.exit(1) 