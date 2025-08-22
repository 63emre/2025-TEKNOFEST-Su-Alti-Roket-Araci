"""
TEKNOFEST 2025 Su Altı Roket Aracı
D300 Derinlik Sensörü Test Scripti

Bu script D300 derinlik ve sıcaklık sensörünü test eder.
I2C adresi: 0x76
"""

import os
import sys
import time
import signal

# Proje dizinini path'e ekle
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from common.d300_sensor import D300Sensor

class D300Tester:
    """D300 sensör test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.sensor = None
        
        # Signal handler ayarla
        signal.signal(signal.SIGINT, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Ctrl+C ile çıkış"""
        print("\n\nTest durduruluyor...")
        self.running = False
        if self.sensor:
            self.sensor.disconnect()
        sys.exit(0)
    
    def test_connection(self):
        """D300 sensör bağlantısını test et"""
        print("="*60)
        print("D300 DERINLIK SENSÖRÜ BAĞLANTI TESİ")  
        print("="*60)
        print("I2C Bus: 1")
        print("I2C Adres: 0x76")
        print("-"*60)
        
        # D300 sensör nesnesi oluştur
        self.sensor = D300Sensor(bus_number=1, address=0x76)
        
        # Bağlantıyı dene
        print("D300 sensörüne bağlanılıyor...")
        if not self.sensor.connect():
            print("❌ BAĞLANTI BAŞARISIZ!")
            print("Kontrol edilecekler:")
            print("- I2C etkin mi? (raspi-config > Interface Options > I2C)")
            print("- D300 sensörü doğru bağlandı mı?")
            print("- I2C adresi doğru mu? (i2cdetect -y 1 ile kontrol edin)")
            print("- 3.3V ve GND bağlantıları kontrol edin")
            return False
        
        print("✅ BAĞLANTI BAŞARILI!")
        print("-"*60)
        return True
    
    def test_i2c_scan(self):
        """I2C tarama testi"""
        print("\nI2C TARAMA TESİ")
        print("Mevcut I2C cihazları aranıyor...")
        print("-"*60)
        
        try:
            import subprocess
            result = subprocess.run(['i2cdetect', '-y', '1'], 
                                    capture_output=True, text=True)
            
            if result.returncode == 0:
                print("I2C Tarama Sonucu:")
                print(result.stdout)
                
                # 0x76 adresinin var olup olmadığını kontrol et
                if "76" in result.stdout:
                    print("✅ 0x76 adresi tespit edildi!")
                else:
                    print("❌ 0x76 adresi bulunamadı!")
                    
            else:
                print("❌ I2C tarama başarısız!")
                print("i2c-tools kurulu mu? (sudo apt install i2c-tools)")
                
        except FileNotFoundError:
            print("❌ i2cdetect komutu bulunamadı!")
            print("i2c-tools kurun: sudo apt install i2c-tools")
        except Exception as e:
            print(f"❌ I2C tarama hatası: {e}")
    
    def test_calibration(self):
        """Kalibrasyon testi"""
        print("\nKALİBRASYON TESİ")
        print("Yüzey seviyesi kalibrasyonu yapılıyor...")
        print("-"*60)
        
        try:
            # Kalibrasyon yap
            self.sensor.calibrate_surface_level()
            print("✅ Kalibrasyon başarılı!")
            
        except Exception as e:
            print(f"❌ Kalibrasyon hatası: {e}")
    
    def test_single_reading(self):
        """Tek okuma testi"""
        print("\nTEK OKUMA TESİ")
        print("Sensörden tek veri paketi okunuyor...")
        print("-"*60)
        
        try:
            data = self.sensor.read_sensor_data()
            
            print(f"Derinlik: {data['depth_m']:.2f} metre")
            print(f"Basınç: {data['pressure_mbar']:.2f} mbar")
            print(f"Sıcaklık: {data['temp_celsius']:.1f} °C")
            print("✅ Tek okuma başarılı!")
            
        except Exception as e:
            print(f"❌ Tek okuma hatası: {e}")
    
    def test_continuous_reading(self):
        """Sürekli okuma testi"""
        print("\nSÜREKLİ OKUMA TESİ")
        print("30 saniye boyunca sürekli veri okunacak...")
        print("Ctrl+C ile durdurun.")
        print("-"*60)
        
        # Sürekli okuma başlat
        self.sensor.start_continuous_reading()
        
        start_time = time.time()
        data_count = 0
        
        try:
            while self.running and (time.time() - start_time) < 30:
                # Güncel verileri al
                data = self.sensor.get_all_data()
                
                # Timestamp kontrol et
                if data["timestamp"] == 0:
                    print("⚠️  Henüz veri alınmadı...")
                    time.sleep(1)
                    continue
                
                # Veri yaşını kontrol et
                data_age = time.time() - data["timestamp"]
                if data_age > 2:
                    print("⚠️  Veri eski (bağlantı sorunu olabilir)")
                
                # Ekrana yazdır
                print(f"📊 Veri #{data_count + 1:3d} | "
                      f"Derinlik: {data['depth_m']:6.2f}m | "
                      f"Basınç: {data['pressure_mbar']:7.2f}mbar | "
                      f"Sıcaklık: {data['temp_celsius']:5.1f}°C | "
                      f"Yaş: {data_age:.1f}s")
                
                data_count += 1
                time.sleep(1)  # 1 Hz gösterim
                
        except KeyboardInterrupt:
            print("\n\nTest kullanıcı tarafından durduruldu")
            
        finally:
            self.sensor.stop_continuous_reading()
            
        print("-"*60)
        print(f"✅ TEST TAMAMLANDI!")
        print(f"Toplam {data_count} veri paketi alındı")
    
    def test_depth_simulation(self):
        """Derinlik simülasyonu (manuel test)"""
        print("\nDERİNLİK SİMÜLASYON TESİ")
        print("Bu test manuel olarak yapılmalıdır:")
        print("1. Sensörü su dolu bir kaba daldırın")
        print("2. Farklı derinliklerde ölçüm alın")
        print("3. Verilerin mantıklı olup olmadığını kontrol edin")
        print("-"*60)
        
        input("Devam etmek için Enter'a basın...")
        
        # 10 saniye boyunca okuma yap
        for i in range(10):
            data = self.sensor.get_all_data()
            print(f"Ölçüm {i+1:2d}: Derinlik={data['depth_m']:6.2f}m, "
                  f"Basınç={data['pressure_mbar']:7.2f}mbar")
            time.sleep(1)
        
        print("✅ Derinlik simülasyon testi tamamlandı")
    
    def test_sensor_limits(self):
        """Sensör limit testi"""
        print("\nSENSÖR LİMİT TESİ")
        print("Sensörün çalışma limitlerini test ediyoruz...")
        print("-"*60)
        
        test_duration = 10
        readings = []
        
        # 10 saniye boyunca okuma yap
        for i in range(test_duration):
            data = self.sensor.read_sensor_data()
            readings.append(data)
            print(f"Okuma {i+1}: {data['depth_m']:.2f}m, "
                  f"{data['pressure_mbar']:.2f}mbar, {data['temp_celsius']:.1f}°C")
            time.sleep(1)
        
        # İstatistikler
        depths = [r['depth_m'] for r in readings]
        pressures = [r['pressure_mbar'] for r in readings]
        temps = [r['temp_celsius'] for r in readings]
        
        print("\nİSTATİSTİKLER:")
        print(f"Derinlik - Min: {min(depths):.2f}m, Max: {max(depths):.2f}m")
        print(f"Basınç - Min: {min(pressures):.2f}mbar, Max: {max(pressures):.2f}mbar")
        print(f"Sıcaklık - Min: {min(temps):.1f}°C, Max: {max(temps):.1f}°C")
        
        # Veri kararlılığını kontrol et
        depth_range = max(depths) - min(depths)
        if depth_range > 0.1:  # 10cm'den fazla sapma
            print("⚠️  Derinlik verisi kararsız (çok fazla sapma)")
        else:
            print("✅ Derinlik verisi stabil")
        
        print("✅ Sensör limit testi tamamlandı")
    
    def run_full_test(self):
        """Tam test senaryosu"""
        try:
            # I2C tarama
            self.test_i2c_scan()
            
            # Bağlantı testi
            if not self.test_connection():
                return
            
            # Tek okuma testi
            self.test_single_reading()
            
            if not self.running:
                return
            
            # Kalibrasyon testi
            self.test_calibration()
            
            if not self.running:
                return
            
            # Sürekli okuma testi
            self.test_continuous_reading()
            
            if not self.running:
                return
            
            # Sensör limitler testi
            self.test_sensor_limits()
            
            # Manuel testler (isteğe bağlı)
            manual_test = input("\nManuel derinlik testi yapılsın mı? (e/h): ").lower()
            if manual_test == 'e':
                self.test_depth_simulation()
            
            print("\n" + "="*60)
            print("🎉 TÜM D300 TESTLER BAŞARIYLA TAMAMLANDI!")
            print("="*60)
            
        except Exception as e:
            print(f"\n❌ TEST HATASI: {e}")
            
        finally:
            if self.sensor:
                self.sensor.disconnect()

def main():
    """Ana test fonksiyonu"""
    tester = D300Tester()
    
    try:
        tester.run_full_test()
    except KeyboardInterrupt:
        print("\nTest kullanıcı tarafından durduruldu")
    except Exception as e:
        print(f"Beklenmeyen hata: {e}")

if __name__ == "__main__":
    main()
