#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - I2C Scanner
D300 Depth Sensor otomatik tespit scripti
"""

import sys
import json
import time
try:
    import smbus2
except ImportError:
    print("❌ smbus2 kütüphanesi yok! Yüklemek için:")
    print("pip install smbus2")
    sys.exit(1)

class I2CScanner:
    def __init__(self):
        self.found_devices = {}
        self.d300_candidates = []
        
    def scan_bus(self, bus_num):
        """Belirtilen I2C bus'ı tara"""
        print(f"📡 I2C Bus {bus_num} taranıyor...")
        devices = []
        
        try:
            bus = smbus2.SMBus(bus_num)
            
            for addr in range(0x03, 0x78):  # I2C adres aralığı
                try:
                    # Test read
                    bus.read_byte(addr)
                    devices.append(addr)
                    print(f"  ✅ Cihaz bulundu: 0x{addr:02x}")
                except:
                    pass
            
            bus.close()
            return devices
            
        except Exception as e:
            print(f"❌ Bus {bus_num} hatası: {e}")
            return []
    
    def test_d300_address(self, bus_num, addr):
        """Belirtilen adreste D300 test et"""
        try:
            bus = smbus2.SMBus(bus_num)
            
            # D300 specific test patterns
            test_results = {}
            
            # Test 1: Multiple reads (D300 should respond consistently)
            read_values = []
            for i in range(5):
                try:
                    value = bus.read_byte(addr)
                    read_values.append(value)
                    time.sleep(0.01)
                except:
                    read_values.append(None)
            
            test_results['consistent_reads'] = len([v for v in read_values if v is not None]) >= 3
            
            # Test 2: Try to read word data (pressure/temperature registers)
            try:
                pressure_raw = bus.read_word_data(addr, 0x00)
                temp_raw = bus.read_word_data(addr, 0x02)
                test_results['word_data'] = True
                test_results['pressure_raw'] = pressure_raw
                test_results['temp_raw'] = temp_raw
            except:
                test_results['word_data'] = False
            
            # Test 3: Check if values are in reasonable range for depth sensor
            if test_results.get('word_data', False):
                pressure = test_results['pressure_raw']
                temp = test_results['temp_raw']
                
                # Reasonable ranges for depth sensor
                reasonable_pressure = 0 < pressure < 65535
                reasonable_temp = 0 < temp < 65535
                
                test_results['reasonable_values'] = reasonable_pressure and reasonable_temp
            else:
                test_results['reasonable_values'] = False
            
            bus.close()
            return test_results
            
        except Exception as e:
            return {'error': str(e)}
    
    def score_d300_candidate(self, test_results):
        """D300 olma ihtimalini skorla"""
        score = 0
        
        if test_results.get('consistent_reads', False):
            score += 30
        
        if test_results.get('word_data', False):
            score += 40
        
        if test_results.get('reasonable_values', False):
            score += 30
        
        return score
    
    def scan_all_buses(self):
        """Tüm I2C bus'ları tara"""
        print("🔍 I2C Bus taraması başlıyor...\n")
        
        # Genellikle kullanılan bus'lar
        buses_to_scan = [0, 1, 2]
        
        for bus_num in buses_to_scan:
            devices = self.scan_bus(bus_num)
            if devices:
                self.found_devices[bus_num] = devices
                
                # Her cihazı D300 için test et
                for addr in devices:
                    print(f"🔬 0x{addr:02x} adresini D300 için test ediliyor...")
                    test_results = self.test_d300_address(bus_num, addr)
                    
                    if 'error' not in test_results:
                        score = self.score_d300_candidate(test_results)
                        
                        if score > 50:  # %50'den fazla eşleşme
                            self.d300_candidates.append({
                                'bus': bus_num,
                                'address': addr,
                                'score': score,
                                'test_results': test_results
                            })
                            print(f"  🎯 D300 adayı! Skor: {score}/100")
            
            print()  # Boş satır
    
    def print_results(self):
        """Tarama sonuçlarını yazdır"""
        print("=" * 60)
        print("📋 I2C TARAMA SONUÇLARI")
        print("=" * 60)
        
        if not self.found_devices:
            print("❌ Hiç I2C cihazı bulunamadı!")
            print("💡 I2C'nin etkin olduğundan emin olun")
            return
        
        # Bulunan cihazlar
        print("📡 Bulunan I2C Cihazları:")
        for bus_num, devices in self.found_devices.items():
            device_list = [f"0x{addr:02x}" for addr in devices]
            print(f"  Bus {bus_num}: {', '.join(device_list)}")
        
        print()
        
        # D300 adayları
        if self.d300_candidates:
            print("🎯 D300 DEPTH SENSOR ADAYLARI:")
            self.d300_candidates.sort(key=lambda x: x['score'], reverse=True)
            
            for i, candidate in enumerate(self.d300_candidates, 1):
                bus = candidate['bus']
                addr = candidate['address']
                score = candidate['score']
                
                print(f"  {i}. Bus {bus}, Adres 0x{addr:02x} - Skor: {score}/100")
                
                # Test detayları
                results = candidate['test_results']
                if results.get('consistent_reads'):
                    print("     ✅ Tutarlı okuma")
                if results.get('word_data'):
                    print(f"     ✅ Word data (P:{results.get('pressure_raw', 0)}, T:{results.get('temp_raw', 0)})")
                if results.get('reasonable_values'):
                    print("     ✅ Makul değer aralığı")
                
                print()
        else:
            print("❌ D300 depth sensor adayı bulunamadı!")
            print("💡 Sensor bağlı ve çalışır durumda mı?")
    
    def update_config(self):
        """En iyi D300 adayı ile config'i güncelle"""
        if not self.d300_candidates:
            print("⚠️ Config güncellenemiyor - D300 bulunamadı")
            return False
        
        # En yüksek skorlu adayı seç
        best_candidate = self.d300_candidates[0]
        bus = best_candidate['bus']
        addr = best_candidate['address']
        
        try:
            # Config dosyasını yükle
            config_path = "config/hardware_config.json"
            try:
                with open(config_path, 'r') as f:
                    config = json.load(f)
            except:
                config = {}
            
            # I2C ayarlarını güncelle
            if "raspberry_pi" not in config:
                config["raspberry_pi"] = {}
            if "i2c" not in config["raspberry_pi"]:
                config["raspberry_pi"]["i2c"] = {}
            
            config["raspberry_pi"]["i2c"]["bus_number"] = bus
            config["raspberry_pi"]["i2c"]["depth_sensor_address"] = f"0x{addr:02x}"
            
            # Config dosyasını kaydet
            with open(config_path, 'w') as f:
                json.dump(config, f, indent=2)
            
            print(f"✅ Config güncellendi!")
            print(f"   Bus: {bus}")
            print(f"   Adres: 0x{addr:02x}")
            return True
            
        except Exception as e:
            print(f"❌ Config güncelleme hatası: {e}")
            return False

def main():
    print("🔍 TEKNOFEST Su Altı ROV - I2C D300 Scanner")
    print("=" * 50)
    
    scanner = I2CScanner()
    
    try:
        # Tüm bus'ları tara
        scanner.scan_all_buses()
        
        # Sonuçları göster
        scanner.print_results()
        
        # Config güncellemesi sor
        if scanner.d300_candidates:
            print()
            response = input("🤖 En iyi adayı config dosyasına kaydedelim mi? (y/n): ")
            if response.lower() in ['y', 'yes', 'evet', 'e']:
                scanner.update_config()
        
    except KeyboardInterrupt:
        print("\n👋 Tarama durduruldu!")
    except Exception as e:
        print(f"❌ Hata: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 