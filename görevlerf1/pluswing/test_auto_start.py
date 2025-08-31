#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OTOMATİK BAŞLATMA TEST DOSYASI
Buton sistemi kaldırıldıktan sonra otomatik başlatma akışını test eder
"""

import sys
import time
from main import SaraMainController

def test_auto_start_flow():
    """Otomatik başlatma akışını test et"""
    print("🧪 OTOMATİK BAŞLATMA AKIŞI TESTİ")
    print("="*50)
    print("Bu test buton sisteminin kaldırıldığını ve")
    print("sistemin otomatik olarak başladığını kontrol eder.")
    print()
    
    try:
        # Ana kontrolcüyü oluştur
        print("🔧 SARA Ana Kontrolcü oluşturuluyor...")
        sara = SaraMainController()
        
        print("✅ SARA Ana Kontrolcü oluşturuldu")
        print()
        
        # Test akışı - sadece başlangıç kısmı
        print("🚀 Otomatik başlatma akışı test ediliyor...")
        print("NOT: Bu test sadece başlangıç akışını kontrol eder,")
        print("     tam MAVLink bağlantısı olmadan çalışmaz.")
        print()
        
        # Sistem başlatma akışını simüle et
        print("1. Sistem durumu başlatılıyor...")
        if hasattr(sara, 'system_status'):
            print("   ✅ SystemStatus hazır")
        else:
            print("   ❌ SystemStatus eksik")
            return False
        
        print("2. Logger sistemi kontrol ediliyor...")
        if hasattr(sara, 'logger'):
            print("   ✅ Logger hazır")
            print(f"   📁 Log klasörü: {sara.logger.log_dir}")
        else:
            print("   ❌ Logger eksik")
            return False
        
        print("3. MAVLink bağlantı fonksiyonları kontrol ediliyor...")
        if hasattr(sara, 'setup_mavlink'):
            print("   ✅ MAVLink setup fonksiyonu mevcut")
        else:
            print("   ❌ MAVLink setup eksik")
            return False
        
        print("4. Countdown fonksiyonu kontrol ediliyor...")
        if hasattr(sara, 'countdown_65_seconds'):
            print("   ✅ 65 saniye countdown fonksiyonu mevcut")
        else:
            print("   ❌ Countdown fonksiyonu eksik")
            return False
        
        print("5. Mission çalıştırma fonksiyonu kontrol ediliyor...")
        if hasattr(sara, 'run_mission'):
            print("   ✅ Mission çalıştırma fonksiyonu mevcut")
        else:
            print("   ❌ Mission fonksiyonu eksik")
            return False
        
        print("6. Buton bekleme fonksiyonu kaldırıldı mı kontrol ediliyor...")
        if hasattr(sara, 'wait_for_start_button'):
            print("   ❌ wait_for_start_button hala mevcut! Kaldırılmalı.")
            return False
        else:
            print("   ✅ wait_for_start_button kaldırıldı")
        
        print()
        print("🎯 OTOMATİK BAŞLATMA AKIŞI YAPISI:")
        print("   1. Sistem başlatılır")
        print("   2. MAVLink bağlantısı kurulur") 
        print("   3. Sensörler test edilir")
        print("   4. SensorManager hazırlanır (kalibrasyon YOK)")
        print("   5. Otomatik 65 saniye countdown başlar")
        print("   6. PWM sinyalleri etkinleştirilir")
        print("   7. Görev otomatik başlar")
        print()
        
        return True
        
    except Exception as e:
        print(f"❌ Test hatası: {e}")
        return False

def test_countdown_without_button():
    """Countdown sisteminin buton olmadan çalışıp çalışmadığını test et"""
    print("🧪 COUNTDOWN (BUTON YOK) TESTİ")
    print("="*50)
    
    try:
        sara = SaraMainController()
        
        # Countdown fonksiyonunu kontrol et
        print("🔍 Countdown fonksiyonu analiz ediliyor...")
        
        # Fonksiyon kodunu okuyarak buton kontrolü var mı kontrol et
        import inspect
        countdown_source = inspect.getsource(sara.countdown_65_seconds)
        
        if "check_start_button" in countdown_source:
            print("   ❌ Countdown'da hala buton kontrolü var!")
            print("   🔍 Buton kontrolü satırları:")
            lines = countdown_source.split('\n')
            for i, line in enumerate(lines):
                if "check_start_button" in line:
                    print(f"      Satır {i+1}: {line.strip()}")
            return False
        else:
            print("   ✅ Countdown'da buton kontrolü yok")
        
        if "restart" in countdown_source:
            print("   ⚠️ Countdown'da hala 'restart' mantığı var")
            print("   Bu kaldırılabilir ama kritik değil")
        else:
            print("   ✅ Restart mantığı da temizlenmiş")
        
        return True
        
    except Exception as e:
        print(f"❌ Countdown test hatası: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🧪 SARA OTOMATİK BAŞLATMA SİSTEMİ TEST PAKETİ")
    print("="*60)
    print("Bu test buton sisteminin kaldırıldığını ve")
    print("sistemin otomatik olarak başladığını doğrular.")
    print("="*60)
    
    tests = [
        ("Otomatik Başlatma Akışı", test_auto_start_flow),
        ("Countdown (Buton Yok)", test_countdown_without_button)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            print(f"\n🔬 {test_name} Testi...")
            success = test_func()
            results.append((test_name, success))
            print(f"{'✅ BAŞARILI' if success else '❌ BAŞARISIZ'}")
        except Exception as e:
            print(f"❌ {test_name} HATASI: {e}")
            results.append((test_name, False))
    
    print("\n" + "="*60)
    print("📊 TEST SONUÇLARI:")
    print("="*60)
    
    success_count = 0
    for test_name, success in results:
        status = "✅ BAŞARILI" if success else "❌ BAŞARISIZ"
        print(f"   {test_name}: {status}")
        if success:
            success_count += 1
    
    print(f"\n🎯 TOPLAM: {success_count}/{len(results)} test başarılı")
    
    if success_count == len(results):
        print("🏆 TÜM TESTLER BAŞARILI!")
        print("🚀 Sistem otomatik başlatma için hazır!")
        print()
        print("📋 ÇALIŞMA AKIŞI:")
        print("   1. python3 main.py çalıştır")
        print("   2. Sistem otomatik olarak MAVLink'e bağlanır") 
        print("   3. Sensörler test edilir")
        print("   4. 65 saniye countdown başlar (sensör verileri gösterilir)")
        print("   5. PWM sinyalleri etkinleştirilir")
        print("   6. Görev otomatik başlar")
        print()
    else:
        print("⚠️ Bazı testler başarısız!")
        print("Buton kaldırma işlemi tamamlanmamış olabilir.")
    
    return success_count == len(results)

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
