#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Import Doğrulama Scripti
Tüm modüllerin doğru import edilip edilemediğini test eder
"""

import sys
import traceback

def test_imports():
    """Tüm modüllerin import edilip edilemediğini test et"""
    print("🧪 Hava Yarışı Test Modülleri Import Testi")
    print("=" * 50)
    
    modules_to_test = [
        ("config_air", "Hava yarışı konfigürasyonu"),
        ("sensors_air", "Sensör yönetimi (D300 YOK)"),
        ("control_air", "Kontrol sistemi"),
        ("utils_air", "Yardımcı fonksiyonlar"),
        ("main_air_test", "Ana test programı"),
        ("mission_air_test", "Test görev mantığı")
    ]
    
    success_count = 0
    total_count = len(modules_to_test)
    
    for module_name, description in modules_to_test:
        try:
            print(f"📦 {module_name:<20} - ", end="")
            
            # Modülü import et
            __import__(module_name)
            
            print(f"✅ OK - {description}")
            success_count += 1
            
        except ImportError as e:
            print(f"❌ IMPORT ERROR - {e}")
        except Exception as e:
            print(f"⚠️ OTHER ERROR - {e}")
    
    print("=" * 50)
    print(f"📊 Sonuç: {success_count}/{total_count} modül başarılı")
    
    if success_count == total_count:
        print("🎉 Tüm modüller başarıyla import edildi!")
        return True
    else:
        print("❌ Bazı modüllerde sorun var!")
        return False

def test_dependencies():
    """Gerekli bağımlılıkları test et"""
    print("\n🔧 Bağımlılık Testi")
    print("=" * 30)
    
    dependencies = [
        ("pymavlink", "MAVLink protokol desteği"),
        ("time", "Zaman fonksiyonları"),
        ("math", "Matematik fonksiyonları"),
        ("threading", "Thread desteği"),
        ("collections", "Koleksiyon türleri")
    ]
    
    success_count = 0
    
    for dep_name, description in dependencies:
        try:
            print(f"📦 {dep_name:<15} - ", end="")
            __import__(dep_name)
            print(f"✅ OK - {description}")
            success_count += 1
        except ImportError:
            print(f"❌ MISSING - {description}")
    
    print(f"\n📊 Bağımlılık: {success_count}/{len(dependencies)} mevcut")
    return success_count == len(dependencies)

def test_gpio_availability():
    """GPIO kütüphanelerinin durumunu test et"""
    print("\n🔌 GPIO Kütüphane Testi")
    print("=" * 35)
    
    gpio_libs = [
        ("RPi.GPIO", "Raspberry Pi GPIO (eski)"),
        ("lgpio", "Raspberry Pi GPIO (yeni)")
    ]
    
    available_count = 0
    
    for lib_name, description in gpio_libs:
        try:
            print(f"📦 {lib_name:<12} - ", end="")
            __import__(lib_name)
            print(f"✅ AVAILABLE - {description}")
            available_count += 1
        except ImportError:
            print(f"⚠️ NOT FOUND - {description}")
    
    if available_count > 0:
        print(f"✅ GPIO desteği mevcut ({available_count} kütüphane)")
    else:
        print("⚠️ GPIO kütüphanesi yok - simülasyon modunda çalışacak")
    
    return available_count > 0

def test_config_values():
    """Konfigürasyon değerlerini test et"""
    print("\n⚙️ Konfigürasyon Testi")
    print("=" * 30)
    
    try:
        import config_air as config
        
        # Kritik değerleri kontrol et
        checks = [
            (hasattr(config, 'USE_DEPTH_CONTROL'), "USE_DEPTH_CONTROL tanımlı"),
            (getattr(config, 'USE_DEPTH_CONTROL', True) == False, "D300 deaktif"),
            (hasattr(config, 'SIMULATE_DEPTH'), "SIMULATE_DEPTH tanımlı"),
            (hasattr(config, 'TEST_MODE'), "TEST_MODE tanımlı"),
            (hasattr(config, 'ARMING_DELAY_SECONDS'), "ARMING_DELAY_SECONDS tanımlı"),
            (hasattr(config, 'MAVLINK_BAUD'), "MAVLINK_BAUD tanımlı")
        ]
        
        success_count = 0
        for check, description in checks:
            print(f"⚙️ {description:<30} - ", end="")
            if check:
                print("✅ OK")
                success_count += 1
            else:
                print("❌ FAIL")
        
        print(f"\n📊 Konfigürasyon: {success_count}/{len(checks)} doğru")
        return success_count == len(checks)
        
    except Exception as e:
        print(f"❌ Konfigürasyon yüklenemedi: {e}")
        return False

def main():
    """Ana test fonksiyonu"""
    print("🚁 Hava Yarışı Test Sistemi - Doğrulama")
    print("=" * 60)
    
    # Tüm testleri çalıştır
    tests = [
        ("Modül Import", test_imports),
        ("Bağımlılık", test_dependencies),
        ("GPIO Kütüphaneleri", test_gpio_availability),
        ("Konfigürasyon", test_config_values)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"\n❌ {test_name} testi hatası: {e}")
            traceback.print_exc()
            results.append((test_name, False))
    
    # Sonuçları özetle
    print("\n" + "=" * 60)
    print("📋 TEST SONUÇLARI")
    print("=" * 60)
    
    success_count = 0
    for test_name, result in results:
        status = "✅ BAŞARILI" if result else "❌ BAŞARISIZ"
        print(f"{test_name:<20} : {status}")
        if result:
            success_count += 1
    
    print("=" * 60)
    overall_success = success_count == len(results)
    
    if overall_success:
        print("🎉 TÜM TESTLER BAŞARILI!")
        print("✅ Hava yarışı test sistemi kullanıma hazır")
        print("\nÇalıştırmak için:")
        print("  python3 main_air_test.py --test-only  # Bağlantı testi")
        print("  python3 main_air_test.py              # Tam test")
    else:
        print("⚠️ BAZI TESTLER BAŞARISIZ!")
        print("❌ Sorunları giderdikten sonra tekrar deneyin")
    
    return overall_success

if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n🛑 Test kullanıcı tarafından durduruldu")
        sys.exit(1)
    except Exception as e:
        print(f"\n💥 Beklenmeyen hata: {e}")
        traceback.print_exc()
        sys.exit(1)
