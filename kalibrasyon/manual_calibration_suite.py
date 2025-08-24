#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MANUAL CALIBRATION SUITE - Manuel Kalibrasyon Paketi
Hareket gerektiren sensörlerin kalibrasyonu için ana program

Bu dosya ile:
1. Accelerometer (İvmeölçer) kalibrasyonu
2. Compass/Magnetometer (Pusula/Manyetometre) kalibrasyonu

yapabilirsiniz. Her kalibrasyon ayrı ayrı çalıştırılabilir.
"""

import sys
import os

def show_menu():
    """Ana menüyü göster"""
    print("\n" + "="*60)
    print("MANUEL KALİBRASYON PAKETİ")
    print("TEKNOFEST 2025 Su Altı Roket Aracı")
    print("="*60)
    print("\nKalibrasyon Seçenekleri:")
    print("1. İvmeölçer Kalibrasyonu (Accelerometer)")
    print("   - Roll ve Pitch açısı hesaplaması için")
    print("   - 6 yönlü kalibrasyon gerektirir")
    print("   - Süre: ~15 dakika")
    print()
    print("2. Pusula Kalibrasyonu (Compass/Magnetometer)")
    print("   - Yaw açısı ve 180° dönüş kontrolü için")
    print("   - 360° dönüş kalibrasyonu gerektirir")
    print("   - Süre: ~5 dakika")
    print()
    print("3. Tüm Kalibrasyonları Çalıştır")
    print("4. Çıkış")
    print("\nSeçiminizi yapın (1-4): ", end="")

def run_accelerometer_calibration():
    """İvmeölçer kalibrasyonunu çalıştır"""
    print("\n🔄 İvmeölçer kalibrasyonu başlatılıyor...")
    
    try:
        from accelerometer_calibration import AccelerometerCalibrator
        calibrator = AccelerometerCalibrator()
        return calibrator.run_calibration()
    except ImportError as e:
        print(f"❌ İvmeölçer kalibrasyon modülü bulunamadı: {e}")
        return False
    except Exception as e:
        print(f"❌ İvmeölçer kalibrasyon hatası: {e}")
        return False

def run_compass_calibration():
    """Pusula kalibrasyonunu çalıştır"""
    print("\n🔄 Pusula kalibrasyonu başlatılıyor...")
    
    try:
        from compass_calibration import CompassCalibrator
        calibrator = CompassCalibrator()
        return calibrator.run_calibration()
    except ImportError as e:
        print(f"❌ Pusula kalibrasyon modülü bulunamadı: {e}")
        return False
    except Exception as e:
        print(f"❌ Pusula kalibrasyon hatası: {e}")
        return False

def run_all_calibrations():
    """Tüm kalibrasyonları sırayla çalıştır"""
    print("\n🔄 Tüm kalibrasyonlar başlatılıyor...")
    
    results = {}
    
    print("\n" + "="*40)
    print("1/2: İVMEÖLÇER KALİBRASYONU")
    print("="*40)
    results['accelerometer'] = run_accelerometer_calibration()
    
    if results['accelerometer']:
        print("\n✅ İvmeölçer kalibrasyonu tamamlandı!")
    else:
        print("\n❌ İvmeölçer kalibrasyonu başarısız!")
        
    input("\nPusula kalibrasyonuna geçmek için ENTER'a basın...")
    
    print("\n" + "="*40)
    print("2/2: PUSULA KALİBRASYONU")
    print("="*40)
    results['compass'] = run_compass_calibration()
    
    if results['compass']:
        print("\n✅ Pusula kalibrasyonu tamamlandı!")
    else:
        print("\n❌ Pusula kalibrasyonu başarısız!")
        
    # Sonuç özeti
    print("\n" + "="*40)
    print("KALİBRASYON SONUÇLARI")
    print("="*40)
    print(f"İvmeölçer: {'✅ Başarılı' if results['accelerometer'] else '❌ Başarısız'}")
    print(f"Pusula: {'✅ Başarılı' if results['compass'] else '❌ Başarısız'}")
    
    success_count = sum(results.values())
    total_count = len(results)
    
    if success_count == total_count:
        print("\n🎉 TÜM KALİBRASYONLAR BAŞARILI!")
    elif success_count > 0:
        print(f"\n⚠️ {success_count}/{total_count} kalibrasyon başarılı")
    else:
        print("\n❌ Hiçbir kalibrasyon başarılı olmadı!")
        
    return success_count == total_count

def check_requirements():
    """Gerekli bağımlılıkları kontrol et"""
    required_modules = ['numpy', 'matplotlib', 'pymavlink']
    missing_modules = []
    
    for module in required_modules:
        try:
            __import__(module)
        except ImportError:
            missing_modules.append(module)
            
    if missing_modules:
        print("❌ Eksik Python modülleri:")
        for module in missing_modules:
            print(f"   - {module}")
        print("\nKurulum için:")
        print("   pip install " + " ".join(missing_modules))
        return False
        
    return True

def main():
    """Ana fonksiyon"""
    print("Manuel Kalibrasyon Paketi başlatılıyor...")
    
    # Gerekli modülleri kontrol et
    if not check_requirements():
        print("\n❌ Gerekli bağımlılıklar eksik!")
        return False
        
    while True:
        show_menu()
        
        try:
            choice = input().strip()
            
            if choice == '1':
                success = run_accelerometer_calibration()
                if success:
                    print("\n✅ İvmeölçer kalibrasyonu başarılı!")
                else:
                    print("\n❌ İvmeölçer kalibrasyonu başarısız!")
                    
            elif choice == '2':
                success = run_compass_calibration()
                if success:
                    print("\n✅ Pusula kalibrasyonu başarılı!")
                else:
                    print("\n❌ Pusula kalibrasyonu başarısız!")
                    
            elif choice == '3':
                run_all_calibrations()
                
            elif choice == '4':
                print("\n👋 Çıkılıyor...")
                break
                
            else:
                print("\n❌ Geçersiz seçim! Lütfen 1-4 arası bir sayı girin.")
                
        except KeyboardInterrupt:
            print("\n\n👋 Kullanıcı tarafından iptal edildi")
            break
        except Exception as e:
            print(f"\n❌ Beklenmeyen hata: {e}")
            
        input("\nDevam etmek için ENTER'a basın...")
        
    return True

if __name__ == "__main__":
    main()
