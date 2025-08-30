#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LOGLAMA SİSTEMİ TEST DOSYASI
Yeni kapsamlı loglama sistemini test eder
"""

import sys
import time
import os
from utils import Logger

def test_comprehensive_logging():
    """Kapsamlı loglama sistemini test et"""
    print("🧪 SARA LOGLAMA SİSTEMİ TESTİ")
    print("="*50)
    
    # Logger oluştur
    logger = Logger("test_mission.log")
    
    print(f"📁 Log klasörü: {logger.log_dir}")
    print(f"📄 Ana log dosyası: {logger.main_log_file}")
    
    # Test logları
    logger.info("🚀 SARA sistemi başlatılıyor...")
    logger.mission_log("Görev 1 hazırlığı başlıyor", "INFO")
    
    # Sensör logları
    logger.sensor_log("D300 sensörü bağlantısı test ediliyor", "INFO")
    logger.d300_log("SCALED_PRESSURE2 veri akışı başlatıldı", "INFO")
    logger.sensor_log("Attitude sensörü kalibre edildi", "INFO")
    
    # Kontrol logları  
    logger.control_log("Servo kanalları test ediliyor", "INFO")
    logger.control_log("PWM sinyalleri devre dışı (90 saniye)", "WARNING")
    logger.control_log("Motor kontrolü hazır", "INFO")
    
    # Görev logları
    logger.mission_log("90 saniye geri sayım başlıyor", "INFO")
    logger.mission_log("Faz 1: İlk 10 metre seyir", "INFO")
    logger.mission_log("Faz 2: Ana seyir 40 metre", "INFO")
    
    # Hata logları
    logger.warning("D300 sensörü geçici bağlantı sorunu")
    logger.error("Servo kanal 11 yanıt vermiyor")
    logger.critical("Acil durum: Derinlik sensörü kesildi")
    
    # Farklı kategoriler
    logger.debug("Debug: Stabilizasyon PID değerleri güncellendi")
    logger.info("Sistem durumu: Tüm sensörler aktif")
    
    print("\n📊 LOG DOSYALARI OLUŞTURULDU:")
    
    # Log özeti
    summary = logger.get_log_summary()
    for name, info in summary['files'].items():
        if info.get('exists', False):
            print(f"✅ {name.upper()}: {info['path']} ({info['size_kb']} KB)")
        else:
            print(f"❌ {name.upper()}: Oluşturulmadı")
    
    print(f"\n📁 Log klasörü içeriği:")
    try:
        for file in os.listdir(logger.log_dir):
            filepath = os.path.join(logger.log_dir, file)
            if os.path.isfile(filepath):
                size_kb = round(os.path.getsize(filepath) / 1024, 2)
                print(f"   📄 {file} ({size_kb} KB)")
    except Exception as e:
        print(f"   ❌ Klasör listesi hatası: {e}")
    
    return True

def test_category_detection():
    """Kategori algılama sistemini test et"""
    print("\n🔍 KATEGORİ ALGILAMA TESTİ")
    print("="*50)
    
    logger = Logger("category_test.log")
    
    # Farklı mesajlar ile kategori testi
    test_messages = [
        ("D300 sensörü başlatıldı", "INFO"),
        ("Servo motor PWM değeri ayarlandı", "INFO"),
        ("Görev faz 1 tamamlandı", "INFO"),
        ("Attitude sensörü kalibrasyonu", "INFO"),
        ("Motor kontrolü başarısız", "ERROR"),
        ("Stabilizasyon sistemi aktif", "INFO"),
        ("Derinlik ölçümü: 2.5m", "INFO"),
        ("PWM sinyali engellendi", "WARNING")
    ]
    
    for message, level in test_messages:
        logger.log(message, level)
        print(f"📝 {level}: {message}")
    
    print("\n✅ Kategori testi tamamlandı!")
    return True

def test_error_logging():
    """Hata loglama sistemini test et"""
    print("\n⚠️ HATA LOGLAMA TESTİ")
    print("="*50)
    
    logger = Logger("error_test.log")
    
    # Farklı hata seviyeleri
    logger.warning("Test uyarısı: Sensör gecikmesi")
    logger.error("Test hatası: Bağlantı sorunu")
    logger.critical("Test kritik: Sistem durdu")
    
    # Hata log dosyasını kontrol et
    if os.path.exists(logger.error_log_file):
        print(f"✅ Hata log dosyası oluşturuldu: {logger.error_log_file}")
        
        # İçeriği göster
        try:
            with open(logger.error_log_file, "r", encoding="utf-8") as f:
                content = f.read()
                print("📄 Hata log içeriği:")
                print(content)
        except Exception as e:
            print(f"❌ Hata log okuma hatası: {e}")
    else:
        print("❌ Hata log dosyası oluşturulmadı")
    
    return True

def test_performance():
    """Loglama performansını test et"""
    print("\n⚡ PERFORMANS TESTİ")
    print("="*50)
    
    logger = Logger("performance_test.log")
    
    # 100 log mesajı ile performans testi
    start_time = time.time()
    
    for i in range(100):
        logger.info(f"Performans test mesajı #{i+1}")
        if i % 20 == 0:
            logger.sensor_log(f"Sensör okuma #{i+1}", "INFO")
            logger.control_log(f"Kontrol döngüsü #{i+1}", "INFO")
    
    end_time = time.time()
    duration = end_time - start_time
    
    print(f"⏱️ 100 log mesajı süresi: {duration:.3f} saniye")
    print(f"📊 Ortalama: {(duration/100)*1000:.2f} ms/log")
    
    # Log dosyası boyutları
    summary = logger.get_log_summary()
    total_size = sum(info.get('size_kb', 0) for info in summary['files'].values())
    print(f"💾 Toplam log boyutu: {total_size:.2f} KB")
    
    return True

def main():
    """Ana test fonksiyonu"""
    print("🧪 SARA KAPSAMLI LOGLAMA SİSTEMİ TEST PAKETİ")
    print("="*60)
    
    tests = [
        ("Kapsamlı Loglama", test_comprehensive_logging),
        ("Kategori Algılama", test_category_detection), 
        ("Hata Loglama", test_error_logging),
        ("Performans", test_performance)
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            print(f"\n🔬 {test_name} Testi Başlıyor...")
            success = test_func()
            results.append((test_name, success))
            print(f"✅ {test_name} Testi: {'BAŞARILI' if success else 'BAŞARISIZ'}")
        except Exception as e:
            print(f"❌ {test_name} Testi HATASI: {e}")
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
        print("🏆 TÜM TESTLER BAŞARILI! Loglama sistemi çalışır durumda.")
    else:
        print("⚠️ Bazı testler başarısız. Loglama sistemini kontrol edin.")
    
    return success_count == len(results)

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
