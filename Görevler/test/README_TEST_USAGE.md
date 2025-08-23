# TEKNOFEST Test Scriptleri Kullanım Kılavuzu

Bu klasör, TEKNOFEST Su Altı Roket Aracı projesi için fiziksel donanım kontrolü ve sistem entegrasyonu test scriptlerini içerir.

## 📁 Test Scriptleri

### 🔧 `test_physical_hardware.py`
**AMAÇ**: Fiziksel donanımın çalışıp çalışmadığını kontrol eder.

**FİZİKSEL KONTROL TESTLERİ**:
- GPIO LED/Buzzer Testi (Görsel/İşitsel Onay)
- Motor PWM Testi (ESC Arming + Throttle)
- Servo PWM Testi (Kanat Hareketi Gözlemi)
- Derinlik Sensörü Testi (D300 + SCALED_PRESSURE)
- IMU Sensörü Testi (Roll/Pitch/Yaw Okuma)
- Arming Interlock Testi (90s Countdown)

**KULLANIM**:
```bash
cd Görevler/test
python3 test_physical_hardware.py
```

**ÖNEMLİ**: Bu test fiziksel gözlem gerektirir! Her test sonrası donanımın çalışıp çalışmadığını gözlemleyip onaylamanız gerekir.

### 🔄 `test_mission_integration.py`
**AMAÇ**: Yeni eklenen özelliklerin mission kodlarıyla entegrasyonunu doğrular.

**ENTEGRASYON TESTLERİ**:
- Config Yükleme ve Validation Testi
- PWM Odometri Entegrasyon Testi
- LED/Buzzer Entegrasyon Testi
- Leak Detection Testi
- Thread Management Testi
- CSV Telemetri Testi

**KULLANIM**:
```bash
cd Görevler/test
python3 test_mission_integration.py
```

### 🎛️ `test_plus_wing_stabilization.py`
**AMAÇ**: Plus-Wing konfigürasyonunda stabilizasyon sistemini test eder.

**GÜNCELLENEN ÖZELLİKLER**:
- Yeni PID parametreleri testi
- PWM güvenlik sınırları kontrolü
- GPIO entegrasyon testi
- Telemetri kayıt testi

### ⚔️ `test_x_wing_stabilization.py`
**AMAÇ**: X-Wing konfigürasyonunda stabilizasyon sistemini test eder.

### 📊 `test_wing_comparison.py`
**AMAÇ**: Plus-Wing ve X-Wing konfigürasyonlarını karşılaştırır.

## 🚀 Test Sırası Önerisi

1. **İlk Kurulum**: `test_physical_hardware.py`
   - Tüm donanım bileşenlerinin çalıştığını doğrulayın
   - Motor propeller'ını çıkarın!

2. **Entegrasyon Kontrolü**: `test_mission_integration.py`
   - Yeni özelliklerin çalıştığını doğrulayın

3. **Stabilizasyon Testi**: 
   - Plus-Wing için: `test_plus_wing_stabilization.py`
   - X-Wing için: `test_x_wing_stabilization.py`

4. **Karşılaştırma**: `test_wing_comparison.py`
   - Her iki konfigürasyonun performansını karşılaştırın

## ⚠️ Güvenlik Uyarıları

### Motor Testleri İçin:
- **PROPELLER ÇIKARIN!** Motor testlerinde propeller takılı olmamalı
- Test alanında yeterli boş alan olduğundan emin olun
- Acil durdurma için hazır olun

### Servo Testleri İçin:
- Servo bağlantılarının sağlam olduğunu kontrol edin
- Kanat hareketlerini gözlemleyin
- Anormal ses veya titreşim varsa testi durdurun

### Derinlik Sensörü Testleri İçin:
- D300 sensörünün I2C bağlantısını kontrol edin ([[memory:4381766]])
- SCALED_PRESSURE verilerinin mantıklı olduğunu doğrulayın

## 📊 Test Raporları

Her test sonrası otomatik rapor oluşturulur:
- `physical_hardware_test_YYYYMMDD_HHMMSS.json`
- `mission_integration_test_YYYYMMDD_HHMMSS.json`
- `mission_1_telemetry_YYYYMMDD_HHMMSS.csv`

## 🔧 Sorun Giderme

### GPIO Hatası:
```
⚠️ RPi.GPIO bulunamadı, LED/Buzzer devre dışı
```
**Çözüm**: `sudo apt install python3-rpi.gpio`

### Hardware Config Hatası:
```
❌ hardware_config.py bulunamadı!
```
**Çözüm**: Otomatik olarak oluşturuldu - `pluswing/hardware_config.py`

### Import Hatası:
```
❌ Plus-Wing modülü yüklenemedi
```
**Çözüm**: 
- `cd Görevler/test` klasöründe olduğunuzdan emin olun
- `../pluswing/mission_1_navigation_plus.py` dosyasının var olduğunu kontrol edin

### MAVLink Bağlantı Hatası:
```
❌ Pixhawk bağlantısı başarısız!
```
**Çözüm**: 
- USB bağlantısını kontrol edin
- `/dev/ttyACM0` portunu kontrol edin
- Pixhawk'ın açık olduğunu doğrulayın

### Config Yükleme Hatası:
```
⚠️ Mission config yüklenemedi
```
**Çözüm**: `config/mission_config.json` dosyasının var olduğunu kontrol edin

## 🎯 Test Başarı Kriterleri

### Fiziksel Hardware Test:
- **%100 başarı**: Tüm donanım çalışıyor
- **%50+ başarı**: Kısmen çalışıyor, bazı bileşenler kontrol edilmeli
- **%50 altı**: Çoğu bileşen çalışmıyor, hardware kontrolü gerekli

### Mission Entegrasyon Test:
- **%100 başarı**: Tüm yeni özellikler entegre
- **%50+ başarı**: Kısmen entegre, bazı özellikler kontrol edilmeli
- **%50 altı**: Entegrasyon sorunları var, kod kontrolü gerekli

## 📞 Destek

Test scriptleri ile ilgili sorunlar için:
1. Test raporlarını kontrol edin
2. Console çıktılarını inceleyin
3. Hardware bağlantılarını doğrulayın
4. Mission kodlarının güncel olduğunu kontrol edin
