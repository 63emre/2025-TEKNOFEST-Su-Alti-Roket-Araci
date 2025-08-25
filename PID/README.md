# 🎯 SARA PID Optimizasyon Sistemi

Bu sistem, 4m çap ve 1m derinlikli test alanında SARA'nın PID parametrelerini otomatik olarak optimize eder.

## 🚀 Özellikler

- **Otomatik Grid Search**: Iteratif daraltma algoritması ile en uygun PID değerlerini bulur
- **Çoklu Derinlik Testi**: 0.3m, 0.5m, 0.7m, 1.0m derinliklerinde test yapar
- **Gerçek Zamanlı Değerlendirme**: Hata, oscillation ve overshoot'u birleştiren skor sistemi
- **Güvenli Test Alanı**: 4m çap sınırları içinde kalır
- **Sonuç Kaydetme**: JSON formatında detaylı sonuçlar ve config güncellemesi

## 📋 Gereksinimler

- Araç suda olmalı (havuz/deniz)
- 4m çap, minimum 1m derinlik test alanı
- MAVLink bağlantısı aktif
- D300 sensörü kalibre edilmiş

## 🎮 Kullanım

### Temel Kullanım:
```bash
cd PID/
python3 main.py
```

### Gelişmiş Parametreler:
```bash
# Farklı test alanı boyutu
python3 main.py --test-area-radius 1.5  # 3m çap

# Farklı maksimum derinlik
python3 main.py --max-depth 0.8  # 80cm max

# Daha fazla iterasyon
python3 main.py --iterations 7

# Daha uzun test süresi
python3 main.py --test-duration 20  # 20 saniye per test
```

## 🔄 Algoritma

### 1. **Round-Robin Grid Search**
- **İterasyon 0**: Kd sabit, Kp-Ki taranır (3x3 = 9 test)
- **İterasyon 1**: Ki sabit, Kp-Kd taranır (3x3 = 9 test)  
- **İterasyon 2**: Kp sabit, Ki-Kd taranır (3x3 = 9 test)
- **İterasyon 3+**: Döngü devam eder
- Her iterasyon: %60 aralık daraltma

### 2. **Sabit Frekanslı Kontrol**
- **20 Hz** sabit güncelleme döngüsü
- `time.monotonic()` ile hassas zaman kontrolü
- Gerçek zamanlı güvenlik sınırları

### 3. **Skor Sistemi**
```
Skor = Ortalama_Hata + (Max_Hata × 0.3) + (Oscillation × 2.0)
```
- **Düşük skor = Daha iyi performans**
- Hata, oscillation ve overshoot'u dengeler

### 4. **Güvenlik Sistemleri**
- **Derinlik Sınırı**: Hedef + 0.3m (soft limit)
- **Pitch/Roll Sınırı**: ±25° (acil durdurma)
- **Test Alanı**: 4m çap otomatik kontrol
- **Erken Çıkış**: Aşırı oscillation tespiti

### 5. **Test Derinlikleri**
- **0.3m**: Yüzey yakını hassasiyet
- **0.5m**: Orta derinlik kararlılık
- **0.7m**: Derin dalış response
- **1.0m**: Maksimum derinlik performans

## 📊 Çıktılar

### 1. **Konsol Logları**
```
🔄 İTERASYON 1/5
İterasyon 1: 9 test (sabit=kd)
kp aralığı: [170.0, 200.0, 230.0]
ki aralığı: [8.0, 10.0, 12.0]
kd sabit: 50.0

Test ediliyor: Kp=200.0, Ki=10.0, Kd=50.0
  → 0.3m derinlik testi başlatılıyor...
    AvgErr=0.045m  MaxErr=0.120m  Osc=0.023m  Skor=0.127
🎯 Yeni en iyi PID bulundu!
   Kp: 225.0, Ki: 8.0, Kd: 50.0, Skor: 0.098
```

### 2. **JSON Sonuç Dosyası**
```json
{
  "timestamp": "20241225_143022",
  "best_pid": {
    "kp": 225.0,
    "ki": 8.0,
    "kd": 45.0
  },
  "best_score": 0.098,
  "all_results": [...]
}
```

### 3. **Config Güncellemesi**
```python
# optimized_config.py
DEPTH_KP = 225.0  # P kontrolcü katsayısı (önceki: 200.0)
DEPTH_KI = 8.0    # I kontrolcü katsayısı (önceki: 10.0)
DEPTH_KD = 45.0   # D kontrolcü katsayısı (önceki: 50.0)
```

## ⚠️ Güvenlik Uyarıları

1. **Test Alanı**: Engelsiz, güvenli su alanı
2. **Derinlik**: Minimum 1.2m su derinliği (güvenlik payı)
3. **Gözetim**: Test sırasında sürekli gözetim
4. **Acil Durdurma**: Emergency stop butonu hazır
5. **Sınırlar**: 4m çap dışına çıkarsa manuel müdahale

## 🔧 Konfigürasyon

### PID Arama Aralıkları:
```python
self.pid_ranges = {
    'kp': {'min': 50.0, 'max': 400.0},   # Proportional gain
    'ki': {'min': 2.0, 'max': 20.0},     # Integral gain
    'kd': {'min': 10.0, 'max': 100.0}    # Derivative gain
}
```

### Test Parametreleri:
```python
self.max_iterations = 5      # İterasyon sayısı
self.test_duration = 15      # Test süresi (saniye)
self.settle_time = 3         # Stabilizasyon bekleme
```

## 📈 Beklenen Sonuçlar

### Tipik Optimizasyon:
- **Başlangıç**: Kp=200, Ki=10, Kd=50 (Skor: ~0.15)
- **Optimize**: Kp=180-250, Ki=6-12, Kd=35-65 (Skor: <0.10)
- **İyileştirme**: %30-50 daha iyi stabilite

### Performans Metrikleri:
- **Ortalama Hata**: <0.05m
- **Maksimum Hata**: <0.15m
- **Oscillation**: <0.03m
- **Settling Time**: <5 saniye

## 🐛 Sorun Giderme

### MAVLink Bağlantısı:
```bash
# Port kontrolü
ls /dev/ttyACM*
# veya
ls /dev/ttyUSB*
```

### Sensör Problemi:
```bash
# D300 sensör testi
i2cdetect -y 1
# 0x76 adresinde görünmeli
```

### Kötü Sonuçlar:
- Su koşulları (akıntı, dalga)
- Sensör kalibrasyonu
- Mekanik sorunlar (servo, motor)
- Test alanı çok küçük

## 📞 Destek

Sorun yaşarsanız:
1. Log dosyalarını kontrol edin
2. Sensör bağlantılarını test edin
3. Test alanı koşullarını gözden geçirin
4. Manuel PID testleri yapın

---

**🎯 Hedef**: En stabil, hızlı ve hassas derinlik kontrolü için optimize edilmiş PID parametreleri!
