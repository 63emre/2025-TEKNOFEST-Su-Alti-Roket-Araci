# Manuel Kalibrasyon Paketi

Bu klasör, hareket gerektiren sensör kalibrasyonları için geliştirilmiştir.

## 📁 Dosyalar

### 1. `accelerometer_calibration.py`
**İvmeölçer Kalibrasyonu** - Roll ve Pitch açısı hesaplaması için
- **Gereksinim**: Kart 6 farklı pozisyona çevrilmeli
- **Süre**: ~15 dakika
- **Pozisyonlar**: Üst, Alt, Sağ, Sol, İleri, Geri
- **Çıktı**: `accelerometer_calibration.json`

### 2. `compass_calibration.py` 
**Pusula/Manyetometre Kalibrasyonu** - Yaw açısı ve 180° dönüş kontrolü için
- **Gereksinim**: Kart yatay pozisyonda 360° dönüş
- **Süre**: ~5 dakika
- **Ortam**: Açık alan, manyetik girişimden uzak
- **Çıktı**: `compass_calibration.json`, `compass_calibration_plot.png`

### 3. `manual_calibration_suite.py`
**Ana Kalibrasyon Programı** - Tüm kalibrasyonları yönetir
- Menü ile kalibrasyon seçimi
- Tüm kalibrasyonları sırayla çalıştırma
- Sonuç raporlama

## 🚀 Kullanım

### Tek Kalibrasyon
```bash
# İvmeölçer kalibrasyonu
python accelerometer_calibration.py

# Pusula kalibrasyonu  
python compass_calibration.py
```

### Tüm Kalibrasyonlar
```bash
python manual_calibration_suite.py
```

## 📋 Gereksinimler

```bash
pip install numpy matplotlib pymavlink
```

## ⚠️ Önemli Notlar

1. **Bu kalibrasyonlar main.py tarafından kullanılmaz**
2. **Sadece manuel kalibrasyon için ayrı çalıştırılır**
3. **Görev sistemi hareketsiz kalibrasyonları otomatik yapar**
4. **Bu dosyalar isteğe bağlı hassas ayar içindir**

## 📊 Kalibrasyon Sonuçları

- `accelerometer_calibration.json`: İvmeölçer offset ve scale değerleri
- `compass_calibration.json`: Pusula hard iron, soft iron kompanzasyonu  
- `compass_calibration_plot.png`: Pusula kalibrasyonu görselleştirme

## 🔧 Teknik Detaylar

### İvmeölçer Kalibrasyonu
- **Yöntem**: 6 yönlü bias ve scale düzeltme
- **Algoritma**: Min/Max analizi ile offset/scale hesaplama
- **Doğruluk**: ±0.01G hassasiyet

### Pusula Kalibrasyonu  
- **Yöntem**: Hard iron ve soft iron kompanzasyonu
- **Algoritma**: Elipsoid fitting ve normalleştirme
- **Kalite**: 0-100 skoru ile değerlendirme

Bu kalibrasyonlar, sensör doğruluğunu maksimize etmek için isteğe bağlı olarak yapılabilir.
