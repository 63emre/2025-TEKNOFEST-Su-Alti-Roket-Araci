# 🎬 KABİLİYET VİDEOSU KLASÖRÜ - TEKNOFEST Su Altı Roket Aracı

Bu klasör, TEKNOFEST yarışması için gerekli **Kabiliyet Gösterim Videosu** (2-5 dk) çekimi scriptlerini içerir.

## 🎯 Video Gereksinimleri (TEKNOFEST Şartnamesi)

### 📹 Teknik Gereksinimler
- **Çözünürlük**: En az 720p
- **Süre**: 2-5 dakika
- **Platform**: YouTube (liste dışı/unlisted)
- **Format**: MP4 önerilen

### 🏆 Gösterilmesi Gereken Yetenekler

#### 1. Sızdırmazlık Testi ✅
- **≥ 1 metre** derinlikte hiçbir noktasından kabarcık çıkmaması
- Statik ve dinamik testler
- Su altı açılır kapak mekanizmasının su almadığını göstermek

#### 2. Hareket Kabiliyeti ✅
- **En az 1 dakika** boyunca kontrollü manevralar
- Seyir, dönüş, yunuslama hareketleri
- İkinci görevde burun kapağı açılmadan yüzeye çıkış
- Kontrolsüz hareket kabul edilmez

#### 3. Roket Ateşleme Gösterimi ✅
- Su yüzeyinde uygun pozisyonda sinyal alımı
- Ayrılma mekanizmasının çalışması
- **Model roket fırlatılması gerekmez**, sadece ayrılma

#### 4. Acil Durdurma Sistemi ✅
- Acil durdurma butonunun çalıştığının gösterilmesi
- Butona basıldığında motorların durması
- Sistemin güvenli şekilde kapanması

## 📋 Script Yapısı

### 🎬 Demo Scriptleri
- `demo_waterproof_test.py` - Sızdırmazlık testi gösterimi
- `demo_maneuver_capabilities.py` - Manevrabilite gösterimi  
- `demo_rocket_separation.py` - Roket ayrılma mekanizması
- `demo_emergency_stop.py` - Acil durdurma sistemi
- `demo_full_capability.py` - Tam kabiliyet gösterimi (tüm testler)

### 📊 Video Çekim Araçları
- `video_telemetry_overlay.py` - Telemetri overlay sistemi
- `video_sequence_manager.py` - Video çekim sırası yöneticisi
- `video_data_logger.py` - Video eşzamanlı veri kaydı

## 🎥 Çekim Rehberi

### Önerilen Çekim Sırası
```
1. 📋 Sistem Tanıtımı (30sn)
   - Aracın genel görünümü
   - X-konfigürasyon fin sistemi
   - Ana bileşenler (D300, LED, Buzzer)
   - Pin mapping standardı

2. 🔧 Acil Durdurma Testi (30sn)
   - 16A power button ile açılış
   - 90 saniye güvenlik gecikmesi
   - GPIO 19 acil durdurma testi
   - 40A relay ile güvenli kapanış

3. 💧 Sızdırmazlık Testi (90sn)
   - D300 sensör ile derinlik ölçümü (≥1m)
   - X-fin sistemi ile statik test
   - Hareket halinde test (fin manevralar)
   - Payload bay kapak mekanizması testi

4. 🚀 Hareket Kabiliyeti (120sn)
   - X-fin roll kontrolü (sol-sağ finler)
   - X-fin pitch kontrolü (ön-arka finler)  
   - X-diagonal yaw kontrolü
   - MAIN 1 motor ile düz seyir
   - RGB LED durum gösterimi
   - Yüzeye çıkış (burun kapağı kapalı)

5. 🎯 Roket Ayrılma (45sn)
   - Güvenli atış bölgesine navigasyon
   - +30° pitch açısı (X-fin kontrolü)
   - AUX 6 payload bay açılması
   - AUX 7 separation mechanism
   - LED/buzzer ile onay sinyali

6. 📊 Sonuçlar (15sn)
   - Test başarı oranları
   - D300 sensör verileri
   - Sistem performans göstergeleri
```

### 📹 Kamera Pozisyonları
- **Su üstü kamera**: Genel görünüm ve yüzey operasyonları
- **Su altı kamera**: Sızdırmazlık ve manevralar
- **Yakın çekim**: Ayrılma mekanizması detayı
- **Telemetri ekranı**: Veri gösterimi

## 🛠️ Kullanım

### Sistem Hazırlık
```bash
# Pin mapping kontrolü (ÖNEMLİ!)
cat ../HARDWARE_PIN_MAPPING.md

# Telemetri sistemi başlat
python video_telemetry_overlay.py &

# Video kayıt sistemi hazırla  
python video_sequence_manager.py
```

### Hardware Gereksinimleri - Pin Standardı
```
🔌 PIXHAWK PIN MAPPING:
  MAIN 1-8: Ana motor (DEGZ M5) MAIN 1'de
  AUX 1-4:  X-konfigürasyon finler (DS3230MG)
  AUX 5-7:  Elevator, Payload, Separation

🤖 RASPBERRY Pi GPIO:
  GPIO 18:  Power Button (16A Metal)
  GPIO 19:  Emergency Stop  
  GPIO 21:  40A Relay Control
  GPIO 2,3: D300 I2C (Derinlik Sensörü)
  GPIO 4,5,6: RGB LED Status
  GPIO 13,25: Buzzer System
```

### Test Çekimleri

#### Sızdırmazlık Testi
```bash
python demo_waterproof_test.py
```

#### Manevrabilite Testi
```bash
python demo_maneuver_capabilities.py
```

#### Roket Ayrılma Testi
```bash
python demo_rocket_separation.py
```

#### Tam Kabiliyet Gösterimi
```bash
python demo_full_capability.py
```

## 📊 Telemetri ve Veri

### Gösterilecek Veriler
- **Attitude**: Roll, Pitch, Yaw açıları (derece)
- **Depth**: Mevcut derinlik (metre)  
- **Speed**: Seyir hızı (m/s)
- **Position**: GPS koordinatları
- **System Status**: Sistem durum bilgileri
- **Battery**: Batarya voltajı ve yüzdesi
- **Time**: Test süresi (saniye)

### Veri Overlay Formatı
```
┌─ TEKNOFEST Su Altı Roket Aracı ─┐
│ Time: 02:34 / 05:00            │
│ Depth: 1.2m  Speed: 0.8 m/s    │
│ Roll: +2.1°  Pitch: -1.5°      │
│ Yaw: 045°   Battery: 87%       │
│ Status: [STABLE] [ROCKET_READY] │
└─────────────────────────────────┘
```

## 🎬 Video Montaj Önerileri

### Başlık Ekranı
```
🚀 TEKNOFEST 2025
Su Altı Roket Aracı
Kabiliyet Gösterim Videosu

Takım Adı: [TAKIMINIZ]
Tarih: [TARİH]
```

### Geçiş Efektleri
- Test arası geçişlerde telemetri ekranı
- Önemli anları slow-motion
- Başarılı testlerde ✅ işareti
- Kritik anları zoom ile vurgulama

### Ses ve Müzik
- Arka plan müziği: Enerjik ama dikkat dağıtmayacak
- Test açıklamaları: Net ve anlaşılır seslendirme
- Önemli sesler: Motor sesi, ayrılma sesi vb.

## ⚠️ Güvenlik Uyarıları

### Video Çekimi Sırasında
1. **Güvenlik ekibi** hazır olmalı
2. **Acil müdahale planı** belirlenmiş olmalı
3. **Yedek sistemler** hazır tutulmalı
4. **Hava koşulları** uygun olmalı
5. **Test bölgesi** güvenli olmalı

### Test Sırası
1. Her test öncesi **sistem kontrolü**
2. **Acil durdurma** butonunu test et
3. **Batarya seviyesi** kontrol et
4. **Haberleşme** kalitesini kontrol et
5. **Kamera açıları** optimize et

## 📋 Kontrol Listesi

### Video Çekimi Öncesi ✅
- [ ] Kameralar şarjlı ve test edilmiş
- [ ] Su altı aydınlatma hazır
- [ ] Telemetri sistemi çalışıyor
- [ ] Tüm test scriptleri kontrol edilmiş
- [ ] Güvenlik ekibi ve malzemeleri hazır
- [ ] Hava durumu uygun
- [ ] Test bölgesi temizlenmiş

### Video Çekimi Sonrası ✅
- [ ] Tüm test verileri kaydedilmiş
- [ ] Video dosyaları yedeklenmiş
- [ ] Telemetri verileri arşivlenmiş
- [ ] Ekipman temizlenmiş ve kontrol edilmiş
- [ ] Test raporu hazırlanmış
- [ ] Video montaj planı oluşturulmuş

## 📈 Değerlendirme Kriterleri

### Sızdırmazlık (Puanlama)
- ✅ Hiç kabarcık yok: Tam puan
- ⚠️ Minimum kabarcık: Düşük puan  
- ❌ Belirgin sızıntı: Sıfır puan

### Manevrabilite (Puanlama)  
- ✅ Tüm hareketler kontrollü: Tam puan
- ⚠️ Bazı instabilite: Orta puan
- ❌ Kontrolsüz hareket: Düşük puan

### Roket Ayrılma (Puanlama)
- ✅ Temiz ayrılma: Tam puan  
- ⚠️ Gecikme var: Orta puan
- ❌ Ayrılma yok: Sıfır puan

---
*Video çekimi öncesi bu README'yi detaylı inceleyin!*
*Başarılı video çekimi için tüm script'leri önceden test edin!* 