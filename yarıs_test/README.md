# Hava Yarışı Test Seti

Bu klasör, D300 derinlik sensörü olmadan havada test yapmak için `pluswing` klasöründen uyarlanmış test dosyalarını içerir.

## Özellikler

- ✅ **D300 Derinlik Sensörü KALDIRILDI** - Su altı sensörü yok
- ✅ **Barometric Altitude Sensörü** - D300 yerine hava basıncı ile yükseklik
- ✅ **Havada Test Optimizasyonu** - Güvenli hız ve hareket profilleri
- ✅ **Kısa Test Süreleri** - 10 saniye geri sayım, 5 dakika maksimum test
- ✅ **GPIO Simülasyon Desteği** - Raspberry Pi olmadan da test edilebilir
- ✅ **Attitude Stabilizasyonu** - Roll, Pitch, Yaw kontrolleri aktif
- ✅ **U-Dönüş Testi** - 180° dönüş manevrası

## Dosya Yapısı

```
yarıs_test/
├── config_air.py          # Hava yarışı konfigürasyonu
├── sensors_air.py         # Sensör yönetimi (D300 YOK)
├── control_air.py         # Kontrol sistemi (altitude kontrolü)
├── utils_air.py           # Yardımcı fonksiyonlar
├── main_air_test.py       # Ana test programı
├── mission_air_test.py    # Test görev mantığı
└── README.md              # Bu dosya
```

## Ana Değişiklikler

### D300 Derinlik Sensörü Kaldırıldı
- `DepthSensor` sınıfı tamamen kaldırıldı
- `AltitudeSensor` sınıfı eklendi (barometric pressure)
- Derinlik kontrolü deaktif, altitude kontrolü aktif

### Hava Testi İçin Optimizasyon
- Motor hızları düşürüldü (güvenlik için)
- Test mesafeleri kısaltıldı (20m + 30m)
- Geri sayım 90 saniyeden 10 saniyeye düşürüldü
- Maksimum test süresi 5 dakika

### GPIO Simülasyon
- Raspberry Pi olmadan da çalışır
- GPIO komutları konsola yazdırılır
- Gerçek Pi'de normal GPIO çalışır

## Kullanım

### 1. Bağlantı Testi
```bash
cd yarıs_test
python3 main_air_test.py --test-only
```

### 2. Tam Test Çalıştırma
```bash
python3 main_air_test.py
```

### Test Akışı
1. **MAVLink Bağlantısı** - Pixhawk bağlantısı
2. **Sensör Testi** - Attitude ve Altitude sensörleri
3. **Kalibrasyon** - Yer seviyesi ve yaw referansı
4. **Buton Bekleme** - Test başlatma butonu
5. **10 Saniye Geri Sayım** - Güvenlik gecikmesi
6. **Test Fazları**:
   - Faz 1: 20m ileri hareket
   - Faz 2: 30m daha ileri (toplam 50m)
   - Faz 3: 180° U-dönüş
   - Faz 4: 50m geri dönüş
   - Faz 5: İniş manevrası
   - Faz 6: Sistem kapatma

## Güvenlik Özellikleri

- **Düşük Motor Hızları**: Hava testinde güvenli hızlar
- **Acil Durdurma**: Buton ile anında durdurma
- **Zaman Aşımı**: Her faz için maksimum süre limiti
- **Altitude Sınırı**: Maksimum 5m test yüksekliği
- **GPIO Temizliği**: Program sonunda güvenli temizlik

## Konfigürasyon

### `config_air.py` Önemli Parametreler

```python
# D300 deaktif
USE_DEPTH_CONTROL = False
SIMULATE_DEPTH = True

# Hava testi süreleri
ARMING_DELAY_SECONDS = 10    # 10 saniye geri sayım
MISSION_TIMEOUT_SECONDS = 300 # 5 dakika maksimum

# Test mesafeleri
TARGET_DISTANCE_PHASE1 = 20.0  # 20m
TARGET_DISTANCE_PHASE2 = 30.0  # 30m

# Güvenli hızlar
SPEED_SLOW = 1550    # Düşük hız
SPEED_MEDIUM = 1650  # Orta hız
SPEED_FAST = 1750    # Maksimum hız
```

## Sensör Durumu

| Sensör | Durum | Açıklama |
|--------|-------|----------|
| D300 Derinlik | ❌ KALDIRILDI | Su altı sensörü yok |
| Altitude | ✅ AKTİF | Barometric pressure |
| Attitude | ✅ AKTİF | Roll/Pitch/Yaw |
| System Status | ✅ AKTİF | Pixhawk durumu |

## Hata Ayıklama

### MAVLink Bağlantı Sorunları
- Port listesini kontrol edin: `ls /dev/ttyACM*`
- Baud rate: 115200
- Heartbeat timeout: 5 saniye

### GPIO Sorunları
- Raspberry Pi'de: `sudo` ile çalıştırın
- Simülasyon modunda: GPIO komutları konsola yazdırılır

### Sensör Sorunları
- Attitude sensörü kritik (test durur)
- Altitude sensörü opsiyonel (simüle edilir)

## Log Dosyaları

Test sırasında `air_race_test.log` dosyasına detaylı loglar yazılır.

## Orijinal Pluswing Farkları

| Özellik | Pluswing | Hava Yarışı |
|---------|----------|-------------|
| D300 Sensörü | ✅ Aktif | ❌ Kaldırıldı |
| Derinlik Kontrolü | ✅ 2-2.5m | ❌ Deaktif |
| Altitude Kontrolü | ❌ Yok | ✅ Yer seviyesi |
| Test Süresi | 90s geri sayım | 10s geri sayım |
| Motor Hızları | Su altı optimize | Hava optimize |
| Mesafeler | 10m + 40m | 20m + 30m |

Bu test seti, orijinal pluswing kodunun havada güvenli şekilde test edilebilmesi için özel olarak uyarlanmıştır.
