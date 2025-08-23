# 🚀 GÖREVLER KLASÖRÜ - TEKNOFEST Su Altı Roket Aracı

Bu klasör, TEKNOFEST Su Altı Roket Aracı Yarışması'nın ana görevlerini gerçekleştiren mission scriptlerini içermektedir.

## 🎯 Yarışma Görevleri

### 📋 Görev 1: Seyir Yapma & Başlangıç Noktasına Geri Dönüş (300 puan)

#### 🎯 Görev Tanımı
- Başlangıç bölgesinden **2 m derinlikte**, düz istikamette **10 m** ilerledikten sonra süre başlatılır
- Kıyıdan **en az 50 m** uzaklaşıldıktan sonra başlangıç noktasına otonom geri dönmelidir
- Şamandıra ile araç dışarıdan izlenebilir olmalıdır

#### 📊 Puanlama Sistemi
- **Seyir Yapma** (en hızlı tamamlama): **150 puan**
- **Başlangıç Noktasında Enerjiyi Kesme** (pozitif sephiye ile yüzme): **90 puan**
- **Görev Boyunca Sızdırmazlık**: **60 puan**
- **Süre Limiti**: 5 dakika
- **Maksimum Puan**: 300

### 🚀 Görev 2: Roket Ateşleme (400 puan)

#### 🎯 Görev Tanımı
- Aracın, yarışma tarafından tanımlanan **güvenli atış bölgesine** otonom ulaşması
- Uygun yunuslama açısıyla yüzeye çıkış ve **+30° eğim** koşulu sağlandığında roket taşıma bölmesinin açılması
- Fiziksel model roket fırlatılması **beklenmez**, yalnızca ayrılma mekanizması gözlemlenmelidir

#### 📊 Puanlama Sistemi
- **Güvenli Atış Bölgesine Ulaşma**: **100 puan**
- **Su Yüzeyine İstenen Açıyla Güvenli Çıkış**: **100 puan**
- **Model Roketin Güvenli Ayrılması**: **150 puan**
- **Sızdırmazlık**: **50 puan**
- **Süre Limiti**: 5 dakika
- **Maksimum Puan**: 400

## 📁 Script Yapısı

### 🎯 Ana Görev Scriptleri

#### **X-Wing Konfigürasyonu (AUX 3,4,5,6)**
- `xwing/mission_1_nav.py` - Görev 1: Seyir & Geri Dönüş (X-konfigürasyon)
- `xwing/mission_2_rocket_launch.py` - Görev 2: Roket Ateşleme 
- `xwing/mission_manager.py` - Ana görev yöneticisi

#### **Plus-Wing Konfigürasyonu (AUX 3,4,5,6)**
- `pluswing/mission_1_navigation_plus.py` - Görev 1: Seyir & Geri Dönüş (Plus-konfigürasyon)

#### **Ortak Konfigürasyon**
- **Motor**: AUX 1 (DEGZ M5)
- **4 Servo**: AUX 3,4,5,6 (DS3230MG)
- **AUX 2**: BOZUK - Kullanılmaz

### 🧠 Navigasyon Sistemi
- `navigation_controller.py` - Otonom navigasyon kontrol sistemi
- `waypoint_manager.py` - Waypoint tabanlı rota planlama
- `position_estimator.py` - Pozisyon tahmini ve GPS entegrasyonu

### 🔧 Kontrol Sistemleri  
- `depth_controller.py` - Derinlik tutma sistemi
- `attitude_controller.py` - Attitude stabilizasyon sistemi
- `motor_controller.py` - Motor kontrolü ve hız yönetimi

### 🛡️ Güvenlik Sistemleri
  Yok

## 🛠️ Sistem Gereksinimleri

### Donanım - TEKNOFEST Standart (HARDWARE_PIN_MAPPING.md)
- **Raspberry Pi 4B** - GPIO kontrol sistemi
- **Pixhawk PX4 PIX 2.4.8** - MAVLink Serial (115200 baud)
- **D300 Derinlik Sensörü** (I2C 0x76, GPIO 2,3) - derinlik ölçümü
- **IMU sensörleri** (Pixhawk entegre MPU6000) - attitude kontrolü
- **4x DS3230MG Servo** (AUX 3,4,5,6) - kanat kontrolü
- **DEGZ M5 Motor + ESC** (AUX 1) - itki sistemi
- **Status LED** (GPIO 4) - durum gösterimi
- **Buzzer** (GPIO 13) - sesli uyarı
- **Power/Emergency Buttons** (GPIO 18,19) - güvenlik kontrolleri

### Yazılım Bağımlılıkları
```bash
# Temel gereksinimler
pip install pymavlink        # MAVLink protokolü (Pixhawk)
pip install RPi.GPIO         # GPIO kontrol (Raspberry Pi)
pip install smbus2           # I2C haberleşme (D300 sensör)
pip install numpy            # Numerik hesaplamalar
pip install scipy            # PID kontrolcü ve filtreler
pip install geopy            # GPS koordinat hesaplamaları
pip install UTM              # Koordinat dönüşümleri

# Görselleştirme (opsiyonel)
pip install matplotlib       # Grafik çizim
pip install plotly           # İnteraktif grafikler

# Pin Mapping Referansı: HARDWARE_PIN_MAPPING.md
# Tüm pin tanımları ve bağlantı şeması burada standardize edilmiştir
```

## 🚀 Kullanım

### Tek Görev Çalıştırma

#### Görev 1 - Seyir Yapma

**X-Wing Konfigürasyonu:**
```bash
cd Görevler/xwing
python mission_1_nav.py
```

**Plus-Wing Konfigürasyonu:**
```bash
cd Görevler/pluswing
python mission_1_navigation_plus.py --start-heading 0.0
```

#### Görev 2 - Roket Ateşleme (X-Wing)
```bash
cd Görevler/xwing
python mission_2_rocket_launch.py --launch-lat 40.123456 --launch-lon 29.123456
```

### Tam Görev Sırası
```bash
# Otomatik görev sırası (1 -> 2)
python mission_manager.py --auto-sequence

# Manuel görev seçimi
python mission_manager.py --interactive
```

### Konfigürasyon Dosyası
```bash
# Görev parametrelerini config dosyasından yükle
python mission_manager.py --config mission_config.json
```

## ⚙️ Konfigürasyon

### Mission Parametreleri
```json
{
  "mission_1": {
    "target_depth": 2.0,
    "straight_distance": 10.0,
    "min_offshore_distance": 50.0,
    "cruise_speed": 1.5,
    "timeout_seconds": 300
  },
  "mission_2": {
    "target_launch_angle": 30.0,
    "surface_depth_threshold": 0.5,
    "approach_speed": 1.0,
    "timeout_seconds": 300
  },
  "safety": {
    "max_depth": 5.0,
    "min_battery_voltage": 20.0,
    "leak_threshold": 10.0,
    "max_roll_angle": 45.0,
    "max_pitch_angle": 60.0
  }
}
```

### Waypoint Örneği
```json
{
  "mission_1_waypoints": [
    {"lat": 40.123456, "lon": 29.123456, "depth": 2.0, "speed": 1.5},
    {"lat": 40.124000, "lon": 29.123456, "depth": 2.0, "speed": 1.5},
    {"lat": 40.124500, "lon": 29.123456, "depth": 2.0, "speed": 1.5}
  ],
  "mission_2_launch_zone": {
    "lat": 40.125000, "lon": 29.124000, "depth": 0.0, "radius": 10.0
  }
}
```

## 📊 Performans İzleme

### Real-time Telemetri
```bash
# Telemetri izleme
python telemetry_monitor.py --mission 1

# Görsel navigasyon takibi
python navigation_visualizer.py --real-time
```

### Log Analizi
```bash
# Görev sonrası analiz
python mission_analyzer.py --log mission_1_log.json

# Performans raporu
python performance_report.py --mission-logs logs/
```

## 🎯 Görev Stratejileri

### Görev 1 Stratejisi
```
1. 📍 Başlangıç pozisyonu kaydet (GPS)
2. 🌊 2m derinliğe kontrollü iniş
3. ➡️ Düz seyir (10m) - süre başlatılır
4. 🚀 Hızlanarak kıyıdan uzaklaş (≥50m)
5. 🔄 U dönüş başlangıç noktasına
6. 📍 Başlangıç noktasında pozitif sephiye
7. ⚡ Sistem enerji kesimi
```

### Görev 2 Stratejisi  
```
1. 🎯 Güvenli atış bölgesine navigasyon
2. 🌊 Su altından yüzeye çıkış
3. 📐 +30° pitch açısı ayarı
4. 🚀 Roket taşıma bölmesi açılması
5. ✅ Ayrılma mekanizması gözlemlenmesi
6. 🛡️ Güvenlik prosedürleri
```

## ⚠️ Güvenlik Protokolleri

### Görev Öncesi Kontroller
- [ ] GPS sinyali yeterli (≥6 uydu)
- [ ] Batarya şarj seviyesi ≥80%
- [ ] Tüm sensörler çalışır durumda
- [ ] Sızdırmazlık testleri başarılı
- [ ] Acil durdurma sistemi aktif
- [ ] İletişim kalitesi test edilmiş

### Görev Sırası Güvenlik
- [ ] Görev başlamadan güvenlik briefingi
- [ ] Gözlemci teknesi hazır pozisyonda
- [ ] Acil müdahale ekipmanları hazır
- [ ] Hava durumu görev için uygun
- [ ] Diğer deniz trafiği bilgilendirilmiş

### Acil Durumlar
- **Sızıntı tespit edilirse**: Derhal yüzeye çık, görev iptal
- **GPS sinyali kaybolursa**: Dead reckoning ile ev dönüşü
- **Batarya düşerse**: Güvenli yüzeye çıkış, acil sinyal
- **İletişim kesintisi**: Önceden programlı acil prosedür
- **Beklenmeyen engel**: Obstacle avoidance, rota değişikliği

## 📈 Performans Optimizasyonu

### Hız Optimizasyonu
- **Seyir hızı**: 1.5-2.0 m/s (max efficiency)
- **Dönüş hızı**: 1.0-1.5 m/s (kontrollü)
- **Yüzey çıkışı**: 0.8-1.2 m/s (güvenli)

### Enerji Yönetimi
- **Cruise mode**: Düşük güç tüketimi
- **Maneuvering**: Orta güç
- **Emergency**: Maksimum güç kullanımı
- **Surface operations**: Minimal güç

### Navigasyon Hassasiyeti
- **GPS precision**: ±2m (surface)
- **Dead reckoning**: ±5m (underwater)
- **Depth accuracy**: ±0.1m
- **Heading accuracy**: ±2°

## 🔍 Debug ve Troubleshooting

### Yaygın Problemler

#### Navigasyon
- **GPS kayıp**: Dead reckoning devreye girer
- **Yön sapması**: Compass kalibrasyonu gerekir
- **Derinlik hatası**: Basınç sensörü kontrolü

#### Kontrol
- **Stabilizasyon**: PID parametrelerini ayarla
- **Motor tepkisi**: ESC kalibrasyonu yap
- **Servo jitter**: PWM frekansını kontrol et

#### İletişim
- **MAVLink timeout**: Bağlantı kalitesini kontrol et
- **Telemetri kayıp**: Anten pozisyonunu optimize et
- **Command delay**: Network latency'i minimize et

### Log Dosyaları
```
logs/
├── mission_1_YYYYMMDD_HHMMSS.json     # Görev 1 log
├── mission_2_YYYYMMDD_HHMMSS.json     # Görev 2 log
├── navigation_YYYYMMDD_HHMMSS.json    # Navigasyon log
├── telemetry_YYYYMMDD_HHMMSS.json     # Telemetri log
└── system_errors_YYYYMMDD.log         # Hata log
```

## 🏆 Yarışma Günü Protokolü

### Sabah Hazırlık (Yarışma Öncesi)
1. **Sistem test süreci** (2 saat)
2. **Güvenlik kontrolleri** (30 dk)
3. **GPS koordinat ayarlama** (15 dk)
4. **Son kalibrasyon** (15 dk)

### Yarışma Sırası
1. **Check-in ve görev briefing**
2. **Sistem hazırlama** (10 dk)
3. **Görev 1 çalıştırma** (5 dk)
4. **Ara değerlendirme** (5 dk)  
5. **Görev 2 çalıştırma** (5 dk)
6. **Sonuç değerlendirmesi**

### Yarışma Sonrası
1. **Veri download** ve yedekleme
2. **Sistem kontrolü** ve temizleme
3. **Performans analizi**
4. **Sonuç raporu** hazırlama

---

## 📞 Destek ve İletişim

**Teknik Destek**: Sistem hatası durumunda test scriptlerini çalıştırın
**Görev Desteği**: README dosyalarını detaylı inceleyin
**Acil Durum**: Güvenlik protokollerini takip edin

---

*Bu README dosyası TEKNOFEST 2025 Su Altı Roket Aracı Yarışması için özel olarak hazırlanmıştır.*
*Başarılı görev tamamlama için talimatları eksiksiz takip edin.*

**🚀 TEKNOFEST 2025 - SU ALTI ROKET ARACI**
