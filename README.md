# 🚀 TEKNOFEST 2025 - Su Altı ROV Kontrol Sistemi

## 🖥️ Terminal GUI (Ana Arayüz)

Gerçek zamanlı terminal arayüzü ile kontrol:
- **📱 Real-time telemetry**
- **⌨️ Klavye kontrolleri**
- **MAVLink** veri akışı

### 🚀 Hızlı Başlangıç:
```bash
git clone https://github.com/kullanici/2025-TEKNOFEST-Su-Alti-Roket-Araci.git
cd 2025-TEKNOFEST-Su-Alti-Roket-Araci/App
sudo bash setup_raspberry_pi.sh
python3 terminal_gui.py
```

## 📋 Proje Hakkında

Bu proje, 2025 TEKNOFEST Su Altı Roket Aracı Yarışması için geliştirilmiş otonom sualtı aracı kontrol sistemidir. Araç, sualtı navigasyonu yaparak belirlenen koordinatlara gidebilme, roket fırlatma ve güvenli şekilde yüzeye çıkabilme kabiliyetlerine sahiptir.

## 🎯 Yarışma Görevleri

### Görev 1: Navigasyon
- Belirlenen koordinatlara otonom navigasyon
- GPS ve sualtı sensör entegrasyonu
- Yol planlama ve engel kaçınma

### Görev 2: Roket Fırlatma
- Hedefe kilitlenme ve roket fırlatma sistemi
- Timing ve hassasiyet kontrolleri
- Güvenlik protokolleri

## 📁 Proje Yapısı

```
2025-TEKNOFEST-Su-Alti-Roket-Araci/
├── Görevler/                          # Ana görev scriptleri
│   ├── mission_1_navigation.py        # Navigasyon görevi
│   ├── mission_2_rocket_launch.py     # Roket fırlatma görevi
│   ├── mission_manager.py             # Görev yöneticisi
│   └── README.md                      # Görevler hakkında detaylı bilgi
├── Kabiliyet Videosu/                 # Demo ve kabiliyet testleri
│   ├── demo_emergency_stop.py         # Acil durum testi
│   ├── demo_full_capability.py        # Tam kabiliyet testi
│   ├── demo_maneuver_capabilities.py  # Manevra kabiliyeti testi
│   ├── demo_rocket_separation.py      # Roket ayrışma testi
│   ├── demo_waterproof_test.py        # Su geçirmezlik testi
│   └── README.md                      # Demo açıklamaları
├── Test/                              # Sistem testleri
│   ├── test_d300_depth_sensor.py      # Derinlik sensörü testi
│   ├── test_depth_hold.py             # Derinlik tutma testi
│   ├── test_full_system.py            # Tam sistem testi
│   ├── test_gpio_button.py            # GPIO buton testi
│   ├── test_led_buzzer.py             # LED ve buzzer testi
│   ├── test_mavlink_connection.py     # MAVLink bağlantı testi
│   ├── test_motor_control.py          # Motor kontrol testi
│   ├── test_servo_control.py          # Servo kontrol testi
│   ├── test_stabilization.py          # Stabilizasyon testi
│   └── README.md                      # Test prosedürleri
├── HARDWARE_PIN_MAPPING.md            # Hardware pin bağlantıları
└── Dökümanlar/                        # PDF yarışma belgeleri
    ├── 2025_SU_ALTI_ROKET_YARIŞMASI_ŞARTNAMESİ_TR_2_KR4v0 (1).pdf
    └── SU_ALTI_ROKET_FIRLATMA_SİSTEMİ_BİLGİLENDİRME_DÖKÜMANI_PQXdm.pdf
```

## 🔧 Sistem Gereksinimleri

### Hardware
- Raspberry Pi 4 (Ana işlemci)
- ArduSub uyumlu flight controller
- D300 Derinlik sensörü
- GPS modülü
- Kamera sistemi
- Motor kontrol ünitesi
- Servo motorlar
- LED ve buzzer
- Emergency stop butonu

### Software
- Python 3.8+
- MAVLink protokolü
- OpenCV (görüntü işleme)
- NumPy, SciPy (matematik hesaplamalar)
- PyMavlink (drone komunikasyonu)

## 🚀 Kurulum ve Çalıştırma

### 1. Bağımlılıkları Yükleme
```bash
# Gerekli Python paketleri
pip install pymavlink opencv-python numpy scipy

# Sistem paketleri (Raspberry Pi için)
sudo apt update
sudo apt install python3-pip python3-opencv
```

### 2. Hardware Bağlantıları
Hardware pin bağlantıları için `HARDWARE_PIN_MAPPING.md` dosyasını kontrol edin.

### 3. Test Prosedürü
```bash
# Temel sistem testleri
python Test/test_mavlink_connection.py      # MAVLink bağlantısını test et
python Test/test_motor_control.py           # Motor kontrolünü test et
python Test/test_depth_hold.py              # Derinlik tutma testı
python Test/test_full_system.py             # Tam sistem testi
```

### 4. Görevleri Çalıştırma
```bash
# Ana görev yöneticisini başlat
python Görevler/mission_manager.py

# Veya tekil görevler
python Görevler/mission_1_navigation.py     # Sadece navigasyon
python Görevler/mission_2_rocket_launch.py  # Sadece roket fırlatma
```

## 🎥 Demo ve Kabiliyet Testleri

Yarışma öncesi kabiliyet videosunu çekmek için demo scriptlerini kullanın:

```bash
# Tam kabiliyet testi
python "Kabiliyet Videosu/demo_full_capability.py"

# Manevra kabiliyetleri
python "Kabiliyet Videosu/demo_maneuver_capabilities.py"

# Su geçirmezlik testi
python "Kabiliyet Videosu/demo_waterproof_test.py"

# Roket ayrışma testi  
python "Kabiliyet Videosu/demo_rocket_separation.py"

# Acil durum testi
python "Kabiliyet Videosu/demo_emergency_stop.py"
```

## ⚠️ Güvenlik Protokolleri

### Acil Durum Prosedürü
1. **Emergency Stop**: Hardware butonu ile sistem derhal durdurulabilir
2. **Manuel Kontrol**: Otopilot devre dışı bırakılarak manuel kontrol geçilebilir
3. **Yüzey Çıkış**: Acil durumda araç otomatik olarak yüzeye çıkar
4. **Roket Güvenlik**: Roket sistemi fail-safe modunda çalışır

### Operasyon Öncesi Kontrol Listesi
- [ ] Hardware bağlantıları kontrol edildi
- [ ] Battery seviyesi %80 üzerinde
- [ ] GPS sinyali alınıyor
- [ ] MAVLink bağlantısı aktif
- [ ] Emergency stop butonu test edildi
- [ ] Roket güvenlik sistemi aktif
- [ ] Derinlik sensörü kalibre edildi
- [ ] Motor testleri başarılı

## 📊 Performans Metrikleri

- **Navigasyon Hassasiyeti**: ±1 metre
- **Derinlik Kontrolü**: ±0.5 metre  
- **Roket Atış Hassasiyeti**: ±2 metre (50m mesafede)
- **Batarya Süresi**: 45-60 dakika
- **Maksimum Derinlik**: 10 metre
- **Maksimum Hız**: 2 m/s

## 🔍 Sorun Giderme

### Yaygın Problemler
1. **MAVLink bağlantı hatası**: Serial port ayarlarını kontrol edin
2. **GPS sinyali yok**: Antenna bağlantısını ve konumu kontrol edin
3. **Motor çalışmıyor**: ESC kalibrasyonu yapın
4. **Derinlik sensörü okumuyor**: I2C bağlantılarını kontrol edin

### Log Dosyaları
Sistem logları `/var/log/underwater_rocket/` dizininde saklanır.

## 📞 İletişim ve Destek

**Takım**: [Takım Adı]
**E-posta**: [takım@email.com]
**Discord**: [Discord Kanalı]

## 📄 Lisans

Bu proje TEKNOFEST 2025 yarışması kapsamında geliştirilmiştir. Eğitim amaçlı kullanım için açık kaynak kodludur.

---

**Not**: Yarışma öncesi tüm sistem testlerinin tamamlandığından ve güvenlik protokollerinin uygulandığından emin olun.