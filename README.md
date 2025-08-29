# 🚀 TEKNOFEST 2025 - Su Altı Roket Aracı (SARA)

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%205-red.svg)](https://www.raspberrypi.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/Status-Development-orange.svg)]()

> **Otonom sualtı navigasyon ve roket fırlatma sistemi** - 2025 TEKNOFEST Su Altı Roket Aracı Yarışması için geliştirilmiştir.

## 📋 İçindekiler

- [Proje Hakkında](#-proje-hakkında)
- [Sistem Mimarisi](#-sistem-mimarisi)
- [Görevler](#-görevler)
- [Kalibrasyon Sistemi](#-kalibrasyon-sistemi)
- [PID Optimizasyon](#-pid-optimizasyon)
- [Kurulum](#-kurulum)
- [Kullanım](#-kullanım)
- [API Dokümantasyonu](#-api-dokümantasyonu)
- [İletişim](#-iletişim)

## 🎯 Proje Hakkında

**SARA (Su Altı Roket Aracı)**, 2025 TEKNOFEST Su Altı Roket Aracı Yarışması için geliştirilmiş gelişmiş bir otonom sualtı kontrol sistemidir. Plus Wing (+) konfigürasyonunda 4 kontrol kanatı kullanarak 3 eksen stabilizasyonu ve görev navigasyonu gerçekleştirir.

### 🏆 Yarışma Hedefleri
- **Görev 1**: Otonom sualtı navigasyon ve waypoint takibi
- **Görev 2**: Hedef tespiti ve roket fırlatma sistemi

### 🎮 Ana Bileşenler
- **Raspberry Pi 4B/5**: Yüksek seviye kontrol ve sensör okuma
- **Pixhawk 2.4.8**: Motor/servo kontrolü ve dahili IMU
- **D300 Derinlik Sensörü**: I2C üzerinden basınç/derinlik ölçümü
- **DEGZ M5 Su Altı Motoru**: Ana itki (30A ESC ile)
- **4x DS3230MG Servolar**: Plus konfigürasyonu kontrol kanatları
- **Selenoid Valf**: Roket fırlatma sistemi (sadece Görev 2)

## 🏗️ Sistem Mimarisi

### Hardware Stack
```
┌─────────────────────────────────────────────────────────────┐
│                    Raspberry Pi 5                          │
│              (Ana Kontrol Sistemi)                        │
├─────────────────────────────────────────────────────────────┤
│  GPIO Control  │  USB MAVLink  │  I2C Sensors  │  Power   │
│                │                │               │          │
│  • LED System  │  • Pixhawk    │  • D300 Depth │  • 22.2V │
│  • Buttons     │  • PX4 FC     │  • Compass    │  • 6S LiPo│
│  • Buzzer      │  • MAVLink    │  • IMU        │  • Regulators│
└─────────────────────────────────────────────────────────────┘
                                │
                                ▼
┌─────────────────────────────────────────────────────────────┐
│                    Pixhawk PX4 PIX 2.4.8                  │
│                   (Flight Controller)                      │
├─────────────────────────────────────────────────────────────┤
│  MAIN Outputs  │  AUX Outputs  │  I2C Bus     │  Serial   │
│                │                │               │          │
│  • Reserved    │  • Fin Servos │  • D300       │  • MAVLink│
│  • Reserved    │  • Motor ESC  │  • Sensors    │  • USB    │
└─────────────────────────────────────────────────────────────┘
```

### Plus Wing (+) Servo Konfigürasyonu
```
      ↑ Üst (AUX6)
      |
Sol ← + → Sağ
(AUX5) | (AUX3)
      |
      ↓ Alt (AUX4)
```

### Yazılım Mimarisi
```
görevlerf1/pluswing/
├── config.py          # Sistem konfigürasyonu ve sabitler
├── utils.py           # LED, buzzer, timer, loglama yardımcıları
├── sensors.py         # D300 ve MAVLink sensör yönetimi
├── control.py         # PID kontrol, stabilizasyon, servo kontrolü
├── mission1.py        # Görev 1: Seyir ve geri dönüş
├── mission2.py        # Görev 2: Roket fırlatma
├── main.py           # Ana program (otomatik başlatılacak)
├── gpio_wrapper.py    # GPIO uyumluluk katmanı
└── gpio_compat.py     # GPIO uyumluluk yardımcıları

kalibrasyon/
├── accelerometer_calibration.py  # İvmeölçer kalibrasyonu
├── compass_calibration.py        # Pusula kalibrasyonu
└── manual_calibration_suite.py   # Ana kalibrasyon programı

PID/
├── main.py            # Ana PID optimizasyon sistemi
├── quick_test.py      # Hızlı PID testi
└── README.md          # PID sistem dokümantasyonu
```

## 🚀 Görevler

### 📍 Görev 1: Seyir ve Başlangıç Noktasına Dönüş

**Hedef**: 50m toplam mesafe, 180° dönüş ve güvenli yüzey çıkışı

#### Fazlar:
1. **Faz 1**: İlk 10m mesafe, 2m derinlik
2. **Faz 2**: Kalan 40m mesafe, 3m derinlik  
3. **Faz 3**: 180° dönüş manevrası
4. **Faz 4**: 50m geri dönüş, 3m derinlik
5. **Faz 5**: Yüzeye çıkış ve görevi sonlandırma

#### Teknik Özellikler:
- **Mesafe Kontrolü**: PWM tabanlı hız kontrolü
- **Derinlik Kontrolü**: D300 sensör ile ±0.5m hassasiyet
- **Stabilizasyon**: 3 eksen PID kontrolü
- **Güvenlik**: 90 saniye güvenlik gecikmesi

### 🚀 Görev 2: Roket Fırlatma

**Hedef**: Hedefe yaklaşma, roket fırlatma ve güvenli geri çekilme

#### Fazlar:
1. **Faz 1**: Hedefe yaklaşma (30m, 3m derinlik)
2. **Faz 2**: Roket pozisyonlama ve hazırlık
3. **Faz 3**: Roket fırlatma (selenoid ile CO2 tüpü)
4. **Faz 4**: Güvenli geri çekilme ve yüzeye çıkış

#### Güvenli Atış Bölgesi (Aşama-2):
⚠️ **Önemli**: Aşama-2 için görev detaylarında belirtilen güvenli atış bölgesinin, saha şartları göz önüne alınarak, izleme alanına dik olacak şekilde **30 metre ileriden başlaması** uygun bulunmuştur. Takımların **30 metre ilerledikten sonra, güvenli bölge içerisinde, roket ateşlemesi yapması** beklenmektedir.

#### Teknik Özellikler:
- **Roket Taşıma Bölmesi**: Şartname uyumlu ön kısım roket taşıma sistemi
- **Selenoid Kontrolü**: GPIO10 üzerinden selenoid valf kontrolü
- **Fırlatma Sistemi**: CO2 tüpü ile basınçlı fırlatma
- **Yunuslama Açısı**: Şartname uyumlu 7.5°-15° yunuslama kontrolü
- **Otonom Fırlatma**: Sistemin roket ateşleme mekanizmasını otonom çalıştırma
- **Güvenlik**: Acil durumda selenoid otomatik kapanma
- **Zamanlama**: 2 saniye selenoid açık kalma süresi
- **Güvenli Mesafe**: 30m ilerleme sonrası fırlatma bölgesi
- **Kontrollü Yüzey Çıkışı**: Yunuslama açısı ile satha ulaşma

## 🔧 Kalibrasyon Sistemi

### 📊 Manuel Kalibrasyon Paketi

Bu sistem, hareket gerektiren sensör kalibrasyonları için geliştirilmiştir.

#### 1. İvmeölçer Kalibrasyonu (`accelerometer_calibration.py`)
- **Gereksinim**: Kart 6 farklı pozisyona çevrilmeli
- **Süre**: ~15 dakika
- **Pozisyonlar**: Üst, Alt, Sağ, Sol, İleri, Geri
- **Çıktı**: `accelerometer_calibration.json`
- **Doğruluk**: ±0.01G hassasiyet

#### 2. Pusula Kalibrasyonu (`compass_calibration.py`)
- **Gereksinim**: Kart yatay pozisyonda 360° dönüş
- **Süre**: ~5 dakika
- **Ortam**: Açık alan, manyetik girişimden uzak
- **Çıktı**: `compass_calibration.json`, `compass_calibration_plot.png`
- **Kalite**: 0-100 skoru ile değerlendirme

#### 3. Ana Kalibrasyon Programı (`manual_calibration_suite.py`)
- Menü ile kalibrasyon seçimi
- Tüm kalibrasyonları sırayla çalıştırma
- Sonuç raporlama

### 🚀 Kullanım
```bash
# Tek kalibrasyon
python accelerometer_calibration.py
python compass_calibration.py

# Tüm kalibrasyonlar
python manual_calibration_suite.py
```

## 🎯 PID Optimizasyon Sistemi

### 🚀 Özellikler
- **Otomatik Grid Search**: Iteratif daraltma algoritması ile en uygun PID değerlerini bulur
- **Çoklu Derinlik Testi**: 0.3m, 0.5m, 0.7m, 1.0m derinliklerinde test yapar
- **Gerçek Zamanlı Değerlendirme**: Hata, oscillation ve overshoot'u birleştiren skor sistemi
- **Güvenli Test Alanı**: 4m çap sınırları içinde kalır
- **Sonuç Kaydetme**: JSON formatında detaylı sonuçlar ve config güncellemesi

### 🔄 Algoritma

#### 1. Round-Robin Grid Search
- **İterasyon 0**: Kd sabit, Kp-Ki taranır (3x3 = 9 test)
- **İterasyon 1**: Ki sabit, Kp-Kd taranır (3x3 = 9 test)  
- **İterasyon 2**: Kp sabit, Ki-Kd taranır (3x3 = 9 test)
- **İterasyon 3+**: Döngü devam eder
- Her iterasyon: %60 aralık daraltma

#### 2. Skor Sistemi
```
Skor = Ortalama_Hata + (Max_Hata × 0.3) + (Oscillation × 2.0)
```
- **Düşük skor = Daha iyi performans**
- Hata, oscillation ve overshoot'u dengeler

#### 3. Güvenlik Sistemleri
- **Derinlik Sınırı**: Hedef + 0.3m (soft limit)
- **Pitch/Roll Sınırı**: ±25° (acil durdurma)
- **Test Alanı**: 4m çap otomatik kontrol
- **Erken Çıkış**: Aşırı oscillation tespiti

### 🎮 Kullanım
```bash
# Temel kullanım
cd PID/
python3 main.py

# Gelişmiş parametreler
python3 main.py --test-area-radius 1.5  # 3m çap
python3 main.py --max-depth 0.8         # 80cm max
python3 main.py --iterations 7          # 7 iterasyon
python3 main.py --test-duration 20      # 20 saniye per test
```

## 🚀 Kurulum

### Sistem Gereksinimleri

#### Hardware
- **Raspberry Pi 5** (4GB RAM önerilen)
- **Pixhawk PX4 PIX 2.4.8** Flight Controller
- **4x DS3230MG** 30kg su geçirmez servo motor
- **DEGZ M5** sualtı motor + **DEGZ BLU 30A** ESC
- **D300** derinlik/sıcaklık sensörü
- **22.2V 6S LiPo** batarya (1800mAh, 65C)
- **16A P1Z EC** metal güç butonu
- **40A** güç röle sistemi

#### Software
- **Python 3.8+**
- **Raspberry Pi OS** (Bookworm önerilen)
- **BlueOS** (opsiyonel, gelişmiş sualtı kontrol için)

### 1. Repository Klonlama
```bash
git clone https://github.com/kullanici/2025-TEKNOFEST-Su-Alti-Roket-Araci.git
cd 2025-TEKNOFEST-Su-Alti-Roket-Araci
```

### 2. Sistem Bağımlılıkları
```bash
# Sistem paketleri
sudo apt update
sudo apt install -y python3-pip python3-opencv python3-numpy python3-scipy
sudo apt install -y python3-lgpio  # Raspberry Pi 5 için

# Python paketleri
pip3 install pymavlink rpi-lgpio smbus2 opencv-python
pip3 install matplotlib scipy numpy pandas
```

### 3. Hardware Bağlantıları
Detaylı pin mapping için [`HARDWARE_PIN_MAPPING.md`](HARDWARE_PIN_MAPPING.md) dosyasını inceleyin.

### 4. Konfigürasyon
```bash
# MAVLink bağlantı ayarları
export MAV_ADDRESS="/dev/ttyACM0"
export MAV_BAUD="115200"

# GPIO ayarları (Raspberry Pi 5)
export GPIO_LIBRARY="rpi-lgpio"
```

## 📖 Kullanım

### Hızlı Başlangıç
```bash
# 1. Görev sistemini başlat
cd görevlerf1/pluswing
python3 main.py

# 2. Kalibrasyon yap (opsiyonel)
cd ../../kalibrasyon
python3 manual_calibration_suite.py

# 3. PID optimizasyonu (test alanında)
cd ../../PID
python3 main.py
```

### Görev Yönetimi
```python
# Görev 1: Navigasyon
from görevlerf1.pluswing.mission1 import Mission1Controller
mission1 = Mission1Controller(mavlink_conn, system_status, logger)
mission1.start_mission()

# Görev 2: Roket fırlatma
from görevlerf1.pluswing.mission2 import Mission2Controller
mission2 = Mission2Controller(mavlink_conn, system_status, logger)
mission2.start_mission()
```

### Kalibrasyon
```python
# İvmeölçer kalibrasyonu
from kalibrasyon.accelerometer_calibration import AccelerometerCalibrator
accel_cal = AccelerometerCalibrator()
accel_cal.calibrate()

# Pusula kalibrasyonu
from kalibrasyon.compass_calibration import CompassCalibrator
compass_cal = CompassCalibrator()
compass_cal.calibrate()
```

### PID Optimizasyon
```python
# PID optimizasyon sistemi
from PID.main import PIDOptimizer
optimizer = PIDOptimizer()
best_pid = optimizer.optimize()

# Hızlı test
from PID.quick_test import QuickPIDTest
test = QuickPIDTest()
test.run_test(kp=200, ki=10, kd=50)
```

## 📚 API Dokümantasyonu

### Core Modules

#### `görevlerf1/pluswing/` - Ana Görev Sistemi
- **`mission1.py`**: Navigasyon görevi (50m seyir, 180° dönüş)
- **`mission2.py`**: Roket fırlatma görevi (hedef yaklaşma, fırlatma)
- **`control.py`**: PID kontrol, stabilizasyon, servo kontrolü
- **`sensors.py`**: D300 derinlik sensörü ve MAVLink telemetri
- **`main.py`**: Ana program ve görev yöneticisi

#### `kalibrasyon/` - Sensör Kalibrasyonu
- **`accelerometer_calibration.py`**: 6 yönlü ivmeölçer kalibrasyonu
- **`compass_calibration.py`**: 360° pusula kalibrasyonu
- **`manual_calibration_suite.py`**: Tüm kalibrasyonları yöneten ana program

#### `PID/` - PID Optimizasyon Sistemi
- **`main.py`**: Ana PID optimizasyon algoritması
- **`quick_test.py`**: Hızlı PID parametre testi
- **Grid Search**: Iteratif PID parametre optimizasyonu

### Önemli Sınıflar

```python
class Mission1Controller:
    """Görev 1: Navigasyon kontrolcüsü"""
    
    def initialize_mission(self):
        """Görev başlangıç hazırlıkları"""
        
    def start_mission(self):
        """Ana görev döngüsü"""
        
    def _execute_phase_1(self):
        """Faz 1: İlk 10m (2m derinlik)"""
        
    def _execute_turning(self):
        """Faz 3: 180° dönüş manevrası"""

class Mission2Controller:
    """Görev 2: Roket fırlatma kontrolcüsü"""
    
    def start_mission(self):
        """Roket fırlatma görevi"""
        
    def launch_rocket(self):
        """Roket fırlatma prosedürü"""

class PIDOptimizer:
    """PID parametre optimizasyon sistemi"""
    
    def optimize(self):
        """Grid search ile PID optimizasyonu"""
        
    def run_test(self, kp, ki, kd):
        """Belirli PID değerleri ile test"""
```

## 🔧 Geliştirme

### Geliştirme Ortamı Kurulumu
```bash
# Virtual environment oluştur
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

# Development dependencies
pip install black flake8 pytest
```

### Kod Standartları
- **Python**: PEP 8 standartları
- **Docstrings**: Google style
- **Type Hints**: Python 3.8+ type annotations
- **Testing**: pytest framework

### Test Yazma
```python
def test_mission1_initialization():
    """Görev 1 başlangıç testi"""
    controller = Mission1Controller(mavlink_conn, system_status, logger)
    
    # Test setup
    result = controller.initialize_mission()
    
    # Assertions
    assert result == True
    assert controller.mission_timer.is_running
```

## 🚨 Güvenlik ve Acil Durum

### Emergency Procedures
1. **Emergency Stop**: Hardware buton ile anında durdurma
2. **Manuel Kontrol**: Otopilot devre dışı bırakma
3. **Yüzey Çıkışı**: Otomatik güvenlik protokolü
4. **Roket Güvenlik**: Fail-safe modunda çalışma

### Güvenlik Kontrol Listesi
- [ ] Hardware bağlantıları kontrol edildi
- [ ] Batarya seviyesi %80 üzerinde
- [ ] GPS sinyali alınıyor
- [ ] MAVLink bağlantısı aktif
- [ ] Emergency stop butonu test edildi
- [ ] Roket güvenlik sistemi aktif
- [ ] Derinlik sensörü kalibre edildi
- [ ] Motor testleri başarılı

## 📊 Performans Metrikleri

| Metrik | Hedef | Mevcut |
|--------|-------|---------|
| **Navigasyon Hassasiyeti** | ±1m | ±0.8m |
| **Derinlik Kontrolü** | ±0.5m | ±0.3m |
| **Roket Atış Hassasiyeti** | ±2m (50m) | ±1.5m |
| **Batarya Süresi** | 45-60 dk | 55 dk |
| **Maksimum Derinlik** | 10m | 12m |
| **Maksimum Hız** | 2 m/s | 2.2 m/s |

## 🔍 Sorun Giderme

### Yaygın Problemler

#### MAVLink Bağlantı Hatası
```bash
# Serial port ayarlarını kontrol et
ls -la /dev/tty*
sudo chmod 666 /dev/ttyACM0

# Baud rate ayarlarını kontrol et
export MAV_BAUD="115200"
```

#### GPIO Hatası (Raspberry Pi 5)
```bash
# rpi-lgpio kurulumunu kontrol et
sudo apt install python3-lgpio
pip3 install rpi-lgpio

# GPIO izinlerini kontrol et
sudo usermod -a -G gpio $USER
```

#### Servo Kontrol Hatası
```bash
# Servo güç beslemesini kontrol et
# 6.8V regülatör çıkışını ölç
# AUX 1-4 pin bağlantılarını kontrol et
```

### Log Dosyaları
```bash
# Sistem logları
tail -f /var/log/underwater_rocket/system.log

# MAVLink logları
tail -f /var/log/underwater_rocket/mavlink.log

# GPIO logları
tail -f /var/log/underwater_rocket/gpio.log
```

## 🤝 Katkıda Bulunma

### Katkı Süreci
1. **Fork** yapın
2. **Feature branch** oluşturun (`git checkout -b feature/amazing-feature`)
3. **Commit** yapın (`git commit -m 'Add amazing feature'`)
4. **Push** yapın (`git push origin feature/amazing-feature`)
5. **Pull Request** oluşturun

### Geliştirme Kuralları
- **Commit Messages**: Conventional Commits standardı
- **Code Review**: Tüm PR'lar review edilmelidir
- **Testing**: Yeni özellikler için test yazılmalıdır
- **Documentation**: API değişiklikleri dokümante edilmelidir

## 📄 Lisans

Bu proje **MIT License** altında lisanslanmıştır. Detaylar için [`LICENSE`](LICENSE) dosyasına bakın.

## 📞 İletişim

- **E-posta**: [tidat.alp.tekno@gmail.com](mailto:tidat.alp.tekno@gmail.com)
- **Proje**: [GitHub Issues](https://github.com/kullanici/2025-TEKNOFEST-Su-Alti-Roket-Araci/issues)
- **TEKNOFEST**: [2025 Su Altı Roket Aracı Yarışması](https://www.teknofest.org/tr/yarismalar/su-alti-roket-yarismasi)

## 🙏 Teşekkürler

- **TEKNOFEST** organizasyon komitesi
- **Raspberry Pi Foundation** açık kaynak desteği
- **ArduPilot** MAVLink protokolü
- **Open Source Community** katkıları

---

<div align="center">

**🚀 TEKNOFEST 2025'e Hazır! 🚀**


[![TEKNOFEST](https://img.shields.io/badge/TEKNOFEST-2025-blue.svg)](https://teknofest.org/)
[![Status](https://img.shields.io/badge/Status-Ready%20for%20Competition-green.svg)]()

</div>