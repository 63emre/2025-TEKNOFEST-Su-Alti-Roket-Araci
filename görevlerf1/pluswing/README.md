# Su Altı Roket Aracı (SARA) - Plus Wing Konfigürasyonu

Teknofest 2025 Su Altı Roket Aracı Yarışması için geliştirilmiş otonom kontrol sistemi.

## 📋 Sistem Özeti

**SARA (Su Altı Roket Aracı)**, Raspberry Pi ve Pixhawk 2.4.8 tabanlı bir otonom su altı aracıdır. Plus Wing (+) konfigürasyonunda 4 kontrol kanatı kullanarak 3 eksen stabilizasyonu ve görev navigasyonu gerçekleştirir.

### Ana Bileşenler

- **Raspberry Pi 4B/5**: Yüksek seviye kontrol ve sensör okuma
- **Pixhawk 2.4.8**: Motor/servo kontrolü ve dahili IMU
- **D300 Derinlik Sensörü**: I2C üzerinden basınç/derinlik ölçümü
- **DEGZ M5 Su Altı Motoru**: Ana itki (30A ESC ile)
- **4x DS3230MG Servolar**: Plus konfigürasyonu kontrol kanatları
- **Selenoid Valf**: Roket fırlatma sistemi (sadece Görev 2)

## 🔧 Donanım Konfigürasyonu

### Pixhawk Bağlantıları

```
AUX1 (Kanal 9)  → Ana Motor (DEGZ M5 + ESC)
AUX2            → BOZUK - Kullanılmaz
AUX3 (Kanal 11) → Sağ Kanat Servos (DS3230MG)
AUX4 (Kanal 12) → Alt Kanat Servo (DS3230MG)
AUX5 (Kanal 13) → Sol Kanat Servo (DS3230MG)
AUX6 (Kanal 14) → Üst Kanat Servo (DS3230MG)
```

### Raspberry Pi GPIO

```
GPIO 21 → Kırmızı LED (Durum göstergesi)
GPIO 9  → Buzzer (Ses sinyalleri)
GPIO 11 → Başlatma/Soft-kill Butonu
GPIO 10 → Selenoid Valf (Sadece Görev 2)
GPIO 2  → D300 SDA (I2C)
GPIO 3  → D300 SCL (I2C)
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

## 🏗️ Yazılım Mimarisi

```
pluswing/
├── README.md           # Bu dosya
├── config.py          # Sistem konfigürasyonu ve sabitler
├── utils.py           # LED, buzzer, timer, loglama yardımcıları
├── sensors.py         # D300 ve MAVLink sensör yönetimi
├── control.py         # PID kontrol, stabilizasyon, servo kontrolü
├── mission1.py        # Görev 1: Seyir ve geri dönüş
├── mission2.py        # Görev 2: Roket fırlatma
└── main.py           # Ana program (otomatik başlatılacak)
```

### Modül Açıklamaları

- **config.py**: Tüm pin mappingleri, PWM değerleri, kontrolcü parametreleri
- **utils.py**: LED/buzzer kontrolü, zamanlayıcılar, buton yönetimi
- **sensors.py**: D300 derinlik sensörü ve Pixhawk telemetri okuma
- **control.py**: 3-eksen stabilizasyon, PID kontrolü, servo komutlama
- **mission1.py**: Otonom seyir görevi (10m+40m, 180° dönüş, yüzeye çıkış)
- **mission2.py**: Roket fırlatma görevi (hedefe yaklaşma, fırlatma, geri çekilme)
- **main.py**: Ana kontrol döngüsü, 90s güvenlik, görev yönetimi

## 🚀 Görevler

### Görev 1: Seyir ve Başlangıç Noktasına Dönüş

1. **Faz 1**: İlk 10m mesafe, 2m derinlik
2. **Faz 2**: Kalan 40m mesafe, 3m derinlik
3. **Faz 3**: 180° dönüş manevrası
4. **Faz 4**: 50m geri dönüş, 3m derinlik
5. **Faz 5**: Yüzeye çıkış ve görevi sonlandırma

### Görev 2: Roket Fırlatma

1. **Faz 1**: Hedefe yaklaşma (30m, 3m derinlik)
2. **Faz 2**: Roket pozisyonlama ve hazırlık
3. **Faz 3**: Roket fırlatma (selenoid ile CO2 tüpü)
4. **Faz 4**: Güvenli geri çekilme ve yüzeye çıkış

## ⚙️ Kurulum ve Çalıştırma

### Gereksinimler

```bash
sudo apt update && sudo apt upgrade
sudo apt install python3 python3-pip

pip3 install pymavlink RPi.GPIO smbus2
```

### I2C Aktivasyonu

```bash
sudo raspi-config
# Interface Options → I2C → Enable
```

### Çalıştırma

```bash
cd pluswing/
sudo python3 main.py
```

### Otomatik Başlatma (systemd)

```bash
sudo nano /etc/systemd/system/sara.service
```

```ini
[Unit]
Description=SARA Su Alti Roket Araci
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/sara/pluswing
ExecStart=/usr/bin/python3 /home/pi/sara/pluswing/main.py
Restart=always

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable sara.service
sudo systemctl start sara.service
```

## 🎮 Kullanım

### Başlatma Sekansı

1. **Güç Verme**: Hard-kill butonu ile sistemi açın
2. **Bekleme**: Kırmızı LED yanıp sönecek
3. **Görev Başlatma**: GPIO11 butonuna basın
4. **90s Geri Sayım**: Buzzer 9+1 pattern ile sayacak
5. **Görev Başlangıcı**: 90s sonra otomatik başlar

### Buton Kontrolleri

- **İlk Basış**: Görev başlatma (90s geri sayım)
- **İkinci Basış**: Görev durdurma/iptal
- **Üçüncü Basış**: Yeniden başlatma
- **Döngüsel**: Toggle mantığı ile çalışır

### LED ve Buzzer Sinyalleri

- **Yanıp Sönme**: Bekleme modu
- **Sabit Yanık**: Görev aktif
- **Hızlı Yanıp Sönme**: Acil durum
- **9+1 Buzzer Pattern**: 90s geri sayım
- **5s Buzzer**: Görev başlangıcı
- **3s'de 1**: Görev bitişi

## 🔧 Parametreler

### Kritik Ayarlar (config.py)

```python
ARMING_DELAY_SECONDS = 90      # Güvenlik gecikmesi
TARGET_DEPTH_FIRST_10M = 2.0   # İlk 10m derinlik
TARGET_DEPTH_MAIN = 3.0        # Ana seyir derinliği
MISSION_DISTANCE = 50.0        # Toplam mesafe
```

### Stabilizasyon Parametreleri

```python
ROLL_K_ANG_US_PER_RAD = 500.0   # Roll kontrolcü kazancı
PITCH_K_ANG_US_PER_RAD = 500.0  # Pitch kontrolcü kazancı
YAW_K_ANG_US_PER_RAD = 400.0    # Yaw kontrolcü kazancı
DEPTH_KP = 200.0                # Derinlik P kontrolcü
```

## 🛠️ Test ve Debugging

### Sensör Testi

```bash
# I2C cihazları kontrol et
i2cdetect -y 1

# MAVLink bağlantı testi
python3 -c "from sensors import *; s = SensorManager(None); print(s.depth.is_connected())"
```

### Manuel Servo Testi

```bash
# Servo test dosyası ile (varsa)
python3 ../Test/test_aux3_servo.py
```

### Log İnceleme

```bash
tail -f sara_mission.log
```

## ⚠️ Güvenlik

### Kritik Güvenlik Kuralları

1. **90s Kural**: Enerji verildikten 90s sonra motor çalışır
2. **Soft-kill**: GPIO11 butonu ile yazılımsal durdurma
3. **Hard-kill**: Donanımsal acil stop (100A röle)
4. **Acil Durum**: Herhangi bir hata durumunda tüm sistemler durur
5. **Yüzeye Çıkış**: Görev sonunda pozitif yüzerlik ile yüzeye çıkar

### Acil Durum Prosedürleri

- **Yazılımsal**: Tüm servolar nötr, motor dur
- **Donanımsal**: Hard-kill butonu ile tam güç kesme

## 📊 Performans

### Beklenen Değerler

- **Stabilizasyon Hızı**: 50Hz (20ms döngü)
- **Derinlik Hassasiyeti**: ±0.2m
- **Yönelim Hassasiyeti**: ±2°
- **Mesafe Tahmini**: ±10% (zaman×hız tabanlı)
- **Görev Süresi**: ~5-8 dakika

### Hız Profilleri

- **Yavaş (1600 PWM)**: 1.0 m/s, dönüş manevrası
- **Orta (1700 PWM)**: 1.5 m/s, ilk 10m fazı
- **Hızlı (1800 PWM)**: 2.0 m/s, ana seyir

## 🔍 Sorun Giderme

### Yaygın Problemler

1. **MAVLink Bağlantısı**: `/dev/ttyACM0` portunu kontrol edin
2. **I2C Sorunu**: `sudo i2cdetect -y 1` ile 0x76 adresini kontrol edin
3. **Servo Hareketsizliği**: PWM değerlerini ve bağlantıları kontrol edin
4. **Derinlik Okuması**: D300 sensör kalibrasyonunu yenileyin
5. **Stabilizasyon**: Deadband ve kazanç parametrelerini ayarlayın

### Hata Kodları

- **MAVLink Timeout**: Pixhawk bağlantısı kesildi
- **Sensor Invalid**: D300 sensöründen veri alınamıyor
- **Stabilization Failed**: Attitude verileri güncel değil
- **Emergency Stop**: Acil durum prosedürü aktif

## 📝 Geliştirme Notları

### Gelecek Geliştirmeler

- [ ] GPS tabanlı navigasyon (su üstü)
- [ ] Akıntı kompensasyonu
- [ ] Adaptif hız kontrolü
- [ ] Çoklu sensör füzyonu
- [ ] Gelişmiş yol planlama

### Test Edilmiş Konfigürasyonlar

- ✅ Plus Wing stabilizasyonu (`full_stabilization2.py`)
- ✅ Servo PWM kontrolleri (`test_aux*_servo.py`)
- ✅ D300 derinlik sensörü okuma
- ✅ MAVLink haberleşme (ArduSub)

## 📞 İletişim

Bu sistem Teknofest 2025 Su Altı Roket Aracı Yarışması için geliştirilmiştir.

**Önemli**: Bu yazılım deneme.txt belgesindeki tüm gereksinimleri karşılamak üzere tasarlanmıştır. Herhangi bir değişiklik yapmadan önce sistem davranışını tam olarak anlayın.

---

_Son güncelleme: 2024_
_Versiyon: 1.0_
_Konfigürasyon: Plus Wing (+)_
