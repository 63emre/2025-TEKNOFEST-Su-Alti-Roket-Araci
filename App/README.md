# 🚀 TEKNOFEST 2025 - Su Altı ROV Kontrol Sistemi

## 📋 Sistem Genel Bakış

Bu uygulama, Teknofest 2025 Su Altı Roket Aracı için geliştirilmiş tam entegre kontrol sistemidir. Real-time ROV pilotlama, otomatik navigation ve titreşim analizi özelliklerini içerir.

### 🎯 Ana Özellikler

- **🌐 Web GUI**: Modern responsive web arayüzü (BlueOS uyumlu)
- **📱 Real-time Kontrol**: Touch/klavye ile anlık servo kontrolü
- **🔄 WebSocket**: Canlı telemetry ve kontrol güncellemeleri
- **⚙️ İki Kontrol Modu**: RAW PWM (titreşimsiz) vs PID (filtreli)
- **🧭 Üç Navigation Sistemi**: GPS, IMU Dead Reckoning, Hibrit
- **🎮 Hareket Komutları**: Parametreli hareket görevleri
- **📊 Real-time Grafikler**: Plot.ly ile IMU ve telemetry grafikleri
- **📡 Titreşim Monitörü**: Canlı vibration analizi
- **🖥️ Desktop GUI**: PyQt5 arayüz (VNC gerekli)

## 🏗️ Sistem Mimarisi

```
App/
├── web_gui.py               # 🌐 Flask Web GUI (Ana Arayüz)
├── main_gui.py              # 🖥️ PyQt5 Desktop GUI
├── templates/
│   └── index.html           # Web arayüzü template
├── control_module.py        # Tkinter Real-time Kontrol  
├── mavlink_handler.py       # RAW vs PID Kontrol
├── navigation_engine.py     # GPS + IMU + Hibrit Navigation
├── vibration_monitor.py     # Titreşim Analizi
├── start_web_gui.sh         # 🚀 Hızlı Web GUI başlatma
├── config/
│   ├── hardware_config.json # Pin Konfigürasyonu
│   └── control_settings.json # Kontrol Parametreleri
└── scripts/
    └── servo_calibration.py # Otomatik Kalibrasyon
```

## ⚙️ Hardware Konfigürasyonu

### Pixhawk Bağlantıları:
- **Servo AUX 1**: Ön Sol Fin (front_left)
- **Servo AUX 3**: Arka Sol Fin (rear_left) 
- **Servo AUX 4**: Arka Sağ Fin (rear_right)
- **Servo AUX 5**: Ön Sağ Fin (front_right)
- **Motor AUX 6**: Ana İtki Motoru

### Raspberry Pi GPIO:
- **GPIO 7**: Buzzer (değiştirilebilir)
- **GPIO 13**: Kontrol Butonu (değiştirilebilir)
- **I2C**: D300 Derinlik Sensörü

## 🚀 Kurulum ve Çalıştırma

### 1. Otomatik Kurulum (Raspberry Pi)
```bash
# Repoyu clone et
git clone https://github.com/kullanici/2025-TEKNOFEST-Su-Alti-Roket-Araci.git
cd 2025-TEKNOFEST-Su-Alti-Roket-Araci/App

# Kurulum scripti çalıştır
sudo bash setup_raspberry_pi.sh
```

### 2. Web GUI Başlat (Önerilen)
```bash
# Hızlı başlatma
./start_web_gui.sh

# Veya manuel
python3 web_gui.py
```

### 3. Web Arayüzüne Erişim
- **Lokal**: `http://127.0.0.1:5000`
- **Network**: `http://192.168.2.2:5000` (ROV IP)
- **Proxy**: `http://192.168.2.2:8080` (Nginx varsa)

### 4. Service Olarak Çalıştır
```bash
# Web GUI service başlat
sudo systemctl start teknofest-rov-web
sudo systemctl status teknofest-rov-web
```

### 5. Desktop GUI (İsteğe Bağlı)
```bash
# VNC kurulumu
sudo apt install tightvncserver
vncserver :1 -geometry 1024x768

# Desktop GUI çalıştır
export DISPLAY=:1
python3 main_gui.py
```

## 🎮 Kontrol Kullanımı

### 🌐 Web GUI Kontrolleri:

#### Sistem Kontrolü:
1. **MAVLink Bağlan**: Pixhawk'a bağlan
2. **ARM/DISARM**: Sistemi aktif/pasif et
3. **Kontrol Modu**: RAW PWM veya PID seç
4. **Navigation Modu**: GPS, IMU veya Hibrit seç

#### Real-time Kontrol (Klavye):
- **W,A,S,D**: Servo kontrolü (Pitch/Roll)
- **Q,E**: Yaw (sola/sağa dönme)
- **Page ↑/↓**: Derinlik kontrolü
- **Space**: 🚨 Acil durum!
- **Esc**: Yüzeye çık!

#### Hareket Komutları (Parametre Girişli):
- **İleri Git**: Metre belirt → Çalıştır
- **Yaw Dön**: Derece belirt → Çalıştır
- **Yukarı Çık**: Metre belirt → Çalıştır
- **Sol/Sağ Git**: Metre belirt → Çalıştır

#### Web Arayüzü Özellikleri:
- **📊 Real-time Grafikler**: IMU telemetry
- **📡 Telemetry Panel**: Roll/Pitch/Yaw/GPS
- **🔋 Sistem Durumu**: Voltaj/Akım/Güç
- **📳 Titreşim Seviyesi**: Görsel bar + kategori
- **📝 Canlı Loglar**: Sistem mesajları

## 📊 GUI Panelleri

### Sol Panel - Kontrol Ayarları:
- **Bağlantı**: MAVLink bağlan/ARM-DISARM
- **Kontrol Modu**: RAW/PID seçimi (checkbox)
- **Navigation Modu**: GPS/IMU/Hibrit seçimi
- **Hareket Komutları**: T,Y,U,G,H butonları
- **Real-time Kontrol**: Klavye kontrol penceresi
- **Acil Durum**: Emergency stop butonu

### Orta Panel - Telemetry:
- **IMU Grafikleri**: YAW/PITCH/ROLL real-time grafikler
- **Sistem Bilgisi**: Telemetry değerleri
- **GPS Durumu**: Koordinat ve sinyal kalitesi  
- **Titreşim Monitörü**: 0-100 scale + frekans analizi

### Sağ Panel - Scripts & System:
- **Uygulama Scriptleri**: Kalibrasyon ve test butonları
- **Konfigürasyon**: Pin ayarları editörü
- **Titreşim Karşılaştırması**: RAW vs PID analizi

### Alt Panel - Terminal:
- **Script Output**: Çalışan scriptlerin çıktıları
- **Log Messages**: Sistem log mesajları

## 🔧 Kontrol Modu Karşılaştırması

### RAW PWM Control:
- ✅ **Titreşim**: Çok düşük (test_aux4_servo.py benzeri)
- ✅ **Response**: 20ms (50Hz)
- ⚠️ **Accuracy**: ±0.5° (manuel kontrol)
- ✅ **CPU Usage**: Düşük

### PID Control:
- ⚠️ **Titreşim**: Orta (spec_servotest.py benzeri)  
- ⚠️ **Response**: 100ms (20Hz)
- ✅ **Accuracy**: ±0.1° (otomatik düzeltme)
- ⚠️ **CPU Usage**: Yüksek

**Öneri**: Test ederek hangisinin daha iyi çalıştığını belirleyin!

## 🧭 Navigation Sistemleri

### GPS Only:
- **Accuracy**: ±1.0m (4+ uydu gerekli)
- **Update Rate**: 5Hz
- **Use Case**: Açık su, uzun mesafe

### IMU Dead Reckoning:  
- **Accuracy**: ±0.5m (başlangıçta, zamanla drift artar)
- **Update Rate**: 50Hz
- **Use Case**: Su altı, hassas manevra

### Hybrid (GPS + IMU):
- **Logic**: GPS varsa GPS, yoksa IMU
- **Switching**: 5 saniye GPS timeout
- **Fusion**: %70 GPS + %30 IMU ağırlık
- **Use Case**: En güvenilir sistem

## 📊 Titreşim Monitörü

### Real-time Analiz:
- **Level**: 0-100 scale (yeşil/sarı/kırmızı)
- **Category**: Low/Medium/High
- **Frequency**: Dominant frekans (Hz)
- **FFT Bands**: 0-5Hz, 5-15Hz, 15-25Hz

### Karşılaştırma Sistemi:
1. RAW modu seç → Test et → Kayıt al
2. PID modu seç → Test et → Kayıt al  
3. **Comparison Report** ile karşılaştır
4. **Recommendation** al

## 🛠️ Script Sistemi

### Mevcut Scriptler:
- **servo_calibration.py**: 4 servo otomatik kalibrasyonu
- **motor_test.py**: AUX6 motor test scripti
- **imu_calibration.py**: IMU kalibrasyon scripti
- **system_check.py**: Genel sistem kontrolü

### Script Çalıştırma:
1. Sağ panelden script butonuna tık
2. Alt panelde output'u takip et
3. Script tamamlanınca sonucu gör

### Yeni Script Ekleme:
```python
#!/usr/bin/env python3
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from mavlink_handler import MAVLinkHandler

# Script kodunuz buraya
```

## ⚡ Acil Durum Protokolleri

### Emergency Stop (Space):
1. Tüm servo/motor durduruluyor
2. Neutral pozisyona geçiliyor  
3. GUI'de uyarı mesajı
4. Log'a kayıt

### Surface Immediately (Esc):
1. Derinlik sensörü kontrolü
2. Yukarı yönlü maksimum thrust
3. Yüzey çıkışına kadar otomatik
4. 5m derinlikten sonra emergency surface

### System Fail-safe:
- **MAVLink Timeout**: 30 saniye
- **Low Battery**: <19.8V otomatik yüzey
- **High Vibration**: >80 level uyarısı
- **GPS Loss**: Otomatik IMU'ya geçiş

## 🔧 Konfigürasyon Düzenleme

### Pin Değiştirme:
`config/hardware_config.json` dosyasını düzenleyin:
```json
{
  "raspberry_pi": {
    "gpio": {
      "buzzer": 7,           # Buzzer pini
      "control_button": 13   # Buton pini (sizin isteğiniz)
    }
  }
}
```

### Kontrol Parametreleri:
`config/control_settings.json` dosyasında PID değerleri, hassasiyet, tolerans ayarları bulunur.

## 🐛 Sorun Giderme

### Bağlantı Sorunu:
- MAVLink bağlantı string'ini kontrol edin: `tcp:127.0.0.1:5777`
- Pixhawk'ın çalıştığından emin olun
- Port çakışması var mı kontrol edin

### Titreşim Sorunu:
- RAW modu deneyin (PID kapatın)
- PWM Hysteresis değerini artırın
- Servo kalibrasyonu yapın
- Fiziksel bağlantıları kontrol edin

### Navigation Sorunu:
- GPS sinyal kalitesini kontrol edin
- IMU kalibrasyonu yapın
- Hibrit modu deneyin
- Tolerans değerlerini artırın

### GUI Sorunu:
- PyQt5 kurulu mu kontrol edin
- Terminal'den çalıştırıp hata mesajlarına bakın
- Real-time control penceresi focus'ta mı?

## 📞 Destek

**Geliştirici**: TEKNOFEST Takımı  
**Versiyon**: 1.0  
**Tarih**: 2025  

### Log Dosyaları:
- GUI logları: Terminal panel
- MAVLink logları: Console output
- Script logları: Alt panel terminal

**🚀 İyi sürüşler! Teknofest 2025'te başarılar!** 