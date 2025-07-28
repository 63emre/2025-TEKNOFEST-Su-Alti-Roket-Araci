# TEKNOFEST 2025 - Su Altı Roket Aracı Kontrol Scriptleri

## 📋 Özelleştirilmiş Script'ler

Bu doküman, 4-4.5 kg ağırlığındaki X-Wing konfigürasyonlu su altı aracı için özel geliştirilmiş 2 script'in kullanım kılavuzudur.

---

## 🌊 1. Derinlik Kontrolü Script'i
### `depth_control_d300.py`

### ✨ Özellikler:
- **D300 Derinlik Sensörü**: I2C adres 0x76 üzerinden hassas derinlik kontrolü
- **Metre/CM Girişi**: `2.5m`, `150cm`, `3.0` formatlarında giriş desteği  
- **GPIO 23 LED**: Hedef derinliğe ulaşınca yanık, uzakta sönek
- **X-Wing Stabilizasyon**: 4 fin ile otomatik denge kontrolü
- **4-4.5kg Optimizasyonu**: Ağır araç için ayarlanmış PID değerleri

### 🚀 Kullanım:
```bash
cd "Kabiliyet Videosu"
python3 depth_control_d300.py
```

### 💻 Komutlar:
```
Hedef derinlik ayarla: 
  - "2.5m"      -> 2.5 metre derinlik
  - "150cm"     -> 1.5 metre derinlik (150cm)
  - "3.0"       -> 3.0 metre derinlik

LED kontrol:
  - "led on"    -> LED'i aç
  - "led off"   -> LED'i kapat

Durum kontrolü:
  - "status"    -> Mevcut derinlik ve sensör durumu
  - "quit"      -> Programı kapat
```

### ⚙️ Teknik Özellikler:
- **PID Değerleri**: Kp=1.2, Ki=0.05, Kd=0.25 (ağır araç için)
- **Stabilizasyon Frekansı**: 50Hz
- **Maksimum Derinlik Değişim Hızı**: 0.5 m/s
- **Güvenlik Limiti**: 0-10 metre arası

---

## 🔄 2. U-Şekli Hareket Script'i  
### `movement_pattern_u_shape.py`

### ✨ Özellikler:
- **U-Şekli Paternler**: 3 farklı U manevrası (küçük, büyük, karmaşık)
- **Stabilize Hareket**: Roll, pitch, yaw ve derinlik otomatik stabilizasyonu
- **Yavaş ve Kontrollü**: Güvenli hareket hızları
- **GPIO 23 LED**: Hareket sırasında yanık
- **Gerçek Zamanlı**: İptal edilebilir hareketler

### 🚀 Kullanım:
```bash
cd "Kabiliyet Videosu"
python3 movement_pattern_u_shape.py
```

### 💻 U-Şekli Paternler:
```
"small_u"   -> Küçük U şekli (2m ileri + 90°dönüş + 1m + 90°dönüş + 2m)
"large_u"   -> Büyük U şekli (4m ileri + 90°dönüş + 2m + 90°dönüş + 4m)  
"complex_u" -> Karmaşık U (strafe + dönüş kombinasyonları)
```

### 🏃 Tekil Hareketler:
```
"forward 3"     -> 3 saniye ileri git
"backward 2"    -> 2 saniye geri git
"right 2"       -> 2 saniye sağa git
"left 2"        -> 2 saniye sola git
"turn_right 90" -> 90 derece sağa dön
"turn_left 45"  -> 45 derece sola dön
```

### 🛠️ Kontrol Komutları:
```
"stop"      -> Acil dur (tüm hareketleri durdur)
"status"    -> Mevcut konum ve durum bilgisi
"led on"    -> LED'i aç
"led off"   -> LED'i kapat
"quit"      -> Programdan çık
```

---

## ⚙️ Sistem Gereksinimleri

### Hardware:
- **Pixhawk PX4 PIX 2.4.8** (115200 baud MAVLink)
- **D300 Derinlik Sensörü** (I2C: 0x76)
- **X-Wing Servo Konfigürasyonu**:
  - AUX1 (Channel 9): Ön Sol Fin
  - AUX3 (Channel 11): Ön Sağ Fin
  - AUX4 (Channel 12): Arka Sol Fin  
  - AUX5 (Channel 13): Arka Sağ Fin
  - AUX6 (Channel 14): Ana Motor
- **GPIO 23**: Kontrol LED'i

### Software:
```bash
# Gerekli Python paketleri
pip3 install pymavlink smbus2 RPi.GPIO gpiozero
```

### Environment Variables:
```bash
export MAV_ADDRESS="/dev/ttyACM0"
export MAV_BAUD="115200"
```

---

## 🎯 Optimizasyon Detayları

### 4-4.5kg Araç için PID Ayarları:

**Derinlik Kontrolü:**
- Kp: 1.2 (ağır araç için artırıldı)
- Ki: 0.05 (yavaş integral artışı)
- Kd: 0.25 (titreşim önleme)

**Stabilizasyon:**
- Roll/Pitch: Kp=1.2, Ki=0.04, Kd=0.18
- Yaw: Kp=1.0, Ki=0.03, Kd=0.15
- Maksimum PWM değişimi: ±400

**Hareket Hızları:**
- Yavaş: 0.3 (güvenli test için)
- Normal: 0.5 (standart operasyon)
- Hızlı: 0.7 (acil durumlar)

---

## 🔧 Sorun Giderme

### D300 Sensör Bulunamıyor:
```bash
# I2C cihazları tarayın
sudo i2cdetect -y 1

# I2C'yi etkinleştirin
sudo raspi-config -> Interface Options -> I2C -> Enable
```

### MAVLink Bağlantı Sorunu:
```bash
# Seri port kontrolü
ls -la /dev/ttyACM*
ls -la /dev/ttyUSB*

# Baud rate test
export MAV_BAUD="57600"  # Alternatif deneme
```

### GPIO LED Çalışmıyor:
```bash
# GPIO durumu kontrol
gpio readall
# veya
pinout  # Pi5 için
```

---

## 🚨 Güvenlik Uyarıları

1. **Su Altı Testler**: Daima görsel teması korunarak test edin
2. **Emergency Stop**: Her zaman 'stop' komutu ile acil durdurma hazır
3. **Derinlik Limiti**: Maksimum 10 metre güvenlik limiti
4. **Batarya İzleme**: Düşük voltajda otomatik yüzeye çıkış
5. **Manuel Kontrol**: Acil durumlarda manuel RC kontrole geçiş

---

## 📞 Destek

Bu script'ler TEKNOFEST 2025 Su Altı Roket Yarışması için özel geliştirilmiştir.

**Test Ortamı**: X-Wing 4 fin + Ana motor konfigürasyonu  
**Ağırlık**: 4-4.5 kg  
**Operasyon Derinliği**: 0-10 metre  

---

*Son güncelleme: 2025 - TEKNOFEST Yarışma Hazırlığı* 