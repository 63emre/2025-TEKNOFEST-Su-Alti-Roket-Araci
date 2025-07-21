# 🧪 TEST KLASÖRÜ - TEKNOFEST Su Altı Roket Aracı

Bu klasör, su altı roket aracının tüm alt sistemlerini test etmek için geliştirilmiş scriptleri içermektedir.

## 🔧 Sistem Gereksinimleri

### Donanım
- **Raspberry Pi 4B** (BlueOS yüklü)
- **Pixhawk 2.4.8** (USB bağlı)
- **4x DS3230MG Servo** (30kg, su geçirmez)
- **DEGZ M5 Su Altı Motor** + ESC (30A)
- **GPIO Buton** (16mm metal, ışıklı)
- **22.2V 6S LiPo Batarya**

### Yazılım Bağımlılıkları
```bash
pip install pymavlink
pip install RPi.GPIO
pip install numpy
pip install time
```

## 📋 Test Scriptleri

### 1. Base Sistem Testleri
- `test_mavlink_connection.py` - MAVLink bağlantı testi
- `test_gpio_button.py` - Güvenlik butonu + 90sn gecikme testi
- `test_led_buzzer.py` - LED ve buzzer sistemi testi (RGB+Status+Alarm)
- `test_d300_depth_sensor.py` - D300 derinlik ve sıcaklık sensörü (I2C)

### 2. Alt Sistem Testleri
- `test_servo_control.py` - 4 servo fin kontrolü (X-konfigürasyon AUX 1-4)
- `test_motor_control.py` - ESC motor kontrolü (MAIN 1)
- `test_stabilization.py` - Stabilizasyon algoritması (X-fin mixing)
- `test_depth_hold.py` - Derinlik tutma sistemi (D300 + PID)

### 3. Navigasyon Testleri
- `test_waypoint_navigation.py` - Waypoint takip sistemi
- `test_heading_control.py` - Yön kontrolü (compass)  
- `test_altitude_control.py` - Derinlik kontrolü

### 4. İntegrasyon Testleri
- `test_full_system.py` - Tam sistem entegrasyonu
- `test_mission_ready.py` - Görev hazırlık testi

## 🚀 Kullanım

```bash
# 1. MAVLink bağlantısını test et
python test_mavlink_connection.py

# 2. GPIO buton sistemini test et  
python test_gpio_button.py

# 3. Servo sistemini kalibre et
python test_servo_control.py

# 4. Motor kontrolünü test et
python test_motor_control.py

# 5. Tam sistem testi
python test_full_system.py
```

## ⚠️ GÜVENLİK UYARILARI

1. **Her test öncesi** güvenlik butonuna basın
2. **90 saniye bekleyin** motor testleri için
3. **Acil durdurma** her zaman hazır olsun
4. **Su altı testler** için gözlemci bulundurun
5. **Batarya voltajını** sürekli kontrol edin

## 📊 Test Protokolleri

### Günlük Test Rutini
1. Sistem başlatma (buton + 90sn)
2. MAVLink bağlantı kontrolü
3. Servo kalibrasyon kontrolü
4. Motor response testi
5. Stabilizasyon performans ölçümü
6. Acil durdurma testi

### Haftalık Test Rutini  
1. Tam sistem entegrasyonu
2. Su altı sızdırmazlık testi
3. Otonom navigasyon testi
4. Roket ayrılma mekanizması
5. Batarya dayanıklılık testi

## 🔍 Debug & Troubleshooting

### MAVLink Sorunları
- Port kontrol: `netstat -tlnp | grep 5777`
- BlueOS durum: `systemctl status blueos`

### GPIO Sorunları
- Pin durum: `gpio readall`
- Permission: `sudo usermod -a -G gpio $USER`

### Servo Sorunları
- PWM sinyali: Oscilloscope ile kontrol
- Güç kaynağı: 6-7.4V arası olmalı

---
*Son güncelleme: 2025 - TEKNOFEST Hazırlık Dönemі* 