# 🧪 TEST KLASÖRÜ - TEKNOFEST Su Altı Roket Aracı

Bu klasör, su altı roket aracının tüm alt sistemlerini test etmek için geliştirilmiş scriptleri içermektedir.

## 🔧 Sistem Gereksinimleri

### Donanım - HARDWARE_PIN_MAPPING.md Standardına Uygun
- **Raspberry Pi 4B** (BlueOS yüklü) - GPIO kontrol sistemi
- **Pixhawk PX4 PIX 2.4.8** (USB MAVLink bağlantısı) - Ana uçuş kontrol
- **4x DS3230MG Servo** (30kg, su geçirmez) - X-konfigürasyon finler (AUX 1-4)  
- **DEGZ M5 Su Altı Motor** + **DEGZ BLU 30A ESC** (MAIN 1)
- **D300 Derinlik/Sıcaklık Sensörü** (I2C 0x77) - GPIO 2,3
- **16A P1Z EC Metal Buton** (GPIO 18) - Güç kontrolü
- **40A Güç Röle Sistemi** (GPIO 21) - Acil kesme
- **RGB LED Sistem** (GPIO 4,5,6) - Durum gösterimi  
- **Status LED'ler** (GPIO 16,20,24) - Sistem durumu
- **PWM Buzzer** (GPIO 13,25) - Sesli uyarı sistemi
- **22.2V 6S LiPo Batarya** (1800mAh, 65C) - 80A sürekli akım

### Yazılım Bağımlılıkları
```bash
# Pi 5 için güncellenmiş gereksinimler
pip install pymavlink        # MAVLink protokolü
pip install rpi-lgpio        # Raspberry Pi 5 uyumlu GPIO (RPi.GPIO yerine)
pip install numpy            # Numerik hesaplamalar
pip install smbus2           # I2C haberleşme (D300 sensör)

# Test ve analiz
pip install matplotlib       # Veri görselleştirme
pip install scipy            # Sinyal işleme

# Veya tüm gereksinimleri yükle
pip install -r ../requirements.txt
pip install json             # Veri kaydetme
pip install threading       # Çoklu işlem

# Pin mapping referansı
# Tüm pin tanımları HARDWARE_PIN_MAPPING.md'de standardize edilmiştir
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

### Temel Test Sırası
```bash
# 1. Pin mapping ve bağlantı kontrolü (ÖNEMLİ!)
cat HARDWARE_PIN_MAPPING.md

# 2. MAVLink bağlantısını test et
python test_mavlink_connection.py

# 3. GPIO buton sistemini test et (90sn güvenlik gecikmesi)  
python test_gpio_button.py

# 4. D300 derinlik sensörü testi (I2C)
python test_d300_depth_sensor.py

# 5. LED ve buzzer sistemi testi  
python test_led_buzzer.py

# 6. X-konfigürasyon servo sistemi (AUX 1-4)
python test_servo_control.py

# 7. Motor kontrolü (MAIN 1)
python test_motor_control.py

# 8. Stabilizasyon algoritması (X-fin mixing)
python test_stabilization.py

# 9. Tam sistem entegrasyon testi
python test_full_system.py
```

### X-Konfigürasyon Fin Sistemi
```
   Ön Sol (AUX1) ──────── Ön Sağ (AUX2)
       \                     /
        \       X-FIN       /
         \   KONFİGÜR.    /
          \               /
           \             /
  Arka Sol (AUX3) ──────── Arka Sağ (AUX4)

Roll:  Sol finler ↔ Sağ finler  
Pitch: Ön finler ↔ Arka finler
Yaw:   X-Diagonal kontrol
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