# D300 Depth Sensor Kurulum Rehberi

## 📊 Durum: MAVLink Üzerinden Çalışıyor

D300 depth sensörü şu anda Pixhawk üzerinden MAVLink ile çalışıyor. Terminal GUI otomatik olarak MAVLink'den depth ve sıcaklık verilerini alıyor.

## 🔧 İki Seçenek

### Seçenek 1: MAVLink (Önerilen - Şu anda aktif)
- ✅ Pixhawk'a bağlı D300 sensörü
- ✅ TCP bağlantısı: `tcp:127.0.0.1:5777`
- ✅ Terminal GUI otomatik olarak veri alıyor
- ✅ Depth, sıcaklık ve basınç verisi

### Seçenek 2: Doğrudan I2C (İsteğe bağlı)

#### I2C Etkinleştirmek için:
```bash
# I2C setup scriptini çalıştır
chmod +x i2c_setup.sh
./i2c_setup.sh

# Sistem yeniden başlat
sudo reboot

# I2C cihazlarını tara
python3 i2c_scanner.py
```

#### I2C Bağlantı Şeması:
```
D300 Sensör -> Raspberry Pi
VCC (3.3V)  -> Pin 1  (3.3V)
GND         -> Pin 6  (GND)  
SDA         -> Pin 3  (GPIO 2)
SCL         -> Pin 5  (GPIO 3)
```

## 🎮 Terminal GUI Kullanımı

### Motor Kontrol:
- **O/L tuşları**: Motor artır/azalt (Ana)
- **Page Up/Down**: Motor artır/azalt (Alternatif)
- **+/- tuşları**: Motor ince ayar

### Çıkış:
- **ESC tuşu**: Çıkış
- **P tuşu**: Çıkış

## 📡 Veri Akışı

```
D300 Sensor -> Pixhawk -> MAVLink -> Terminal GUI
     |              |         |          |
  I2C/SPI      ArduSub    TCP:5777   Python App
```

## 🔍 Sorun Giderme

### MAVLink'de veri yok:
1. Pixhawk bağlantısını kontrol et
2. ArduSub firmware güncel mi?
3. Depth sensor Pixhawk'a doğru bağlı mı?

### I2C'de cihaz yok:
1. I2C etkin mi: `sudo i2cdetect -y 1`
2. Kablolama doğru mu?
3. Sensor çalışıyor mu?

## 📋 Test Komutları

```bash
# MAVLink bağlantı testi
python3 -c "from mavlink_handler import MAVLinkHandler; m=MAVLinkHandler(); print(m.connect())"

# I2C tarama
python3 i2c_scanner.py

# Terminal GUI başlat
python3 terminal_gui.py
```

## 💡 İpuçları

- **0x76 adresi**: Standart D300 I2C adresi
- **MAVLink Priority**: Terminal GUI önce MAVLink'den veri almaya çalışır
- **Real-time**: Veriler 0.2s aralıklarla güncellenir
- **Derinlik hesabı**: Basınctan otomatik hesaplanır (1 mbar ≈ 1 cm su)

## 🚀 Optimizasyon

MAVLink üzerinden veri alım en stabil yöntem. I2C sadece debug/test için gerektiğinde kullanın. 