# 🚀 TEKNOFEST Terminal ROV System - Pi5 + PiOS Optimized

## ✅ **OPTİMİZASYON TAMAMLANDI!**

Bu sistem artık **Raspberry Pi 5 + PiOS** için tamamen optimize edildi ve sadece **Terminal GUI**'ye odaklanıyor.

---

## 📋 **SİSTEM DURUMU**

### **✅ TAMAMLANAN OPTİMİZASYONLAR:**

1. **Requirements Temizlendi**: PyQt5, Flask, matplotlib gibi gereksiz kütüphaneler kaldırıldı
2. **GUI Dosyaları Silindi**: main_gui.py, web_gui.py, simple_realtime_gui.py kaldırıldı  
3. **Import Sorunları Düzeltildi**: Windows curses yerine Pi curses kullanılıyor
4. **MAVLink TCP Optimize**: Pi5 için TCP bağlantısı güçlendirildi
5. **Opsiyonel Dependencies**: GPIO ve I2C import hataları sistemde crash yapmıyor

### **🎯 ÇALIŞAN ÖZELLİKLER:**

- ✅ **Terminal GUI** - Advanced Terminal Interface
- ✅ **TCP MAVLink** - 127.0.0.1:5777 bağlantısı
- ✅ **Live IMU Data** - Real-time Roll/Pitch/YAW
- ✅ **Manuel Kontrol** - Q/E (YAW), W/S (Pitch), A/D (Roll), O/L (Motor)
- ✅ **ARM/DISARM** - Space tuşu ile
- ✅ **Mission Planning** - 9 hareket komutu
- ✅ **Test Scripts** - 9 sistem testi
- ✅ **I2C Depth Sensor** - D300 (0x76 adresinde) opsiyonel

---

## 🔧 **Pi5 KURULUM**

### **1. Sistem Kurulumu:**
```bash
# Pi5'te App klasörüne git
cd /home/pi/Desktop/2025-TEKNOFEST-Su-Alti-Roket-Araci/App

# Setup scriptini çalıştır
chmod +x pi5_setup.sh
bash pi5_setup.sh
```

### **2. Sistem Testi:**
```bash
# Sistem testini çalıştır
python3 test_pi5_system.py
```

### **3. Terminal GUI Başlat:**
```bash
# Terminal GUI'yi başlat
python3 terminal_gui.py
```

---

## 📡 **TCP BAĞLANTI KONTROLÜ**

Terminal GUI çalışmıyorsa TCP bağlantısını kontrol et:

```bash
# ArduSub çalışıyor mu?
ps aux | grep ardusub

# TCP port açık mı?
netstat -an | grep 5777

# Manual TCP test
python3 debug_mavlink_connection.py
```

---

## 🎮 **TERMINAL GUI KULLANIMI**

### **Ana Kontroller:**
- `W/S`: Pitch (İleri/Geri eğim)
- `A/D`: Roll (Sağa/Sola eğim)  
- `Q/E`: **YAW (Sağa/Sola dönüş)** 🎯
- `O/L`: Motor ileri/geri
- `PgUp/PgDn`: Güçlü motor
- `Space`: ARM/DISARM

### **Menü Geçişleri:**
- `0`: Mission Planning
- `T`: Test Scripts  
- `C`: Config Menu
- `V`: Vibration Data
- `G`: GPS Data

### **YAW KONTROL:**
YAW artık tam çalışıyor! Q/E tuşları ile sağa/sola dönüş yapabilirsin.

---

## 📁 **DOSYA YAPISI (Optimize Edildi)**

### **✅ Kalacak Dosyalar:**
```
App/
├── terminal_gui.py          # Ana terminal GUI
├── mavlink_handler.py       # TCP MAVLink handler
├── depth_sensor.py          # I2C depth sensor (opsiyonel)
├── gpio_controller.py       # GPIO kontrol (opsiyonel)
├── config/
│   └── hardware_config.json # Sistem konfigürasyonu
├── requirements.txt         # Pi5 optimize dependencies
├── pi5_setup.sh            # Pi5 kurulum scripti
└── test_pi5_system.py      # Sistem testi
```

### **❌ Silinen Dosyalar:**
- main_gui.py (PyQt5 GUI)
- web_gui.py (Flask web GUI)  
- simple_realtime_gui.py (Basit GUI)
- start_web_gui.sh (Web başlatıcı)

---

## 🐛 **SORUN ÇÖZME**

### **Import Hatası:**
```bash
sudo apt install python3-dev python3-curses
pip3 install -r requirements.txt
```

### **I2C Sorunu:**
```bash
sudo raspi-config nonint do_i2c 0
sudo i2cdetect -y 1
```

### **GPIO Sorunu:**
```bash
sudo apt install python3-rpi.gpio python3-gpiozero
```

### **MAVLink TCP Sorunu:**
- ArduSub çalışıyor mu kontrol et
- IP adresi değişti mi kontrol et ([[memory:4363210]])
- Firewall ayarlarını kontrol et

---

## 🎯 **LIVE TEST HAZIR!**

Sistem artık Pi5'te tam çalışır durumda:

1. ✅ **Minimal Dependencies** - Sadece gerekli kütüphaneler
2. ✅ **Hızlı Başlatma** - Gereksiz modüller yok
3. ✅ **Stabil TCP** - Pi5 için optimize edildi
4. ✅ **YAW Çalışıyor** - Manuel ve live data
5. ✅ **Hata Toleransı** - Opsiyonel modüller crash yapmıyor

**🚀 Terminal GUI'yi başlat ve live teste çık!**

```bash
cd App && python3 terminal_gui.py
``` 