# 🔧 Pi5 TCP MAVLink Sorun Giderme Kılavuzu

## ⚠️ **TESPİT EDİLEN SORUNLAR**

Senin sisteminde bu sorunlar var:

1. **Socat Config Hatları**: `tcgetattr` hataları - terminal ayarları uyumsuz
2. **Python PEP 668**: Sistem Python'una paket yüklenemez  
3. **Pixhawk Port**: `/dev/ttyACM0` doğru port mu bilinmiyor
4. **TCP Broken Pipe**: Socat bağlantı kopmaları

---

## 🚀 **ADIM ADIM ÇÖZÜM**

### **ADIM 1: Pixhawk Port Kontrolü**

```bash
# Port tespit scripti çalıştır
chmod +x pixhawk_port_check.sh
bash pixhawk_port_check.sh

# Manuel kontrol
ls -la /dev/ttyACM* /dev/ttyUSB*
lsusb | grep -i "3D\|ArduPilot\|PX4"
```

**🎯 Beklenen Çıktı:** `/dev/ttyACM0` veya `/dev/ttyUSB0` görünmeli

### **ADIM 2: Düzeltilmiş Socat Service**

```bash
# Eski service'i durdur
sudo systemctl stop mavtcp
sudo systemctl disable mavtcp

# Yeni düzeltilmiş service yükle
sudo cp mavtcp_fixed.service /etc/systemd/system/mavtcp.service
sudo systemctl daemon-reload
sudo systemctl enable mavtcp
sudo systemctl start mavtcp

# Durumu kontrol et
sudo systemctl status mavtcp
```

**🎯 Beklenen Çıktı:** `Active: active (running)` ve hata yok

### **ADIM 3: Python Venv Kurulumu**

```bash
# PEP 668 uyumlu venv kurulumu
chmod +x pi5_venv_setup.sh
bash pi5_venv_setup.sh
```

**🎯 Beklenen Çıktı:** `✅ Venv kurulumu tamamlandı!`

### **ADIM 4: TCP Bağlantı Testi**

```bash
# Venv aktifleştir
source venv/bin/activate

# TCP test çalıştır
python3 test_tcp_connection.py
```

**🎯 Beklenen Çıktı:**
```
✅ TCP Port     : PASS
✅ MAVLink      : PASS
🚀 Sistem hazır!
```

### **ADIM 5: Terminal GUI Başlat**

```bash
# Wrapper script ile başlat
chmod +x run_terminal_gui.sh
bash run_terminal_gui.sh
```

---

## 🔍 **SORUN GİDERME**

### **Sorun: "tcgetattr: Inappropriate ioctl for device"**

**Sebep:** Socat terminal ayarları yanlış  
**Çözüm:**
```bash
# Service dosyasını kontrol et
sudo systemctl edit mavtcp --full

# Bu satırı bul ve değiştir:
# ESKI: ExecStart=/usr/bin/socat TCP-LISTEN:5777,reuseaddr,fork /dev/ttyACM0,raw,echo=0,nonblock,b115200
# YENİ: ExecStart=/usr/bin/socat -d -d TCP-LISTEN:5777,bind=0.0.0.0,reuseaddr,fork OPEN:/dev/ttyACM0,nonblock
```

### **Sorun: "Broken pipe"**

**Sebep:** Pixhawk bağlantısı kesiliyor  
**Çözüm:**
```bash
# USB kablosunu kontrol et
# Pixhawk power cycling yap
# Service restart et
sudo systemctl restart mavtcp
```

### **Sorun: "externally-managed-environment"**

**Sebep:** PEP 668 - sistem Python korumalı  
**Çözüm:**
```bash
# Venv kullan (önerilen)
bash pi5_venv_setup.sh

# VEYA sistem override (riskli)
pip install --break-system-packages pymavlink
```

### **Sorun: Port 5777 dinlenmiyor**

**Sebep:** Socat başlamamış  
**Çözüm:**
```bash
# Service durumu
sudo systemctl status mavtcp

# Log kontrol
sudo journalctl -u mavtcp -f

# Manuel socat testi
sudo socat TCP-LISTEN:5777,bind=0.0.0.0,reuseaddr,fork OPEN:/dev/ttyACM0,nonblock
```

### **Sorun: Pixhawk tanınmıyor**

**Sebep:** USB driver sorunu  
**Çözüm:**  
```bash
# USB reset
sudo modprobe -r cdc_acm
sudo modprobe cdc_acm

# Permissions kontrol
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0
```

---

## ✅ **BAŞARILI KURULUM KONTROLLİSTESİ**

- [ ] Pixhawk `/dev/ttyACM0` veya `/dev/ttyUSB0`'da görünüyor
- [ ] `sudo systemctl status mavtcp` = active (running)
- [ ] `ss -tlnp | grep 5777` = port dinleniyor
- [ ] `python3 test_tcp_connection.py` = tüm testler PASS
- [ ] `bash run_terminal_gui.sh` = GUI başlıyor
- [ ] Terminal GUI'de "✅ TCP MAVLink bağlantısı kuruldu" mesajı
- [ ] Live IMU verileri görünüyor (Roll/Pitch/YAW)
- [ ] Q/E tuşları YAW kontrolü yapıyor

---

## 📞 **DESTEK**

Hala sorun varsa şu bilgileri paylaş:

```bash
# Sistem durumu raporu
echo "=== SİSTEM DURUMU ==="
uname -a
lsusb | grep -i "3D\|ArduPilot\|PX4"
ls -la /dev/ttyACM* /dev/ttyUSB*
sudo systemctl status mavtcp
ss -tlnp | grep 5777
source venv/bin/activate && python3 -c "import pymavlink; print('PyMAVLink OK')"
```

Bu çıktıyı gönder, hızlı analiz yapalım! 🚀 