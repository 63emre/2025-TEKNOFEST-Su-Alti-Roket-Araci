# TEKNOFEST 2025 - Su Altı ROV 
## Raspberry Pi Kurulum ve Kullanım Kılavuzu

### 🚀 Hızlı Başlangıç

#### 1. Sistem Kurulumu
```bash
# Kurulum scriptini çalıştır
chmod +x setup_raspberry_pi.sh
./setup_raspberry_pi.sh
```

#### 2. Manuel Kurulum (Alternatif)
```bash
# Paket güncellemesi
sudo apt update && sudo apt upgrade -y

# Python bağımlılıkları
sudo apt install -y python3 python3-pip python3-venv python3-pyqt5

# GPIO izinleri
sudo usermod -a -G gpio,dialout,i2c,spi $USER

# Python paketleri
pip3 install -r requirements.txt
```

### 🔧 Konfigürasyon

#### Hardware Config (`config/hardware_config.json`)
```json
{
  "pixhawk": {
    "servos": {
      "front_left": 1,
      "rear_left": 3,
      "rear_right": 4,
      "front_right": 5
    },
    "motor": 6
  },
  "raspberry_pi": {
    "gpio": {
      "buzzer": 7,
      "control_button": 13,
      "led_red": 4
    }
  }
}
```

#### MAVLink Bağlantısı
- **BlueOS**: `tcp:127.0.0.1:5777`
- **ArduSub**: `udp:192.168.1.1:14550`
- **Serial**: `/dev/ttyUSB0:57600`

### 🚨 Bilinen Sorunlar ve Çözümler

#### 1. Indentasyon Hatası (main_gui.py Line 1120)
```python
# HATALI KOD:
# MAVLink defaults
self.connection_string.setText("tcp:127.0.0.1:5777")

# DOĞRU KOD:
            # MAVLink defaults
            self.connection_string.setText("tcp:127.0.0.1:5777")
```

**Çözüm:**
```bash
# Otomatik düzeltme
sed -i '1120,1122s/^/            /' App/main_gui.py
```

#### 2. PyQt5 Display Problemi
```bash
export DISPLAY=:0
xhost +local:root
```

#### 3. GPIO İzin Problemi
```bash
sudo usermod -a -G gpio pi
sudo reboot
```

#### 4. MAVLink Bağlantı Problemi
```bash
# Port kontrolü
netstat -tulpn | grep 5777

# MAVProxy test
mavproxy.py --master=tcp:127.0.0.1:5777
```

### 📱 GUI Kullanımı

#### Başlatma
```bash
cd App
source venv/bin/activate  # Eğer venv kullanıyorsanız
python3 main_gui.py
```

#### Ana Özellikler
- **MAVLink Bağlantısı**: Pixhawk ile iletişim
- **Real-time Kontrol**: W,A,S,D tuşları ile hareket
- **Titreşim Analizi**: RAW vs PID karşılaştırması
- **Script Entegrasyonu**: Servo/motor kalibrasyonu
- **Görev Planlama**: Otomatik navigation

#### Kontrol Modları
1. **RAW PWM**: Doğrudan servo kontrolü (titreşimsiz)
2. **PID Control**: Filtreli kontrol (smooth)

#### Navigation Modları
1. **GPS Only**: GPS tabanlı navigation
2. **IMU Only**: Dead reckoning
3. **Hybrid**: GPS + IMU kombine

### 🔧 Troubleshooting

#### GUI Çalışmıyor
1. X11 forwarding kontrol edin
2. PyQt5 kurulumunu kontrol edin
3. Display değişkenini ayarlayın

#### MAVLink Bağlanamıyor
1. BlueOS/ArduSub çalışıyor mu?
2. Port açık mı? (5777, 14550)
3. Firewall bloke ediyor mu?

#### GPIO Çalışmıyor
1. Kullanıcı gpio grubunda mı?
2. I2C/SPI etkin mi?
3. Pin conflict var mı?

### 📊 Performans Optimizasyonu

#### Raspberry Pi 4 Önerilen Ayarlar
```bash
# GPU memory split
sudo raspi-config
# Advanced Options -> Memory Split -> 128

# CPU governor
echo performance | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

#### Python Optimizasyonu
```bash
# Gerekli olmayan servisleri durdur
sudo systemctl disable bluetooth
sudo systemctl disable wifi-powersave

# Swap artır
sudo dphys-swapfile swapoff
sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=1024/' /etc/dphys-swapfile
sudo dphys-swapfile setup
sudo dphys-swapfile swapon
```

### 🔄 Otomatik Başlatma

#### Systemd Service
```bash
sudo systemctl enable teknofest-rov.service
sudo systemctl start teknofest-rov.service
sudo systemctl status teknofest-rov.service
```

#### Desktop Autostart
```bash
mkdir -p ~/.config/autostart
cat > ~/.config/autostart/teknofest-rov.desktop << EOF
[Desktop Entry]
Type=Application
Name=TEKNOFEST ROV
Exec=/home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/main_gui.py
Hidden=false
NoDisplay=false
X-GNOME-Autostart-enabled=true
EOF
```

### 📞 Destek

- **Hardware Pin Mapping**: `HARDWARE_PIN_MAPPING.md`
- **Test Scripts**: `Test/` klasörü
- **Demo Videos**: `Kabiliyet Videosu/` klasörü

### 🚨 Acil Durum

#### Emergency Stop
```bash
# Script ile
python3 scripts/emergency_stop.py

# GPIO ile
echo 1 > /sys/class/gpio/gpio13/value
```

#### Sistem Sıfırlama
```bash
sudo systemctl stop teknofest-rov.service
pkill -f main_gui.py
sudo reboot
``` 