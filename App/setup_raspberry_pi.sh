#!/bin/bash
# TEKNOFEST 2025 - Su Altı ROV
# Raspberry Pi Kurulum Scripti

echo "🚀 TEKNOFEST Su Altı ROV - Raspberry Pi Kurulumu Başlıyor..."

# Sistem güncellemesi
echo "📦 Sistem paketleri güncelleniyor..."
sudo apt update && sudo apt upgrade -y

# Python3 ve pip kurulumu
echo "🐍 Python3 ve pip kurulumu..."
sudo apt install -y python3 python3-pip python3-venv python3-dev

# PyQt5 sistem paketleri
echo "🖥️ PyQt5 sistem bağımlılıkları..."
sudo apt install -y python3-pyqt5 python3-pyqt5-dev qt5-default

# GPIO ve I2C etkinleştirme
echo "⚡ GPIO ve I2C etkinleştiriliyor..."
sudo raspi-config nonint do_i2c 0
sudo raspi-config nonint do_spi 0
sudo raspi-config nonint do_serial 0

# Gerekli sistem kütüphaneleri
echo "📚 Sistem kütüphaneleri kuruluyor..."
sudo apt install -y \
    libatlas-base-dev \
    libhdf5-dev \
    libhdf5-serial-dev \
    libatlas-base-dev \
    libjasper-dev \
    libqtgui4 \
    libqt4-test \
    libgfortran5 \
    git \
    cmake \
    build-essential \
    avahi-daemon \
    avahi-utils

# Python virtual environment oluşturma
echo "🐍 Python virtual environment oluşturuluyor..."
python3 -m venv venv
source venv/bin/activate

# Python paketlerini kurma
echo "📦 Python paketleri kuruluyor..."
pip install --upgrade pip
pip install -r requirements.txt

# Kullanıcıyı dialout grubuna ekleme (serial access için)
echo "👤 Kullanıcı izinleri ayarlanıyor..."
sudo usermod -a -G dialout $USER
sudo usermod -a -G gpio $USER
sudo usermod -a -G i2c $USER
sudo usermod -a -G spi $USER

# MAVProxy kurulumu (opsiyonel)
echo "✈️ MAVProxy kurulumu..."
pip install MAVProxy

# Config klasörü kontrolü
echo "⚙️ Konfigürasyon dosyaları kontrol ediliyor..."
if [ ! -d "config" ]; then
    mkdir config
    echo "📁 Config klasörü oluşturuldu"
fi

# Systemd service dosyası oluşturma (opsiyonel)
echo "🔧 Systemd service oluşturuluyor..."
sudo tee /etc/systemd/system/teknofest-rov.service > /dev/null <<EOF
[Unit]
Description=TEKNOFEST ROV Control System
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App
Environment=DISPLAY=:0
ExecStart=/home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/venv/bin/python /home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/terminal_gui.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Service'i etkinleştirme
sudo systemctl enable teknofest-rov.service

echo "✅ Kurulum tamamlandı!"
echo ""
echo "🖥️ Terminal GUI KULLANIMI:"
echo "1. Servis başlatma: sudo systemctl start teknofest-rov"
echo "2. Terminal GUI: python3 terminal_gui.py"
echo ""
echo "🔧 SERVİS KONTROL:"
echo "sudo systemctl status teknofest-rov"
echo "sudo systemctl stop teknofest-rov"
echo "sudo systemctl restart teknofest-rov"
echo ""
echo "📋 NOT: Web GUI ve Nginx yapılandırmaları kaldırıldı."