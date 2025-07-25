k#!/bin/bash
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
    build-essential

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
ExecStart=/home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/venv/bin/python /home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/main_gui.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Service'i etkinleştirme
sudo systemctl enable teknofest-rov.service

echo "✅ Kurulum tamamlandı!"
echo ""
echo "📋 Sonraki adımlar:"
echo "1. Raspberry Pi'yi yeniden başlatın: sudo reboot"
echo "2. Terminal'de: cd App && source venv/bin/activate"
echo "3. Uygulamayı çalıştırın: python main_gui.py"
echo ""
echo "🔧 Service olarak çalıştırmak için:"
echo "sudo systemctl start teknofest-rov.service"
echo "sudo systemctl status teknofest-rov.service"
echo ""
echo "🚨 UYARI: main_gui.py'deki indentasyon hatasını manuel olarak düzeltmeniz gerekiyor!"
echo "Line 1120 civarındaki 'self.connection_string.setText' satırının indentasyonunu düzeltin."

