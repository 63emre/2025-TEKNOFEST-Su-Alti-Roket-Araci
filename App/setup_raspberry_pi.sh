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
ExecStart=/home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/venv/bin/python /home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App/main_gui.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

# Service'i etkinleştirme
sudo systemctl enable teknofest-rov.service

echo "🌐 Web GUI servisi kuruluyor..."
sudo tee /etc/systemd/system/teknofest-rov-web.service > /dev/null << 'EOF'
[Unit]
Description=TEKNOFEST ROV Web GUI
After=network.target
Wants=network.target

[Service]
Type=simple
User=pi
Group=pi
WorkingDirectory=/home/pi/2025-TEKNOFEST-Su-Alti-Roket-Araci/App
Environment=PYTHONPATH=/usr/lib/python3/dist-packages
ExecStart=/usr/bin/python3 web_gui.py
Restart=on-failure
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable teknofest-rov-web.service

echo "🔗 Nginx proxy konfigürasyonu (isteğe bağlı)..."
if command -v nginx > /dev/null 2>&1; then
    sudo tee /etc/nginx/sites-available/rov-web > /dev/null << 'EOF'
server {
    listen 8080;
    server_name _;
    
    location / {
        proxy_pass http://127.0.0.1:5000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
        proxy_set_header X-Real-IP $remote_addr;
        proxy_set_header X-Forwarded-For $proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto $scheme;
        proxy_read_timeout 86400;
    }
}
EOF

    sudo ln -sf /etc/nginx/sites-available/rov-web /etc/nginx/sites-enabled/
    sudo nginx -t && sudo systemctl reload nginx 2>/dev/null || echo "⚠️ Nginx konfigürasyonu atlandı"
else
    echo "ℹ️ Nginx bulunamadı, proxy konfigürasyonu atlandı"
fi

echo "✅ Kurulum tamamlandı!"
echo ""
echo "🌐 WEB GUI KULLANIMI (Önerilen):"
echo "1. Otomatik başlatma: sudo systemctl start teknofest-rov-web"
echo "2. Web arayüz: http://192.168.2.2:5000 (ROV IP)"
echo "3. Nginx proxy: http://192.168.2.2:8080 (varsa)"
echo ""
echo "📱 MANUEL BAŞLATMA:"
echo "cd App && python3 web_gui.py"
echo ""
echo "🔧 SERVİS KONTROL:"
echo "sudo systemctl status teknofest-rov-web"
echo "sudo systemctl stop teknofest-rov-web"
echo "sudo systemctl restart teknofest-rov-web"
echo ""
echo "📋 DESKTOP GUI (VNC gerekli):"
echo "1. VNC kurulumu: sudo apt install tightvncserver"
echo "2. VNC başlat: vncserver :1 -geometry 1024x768"
echo "3. Display ayarla: export DISPLAY=:1 && python3 main_gui.py"
echo "4. VNC erişim: 192.168.2.2:5901" 

cd ~/2025-TEKNOFEST-Su-Alti-Roket-Araci/App

# Venv'i tamamen sil
rm -rf venv

# System packages kullan (venv KULLANMA!)
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"