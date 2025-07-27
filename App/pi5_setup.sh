#!/bin/bash
# TEKNOFEST 2025 - Pi5 + PiOS Optimized Setup
# Terminal GUI için minimal kurulum

echo "🚀 Pi5 + PiOS Terminal GUI Kurulumu Başlatılıyor..."
echo "📅 $(date)"

# Sistem güncellemesi
echo "🔄 Sistem güncelleniyor..."
sudo apt update && sudo apt upgrade -y

# Python3 ve pip kurulumu
echo "🐍 Python3 ve pip kontrol ediliyor..."
sudo apt install -y python3 python3-pip python3-dev python3-venv

# Terminal için gerekli sistem kütüphaneleri
echo "📺 Terminal UI kütüphaneleri yükleniyor..."
sudo apt install -y python3-curses python3-ncurses-dev

# I2C ve GPIO için Pi5 kütüphaneleri
echo "🔌 Pi5 GPIO ve I2C kütüphaneleri yükleniyor..."
sudo apt install -y python3-rpi.gpio python3-gpiozero i2c-tools

# I2C aktifleştir
echo "⚡ I2C aktifleştiriliyor..."
sudo raspi-config nonint do_i2c 0

# Python dependencies yükle
echo "📦 Python paketleri yükleniyor..."
cd /home/pi/Desktop/2025-TEKNOFEST-Su-Alti-Roket-Araci/App
pip3 install -r requirements.txt

# Test için I2C scan
echo "🔍 I2C cihazları taranıyor..."
sudo i2cdetect -y 1

# Terminal GUI test
echo "🧪 Terminal GUI test ediliyor..."
python3 -c "
import curses
print('✅ Curses OK')
import pymavlink
print('✅ MAVLink OK') 
try:
    import RPi.GPIO
    print('✅ GPIO OK')
except:
    print('⚠️ GPIO import hatası - normal olabilir')
"

echo "✅ Pi5 + PiOS kurulumu tamamlandı!"
echo "🚀 Terminal GUI başlatmak için: cd App && python3 terminal_gui.py" 