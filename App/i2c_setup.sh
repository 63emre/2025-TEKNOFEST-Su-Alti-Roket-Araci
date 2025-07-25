#!/bin/bash
# TEKNOFEST Su Altı ROV - I2C Setup Script
# Raspberry Pi'da I2C'yi etkinleştir

echo "🔧 TEKNOFEST Su Altı ROV - I2C Setup"
echo "===================================="

# I2C'yi etkinleştir
echo "📡 I2C etkinleştiriliyor..."

# /boot/config.txt'ye I2C ekle
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" | sudo tee -a /boot/config.txt
    echo "✅ I2C ARM etkinleştirildi"
else
    echo "✅ I2C ARM zaten etkin"
fi

# I2C modüllerini yükle
echo "📦 I2C modülleri kontrol ediliyor..."

if ! grep -q "i2c-dev" /etc/modules; then
    echo "i2c-dev" | sudo tee -a /etc/modules
    echo "✅ i2c-dev modülü eklendi"
else
    echo "✅ i2c-dev modülü zaten var"
fi

if ! grep -q "i2c-bcm2708" /etc/modules; then
    echo "i2c-bcm2708" | sudo tee -a /etc/modules
    echo "✅ i2c-bcm2708 modülü eklendi"
else
    echo "✅ i2c-bcm2708 modülü zaten var"
fi

# I2C araçlarını yükle
echo "🛠️ I2C araçları yükleniyor..."
sudo apt update
sudo apt install -y i2c-tools python3-smbus

# I2C'yi modül olarak yükle
echo "🔄 I2C modüllerini yükleniyor..."
sudo modprobe i2c-dev
sudo modprobe i2c-bcm2708

# I2C cihazlarını listele
echo "📋 I2C cihazları taraniyor..."
echo "Bus 0:"
sudo i2cdetect -y 0 2>/dev/null || echo "  Bus 0 bulunamadı"

echo "Bus 1:"
sudo i2cdetect -y 1 2>/dev/null || echo "  Bus 1 bulunamadı"

# Kullanıcı izinleri
echo "👤 Kullanıcı izinleri ayarlanıyor..."
sudo usermod -a -G i2c $USER

echo ""
echo "✅ I2C setup tamamlandı!"
echo "💡 Değişikliklerin etkin olması için sistem yeniden başlatın:"
echo "   sudo reboot"
echo ""
echo "🔍 I2C cihazları taramak için:"
echo "   python3 i2c_scanner.py" 