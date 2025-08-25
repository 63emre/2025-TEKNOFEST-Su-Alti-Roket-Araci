#!/bin/bash
# Raspberry Pi 5 GPIO Dependencies Installer
# Pi 5 için rpi-lgpio kütüphanesini kurar

set -e  # Hata durumunda çık

echo "========================================"
echo "Raspberry Pi 5 GPIO Dependencies Installer"
echo "========================================"

# Sistem bilgilerini kontrol et
echo "Sistem bilgileri kontrol ediliyor..."
cat /proc/device-tree/model 2>/dev/null || echo "Model bilgisi alınamadı"

# Pi versiyonunu tespit et
if grep -q "Raspberry Pi 5" /proc/device-tree/model 2>/dev/null; then
    echo "✅ Raspberry Pi 5 tespit edildi"
    PI5_DETECTED=true
else
    echo "⚠️  Pi 5 tespit edilemedi, yine de devam ediliyor..."
    PI5_DETECTED=false
fi

# Mevcut Python versiyonunu kontrol et
PYTHON_VERSION=$(python3 --version 2>&1)
echo "Python versiyonu: $PYTHON_VERSION"

# Eski RPi.GPIO'yu kaldır (Pi 5'te çalışmaz)
echo ""
echo "Eski RPi.GPIO kaldırılıyor..."
pip3 uninstall -y RPi.GPIO 2>/dev/null || echo "RPi.GPIO zaten yüklü değil"

# Pi 5 için gerekli paketleri kur
echo ""
echo "Pi 5 için GPIO kütüphaneleri kuruluyor..."

# lgpio kütüphanesi (C library)
echo "lgpio C kütüphanesi kuruluyor..."
sudo apt update
sudo apt install -y lgpio

# rpi-lgpio Python kütüphanesi
echo "rpi-lgpio Python kütüphanesi kuruluyor..."
pip3 install rpi-lgpio

# Alternatif: pigpio da kurulabilir (backup için)
echo "pigpio kütüphanesi kuruluyor (backup için)..."
sudo apt install -y pigpio python3-pigpio
pip3 install pigpio

# Diğer gerekli kütüphaneleri kontrol et ve kur
echo ""
echo "Diğer gerekli kütüphaneler kontrol ediliyor..."

REQUIRED_PACKAGES=(
    "pymavlink"
    "smbus2"
)

for package in "${REQUIRED_PACKAGES[@]}"; do
    if pip3 show "$package" > /dev/null 2>&1; then
        echo "✅ $package zaten yüklü"
    else
        echo "📦 $package kuruluyor..."
        pip3 install "$package"
    fi
done

# GPIO izinlerini ayarla
echo ""
echo "GPIO izinleri ayarlanıyor..."
sudo usermod -a -G gpio $USER || echo "GPIO grup ayarı başarısız (normal olabilir)"

# Test dosyasını çalıştır
echo ""
echo "GPIO wrapper test ediliyor..."
cd "$(dirname "$0")"

if python3 gpio_wrapper.py; then
    echo "✅ GPIO wrapper test başarılı!"
else
    echo "⚠️  GPIO wrapper test başarısız, manuel kontrol gerekli"
fi

# Sistem yeniden başlatma önerisi
echo ""
echo "========================================"
echo "KURULUM TAMAMLANDI!"
echo "========================================"

if [ "$PI5_DETECTED" = true ]; then
    echo "✅ Raspberry Pi 5 için GPIO kütüphaneleri başarıyla kuruldu"
    echo ""
    echo "Kurulum sonrası öneriler:"
    echo "1. Sistemi yeniden başlatın: sudo reboot"
    echo "2. GPIO izinlerinin aktif olması için tekrar giriş yapın"
    echo "3. Test komutu: python3 main.py"
else
    echo "⚠️  Pi 5 tespit edilemedi, ancak kütüphaneler kuruldu"
    echo "   Pi 4 ve öncesi modellerde RPi.GPIO fallback kullanılacak"
fi

echo ""
echo "Kurulu GPIO kütüphaneleri:"
pip3 list | grep -E "(gpio|lgpio|pigpio)" || echo "GPIO kütüphanesi listesi alınamadı"

echo ""
echo "Manuel test için:"
echo "  python3 gpio_wrapper.py"
echo ""
