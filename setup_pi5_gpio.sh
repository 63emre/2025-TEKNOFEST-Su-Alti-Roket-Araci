#!/bin/bash
# SARA GPIO Kurulum Scripti - Raspberry Pi 5 Uyumlu

echo "🚀 SARA - Su Altı Roket Aracı GPIO Kurulum Scripti"
echo "🔧 Raspberry Pi 5 uyumlu GPIO kütüphanesi kurulumu"
echo "=================================================="

# Sistem bilgisi
echo "📋 Sistem Bilgisi:"
echo "Pi Model: $(cat /proc/device-tree/model 2>/dev/null || echo 'Bilinmiyor')"
echo "OS: $(lsb_release -d 2>/dev/null | cut -f2 || echo 'Bilinmiyor')"
echo ""

# Eski GPIO kütüphanesini kaldır
echo "🗑️ Eski GPIO kütüphanesi temizleniyor..."
sudo pip3 uninstall RPi.GPIO -y 2>/dev/null || true
pip3 uninstall RPi.GPIO -y 2>/dev/null || true
sudo apt remove python3-rpi.gpio -y 2>/dev/null || true

# Sistem güncellemeleri
echo "📦 Sistem güncellemeleri yapılıyor..."
sudo apt update && sudo apt upgrade -y
sudo apt install python3 python3-pip git -y

# Pi 5 uyumlu GPIO kütüphanesi
echo "🔌 rpi-lgpio kütüphanesi kuruluyor..."
sudo pip3 install rpi-lgpio --force-reinstall

# Diğer gereksinimler
echo "📚 Diğer kütüphaneler kuruluyor..."
sudo pip3 install pymavlink smbus2 numpy colorlog

# User'ı GPIO grubuna ekle
echo "👤 Kullanıcı GPIO grubuna ekleniyor..."
sudo usermod -a -G gpio $USER

# GPIO test
echo "🧪 GPIO testi yapılıyor..."
python3 -c "
try:
    import lgpio
    h = lgpio.gpiochip_open(0)
    print('✅ GPIO chip başarıyla açıldı')
    lgpio.gpiochip_close(h)
    print('✅ GPIO kütüphanesi çalışıyor')
except Exception as e:
    print(f'❌ GPIO testi başarısız: {e}')
    exit(1)
" || {
    echo "❌ GPIO kurulumu başarısız!"
    exit 1
}

# Proje dosyalarını test et
if [[ -f "görevlerf1/pluswing/gpio_compat.py" ]]; then
    echo "🧪 GPIO uyumluluk katmanı testi..."
    cd görevlerf1/pluswing
    python3 gpio_compat.py || {
        echo "⚠️ GPIO uyumluluk testi başarısız"
        cd - > /dev/null
    }
    cd - > /dev/null
else
    echo "⚠️ GPIO uyumluluk dosyası bulunamadı"
fi

echo ""
echo "🎉 Kurulum tamamlandı!"
echo ""
echo "📝 Sonraki adımlar:"
echo "1. Çıkış yapın ve tekrar giriş yapın (GPIO grup izinleri için)"
echo "2. 'python3 görevlerf1/pluswing/gpio_compat.py' ile test edin"
echo "3. Ana programı çalıştırın: 'python3 görevlerf1/pluswing/main.py'"
echo ""
echo "📖 Detaylı bilgi: PI5_SETUP_INSTRUCTIONS.md"
