#!/bin/bash
# Hava Yarışı Test Çalıştırma Scripti

echo "🚁 Hava Yarışı Test Sistemi"
echo "=============================="

# Klasöre git
cd "$(dirname "$0")"

# Python path ayarla
export PYTHONPATH="$(pwd):$PYTHONPATH"

# Bağımlılık kontrolü
echo "📦 Bağımlılık kontrolü..."
python3 -c "import pymavlink" 2>/dev/null || {
    echo "❌ pymavlink bulunamadı!"
    echo "Kurulum: pip3 install pymavlink"
    exit 1
}

# GPIO kontrolü (opsiyonel)
echo "🔧 GPIO kontrolü..."
if python3 -c "import RPi.GPIO" 2>/dev/null; then
    echo "✅ RPi.GPIO mevcut"
elif python3 -c "import lgpio" 2>/dev/null; then
    echo "✅ lgpio mevcut"
else
    echo "⚠️ GPIO kütüphanesi yok - simülasyon modunda çalışacak"
fi

# Test tipini sor
echo ""
echo "Test tipi seçin:"
echo "1) Sadece bağlantı testi"
echo "2) Tam hava yarışı testi"
read -p "Seçiminiz (1-2): " choice

case $choice in
    1)
        echo "🧪 Bağlantı testi başlatılıyor..."
        python3 main_air_test.py --test-only
        ;;
    2)
        echo "🚁 Tam hava yarışı testi başlatılıyor..."
        echo "⚠️ UYARI: Aracın güvenli bir alanda olduğundan emin olun!"
        echo "⚠️ Test sırasında Ctrl+C ile acil durdurma yapabilirsiniz"
        read -p "Devam etmek için Enter'a basın..."
        python3 main_air_test.py
        ;;
    *)
        echo "❌ Geçersiz seçim!"
        exit 1
        ;;
esac

echo ""
echo "🏁 Test tamamlandı!"
