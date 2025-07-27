#!/bin/bash
# Terminal GUI Venv Wrapper

echo "🚀 TEKNOFEST Terminal GUI Başlatılıyor..."

# Venv kontrol et
if [ ! -d "venv" ]; then
    echo "❌ Python venv bulunamadı!"
    echo "💡 Önce kurulum yap: bash pi5_venv_setup.sh"
    exit 1
fi

# Venv aktifleştir
echo "⚡ Python venv aktifleştiriliyor..."
source venv/bin/activate

# Python ve paketleri kontrol et
echo "📦 Paket kontrolü..."
python3 -c "
try:
    import pymavlink
    import curses
    print('✅ Gerekli paketler hazır')
except ImportError as e:
    print(f'❌ Paket eksik: {e}')
    exit(1)
"

if [ $? -ne 0 ]; then
    echo "💡 Paketleri yükle: bash pi5_venv_setup.sh"
    exit 1
fi

# TCP bağlantı hızlı testi
echo "🔌 TCP 5777 hızlı test..."
timeout 3 bash -c '</dev/tcp/127.0.0.1/5777' 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✅ TCP 5777 erişilebilir"
else
    echo "⚠️ TCP 5777 erişilemeyebilir"
    echo "💡 Kontrol et: sudo systemctl status mavtcp"
fi

# Terminal GUI başlat
echo "🖥️ Terminal GUI başlatılıyor..."
echo "💡 Çıkmak için: Ctrl+C veya X tuşu"
echo ""

# GUI'yi başlat
python3 terminal_gui.py

# Temizlik
echo ""
echo "👋 Terminal GUI kapatıldı"
deactivate 