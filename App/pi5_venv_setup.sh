#!/bin/bash
# Pi5 + PiOS PEP 668 Venv Kurulumu

echo "🐍 Python Venv Kurulumu (PEP 668 Uyumlu)..."

# Gerekli sistem paketleri
echo "📦 Sistem paketlerini kontrol et..."
sudo apt install -y python3-full python3-venv python3-dev

# Mevcut venv varsa sil
if [ -d "venv" ]; then
    echo "🗑️ Eski venv temizleniyor..."
    rm -rf venv
fi

# Yeni venv oluştur
echo "🆕 Python venv oluşturuluyor..."
python3 -m venv venv

# Venv aktifleştir
echo "⚡ Venv aktifleştiriliyor..."
source venv/bin/activate

echo "📦 Python paketleri yükleniyor (venv içinde)..."

# Pip güncelle
pip install --upgrade pip

# Requirements yükle
pip install -r requirements.txt

echo "✅ Venv kurulumu tamamlandı!"
echo ""
echo "🚀 Kullanım:"
echo "   source venv/bin/activate    # Venv aktifleştir"  
echo "   python3 terminal_gui.py     # GUI başlat"
echo "   deactivate                  # Venv kapat"
echo ""
echo "📁 Venv bilgileri:"
which python3
python3 --version
pip list | head -10 