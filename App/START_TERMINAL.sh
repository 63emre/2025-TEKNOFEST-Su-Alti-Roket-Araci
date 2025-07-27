#!/bin/bash

# TEKNOFEST Su Altı ROV - Terminal GUI Quick Starter
echo "🚀 TEKNOFEST ROV Terminal GUI - Quick Start"
echo "============================================"

# Check if running from correct directory
if [ ! -f "terminal_gui.py" ]; then
    echo "❌ Hata: App/ klasörünün içinde çalıştırın!"
    echo "💡 Çözüm: cd App && bash START_TERMINAL.sh"
    exit 1
fi

# Check Python version
python_version=$(python3 --version 2>&1)
echo "🐍 Python Version: $python_version"

# Check required libraries
echo "📦 Gerekli kütüphaneler kontrol ediliyor..."

if ! python3 -c "import curses" 2>/dev/null; then
    echo "❌ Curses library yok!"
    echo "🔧 Yükleniyor: sudo apt update && sudo apt install python3-dev"
    sudo apt update && sudo apt install python3-dev python3-curses -y
fi

if ! python3 -c "import pymavlink" 2>/dev/null; then
    echo "❌ PyMAVLink library yok!"
    echo "🔧 Yükleniyor: pip install pymavlink"
    pip install pymavlink
fi

echo "✅ Kütüphaneler hazır!"

# Set terminal size (if possible)
echo "🖥️ Terminal ayarları optimizing..."
if command -v resize >/dev/null 2>&1; then
    resize -s 35 130 2>/dev/null || true
fi

# Set environment variables
export PYTHONIOENCODING=utf-8
export TERM=${TERM:-xterm-256color}

echo ""
echo "🎮 TERMINAL GUI KONTROLLER:"
echo "   W/S: Pitch ±        A/D: Roll ±         Q/E: Yaw ±"
echo "   O/L: Motor ±        Space: ARM/DISARM"  
echo "   0: Mission Planning T: Test Scripts     X: Exit"
echo ""
echo "⏳ Terminal GUI başlatılıyor..."
echo ""

# Start the terminal GUI
python3 run_real_terminal.py

echo ""
echo "👋 Terminal GUI sonlandırıldı!"
echo "🔄 Tekrar başlatmak için: bash START_TERMINAL.sh" 