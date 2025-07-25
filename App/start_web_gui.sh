#!/bin/bash
# TEKNOFEST ROV - Web GUI Hızlı Başlatma

echo "🚀 TEKNOFEST ROV Web GUI başlatılıyor..."

# Çalışma dizinini ayarla
cd "$(dirname "$0")"

# Python path ayarla
export PYTHONPATH="/usr/lib/python3/dist-packages:$PYTHONPATH"

# Port kontrolü
if lsof -Pi :5000 -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️ Port 5000 zaten kullanımda!"
    echo "Mevcut process'i durduruyor..."
    sudo pkill -f web_gui.py
    sleep 2
fi

# Network interface kontrol
echo "🔗 Network durumu kontrol ediliyor..."
IP_ADDRESS=$(hostname -I | awk '{print $1}')
if [ -z "$IP_ADDRESS" ]; then
    echo "⚠️ Network bağlantısı bulunamadı, localhost kullanılacak"
    IP_ADDRESS="127.0.0.1"
fi

echo "📡 Web GUI erişim adresleri:"
echo "   - Lokal: http://127.0.0.1:5000"
echo "   - Network: http://$IP_ADDRESS:5000"

# Dependencies kontrolü
echo "📦 Dependencies kontrol ediliyor..."
python3 -c "import flask, flask_socketio" 2>/dev/null || {
    echo "❌ Flask/SocketIO bulunamadı!"
    echo "Kurmak için: pip3 install --user flask flask-socketio"
    exit 1
}

# Gerekli klasörleri oluştur
mkdir -p templates static logs

# Web GUI'yi başlat
echo "🌐 Web GUI başlatılıyor..."
echo "🎯 Ctrl+C ile durdurmak için bekleyin..."
echo ""

# Log dosyasını başlat
LOG_FILE="logs/web_gui_$(date +%Y%m%d_%H%M%S).log"
touch "$LOG_FILE"

# Python scripti çalıştır
python3 web_gui.py 2>&1 | tee "$LOG_FILE" 