#!/bin/bash
# TEKNOFEST 2025 - Pi5 Hardware Test Launcher

echo "🚀 TEKNOFEST ROV - Pi5 Hardware Test"
echo "=================================="

# Test tipi seç
echo "Test modunu seçin:"
echo "1) Headless Test (GUI yok, sadece sensör testleri)"
echo "2) EGLFS GUI (Framebuffer üzerinde GUI)"
echo "3) Offscreen GUI (Görünmez GUI - test için)"
echo "4) VNC Setup (Uzaktan bağlantı için)"

read -p "Seçiminiz (1-4): " choice

case $choice in
    1)
        echo "🔧 Headless test modu başlatılıyor..."
        python3 main_gui.py --headless
        ;;
    2)
        echo "🖥️ EGLFS GUI modu başlatılıyor..."
        export QT_QPA_PLATFORM=eglfs
        python3 main_gui.py
        ;;
    3)
        echo "👻 Offscreen GUI modu başlatılıyor..."
        export QT_QPA_PLATFORM=offscreen
        python3 main_gui.py
        ;;
    4)
        echo "📡 VNC kurulumu yapılıyor..."
        
        # VNC etkinleştir
        echo "VNC etkinleştiriliyor..."
        sudo raspi-config nonint do_vnc 0
        
        # VNC server başlat
        echo "VNC server başlatılıyor..."
        vncserver :1 -geometry 1024x768 -depth 24
        
        echo "✅ VNC kurulumu tamamlandı!"
        echo "📱 PC'nizden RealVNC Viewer ile bağlanın:"
        echo "   IP: $(hostname -I | awk '{print $1}'):5901"
        echo "   Kullanıcı: pi"
        echo ""
        echo "VNC oturumunda şu komutu çalıştırın:"
        echo "   cd ~/2025-TEKNOFEST-Su-Alti-Roket-Araci/App"
        echo "   python3 main_gui.py"
        ;;
    *)
        echo "❌ Geçersiz seçim!"
        exit 1
        ;;
esac 