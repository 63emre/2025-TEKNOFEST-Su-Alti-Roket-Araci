#!/bin/bash
echo "🔍 Pixhawk Port Tespiti..."

echo "📡 USB Serial cihazları:"
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "❌ Hiç USB serial cihaz yok"

echo -e "\n📡 USB cihaz detayları:"
lsusb | grep -i "3D\|ArduPilot\|PX4\|Pixhawk" || echo "ℹ️ Bilinen flight controller bulunamadı"

echo -e "\n📡 Tüm USB cihazları:"
lsusb

echo -e "\n📡 dmesg'de USB bağlantıları:"
dmesg | tail -20 | grep -i "tty\|usb\|acm"

echo -e "\n📡 Aktif serial portlar:"
ls -la /dev/serial/by-id/ 2>/dev/null || echo "❌ /dev/serial/by-id/ yok"

echo -e "\n💡 Pixhawk'ı çıkar/tak ve bu scripti tekrar çalıştır" 