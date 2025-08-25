# RASPBERRY PI 5 KURULUM TALİMATLARI

## Raspberry Pi 5 için GPIO Uyumluluk Güncellemeleri

### Problem
Raspberry Pi 5'te eski `RPi.GPIO` kütüphanesi çalışmaz ve şu hatayı verir:
```
RuntimeError: Cannot determine SOC peripheral base address
```

### Çözüm
Bu proje `rpi-lgpio` kütüphanesi kullanacak şekilde güncellenmiştir.

## Kurulum Adımları

### 1. Sistem Güncellemeleri
```bash
sudo apt update && sudo apt upgrade -y
sudo apt install python3 python3-pip git -y
```

### 2. Eski GPIO Kütüphanesini Kaldır (Eğer Yüklüyse)
```bash
sudo pip3 uninstall RPi.GPIO -y
pip3 uninstall RPi.GPIO -y
```

### 3. Yeni GPIO Kütüphanesini Yükle
```bash
# Ana GPIO kütüphanesi (Pi 5 uyumlu)
sudo pip3 install rpi-lgpio

# Diğer gereksinimler
sudo pip3 install pymavlink smbus2 numpy colorlog
```

### 4. Proje Gereksinimlerini Yükle
```bash
cd /path/to/project
sudo pip3 install -r requirements.txt
```

### 5. GPIO Test
```bash
# Pi 5 için test
python3 görevlerf1/pluswing/gpio_compat.py

# Manuel test
python3 -c "
import lgpio
h = lgpio.gpiochip_open(0)
print('✓ GPIO chip açıldı başarıyla')
lgpio.gpiochip_close(h)
"
```

## Değişiklik Özeti

### Değiştirilen Dosyalar:
- `görevlerf1/pluswing/gpio_compat.py` → **YENİ** - GPIO uyumluluk katmanı
- `görevlerf1/pluswing/utils.py` → GPIO import değiştirildi
- `görevlerf1/pluswing/mission2.py` → GPIO import değiştirildi  
- `Test/test_gpio_button.py` → GPIO import değiştirildi
- `Test/test_led_buzzer.py` → GPIO import değiştirildi
- `Test/test_x_wing_realtime_control.py` → GPIO import değiştirildi
- `requirements.txt` → **YENİ** - Pi 5 uyumlu gereksinimler
- `README.md` → Kurulum talimatları güncellendi
- `HARDWARE_PIN_MAPPING.md` → Test örnekleri güncellendi

### GPIO API Değişiklikleri:
- `import RPi.GPIO as GPIO` → `from gpio_compat import GPIO`
- GPIO API aynı kaldı (uyumluluk katmanı sayesinde)
- Otomatik fallback: lgpio → RPi.GPIO (eski Pi'ler için)

## Doğrulama

### 1. GPIO Çalışma Testi
```bash
cd görevlerf1/pluswing
python3 gpio_compat.py
```

### 2. Ana Program Testi  
```bash
cd görevlerf1/pluswing
python3 main.py
```

### 3. LED/Buzzer Testi
```bash
cd Test
python3 test_led_buzzer.py
```

## Sorun Giderme

### "lgpio not found" Hatası:
```bash
sudo pip3 install rpi-lgpio --force-reinstall
```

### İzin Sorunları:
```bash
sudo usermod -a -G gpio $USER
# Çıkış yap ve tekrar giriş yap
```

### Eski GPIO Artıkları:
```bash
sudo pip3 uninstall RPi.GPIO -y
pip3 uninstall RPi.GPIO -y
sudo apt remove python3-rpi.gpio -y
```

## Başarı Kriterleri

✅ `gpio_compat.py` test dosyası çalışır  
✅ Ana program başlar (MAVLink bağlantı hatası normal)  
✅ LED/Buzzer testleri çalışır  
✅ GPIO pinleri kontrol edilebilir  
✅ "Cannot determine SOC peripheral base address" hatası alınmaz  

---

**Not:** Bu güncellemeler sayesinde proje hem Raspberry Pi 5'te hem de eski Pi modellerinde çalışacaktır.
