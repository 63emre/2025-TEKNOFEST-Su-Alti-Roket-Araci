# 🔊 Buzzer ve LED (Pinger) Sinyalleri Klavuzu

## 📋 Sistem Durumları ve Sinyaller

### **🔌 Sistem Başlangıcı**

| **Durum** | **Buzzer Sinyali** | **LED Durumu** | **Açıklama** |
|-----------|-------------------|----------------|--------------|
| **Güç verildiğinde** | 3 kısa bip (0.2s) | 3x yanıp sönme (0.5s) | Sistem başlatılıyor |
| **Kalibrasyon sırasında** | 6 hızlı bip (0.1s) | Hızlı yanıp sönme (0.2s) | Sensörler otomatik kalibre ediliyor |
| **Kalibrasyon başarılı** | 2 uzun bip (0.5s) | 2x yavaş yanıp sönme (2.0s) | Kalibrasyon tamamlandı |
| **Kalibrasyon başarısız** | 10 hızlı bip (0.1s) | Çok hızlı yanıp sönme (0.05s) | Kalibrasyon hatası |

### **⏳ Görev Öncesi**

| **Durum** | **Buzzer Sinyali** | **LED Durumu** | **Açıklama** |
|-----------|-------------------|----------------|--------------|
| **Buton bekleme** | Sessiz | Yavaş yanıp sönme (1.0s) | Başlatma butonu bekleniyor |
| **Buton basıldı** | 3 orta bip (0.3s) | Sürekli açık | Görev başlatıldı |
| **90s geri sayım** | Kısa/uzun bip patterni | Çok hızlı yanıp sönme (0.1s) | Güvenlik gecikmesi |

### **🚀 Görev Fazları - Mission 1**

| **Faz** | **Buzzer Sinyali** | **LED Durumu** | **Açıklama** |
|---------|-------------------|----------------|--------------|
| **Faz 1 (İlk 10m)** | 1 bip | Orta hızda yanıp sönme (0.3s) | 2m derinlikte 10m ilerleme |
| **Faz 2 (Ana seyir)** | 2 bip | Orta hızda yanıp sönme (0.3s) | 3m derinlikte 40m ilerleme |
| **180° Dönüş** | 3 orta bip | Orta hızda yanıp sönme (0.3s) | Yön değiştirme |
| **Geri dönüş** | 4 bip | Orta hızda yanıp sönme (0.3s) | Başlangıç noktasına dönüş |
| **Yüzeye çıkış** | 2 uzun bip (1.0s) | Orta hızda yanıp sönme (0.3s) | Yüzeye çıkış |

### **🚀 Görev Fazları - Mission 2**

| **Faz** | **Buzzer Sinyali** | **LED Durumu** | **Açıklama** |
|---------|-------------------|----------------|--------------|
| **Hedefe yaklaşma** | 1 bip | Orta hızda yanıp sönme (0.3s) | 30m mesafe, 3m derinlik |
| **Roket hazırlık** | 5 orta bip (0.5s) | Orta hızda yanıp sönme (0.3s) | Pozisyonlama ve yunuslama açısı |
| **Roket fırlatma** | 1 çok uzun bip (2.0s) | Sürekli açık | CO2 sistemi aktif |
| **Geri çekilme** | 4 bip | Orta hızda yanıp sönme (0.3s) | Güvenli mesafeye çekilme |
| **Yüzeye çıkış** | 2 uzun bip (1.0s) | Orta hızda yanıp sönme (0.3s) | Yunuslama açısı ile yüzey |

### **✅ Görev Sonuçları**

| **Durum** | **Buzzer Sinyali** | **LED Durumu** | **Açıklama** |
|-----------|-------------------|----------------|--------------|
| **Görev başarılı** | 4 orta bip (0.5s) | Çok yavaş yanıp sönme (2.0s) | Tüm fazlar tamamlandı |
| **Görev başarısız** | 16 hızlı bip (0.1s) | Hızlı yanıp sönme (0.2s) | Görev iptal edildi |
| **Acil durum** | 20 çok hızlı bip (0.1s) | Çok hızlı yanıp sönme (0.05s) | Emergency stop aktif |

## 🔧 Teknik Detaylar

### **Otomatik Kalibrasyon**
- **Tetikleyici**: Güç verildiğinde otomatik (butona basılmadan)
- **Config ayarı**: `AUTO_CALIBRATION_ON_POWER = True`
- **Süre**: D300 için 6 saniye (deniz suyu)
- **Başarı kriteri**: En azından D300 sensörü kalibre olmalı

### **GPIO Pin Tanımları**
```python
GPIO_LED_RED = 21        # Kırmızı LED (Pinger)
GPIO_BUZZER = 9          # Buzzer
GPIO_START_BUTTON = 11   # Başlatma/Soft-kill butonu
```

### **Sinyal Süreleri (saniye)**
```python
# Sistem başlangıç
BUZZER_POWER_ON = [0.2, 0.1, 0.2, 0.1, 0.2]     # 3 kısa bip
BUZZER_CALIBRATION = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 6 hızlı bip
BUZZER_CALIBRATION_OK = [0.5, 0.2, 0.5]          # 2 uzun bip
BUZZER_CALIBRATION_FAIL = [0.1, 0.1] * 5         # 10 hızlı bip

# Görev fazları
BUZZER_PHASE_1 = [0.2]                               # 1 bip
BUZZER_PHASE_2 = [0.2, 0.1, 0.2]                    # 2 bip
BUZZER_TURNING = [0.3, 0.1, 0.3, 0.1, 0.3]         # 3 bip
BUZZER_RETURN = [0.2, 0.1, 0.2, 0.1, 0.2, 0.1, 0.2]  # 4 bip
BUZZER_SURFACING = [1.0, 0.3, 1.0]                  # 2 uzun bip

# Roket görevleri
BUZZER_ROCKET_PREP = [0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5]  # 5 orta bip
BUZZER_ROCKET_LAUNCH = [2.0]                         # 1 çok uzun bip

# Sonuçlar
BUZZER_MISSION_SUCCESS = [0.5, 0.2, 0.5, 0.2, 0.5, 0.2, 0.5]  # 4 orta bip
BUZZER_MISSION_FAIL = [0.1, 0.1] * 8                 # 16 hızlı bip
BUZZER_EMERGENCY = [0.1, 0.1] * 10                   # 20 hızlı bip
```

### **LED Yanıp Sönme Hızları (saniye)**
```python
LED_POWER_ON_BLINK = 0.5        # Güç verildiğinde
LED_CALIBRATION_BLINK = 0.2     # Kalibrasyon sırasında
LED_WAITING_BLINK = 1.0         # Buton bekleme
LED_COUNTDOWN_BLINK = 0.1       # Geri sayım
LED_PHASE_TRANSITION = 0.3      # Faz geçişlerinde
LED_EMERGENCY_BLINK = 0.05      # Acil durum
LED_SUCCESS_SLOW_BLINK = 2.0    # Görev başarılı
```

## 🎯 Kullanım Senaryoları

### **Normal Görev Akışı (Mission 1)**
1. **Güç ver** → 3 kısa bip + LED 3x yanıp sönme
2. **Otomatik kalibrasyon** → 6 hızlı bip + LED hızlı yanıp sönme
3. **Kalibrasyon OK** → 2 uzun bip + LED 2x yavaş yanıp sönme
4. **Buton bekle** → Sessiz + LED yavaş yanıp sönme
5. **Buton bas** → 3 orta bip + LED sürekli açık
6. **90s geri sayım** → Pattern + LED çok hızlı yanıp sönme
7. **Faz 1** → 1 bip + LED orta hızda yanıp sönme
8. **Faz 2** → 2 bip + LED orta hızda yanıp sönme
9. **180° dönüş** → 3 bip + LED orta hızda yanıp sönme
10. **Geri dönüş** → 4 bip + LED orta hızda yanıp sönme
11. **Yüzeye çıkış** → 2 uzun bip + LED orta hızda yanıp sönme
12. **Görev başarılı** → 4 orta bip + LED çok yavaş yanıp sönme

### **Hata Durumları**
- **Kalibrasyon başarısız** → 10 hızlı bip + LED çok hızlı yanıp sönme
- **Görev başarısız** → 16 hızlı bip + LED hızlı yanıp sönme
- **Acil durum** → 20 çok hızlı bip + LED çok hızlı yanıp sönme

## 🔍 Test Prosedürü

### **Buzzer Testi**
```bash
# Tüm sinyalleri test et
python3 -c "
from config import *
from utils import BuzzerController
buzzer = BuzzerController()
buzzer.beep_pattern(BUZZER_POWER_ON)
"
```

### **LED Testi**
```bash
# LED yanıp sönme testi
python3 -c "
from config import *
from utils import LEDController
led = LEDController()
led.blink(LED_WAITING_BLINK, count=5)
"
```

---

**📅 Güncelleme:** Buzzer ve LED sinyalleri tam otomatik
**🎯 Hedef:** Operatörsüz çalışma, sesli/görsel durum bildirimi
**✅ Durum:** Tüm faz geçişleri ve durumlar tanımlandı!
