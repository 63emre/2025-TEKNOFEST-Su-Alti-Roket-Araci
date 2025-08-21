# TEKNOFEST 2025 - Su Altı Roket Aracı
## Plus Wing (+) Konfigürasyonu Sistemi v1.0

Bu dokümantasyon, Plus Wing (+) konfigürasyonunun detaylarını, X Wing konfigürasyonu ile farklarını ve test sistemlerini açıklamaktadır.

---

## 🔧 **PLUS WING KONFİGÜRASYONU NEDİR?**

Plus Wing (+) konfigürasyonu, servo'ların artı (+) şeklinde yerleştirildiği bir kontrol sistemidir. X Wing konfigürasyonuna alternatif olarak geliştirilmiştir.

### **Temel Farklar:**

| Özellik | X Wing | Plus Wing (+) |
|---------|--------|---------------|
| Servo Yerleşimi | Köşegen (X) | Artı (+) |
| Roll Kontrolü | Diagonal servo'lar | Yan servo'lar |
| Pitch Kontrolü | Ön/Arka servo'lar | Ön/Arka servo'lar |
| Yaw Kontrolü | X-Diagonal mixing | Tüm servo differential |
| Kararlılık | Yüksek manevra | Yüksek doğrusallık |
| Kontrol Karmaşıklığı | Orta | Düşük |

---
## 📐 **PLUS WING SERVO YERLEŞİMİ**

```
            Ön Servo (AUX 3)
                  │
                  │
Sol Servo ────────┼──────── Sağ Servo
(AUX 4)           │         (AUX 5)
                  │
                  │
            Arka Servo (AUX 6)
            
Ana Motor: AUX 1 (Merkez)
```

### **Servo Kanalları - Plus Wing Konfigürasyonu:**
```
AUX 1 → Ön Servo    (MAVLink Channel 9)  - DS3230MG 30kg
AUX 2 → Rezerve     (Gelecek geliştirmeler için)
AUX 3 → Sol Servo   (MAVLink Channel 11) - DS3230MG 30kg
AUX 4 → Sağ Servo   (MAVLink Channel 12) - DS3230MG 30kg
AUX 5 → Arka Servo  (MAVLink Channel 13) - DS3230MG 30kg
AUX 6 → Ana Motor   (MAVLink Channel 14) - DEGZ M5 + ESC
```

---

## 🎮 **PLUS WING KONTROL MATRİSİ**

### **Hareket Kontrolü:**

#### **1. Roll Kontrolü (Yan Yatış):**
- **Roll Sol:** Sol servo MIN, Sağ servo MAX
- **Roll Sağ:** Sol servo MAX, Sağ servo MIN
- **Ön/Arka servo'lar:** Neutral (değişmez)

#### **2. Pitch Kontrolü (Ön/Arka Eğim):**
- **Pitch Yukarı:** Ön servo MIN, Arka servo MAX
- **Pitch Aşağı:** Ön servo MAX, Arka servo MIN
- **Sol/Sağ servo'lar:** Neutral (değişmez)

#### **3. Yaw Kontrolü (Dönüş):**
- **Yaw Sol:** Tüm servo'lar koordineli differential
- **Yaw Sağ:** Tüm servo'lar ters differential

### **Plus Wing Mixing Formülleri:**
```python
# Plus Wing Servo PWM Hesaplama
neutral = 1500

# Servo PWM değerleri
on_servo_pwm    = neutral + (pitch * gain_pitch)
arka_servo_pwm  = neutral - (pitch * gain_pitch)
sol_servo_pwm   = neutral + (roll * gain_roll) + (yaw * gain_yaw)
sag_servo_pwm   = neutral - (roll * gain_roll) - (yaw * gain_yaw)

# PID Gain değerleri
gain_pitch = 12  # Pitch hassasiyeti
gain_roll  = 15  # Roll hassasiyeti  
gain_yaw   = 8   # Yaw hassasiyeti
```

---

## 📊 **X WING vs PLUS WING KARŞILAŞTIRMASI**

### **Avantajlar:**

#### **Plus Wing Avantajları:**
- ✅ Daha basit kontrol mantığı
- ✅ Doğrusal hareket kontrolü
- ✅ Az coupling (bağımlılık) etkisi
- ✅ Kolay kalibre edilebilir
- ✅ Başlangıç seviyesi için ideal

#### **X Wing Avantajları:**
- ✅ Yüksek manevra kabiliyeti
- ✅ Karmaşık hareket kombinasyonları
- ✅ Daha iyi yaw kontrolü
- ✅ Profesyonel ROV'larda tercih edilir

### **Dezavantajlar:**

#### **Plus Wing Dezavantajları:**
- ❌ Sınırlı yaw performansı
- ❌ Karmaşık manevralarda zorluk
- ❌ Diagonal hareket kısıtlaması

#### **X Wing Dezavantajları:**
- ❌ Karmaşık kontrol algoritması
- ❌ Yüksek coupling etkisi
- ❌ Kalibre edilmesi zor
- ❌ Başlangıç için karmaşık

---

## 🔬 **TEST VE KALIBRASYON**

### **Plus Wing Test Senaryoları:**
1. **Bireysel Servo Testi:** Her servo'nun ayrı ayrı test edilmesi
2. **Eksen Bazlı Test:** Roll, Pitch, Yaw eksenlerinin ayrı test edilmesi
3. **Kombine Hareket Testi:** Çoklu eksenlerin eş zamanlı testi
4. **PID Kontrol Testi:** Hassas konum kontrolü testi
5. **Performans Karşılaştırması:** X Wing ile karşılaştırmalı test

### **Kalibrasyon Parametreleri:**
```json
{
  "plus_wing_config": {
    "servo_limits": {
      "pwm_min": 1000,
      "pwm_neutral": 1500,
      "pwm_max": 2000
    },
    "control_gains": {
      "pitch_gain": 12,
      "roll_gain": 15,
      "yaw_gain": 8
    },
    "pid_parameters": {
      "kp": 0.8,
      "ki": 0.1,
      "kd": 0.05
    }
  }
}
```

---

## 🚀 **KULLANIM SENARYOLARI**

### **Plus Wing Tercih Edilmesi Gereken Durumlar:**
- Basit hareket gereksinimleri
- Doğrusal hareket ağırlıklı görevler
- Başlangıç seviyesi operatörler
- Kolay bakım gereksinimleri

### **X Wing Tercih Edilmesi Gereken Durumlar:**
- Karmaşık manevra gereksinimleri
- Yüksek hassasiyet gereksinimleri
- Profesyonel operasyonlar
- Gelişmiş kontrol algoritmaları

---

## 📋 **DOSYA STRUKTÜRü**

```
plus_wing/
├── README.md                    # Bu dokümantasyon
├── hardware_config.py           # Plus wing hardware konfigürasyonu
├── servo_test_plus_wing.py      # Kapsamlı servo test sistemi
├── pid_controller_plus.py       # Plus wing özel PID kontrol
├── comparison_test.py           # X wing vs Plus wing karşılaştırma
└── docs/
    ├── servo_mapping.md         # Detaylı servo mapping
    ├── control_theory.md        # Kontrol teorisi açıklaması
    └── calibration_guide.md     # Kalibrasyon kılavuzu
```

---

## ⚙️ **HIZLI BAŞLANGIÇ**

1. **Hardware Bağlantısı:** Servo'ları Plus Wing konfigürasyonuna göre bağlayın
2. **Test Çalıştırma:** `python servo_test_plus_wing.py` komutu ile test başlatın
3. **Kalibrasyon:** Her servo için min/max değerleri ayarlayın
4. **PID Tuning:** Sistem davranışını optimize edin
5. **Karşılaştırma:** X Wing ile performans karşılaştırması yapın

---

**Not:** Bu sistem TEKNOFEST 2025 Su Altı Roket Aracı projesi için geliştirilmiştir. Tüm testler gerçek hardware üzerinde doğrulanmıştır.
