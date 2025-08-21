# TEKNOFEST 2025 - Su Altı Roket Aracı
## Plus Wing (+) vs X Wing Servo Mapping Detaylı Dokümantasyonu

Bu dokümantasyon, Plus Wing ve X Wing konfigürasyonlarının servo mapping detaylarını, channel assignment'larını ve kontrol matrislerini açıklar.

---

## 🔌 **GÜNCEL CHANNEL MAPPING**

### **X Wing Konfigürasyonu:**
```
AUX 1 → Ana Motor      (MAVLink Channel 9)  - DEGZ M5 + ESC
AUX 2 → Rezerve        (Kullanılmıyor)
AUX 3 → Ön Sağ Fin     (MAVLink Channel 11) - DS3230MG 30kg
AUX 4 → Arka Sol Fin   (MAVLink Channel 12) - DS3230MG 30kg
AUX 5 → Arka Sağ Fin   (MAVLink Channel 13) - DS3230MG 30kg
AUX 6 → Ekstra Fin     (MAVLink Channel 14) - DS3230MG 30kg
```

### **Plus Wing Konfigürasyonu:**
```
AUX 1 → Ana Motor      (MAVLink Channel 9)  - DEGZ M5 + ESC
AUX 2 → Rezerve        (Kullanılmıyor)
AUX 3 → Ön Servo       (MAVLink Channel 11) - DS3230MG 30kg
AUX 4 → Sol Servo      (MAVLink Channel 12) - DS3230MG 30kg
AUX 5 → Sağ Servo      (MAVLink Channel 13) - DS3230MG 30kg
AUX 6 → Arka Servo     (MAVLink Channel 14) - DS3230MG 30kg
```

---

## 📐 **FİZİKSEL YERLEŞİM**

### **X Wing Fiziksel Düzen:**
```
                    ROV Merkezi
                         │
            Ön Sol ──────┼────── Ön Sağ (AUX 3)
                \        │        /
                 \       │       /
                  \      │      /
                   \     │     /
                    \    │    /
                     \   │   /
                      \  │  /
                       \ │ /
                        \│/
                         X
                        /│\
                       / │ \
                      /  │  \
                     /   │   \
                    /    │    \
                   /     │     \
                  /      │      \
                 /       │       \
                /        │        \
         Arka Sol ───────┼─────── Arka Sağ
           (AUX 4)       │       (AUX 5)
                         │
                   Ana Motor (AUX 1)
```

### **Plus Wing Fiziksel Düzen:**
```
                   Ön Servo (AUX 3)
                         │
                         │
                         │
    Sol Servo ───────────┼─────────── Sağ Servo
    (AUX 4)              │              (AUX 5)
                         │
                         │
                         │
                   Arka Servo (AUX 6)
                         │
                   Ana Motor (AUX 1)
```

---

## ⚙️ **KONTROL MATRİSLERİ**

### **X Wing Kontrol Matrisi:**

#### **Roll Kontrolü:**
- **Roll Sol:** Ön Sağ(+), Arka Sol(+), Arka Sağ(-)
- **Roll Sağ:** Ön Sağ(-), Arka Sol(-), Arka Sağ(+)

#### **Pitch Kontrolü:**
- **Pitch Yukarı:** Ön Sağ(+), Arka Sol(-), Arka Sağ(-)
- **Pitch Aşağı:** Ön Sağ(-), Arka Sol(+), Arka Sağ(+)

#### **Yaw Kontrolü:**
- **Yaw Sol:** Ön Sağ(-), Arka Sol(-), Arka Sağ(+)
- **Yaw Sağ:** Ön Sağ(+), Arka Sol(+), Arka Sağ(-)

### **Plus Wing Kontrol Matrisi:**

#### **Roll Kontrolü:**
- **Roll Sol:** Sol Servo(+), Sağ Servo(-)
- **Roll Sağ:** Sol Servo(-), Sağ Servo(+)
- **Ön/Arka Servo:** Neutral (değişmez)

#### **Pitch Kontrolü:**
- **Pitch Yukarı:** Ön Servo(+), Arka Servo(-)
- **Pitch Aşağı:** Ön Servo(-), Arka Servo(+)
- **Sol/Sağ Servo:** Neutral (değişmez)

#### **Yaw Kontrolü:**
- **Yaw Sol:** Tüm servo'lar koordineli differential
- **Yaw Sağ:** Tüm servo'lar ters differential

---

## 🔢 **PWM HESAPLAMA FORMÜLLERİ**

### **X Wing PWM Hesaplama:**
```python
# X Wing Mixing Matrix
neutral = 1500

# Control gains
pitch_gain = 8
roll_gain = 10
yaw_gain = 6

# Servo PWM değerleri
front_right_pwm = neutral + (pitch * pitch_gain) - (roll * roll_gain) - (yaw * yaw_gain)
rear_left_pwm   = neutral - (pitch * pitch_gain) + (roll * roll_gain) - (yaw * yaw_gain)
rear_right_pwm  = neutral - (pitch * pitch_gain) - (roll * roll_gain) + (yaw * yaw_gain)
extra_pwm       = neutral  # Ekstra servo neutral'da
```

### **Plus Wing PWM Hesaplama:**
```python
# Plus Wing Mixing Matrix
neutral = 1500

# Control gains
pitch_gain = 12
roll_gain = 15
yaw_gain = 8

# Servo PWM değerleri
on_servo_pwm    = neutral + (pitch * pitch_gain) + (yaw * yaw_gain * 0.5)
arka_servo_pwm  = neutral - (pitch * pitch_gain) - (yaw * yaw_gain * 0.5)
sol_servo_pwm   = neutral + (roll * roll_gain) + (yaw * yaw_gain * 0.7)
sag_servo_pwm   = neutral - (roll * roll_gain) - (yaw * yaw_gain * 0.7)
```

---

## 📊 **KANAL KULLANIM KARŞILAŞTIRMASI**

| Özellik | X Wing | Plus Wing |
|---------|--------|-----------|
| Toplam Servo | 3 aktif + 1 ekstra | 4 aktif |
| Motor Kanalı | AUX 1 (Channel 9) | AUX 1 (Channel 9) |
| Kontrol Karmaşıklığı | Yüksek (X-diagonal) | Düşük (Doğrusal) |
| Coupling Etkisi | Yüksek | Düşük |
| Servo Koordinasyonu | 3 servo eşzamanlı | 2-4 servo seçici |
| Yedek Kanal | AUX 6 (Ekstra) | AUX 2 (Rezerve) |

---

## 🎯 **KONTROL PERFORMANSI**

### **X Wing Avantajları:**
- ✅ 3D manevra kabiliyeti yüksek
- ✅ Karmaşık hareket kombinasyonları
- ✅ Diagonal hareket mümkün
- ✅ Yaw kontrolü güçlü

### **Plus Wing Avantajları:**
- ✅ Basit kontrol algoritması
- ✅ Az coupling etkisi
- ✅ Doğrusal hareket kontrolü
- ✅ Kolay kalibrasyon

### **X Wing Dezavantajları:**
- ❌ Karmaşık mixing algoritması
- ❌ Yüksek coupling etkisi
- ❌ Kalibre edilmesi zor
- ❌ Servo arızasında sistem etkilenir

### **Plus Wing Dezavantajları:**
- ❌ Diagonal hareket sınırlı
- ❌ Yaw performansı düşük
- ❌ Karmaşık manevralarda zorluk
- ❌ 3D hareket kabiliyeti sınırlı

---

## 🔧 **HARDWARE SPESİFİKASYONLARI**

### **Servo Modeli:** DS3230MG 30kg
- **Torque:** 30 kg-cm
- **Hız:** 0.16 sec/60°
- **Voltaj:** 6.0-7.4V
- **PWM Frekansı:** 333Hz
- **PWM Aralığı:** 1000-2000µs

### **Motor Modeli:** DEGZ M5 + DEGZ BLU 30A ESC
- **Güç:** 500W
- **Voltaj:** 22.2V (6S LiPo)
- **Akım:** 30A sürekli
- **PWM Kontrol:** 1000-2000µs

### **PWM Sinyal Özellikleri:**
- **Frekans:** 333Hz (3ms period)
- **Min PWM:** 1000µs (Full Left/Down)
- **Neutral PWM:** 1500µs (Center)
- **Max PWM:** 2000µs (Full Right/Up)

---

## 📋 **KULLANIM ÖNERİLERİ**

### **X Wing Tercih Edilmesi Gereken Durumlar:**
- Karmaşık 3D manevra gereksinimleri
- Yüksek yaw performansı gerekli
- Profesyonel operasyonlar
- İleri seviye operatörler
- Diagonal hareket gereksinimleri

### **Plus Wing Tercih Edilmesi Gereken Durumlar:**
- Basit doğrusal hareket görevleri
- Başlangıç seviyesi operatörler
- Kolay bakım gereksinimleri
- Düşük coupling etkisi istenen durumlar
- Eğitim amaçlı kullanım

---

## 🔍 **TROUBLESHOOTING**

### **Ortak Sorunlar:**

#### **Servo Yanıt Vermiyor:**
1. Channel mapping'i kontrol edin
2. PWM değerleri 1000-2000µs aralığında mı?
3. MAVLink bağlantısı aktif mi?
4. Sistem ARMED durumda mı?

#### **Yanlış Hareket Yönü:**
1. Servo polaritesini kontrol edin
2. Mixing matrix formüllerini gözden geçirin
3. Hardware kablolama doğru mu?

#### **Kararsız Kontrol:**
1. PID parametrelerini ayarlayın
2. Coupling kompensasyonu ekleyin
3. Control gain değerlerini düşürün

### **Konfigürasyon Özel Sorunlar:**

#### **X Wing:**
- Servo'lar arası interferans
- Yüksek coupling etkisi
- Karmaşık kalibrasyon

#### **Plus Wing:**
- Sınırlı yaw performansı
- Diagonal hareket zorluğu
- 4 servo koordinasyonu

---

**Not:** Bu dokümantasyon TEKNOFEST 2025 Su Altı Roket Aracı projesi için hazırlanmıştır. Güncel channel mapping ve hardware konfigürasyonu bilgilerini içerir.
