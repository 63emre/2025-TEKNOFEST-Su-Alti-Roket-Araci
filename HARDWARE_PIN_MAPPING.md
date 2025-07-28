# TEKNOFEST 2025 - Su Altı Roket Aracı
## Hardware Pin Mapping Standardı v1.0

Bu dokuman, tüm sistemde kullanılan pin bağlantılarının standardını tanımlar. **TÜM KODLAR BU STANDARDA GÖRE YAZILMIŞTIR.**

---

## 🔌 **PIXHAWK PX4 PIX 2.4.8 Pin Mapping**

  ### **MAIN OUTPUT (PWM Çıkışları)**
  ```
  MAIN 1  → Rezerve (Gelecek geliştirmeler için)
  MAIN 2  → Rezerve (Gelecek geliştirmeler için)
  MAIN 3  → Rezerve 
  MAIN 4  → Rezerve
  MAIN 5  → Rezerve
  MAIN 6  → Rezerve
  MAIN 7  → Rezerve
  MAIN 8  → Rezerve
  ```
  
  ### **AUX OUTPUT (Auxiliary PWM Çıkışları) - Pi5 Test Konfigürasyonu**
  ```
  AUX 1   → Fin Servo 1 - Ön Sol (X Düzeninde) (DS3230MG 30kg)
  AUX 2   → Rezerve
  AUX 3   → Fin Servo 2 - Ön Sağ (X Düzeninde) (DS3230MG 30kg)  
  AUX 4   → Fin Servo 3 - Arka Sol (X Düzeninde) (DS3230MG 30kg)
  AUX 5   → Fin Servo 4 - Arka Sağ (X Düzeninde) (DS3230MG 30kg)
  AUX 6   → Ana Motor (DEGZ M5 + DEGZ BLU 30A ESC)
  ```
  
  ### **I2C Port**
  ```
  I2C SCL → D300 Derinlik Sensörü SCL
  I2C SDA → D300 Derinlik Sensörü SDA  
  I2C VCC → +5V (D300 için)
  I2C GND → Ground
  ```

## **Power Module**
```
Power Module → 22.2V DC to 5V DC
              →  6S 22.2V LiPo Batarya (1800mAh 65C)
              → Akım sensörü entegreli
              → Voltaj monitörleme
```

---

## 📟 **RASPBERRY PI 4B GPIO Pin Mapping**

  ### **GPIO Pinleri (BCM Numaralama)**
  ```
  GPIO 2 → Pi Fan (+) (Rezerve)
  GPIO 3 → Pi Fan (-) (Rezerve)
  
  GPIO 4  → Status LED Red (Kırmızı Durum LED)
  GPIO 5  → Status LED Green (Yeşil Durum LED)
  GPIO 6  → Status LED Blue (Mavi Durum LED)
  
  GPIO 7 → Buzzer PWM Output (Sesli Uyarı/Müzik)
  GPIO 8 → External Warning LED (Harici Uyarı LED)
  GPIO 9 → Power Button Input System ON/OFF (16A P1Z EC Metal Buton)
  
  GPIO 10 → System Status LED (Ana Sistem Durumu)
  
  GPIO 11 → Spare GPIO (Rezerve)
  
  
  ```

## **Güç Sistemi**
```
Regülatörler
5V → Pixhawk güç beslemesi - Power Input(2x 22.2V to 5V Power Module GM V1.0 (a-b))
5V → Raspery Pi Beslemesi - USB-C (2x 22.2V to 5V 3A Regülatör (e-f))

6.8V → Servo Güç Beslemesi (2x 22.2V to 6.8V 8A Regülatör (c-d))

12V → Selonoid Beslemesi (1x 22.2V to 12V 8A Regülatör (g))
12V -Ateşleme Sistemi (1x 22.2V to 12 V 8A Regülatör (g))


Kart Üstünden Besleme
3.3V → GPIO pull-up dirençleri ?
22.2v → Ana Motor Besleme (ESC üzerinden)
GND  → Ortak topraklama
USB  → Pixhawk MAVLink bağlantısı

Li-Polimer Pil Besleme
100A Röle Besleme (2x9V Li-PO)
```

---

## ⚡ **GÜÇ SİSTEMİ ŞEMASI**

### **Ana Güç Dağılımı**
```
┌─────────────────┐
│ 6S LiPo Batarya │ (22.2V, 1800mAh, 65C = ~117A peak)
│    80A sürekli  │
└─────────┬───────┘
          │
    ┌─────▼─────┐
    │ 100A Relay│ ◄── Acil Stop Buton 
    │(Acil Kesme│  
    │ Kontrolü) │    
    └─────┬─────┘
          │
    ┌─────▼─────┐
    │Power Dist.│
    │   Module  │
    └─┬─┬─┬─┬─┬─┘
      │ │ │ │ │
      │ │ │ │ └── +5V 3A USB-C → Raspberry Pi
      │ │ │ └──── +5V Pixhawk POWER PIN  → Pixhawk  
      │ │ └────── +6.8V x 4 → Servo
      │ └──────── +12V X 2 → Selenoid + Fırlatma Sistemi
      └────────── MAIN 1 → ESC (30A)
```

### **Güvenlik Sistemi**
- **100A Relay**: Ana sistem gücü kesintisi için
- **16A Metal Buton**: Sistem on/off kontrolü 
- **Acil Stop**: Anında tüm motor durdurmak için
- **80A Peak Current**: Bataryadan gelen maksimum akım

---

## 🔧 **SERVO KANAL DETAYLARI**

### **Fin Control Matrix - X Konfigürasyonu (Pi5 Test: AUX 1,3,4,5)**
```
   Ön Sol (AUX 1) ────────────── Ön Sağ (AUX 3)
       │   \                 /   │
       │    \               /    │
       │     \             /     │
       │      \           /      │
       │       \         /       │
       │        \       /        │
       │         \     /         │
       │          \ X /          │
       │          / X \          │
       │         /     \         │
       │        /       \        │
       │       /         \       │
       │      /           \      │
       │     /             \     │
       │    /               \    │
       │   /                 \   │
  Arka Sol (AUX 4) ────────────── Arka Sağ (AUX 5)

🚀 Pi5 Test X-Konfigürasyon Kontrol Matrisi:
Roll Control  → AUX 1 & AUX 4 vs AUX 3 & AUX 5 (Sol/Sağ Differential)
Pitch Control → AUX 1 & AUX 3 vs AUX 4 & AUX 5 (Ön/Arka Differential)  
Yaw Control   → AUX 1 & AUX 5 vs AUX 3 & AUX 4 (X-Diagonal)
Motor Control → AUX 6 (Ana İtki)
```

### **PWM Signal Specs**
```
PWM Frequency: 333Hz (3ms period)
PWM Range:     1000-2000 μs
Neutral:       1500 μs
Min:           1000 μs (Full Left/Down)
Max:           2000 μs (Full Right/Up)
```

---

## 📡 **SENSOR INTERFACES**

### **D300 Derinlik Sensörü (I2C) - Pi5 Test Konfigürasyonu**
```
I2C Address: 0x76 (Hardware Test)
Voltage:     3.3V - 5V
Interface:   I2C (100kHz - 400kHz)
Data Rate:   10Hz maksimum
Resolution:  Depth: 0.01m, Temp: 0.01°C
Range:       0-300m depth, -20°C to +85°C

Raspberry Pi 5 I2C Connection:
GPIO 2 (SDA) ── SDA Pin
GPIO 3 (SCL) ── SCL Pin  
5V           ── VCC Pin
GND          ── GND Pin
```

### **Pixhawk Internal Sensors**
```
IMU:         MPU6000 (Gyro + Accelerometer)
Magnetometer: HMC5883L
Barometer:   MS5611 (Backup depth reference)
GPS:         External GPS module (Serial)
```

---

## 🚨 **ACİL GÜVENLİK SİSTEMİ**

### **40A Relay Kontrol Sistemi**
```
Relay Kapasitesi: 100A @ 24VDC
Kontrol Voltajı:  9V (Li-PO pil)
Ana Devreleme:    6S LiPo → Tüm sistem
Kesme Süresi:     <50ms
Fail-Safe:        Power loss = Relay açık
```

### **Buton Hiyerarşisi**
```
1. Acil Stop Buton (Enerjilendirme) (GPIO 19):
   - Anında motor durdurma
   - 100A relay kontrolü
   - Power Line Level Kapama
   - Emergency surface protocol
2. 16A Metal Buton (GPIO 18):
   - Sistem açma/kapama
   - 90 saniye güvenlik gecikmesi
   - Soft shutdown


```

---

## 📋 **SOFTWARE CHANNEL MAPPING**

### **Python Code Standards**
```python
# Pi5 Hardware Test Konfigürasyonu

# Motor Channels
MOTOR_CHANNEL = 6           # AUX 6

# Servo Channels - X Konfigürasyonu (Pi5 Test)
SERVO_FIN_FRONT_LEFT = 1    # AUX 1 - Ön Sol
SERVO_FIN_FRONT_RIGHT = 3   # AUX 3 - Ön Sağ  
SERVO_FIN_REAR_LEFT = 4     # AUX 4 - Arka Sol
SERVO_FIN_REAR_RIGHT = 5    # AUX 5 - Arka Sağ
# Rezerve Kanallar:
# AUX 2 - Rezerve
# AUX 7-8 - Gelecek geliştirmeler

# GPIO Pins - Kontrol
GPIO_POWER_BUTTON = 18      # Güç Butonu
GPIO_EMERGENCY_STOP = 19    # Acil Kesme 
GPIO_POWER_RELAY = 21       # Ana Güç Rölesi
GPIO_PAYLOAD_SENSOR = 22    # Payload Durumu
GPIO_WATER_DETECT = 23      # Su Algılama

# GPIO Pins - LED ve Buzzer
GPIO_LED_RED = 4            # Kırmızı LED
GPIO_LED_GREEN = 5          # Yeşil LED  
GPIO_LED_BLUE = 6           # Mavi LED
GPIO_BUZZER_PWM = 13        # PWM Buzzer
GPIO_WARNING_LED = 16       # Uyarı LED
GPIO_SYSTEM_LED = 20        # Sistem LED
GPIO_MISSION_LED = 24       # Görev LED
GPIO_EXT_BUZZER = 25        # Dış Buzzer
GPIO_RGB_STRIP = 26         # RGB LED Strip

# I2C Sensors
I2C_D300_ADDRESS = 0x76     # D300 Derinlik Sensörü (Pi5 Test)

# X-Fin Kontrol Matrisi (Pi5 Test - AUX 1,3,4,5)
FIN_MATRIX = {
    'roll_positive': [SERVO_FIN_FRONT_LEFT, SERVO_FIN_REAR_LEFT],    # Sol finler (AUX1, AUX4)
    'roll_negative': [SERVO_FIN_FRONT_RIGHT, SERVO_FIN_REAR_RIGHT],  # Sağ finler (AUX3, AUX5)
    'pitch_positive': [SERVO_FIN_FRONT_LEFT, SERVO_FIN_FRONT_RIGHT], # Ön finler (AUX1, AUX3)
    'pitch_negative': [SERVO_FIN_REAR_LEFT, SERVO_FIN_REAR_RIGHT],   # Arka finler (AUX4, AUX5)
    'yaw_positive': [SERVO_FIN_FRONT_LEFT, SERVO_FIN_REAR_RIGHT],    # X-Diagonal 1 (AUX1, AUX5)
    'yaw_negative': [SERVO_FIN_FRONT_RIGHT, SERVO_FIN_REAR_LEFT]     # X-Diagonal 2 (AUX3, AUX4)
}
```

### **MAVLink Channel Usage**
```python
# MAVLink servo command format:
# mavutil.mavlink.MAV_CMD_DO_SET_SERVO
# Parametreler: (channel, pwm_value, 0, 0, 0, 0, 0)

# Motor ESC komutları:
# Channel 1 = MAIN 1 output
# PWM 1000-2000 range (1500 = neutral/stop)
```

---

## 🔍 **BAĞLANTI TEST PROTOKOLÜ**

### **1. Güç Sistemi Testi**
```bash
# Voltaj kontrolleri
echo "Testing main power..."
# 22.2V nominal, 19.8V minimum cutoff
# Current draw: <5A idle, <30A operational
```

### **2. GPIO Test Sequence** 
```bash
# Raspberry Pi GPIO test
sudo python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(18, GPIO.IN); print(GPIO.input(18))"
```

### **3. I2C Device Detection (Pi5 Test)**
```bash
# D300 sensör tespiti (Pi5)
i2cdetect -y 1
# 0x76 adresinde D300 görünmeli (Hardware Test Konfigürasyonu)
```

### **4. Pixhawk Connection Test**
```bash
# MAVLink bağlantı testi
python3 -c "from pymavlink import mavutil; m=mavutil.mavlink_connection('tcp:127.0.0.1:5777'); m.wait_heartbeat(); print('OK')"
```

---

## ⚠️ **KRITIK UYARILAR**

### **Güç Sistemi**
- ⚡ **80A pil akımına karşı 40A relay kullanımı**: Relay motor akımını değil, sistem gücünü keser
- ⚡ **ESC 30A**, motor max 25A çeker, güvenli margin var
- ⚡ **16A buton**, sadece 3.3V GPIO sinyali taşır, ana akım relay üzerinden
- ⚡ **Termal koruma**: ESC ve motor sıcaklık monitörleme gerekli

### **Su Geçirgenlik**  
- 💧 Tüm bağlantılar IP67+ standart
- 💧 D300 sensör inherently waterproof
- 💧 GPIO bağlantıları waterproof connector ile
- 💧 Test depth: Minimum 3m, operasyonel 2m

### **Software Fail-Safes**
- 🛡️ MAVLink heartbeat timeout: 30 saniye
- 🛡️ GPIO button debounce: 50ms minimum
- 🛡️ Emergency surface depth trigger: >5m
- 🛡️ Low battery cutoff: <19.8V (3.3V/cell)

---

## 📖 **REFERANS LINKLER**

- **DEGZ BLU 30A ESC**: https://www.mucif.com/urunler/degz-blu-30a-esc-fircasiz-motor-surucu
- **DS3230MG Servo**: https://www.motorobit.com/ds3230mg-30kg-su-gecirmez-dijital-servo-motor  
- **DEGZ M5 Motor**: https://www.mucif.com/urunler/degz-m5-su-gecirmez-su-alti-motoru
- **D300 Depth Sensor**: https://www.mucif.com/urunler/d300-derinlik-ve-su-sicakligi-sensoru
- **Metal Buton**: https://www.motorobit.com/16a-p1z-ec-16mm-duz-anahtarli-isikli-power-metal-buton-yesil
- **LiPo Batarya**: https://www.motorobit.com/222v-6s-1800mah-65c-lipo-batarya

---

**⚠️ Bu pin mapping standardı tüm sistem kodlarında kullanılmaktadır. Değişiklik yapmadan önce tüm referansları kontrol edin!** 
