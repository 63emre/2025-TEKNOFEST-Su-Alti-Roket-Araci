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
  
  ### **AUX OUTPUT (Auxiliary PWM Çıkışları) - TEKNOFEST Standart Konfigürasyonu**
  ```
  AUX 1   → Ana Motor (DEGZ M5 + DEGZ BLU 30A ESC)
  AUX 2   → BOZUK - Kullanılmaz
  AUX 3   → Fin Servo 1 (DS3230MG 30kg) - X: Ön Sağ / Plus: Sağ Kanat
  AUX 4   → Fin Servo 2 (DS3230MG 30kg) - X: Arka Sol / Plus: Alt Kanat  
  AUX 5   → Fin Servo 3 (DS3230MG 30kg) - X: Arka Sağ / Plus: Sol Kanat
  AUX 6   → Fin Servo 4 (DS3230MG 30kg) - X: Ön Sol / Plus: Üst Kanat
  ```
  
  ### **I2C Port**
  ```
  Pixhawk I2C Portuna bağlı Adres : 0x76
  I2C SCL → D300 Derinlik Sensörü SCL
  I2C SDA → D300 Derinlik Sensörü SDA  
  I2C VCC → +5V (D300 için)
  I2C GND → Ground
  ```

  ### **Serial MAVLink Connection**
  ```
  USB Port → Raspberry Pi 5 / Development Computer
  Baud Rate: 115200 (Configurable via MAV_BAUD environment variable)
  Protocol: MAVLink v2.0
  
  Environment Variables:
  - MAV_ADDRESS="/dev/ttyACM0" (default USB serial port)
  - MAV_BAUD="115200" (default baud rate)
  
  Alternative Ports:
  - /dev/ttyUSB0 (USB-UART adapter)
  - /dev/ttyUSB1 (Secondary USB-UART)
  - /dev/ttyAMA0 (Raspberry Pi UART pins)
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

  ### **GPIO Pinleri (BCM Numaralama) - TEKNOFEST Standart**
  ```
  GPIO 2 → I2C SDA (D300 Derinlik Sensörü)
  GPIO 3 → I2C SCL (D300 Derinlik Sensörü)
  
  GPIO 4  → Status LED (Ana Durum LED - Kırmızı/Yeşil/Mavi)
  GPIO 5  → Rezerve
  GPIO 6  → Rezerve
  
  GPIO 13 → Buzzer PWM Output (Sesli Uyarı/Müzik/Beep)
  GPIO 18 → Power Button Input (16A P1Z EC Metal Buton)
  GPIO 19 → Emergency Stop Input (Acil Durdurma Butonu)
  
  GPIO 20 → Rezerve
  GPIO 21 → Rezerve
  
  # Kullanılan GPIO'lar:
  # - GPIO 2,3: I2C (D300 sensör)
  # - GPIO 4: LED (durum gösterimi)
  # - GPIO 13: Buzzer (sesli uyarı)
  # - GPIO 18,19: Butonlar (güvenlik)
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
USB  → Pixhawk Serial MAVLink bağlantısı
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

### **Kanat Konfigürasyonları - TEKNOFEST Standart**

#### **X-Konfigürasyon (AUX 3,4,5,6)**
```
   Ön Sol (AUX 6) ────────────── Ön Sağ (AUX 3)
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

🚀 X-Konfigürasyon Kontrol Matrisi:
Roll Control  → AUX 6 & AUX 4 vs AUX 3 & AUX 5 (Sol/Sağ Differential)
Pitch Control → AUX 6 & AUX 3 vs AUX 4 & AUX 5 (Ön/Arka Differential)  
Yaw Control   → AUX 6 & AUX 5 vs AUX 3 & AUX 4 (X-Diagonal)
Motor Control → AUX 1 (Ana İtki)
```

#### **Plus-Konfigürasyon (AUX 3,4,5,6)**
```
        Üst (AUX 6)
           │
           │
Sol (AUX 5) ┼ Sağ (AUX 3)
           │
           │
        Alt (AUX 4)

🚀 Plus-Konfigürasyon Kontrol Matrisi:
Roll Control  → AUX 5 vs AUX 3 (Sol/Sağ Differential)
Pitch Control → AUX 6 vs AUX 4 (Üst/Alt Differential)
Yaw Control   → Tüm kanatlar koordineli (CCW/CW rotation)
Motor Control → AUX 1 (Ana İtki)
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
# TEKNOFEST 2025 Standart Konfigürasyonu

# Motor Channel
MOTOR_CHANNEL = 1           # AUX 1 - Ana Motor

# Servo Channels - Standart Mapping
SERVO_CHANNELS = {
    'fin_1': 3,  # AUX 3 - X: Ön Sağ / Plus: Sağ Kanat
    'fin_2': 4,  # AUX 4 - X: Arka Sol / Plus: Alt Kanat
    'fin_3': 5,  # AUX 5 - X: Arka Sağ / Plus: Sol Kanat
    'fin_4': 6   # AUX 6 - X: Ön Sol / Plus: Üst Kanat
}
# Not: AUX 2 BOZUK - Kullanılmaz

# GPIO Pins - Standart
GPIO_POWER_BUTTON = 18      # Güç Butonu
GPIO_EMERGENCY_STOP = 19    # Acil Durdurma
GPIO_STATUS_LED = 4         # Durum LED
GPIO_BUZZER_PWM = 13        # Buzzer PWM

# I2C Sensors
I2C_D300_ADDRESS = 0x76     # D300 Derinlik Sensörü

# X-Konfigürasyon Kontrol Matrisi (AUX 3,4,5,6)
X_WING_MATRIX = {
    'roll_positive': [6, 4],    # Sol finler (AUX6, AUX4)
    'roll_negative': [3, 5],    # Sağ finler (AUX3, AUX5)
    'pitch_positive': [6, 3],   # Ön finler (AUX6, AUX3)
    'pitch_negative': [4, 5],   # Arka finler (AUX4, AUX5)
    'yaw_positive': [6, 5],     # X-Diagonal 1 (AUX6, AUX5)
    'yaw_negative': [3, 4]      # X-Diagonal 2 (AUX3, AUX4)
}

# Plus-Konfigürasyon Kontrol Matrisi (AUX 3,4,5,6)
PLUS_WING_MATRIX = {
    'roll_positive': [5],       # Sol kanat (AUX5)
    'roll_negative': [3],       # Sağ kanat (AUX3)
    'pitch_positive': [6],      # Üst kanat (AUX6)
    'pitch_negative': [4],      # Alt kanat (AUX4)
    'yaw_ccw': [3, 4, 5, 6],   # Tüm kanatlar CCW
    'yaw_cw': [3, 4, 5, 6]     # Tüm kanatlar CW
}
```

### **MAVLink Channel Usage - Serial Connection**
```python
# MAVLink servo command format:
# mavutil.mavlink.MAV_CMD_DO_SET_SERVO
# Parametreler: (channel, pwm_value, 0, 0, 0, 0, 0)

# Serial Connection Configuration
import os
from pymavlink import mavutil

# Environment variable support
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

# MAVLink connection establishment
master = mavutil.mavlink_connection(MAV_ADDRESS, baud=MAV_BAUD, autoreconnect=True)

# Motor ESC komutları:
# Channel 1 = AUX 1 output (TEKNOFEST Standart)
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

### **4. Pixhawk Serial Connection Test**
```bash
# Serial MAVLink bağlantı testi
export MAV_ADDRESS="/dev/ttyACM0"
export MAV_BAUD="115200"

python3 -c "
import os
from pymavlink import mavutil
port = os.getenv('MAV_ADDRESS', '/dev/ttyACM0')
baud = int(os.getenv('MAV_BAUD', '115200'))
print(f'Testing connection: {port}@{baud}')
master = mavutil.mavlink_connection(port, baud=baud)
master.wait_heartbeat()
print('Serial MAVLink connection successful!')
"
```

### **5. Alternative Connection Methods**
```bash
# USB Serial Adapter Test
export MAV_ADDRESS="/dev/ttyUSB0"
export MAV_BAUD="115200"

# Raspberry Pi UART Test  
export MAV_ADDRESS="/dev/ttyAMA0"
export MAV_BAUD="115200"

# TCP Fallback (if MAVLink proxy is running)
export MAV_ADDRESS="tcp:127.0.0.1:5777"
```

---

## ⚠️ **KRITIK UYARILAR**

### **Güç Sistemi**
- ⚡ **80A pil akımına karşı 40A relay kullanımı**: Relay motor akımını değil, sistem gücünü keser
- ⚡ **ESC 30A**, motor max 25A çeker, güvenli margin var
- ⚡ **16A buton**, sadece 3.3V GPIO sinyali taşır, ana akım relay üzerinden
- ⚡ **Termal koruma**: ESC ve motor sıcaklık monitörleme gerekli

### **Serial Communication**
- 📡 **Serial port permissions**: `/dev/ttyACM0` erişimi için user'ı `dialout` grubuna ekle
- 📡 **Baud rate**: 115200 baud standart, 57600/921600 alternatif seçenekler
- 📡 **USB cable quality**: Veri hatları için kaliteli USB kablo kullan
- 📡 **Auto-reconnection**: MAVLink bağlantısı koptuğunda otomatik yeniden bağlanma

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
