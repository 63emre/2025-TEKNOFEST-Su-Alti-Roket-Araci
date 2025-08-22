# TEKNOFEST 2025 SU ALTI ROKET ARACI

## 🚀 Proje Genel Bakış

Bu proje, TEKNOFEST 2025 Su Altı Roket Yarışması için geliştirilmiş otonom sualtı aracı kontrol sistemidir. Araç, suya girdikten sonra belirlenen koordinatlara gidip roket fırlatma görevini gerçekleştirir.

## 📋 Sistem Bileşenleri

### Hardware
- **Pixhawk PX4 PIX 2.4.8 32 Bit**: Ana kontrol ünitesi
- **Raspberry Pi 5**: Yüksek seviye kontrol ve sensör işleme
- **DEGZ M5 Su Geçirmez Motor**: Ana itki motoru
- **DS3230MG Servo Motor (4x)**: Fin kontrol servolar
- **D300 Derinlik Sensörü**: Su derinliği ve sıcaklık ölçümü
- **6S LiPo Batarya (22.2V, 1800mAh)**: Güç kaynağı
- **30A ESC**: Motor kontrol ünitesi

### Sensörler
- **D300 Derinlik Sensörü** (I2C: 0x76): Basınç ve sıcaklık ölçümü
- **Pixhawk IMU**: Roll, pitch, yaw açıları
- **Pixhawk Mesafe Sensörü**: Engel tespiti ve navigasyon

## 🎯 Kanat Konfigürasyonları

### X Wing Konfigürasyonu (`x_wing/`)
- **Fin Düzeni**: Çapraz yerleşim (4 köşe)
- **Avantajlar**: Daha hassas hareket kontrolü, karmaşık manevralar
- **Kullanım**: Hassas navigasyon gerektiren görevler
- **Pin Haritası**: [X_WING_PINMAP.md](x_wing/X_WING_PINMAP.md)

### + Wing Konfigürasyonu (`+_wing/`)
- **Fin Düzeni**: Dikey/yatay yerleşim (artı şekli)
- **Avantajlar**: Basit kontrol, doğrudan hareket
- **Kullanım**: Hızlı hareket gerektiren görevler
- **Pin Haritası**: [PLUS_WING_PINMAP.md](+_wing/PLUS_WING_PINMAP.md)

## 📁 Proje Yapısı

```
2025-TEKNOFEST-Su-Alti-Roket-Araci/
├── x_wing/                     # X Wing konfigürasyonu
│   ├── hardware_pinmap.py      # X Wing pin haritası
│   ├── X_WING_PINMAP.md       # X Wing dokümantasyonu
│   ├── Görevler/              # X Wing görev scriptleri
│   │   ├── autonomous_mission_1.py
│   │   ├── autonomous_mission_2.py
│   │   ├── manual_mission_1.py
│   │   ├── manual_mission_2.py
│   │   ├── mission_1_params.json
│   │   └── mission_2_params.json
│   └── Test/                  # X Wing test scriptleri
│       ├── test_servo_control.py
│       └── test_stabilization.py
├── +_wing/                    # + Wing konfigürasyonu
│   ├── hardware_pinmap.py     # + Wing pin haritası
│   ├── PLUS_WING_PINMAP.md   # + Wing dokümantasyonu
│   ├── Görevler/             # + Wing görev scriptleri
│   │   ├── autonomous_mission_1.py
│   │   ├── autonomous_mission_2.py
│   │   ├── manual_mission_1.py
│   │   ├── manual_mission_2.py
│   │   ├── mission_1_params.json
│   │   └── mission_2_params.json
│   └── Test/                 # + Wing test scriptleri
│       ├── test_servo_control.py
│       └── test_stabilization.py
├── common/                   # Ortak modüller
│   ├── __init__.py
│   ├── button_system.py      # Buton kontrol sistemi
│   ├── d300_sensor.py        # D300 sensör sürücüsü
│   ├── gpio_helper.py        # GPIO kontrol helper'ı
│   ├── mavlink_helper.py     # MAVLink iletişim helper'ı
│   ├── pid_controller.py     # PID kontrol algoritması
│   └── servo_controller.py   # Servo kontrol helper'ı
├── Test/                     # Genel test scriptleri
│   ├── test_d300_sensor.py   # D300 sensör testi
│   ├── test_gpio_components.py # GPIO bileşen testleri
│   └── test_mavlink_connection.py # MAVLink bağlantı testi
└── README.md                 # Bu dosya
```

## 🎮 Görev Tipleri

### Manuel Görevler
- **manual_mission_1.py**: İnteraktif kontrol modu
- **manual_mission_2.py**: Manuel roket fırlatma

### Otonom Görevler
- **autonomous_mission_1.py**: Otomatik navigasyon
- **autonomous_mission_2.py**: Mesafe sensörü tabanlı navigasyon ve roket fırlatma

## ⚙️ Kurulum ve Çalıştırma

### Gereksinimler
```bash
# Python paketleri
pip install pymavlink
pip install smbus2
pip install RPi.GPIO
pip install threading
```

### Sistem Başlatma

#### X Wing Konfigürasyonu
```bash
# Manuel görev
python x_wing/Görevler/manual_mission_1.py

# Otonom görev
python x_wing/Görevler/autonomous_mission_1.py
```

#### + Wing Konfigürasyonu
```bash
# Manuel görev
python +_wing/Görevler/manual_mission_1.py

# Otonom görev
python +_wing/Görevler/autonomous_mission_1.py
```

### Test Scriptleri
```bash
# D300 sensör testi
python Test/test_d300_sensor.py

# GPIO bileşen testi
python Test/test_gpio_components.py

# MAVLink bağlantı testi
python Test/test_mavlink_connection.py
```

## 🔧 Common Modüller

### D300 Sensor (`common/d300_sensor.py`)
- **İşlev**: Derinlik ve sıcaklık ölçümü
- **I2C Adresi**: 0x76
- **Özellikler**: 
  - Sürekli veri okuma (10Hz)
  - Otomatik kalibrasyon
  - Thread-safe operasyonlar

### Servo Controller (`common/servo_controller.py`)
- **İşlev**: Fin servo kontrolü
- **Özellikler**:
  - MAVLink üzerinden PWM kontrolü
  - Yumuşak hareket algoritması
  - Çoklu servo koordinasyonu
  - Acil durdurma sistemi

### GPIO Helper (`common/gpio_helper.py`)
- **İşlev**: LED, buzzer, buton kontrolü
- **GPIO Pinleri**:
  - Buton: GPIO 18
  - LED: GPIO 22
  - Buzzer: GPIO 23

### MAVLink Helper (`common/mavlink_helper.py`)
- **İşlev**: Pixhawk ile iletişim
- **Port**: `/dev/ttyACM0` (115200 baud)
- **Özellikler**:
  - Servo PWM kontrolü
  - Sensör veri okuma
  - Sistem durumu izleme

### PID Controller (`common/pid_controller.py`)
- **İşlev**: Otomatik stabilizasyon
- **Kontrol Eksenleri**: Roll, Pitch, Yaw, Derinlik, Hız, Pozisyon
- **Özellikler**:
  - Adaptive PID parametreleri
  - Integral windup koruması
  - Çıkış sınırlama

### Button System (`common/button_system.py`)
- **İşlev**: Görev seçimi ve kontrol
- **Özellikler**:
  - Manuel/Otonom mod seçimi
  - Acil durdurma
  - Sistem durumu gösterimi

## 📊 Görev Parametreleri

### Mission 1 Parametreleri
```json
{
  "target_depth": 2.0,           // Hedef derinlik (metre)
  "straight_distance": 10.0,     // Düz gidiş mesafesi (metre)
  "min_offshore_distance": 50.0, // Minimum açık deniz mesafesi
  "cruise_speed_pwm": 1620,      // Seyir hızı PWM değeri
  "return_speed_pwm": 1650,      // Dönüş hızı PWM değeri
  "timeout_seconds": 300,        // Görev timeout süresi
  "position_tolerance": 2.0,     // Pozisyon toleransı (metre)
  "depth_tolerance": 0.2,        // Derinlik toleransı (metre)
  "distance_measurement_interval": 0.5, // Mesafe ölçüm aralığı
  "control_frequency": 10        // Kontrol döngüsü frekansı (Hz)
}
```

### Mission 2 Parametreleri
```json
{
  "safe_launch_zone_distance": 30.0,  // Güvenli fırlatma mesafesi
  "launch_depth": 1.5,                // Fırlatma derinliği (metre)
  "surface_approach_angle": 30.0,     // Yüzey yaklaşım açısı (derece)
  "required_pitch_angle": 30.0,       // Gerekli pitch açısı (derece)
  "timeout_seconds": 300,             // Görev timeout süresi
  "depth_tolerance": 0.3,             // Derinlik toleransı (metre)
  "angle_tolerance": 5.0,             // Açı toleransı (derece)
  "launch_motor_speed": 35,           // Fırlatma motor hızı
  "surface_motor_speed": 40,          // Yüzey motor hızı
  "control_frequency": 10             // Kontrol döngüsü frekansı (Hz)
}
```

## 🧪 Test Sistemi

### D300 Sensör Testi (`Test/test_d300_sensor.py`)
- I2C bağlantı kontrolü
- Tek okuma testi
- Sürekli okuma testi
- Kalibrasyon testi
- Sensör limit testleri

### GPIO Bileşen Testi (`Test/test_gpio_components.py`)
- LED kontrol testi
- Buzzer kontrol testi
- Buton input testi
- Sequence testleri
- Stress testleri

### MAVLink Bağlantı Testi (`Test/test_mavlink_connection.py`)
- Pixhawk bağlantı testi
- Servo komut testi
- Sensör veri okuma testi
- Motor kontrol testi

## 🛡️ Güvenlik Özellikleri

### Acil Durdurma Sistemi
1. **Fiziksel Buton**: GPIO 18 üzerinden acil durdurma
2. **Yazılım Watchdog**: Otomatik sistem izleme
3. **Voltaj İzleme**: Düşük batarya uyarısı (18V eşiği)
4. **Timeout Koruması**: Görev süresi aşımında otomatik durdurma

### Güvenlik Kontrolleri
- Servo PWM değerlerinin sınırlandırılması (1000-2000)
- Motor hızının güvenli aralıkta tutulması
- Derinlik limitlerinin kontrol edilmesi
- Sistem durumu sürekli izleme

## 📡 İletişim Protokolleri

### MAVLink İletişimi
- **Protokol**: MAVLink v2
- **Bağlantı**: USB Serial (ttyACM0)
- **Baud Rate**: 115200
- **Mesaj Tipleri**: 
  - SERVO_OUTPUT_RAW (servo kontrolü)
  - ATTITUDE (açı verileri)
  - DISTANCE_SENSOR (mesafe verileri)

### I2C İletişimi
- **Bus**: I2C Bus 1
- **D300 Adresi**: 0x76
- **Hız**: Standard mode (100kHz)

## 🎛️ Kontrol Algoritmaları

### PID Kontrolü
Her iki konfigürasyon için optimize edilmiş PID parametreleri:

#### X Wing PID Değerleri
- **Roll**: Kp=2.5, Ki=0.1, Kd=0.8
- **Pitch**: Kp=2.8, Ki=0.15, Kd=0.9
- **Yaw**: Kp=1.8, Ki=0.05, Kd=0.6

#### + Wing PID Değerleri
- **Roll**: Kp=3.2, Ki=0.12, Kd=1.0
- **Pitch**: Kp=3.5, Ki=0.18, Kd=1.1
- **Yaw**: Kp=2.2, Ki=0.08, Kd=0.7

### Fin Mixing Algoritması
- **X Wing**: Çapraz fin karıştırma matrisi
- **+ Wing**: Dikey/yatay fin karıştırma matrisi

## 🚀 Görev Senaryoları

### Görev 1: Navigasyon
1. **Başlangıç**: Yüzeyden suya giriş
2. **Dalış**: Hedef derinliğe (2.0m) inme
3. **Navigasyon**: Düz hat boyunca (10m) ilerleme
4. **Dönüş**: Başlangıç noktasına geri dönüş
5. **Çıkış**: Yüzeye güvenli çıkış

### Görev 2: Roket Fırlatma
1. **Başlangıç**: Yüzeyden suya giriş
2. **Güvenli Bölge**: Fırlatma bölgesine gitme (30m mesafe)
3. **Dalış**: Fırlatma derinliğine (1.5m) inme
4. **Pozisyonlama**: 30° açıyla yüzey yaklaşımı
5. **Fırlatma**: Roket ayrılma mekanizması
6. **Dönüş**: Güvenli geri çekilme

## 💻 Kullanım Kılavuzu

### Hızlı Başlangıç

1. **Sistem Kontrolü**
```bash
# Tüm bileşenleri test et
python Test/test_gpio_components.py
python Test/test_d300_sensor.py
python Test/test_mavlink_connection.py
```

2. **Manuel Kontrol**
```bash
# X Wing manuel kontrol
python x_wing/Görevler/manual_mission_1.py

# + Wing manuel kontrol
python +_wing/Görevler/manual_mission_1.py
```

3. **Otonom Görev**
```bash
# X Wing otonom görev
python x_wing/Görevler/autonomous_mission_1.py

# + Wing otonom görev
python +_wing/Görevler/autonomous_mission_1.py
```

### Konfigürasyon Ayarları

#### Görev Parametrelerini Değiştirme
```bash
# Mission 1 parametreleri
nano x_wing/Görevler/mission_1_params.json
nano +_wing/Görevler/mission_1_params.json

# Mission 2 parametreleri
nano x_wing/Görevler/mission_2_params.json
nano +_wing/Görevler/mission_2_params.json
```

#### PID Parametrelerini Ayarlama
```bash
# X Wing PID ayarları
nano x_wing/hardware_pinmap.py

# + Wing PID ayarları
nano +_wing/hardware_pinmap.py
```

## 🔍 Kod Açıklamaları

### Common Modüller

#### D300 Sensor Sınıfı
```python
# D300 sensörü başlatma
sensor = D300Sensor(bus_number=1, address=0x76)
sensor.connect()                    # I2C bağlantısı kur
sensor.calibrate_surface_level()    # Yüzey seviyesi kalibrasyonu
sensor.start_continuous_reading()   # Sürekli veri okuma başlat

# Veri okuma
depth = sensor.get_depth()          # Derinlik (metre)
pressure = sensor.get_pressure()    # Basınç (mbar)
temperature = sensor.get_temperature() # Sıcaklık (°C)
```

#### Servo Controller Sınıfı
```python
# Servo kontrolcüsü başlatma
servo_controller = ServoController(mav_controller, fin_config)

# Tek servo kontrolü
servo_controller.set_servo_position("upper_right", 1800)

# Çoklu servo kontrolü
positions = {"upper_right": 1800, "upper_left": 1200}
servo_controller.set_multiple_servos(positions)

# Hareket komutları
servo_controller.execute_movement_command(movement_commands["yukarı"])

# Acil durdurma
servo_controller.emergency_stop()
```

#### GPIO Controller Sınıfı
```python
# GPIO kontrolcüsü başlatma
gpio = GPIOController(button_pin=18, led_pin=22, buzzer_pin=23)
gpio.setup_gpio()

# LED kontrolü
gpio.led_on()                       # LED aç
gpio.led_off()                      # LED kapat
gpio.led_blink(duration=5, interval=0.5) # 5 saniye yanıp söndür

# Buzzer kontrolü
gpio.buzzer_on()                    # Buzzer aç
gpio.buzzer_off()                   # Buzzer kapat
gpio.buzzer_beep(count=3, duration=0.2) # 3 kez bip

# Buton kontrolü
gpio.set_button_callback(callback_function) # Buton basma callback'i
```

#### MAVLink Controller Sınıfı
```python
# MAVLink bağlantısı
mav = MAVLinkController("/dev/ttyACM0", 115200)
mav.connect()

# Servo kontrolü
mav.set_servo_pwm(aux_port=3, pwm_value=1800)

# Sensör veri okuma
attitude = mav.get_attitude()       # Roll, pitch, yaw açıları
distance = mav.get_distance_sensor() # Mesafe sensörü verisi

# Motor kontrolü
mav.set_motor_speed(speed_pwm=1600)
```

#### PID Controller Sınıfı
```python
# PID kontrolcüsü başlatma
pid_config = {"kp": 2.5, "ki": 0.1, "kd": 0.8}
pid = PIDController(pid_config)

# PID hesaplama
output = pid.compute(setpoint=0.0, current_value=5.2)

# PID sıfırlama
pid.reset()
```

## 🎯 Görev Algoritmaları

### Otonom Navigasyon Algoritması
1. **Sistem Başlatma**: Tüm bileşenlerin kontrolü
2. **Kalibrasyon**: Sensör kalibrasyonu ve başlangıç pozisyonu
3. **Dalış Fazı**: Hedef derinliğe kontrollü dalış
4. **Navigasyon Fazı**: Mesafe ve açı kontrolü ile hedefe gitme
5. **Dönüş Fazı**: Başlangıç noktasına güvenli dönüş
6. **Çıkış Fazı**: Yüzeye kontrollü çıkış

### Roket Fırlatma Algoritması
1. **Güvenli Bölge**: Fırlatma için uygun mesafeye gitme
2. **Pozisyonlama**: Doğru açı ve derinlikte konumlanma
3. **Fırlatma Hazırlığı**: Sistem kontrolü ve onay
4. **Roket Ayrılması**: Servo ile kapak açma mekanizması
5. **Güvenli Çekilme**: Fırlatma sonrası güvenli uzaklaşma

## 🔧 Kalibrasyon ve Ayarlama

### Servo Kalibrasyonu
```python
# Manuel servo kalibrasyonu
servo_controller.calibrate_servo_limits("upper_right")

# Servo test modu
servo_controller.test_servo_range("upper_right", test_duration=3.0)

# Tüm servoları test et
servo_controller.test_all_servos()
```

### D300 Sensör Kalibrasyonu
```python
# Yüzey seviyesi kalibrasyonu (aracı yüzeyde iken)
sensor.calibrate_surface_level()

# Manuel derinlik offset ayarı
sensor.depth_offset = 0.05  # 5cm offset
```

### PID Parametre Ayarlama
1. **Kp (Proportional)**: Tepki hızını kontrol eder
2. **Ki (Integral)**: Kalıcı hata düzeltimi
3. **Kd (Derivative)**: Aşım önleme ve stabilite

## 🚨 Hata Giderme

### Yaygın Sorunlar

#### D300 Sensörü Bağlanmıyor
```bash
# I2C tarama
i2cdetect -y 1

# Beklenen çıktı: 0x76 adresinde cihaz görünmeli
```

#### Servo Hareket Etmiyor
1. MAVLink bağlantısını kontrol edin
2. AUX port bağlantılarını kontrol edin
3. PWM değerlerinin doğru aralıkta olduğunu kontrol edin

#### Motor Çalışmıyor
1. ESC kalibrasyonunu yapın
2. Batarya voltajını kontrol edin (>18V)
3. Motor PWM değerlerini kontrol edin (1100-2000)

### Log Dosyaları
- Sistem logları: `/tmp/submarine_log.txt`
- Hata logları: `/tmp/submarine_error.log`
- Performance logları: `/tmp/submarine_performance.log`

## 📈 Performans İzleme

### Sistem Metrikleri
- **Kontrol Döngüsü**: 50Hz (20ms)
- **Sensör Okuma**: 10Hz (100ms)
- **MAVLink İletişim**: 20Hz (50ms)
- **Log Yazma**: 1Hz (1000ms)

### Performans Optimizasyonu
- Thread-based sensör okuma
- Async MAVLink iletişimi
- Optimized PID hesaplamaları
- Efficient memory usage

## 🤝 Katkıda Bulunma

### Geliştirme Rehberi
1. Yeni özellikler için önce test scripti yazın
2. Common modülleri kullanarak kod tekrarından kaçının
3. Türkçe yorum ve değişken isimleri kullanın
4. PID parametrelerini değiştirmeden önce test edin

### Kod Standartları
- Python 3.8+ uyumluluğu
- Type hints kullanımı
- Logging sistemi entegrasyonu
- Exception handling

## 📞 İletişim

TEKNOFEST 2025 Su Altı Roket Yarışması
Takım: [Takım Adı]
Proje: Sualtı Roket Fırlatma Sistemi

---

**Not**: Bu sistem gerçek sualtı operasyonları için tasarlanmıştır. Test aşamasında güvenlik önlemlerini almayı unutmayın.
