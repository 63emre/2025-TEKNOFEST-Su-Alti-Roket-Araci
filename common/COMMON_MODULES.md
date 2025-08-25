# COMMON MODÜLLER DOKÜMANTASYONU

## 📋 Genel Bakış

Common modüller, hem X Wing hem de + Wing konfigürasyonları tarafından paylaşılan ortak işlevselliği sağlar. Bu modüller, kod tekrarını önler ve sistem tutarlılığını sağlar.

## 📁 Modül Listesi

| Modül | Dosya | İşlev | Açıklama |
|-------|-------|-------|----------|
| D300 Sensor | `d300_sensor.py` | Derinlik/Sıcaklık | I2C üzerinden D300 sensör kontrolü |
| Servo Controller | `servo_controller.py` | Servo Kontrolü | MAVLink ile fin servo kontrolü |
| GPIO Helper | `gpio_helper.py` | GPIO Kontrolü | LED, buzzer, buton kontrolü |
| MAVLink Helper | `mavlink_helper.py` | Pixhawk İletişim | MAVLink protokol yönetimi |
| PID Controller | `pid_controller.py` | Stabilizasyon | PID kontrol algoritmaları |
| Button System | `button_system.py` | Görev Yönetimi | Buton ile görev başlatma |

---

## 🌊 D300 Sensor Modülü

### Genel Özellikler
- **I2C Adresi**: 0x76 [[memory:4381766]]
- **I2C Bus**: 1 (Raspberry Pi)
- **Okuma Frekansı**: 10Hz (100ms aralık)
- **Thread-Safe**: Evet

### Ana Sınıf: `D300Sensor`

#### Başlatma
```python
from common.d300_sensor import D300Sensor

# Sensörü başlat
sensor = D300Sensor(bus_number=1, address=0x76)
sensor.connect()                    # I2C bağlantısı kur
sensor.calibrate_surface_level()    # Yüzey seviyesi kalibrasyonu
sensor.start_continuous_reading()   # Sürekli veri okuma başlat
```

#### Veri Okuma
```python
# Tek seferlik veri okuma
data = sensor.read_sensor_data()
print(f"Derinlik: {data['depth_m']:.2f}m")
print(f"Basınç: {data['pressure_mbar']:.2f}mbar")
print(f"Sıcaklık: {data['temp_celsius']:.1f}°C")

# Sürekli veri okuma
depth = sensor.get_depth()          # Güncel derinlik (metre)
pressure = sensor.get_pressure()    # Güncel basınç (mbar)
temperature = sensor.get_temperature() # Güncel sıcaklık (°C)

# Tüm veriler
all_data = sensor.get_all_data()
```

#### Kalibrasyon
```python
# Yüzey seviyesi kalibrasyonu (aracı yüzeyde iken çalıştırın)
sensor.calibrate_surface_level()

# Manuel offset ayarı
sensor.depth_offset = 0.05  # 5cm offset
sensor.pressure_sea_level = 1013.25  # Deniz seviyesi basıncı
```

#### Bağlantı Kontrolü
```python
# Bağlantı durumu
if sensor.is_connected():
    print("Sensör bağlı ve çalışıyor")

# Durum özeti
status = sensor.get_status_summary()
print(f"Bağlantı: {status['connected']}")
print(f"Son okuma: {status['last_reading']} saniye önce")
```

---

## 🎛️ Servo Controller Modülü

### Genel Özellikler
- **PWM Aralığı**: 1000-2000 μs
- **Nötr Pozisyon**: 1500 μs
- **Kontrol Protokolü**: MAVLink üzerinden Pixhawk AUX çıkışları
- **Servo Sayısı**: 4 (fin kontrolü)

### Ana Sınıf: `ServoController`

#### Başlatma
```python
from common.servo_controller import ServoController
from common.mavlink_helper import MAVLinkController

# MAVLink bağlantısı
mav = MAVLinkController("/dev/ttyACM0", 115200)
mav.connect()

# Servo kontrolcüsü (wing config'e göre)
from x_wing.hardware_pinmap import FinControlConfig  # X Wing için
# veya
from plus_wing.hardware_pinmap import FinControlConfig  # + Wing için

servo_controller = ServoController(mav, FinControlConfig.FINS)
```

#### Tek Servo Kontrolü
```python
# Belirli servo kontrolü
servo_controller.set_servo_position("upper_right", 1800)  # X Wing
servo_controller.set_servo_position("upper", 1800)        # + Wing

# Yumuşak hareket (3 saniyede hedefe git)
servo_controller.move_servo_smooth("upper_right", 1800, duration=3.0)
```

#### Çoklu Servo Kontrolü
```python
# Birden fazla servo aynı anda
positions = {
    "upper_right": 1800,
    "upper_left": 1200,
    "lower_left": 1600,
    "lower_right": 1400
}
servo_controller.set_multiple_servos(positions)

# Hareket komutları (pinmap'ten)
from x_wing.hardware_pinmap import FinControlConfig
servo_controller.execute_movement_command(FinControlConfig.MOVEMENT_COMMANDS["yukarı"])
```

#### Servo Test ve Kalibrasyon
```python
# Tek servo test (min-max-neutral döngüsü)
servo_controller.test_servo_range("upper_right", test_duration=3.0)

# Tüm servoları test et
servo_controller.test_all_servos()

# Manuel kalibrasyon modu
servo_controller.calibrate_servo_limits("upper_right")
```

#### Güvenlik Fonksiyonları
```python
# Tüm servoları nötr konuma getir
servo_controller.all_servos_neutral()

# Acil durdurma
servo_controller.emergency_stop()

# Servo durumları
status = servo_controller.get_servo_status()
positions = servo_controller.get_current_positions()
```

---

## 🔌 GPIO Helper Modülü

### Genel Özellikler
- **Buton**: GPIO 18 (Pull-up dirençli)
- **LED**: GPIO 22 (Çıkış)
- **Buzzer**: GPIO 23 (Çıkış)
- **Debounce**: 500ms buton debounce

### Ana Sınıf: `GPIOController`

#### Başlatma
```python
from common.gpio_helper import GPIOController

# GPIO kontrolcüsü
gpio = GPIOController(button_pin=18, led_pin=22, buzzer_pin=23)
gpio.setup_gpio()
```

#### LED Kontrolü
```python
# Temel LED kontrolü
gpio.led_on()           # LED aç
gpio.led_off()          # LED kapat
gpio.led_toggle()       # LED durumunu değiştir

# LED yanıp sönme
gpio.led_blink(duration=5, interval=0.5)  # 5 saniye, 0.5s aralık
gpio.led_blink(0.2, 0.3, 5)              # 5 kez blink
gpio.stop_led_blink()                     # Blinking'i durdur

# Özel sequence'lar
gpio.startup_sequence()   # Başlangıç sequence
gpio.success_sequence()   # Başarı sequence
gpio.error_sequence()     # Hata sequence
```

#### Buzzer Kontrolü
```python
# Temel buzzer kontrolü
gpio.buzzer_on()        # Buzzer aç
gpio.buzzer_off()       # Buzzer kapat

# Buzzer bip'leri
gpio.buzzer_beep(duration=0.5)           # Tek bip
gpio.buzzer_beep(0.2, 0.1, 3)           # 3 kez bip
gpio.buzzer_beep_pattern([(0.1, 0.1), (0.3, 0.2), (0.1, 0)])  # Özel pattern
```

#### Buton Kontrolü
```python
# Buton callback ayarla
def button_pressed():
    print("Buton basıldı!")

gpio.set_button_callback(button_pressed)

# Buton durumu kontrol
if gpio.is_button_pressed():
    print("Buton şu anda basılı")

# Uzun basma testi
if gpio.wait_for_long_press(threshold=3.0):
    print("Uzun basma algılandı")
```

#### Sistem Durumu
```python
# GPIO durumu
status = gpio.get_status()
print(f"GPIO Hazır: {status['setup_complete']}")
print(f"LED Durumu: {status['led_state']}")
print(f"Buzzer Aktif: {status['buzzer_active']}")

# Temizlik
gpio.cleanup_gpio()
```

---

## 📡 MAVLink Helper Modülü

### Genel Özellikler
- **Port**: `/dev/ttyACM0` (USB Serial)
- **Baud Rate**: 115200
- **Protokol**: MAVLink v2
- **Veri Okuma**: 100Hz (10ms aralık)

### Ana Sınıf: `MAVLinkController`

#### Başlatma ve Bağlantı
```python
from common.mavlink_helper import MAVLinkController

# MAVLink kontrolcüsü
mav = MAVLinkController("/dev/ttyACM0", 115200)

# Bağlantı kur
if mav.connect():
    print("Pixhawk'a başarıyla bağlanıldı!")
else:
    print("Bağlantı hatası!")
```

#### Servo Kontrolü
```python
# Tek servo kontrolü (AUX port numarası ile)
mav.set_servo_pwm(servo_num=3, pwm_value=1800)  # AUX 3'e 1800μs PWM

# Motor kontrolü (yüzde ile)
mav.set_motor_speed(speed_percent=50)  # %50 hız
```

#### Sensör Veri Okuma
```python
# Açı verileri (IMU'dan)
attitude = mav.get_attitude()
print(f"Roll: {attitude['roll']:.1f}°")
print(f"Pitch: {attitude['pitch']:.1f}°")
print(f"Yaw: {attitude['yaw']:.1f}°")

# Mesafe sensörü
distance = mav.get_distance()
print(f"Mesafe: {distance:.2f}m")

# Batarya durumu
battery = mav.get_battery_status()
print(f"Voltaj: {battery['voltage']:.1f}V")
print(f"Akım: {battery['current']:.1f}A")
print(f"Kalan: {battery['remaining']}%")
```

#### Araç Kontrolü
```python
# Araç arm/disarm
mav.arm_vehicle()       # Motoru etkinleştir
mav.disarm_vehicle()    # Motoru devre dışı bırak

# Acil durdurma
mav.emergency_stop()    # Tüm servolar nötr, motor durdur
```

#### Bağlantı Yönetimi
```python
# Bağlantı durumu
if mav.is_connected():
    print("Pixhawk bağlı")

# Durum özeti
status = mav.get_status_summary()
print(f"Bağlı: {status['connected']}")
print(f"Attitude: {status['attitude']}")
print(f"Mesafe: {status['distance']}")

# Bağlantıyı kapat
mav.disconnect()
```

---

## 🎯 PID Controller Modülü

### Genel Özellikler
- **Kontrol Eksenleri**: Roll, Pitch, Yaw, Derinlik, Hız, Pozisyon
- **Güncelleme Frekansı**: 50Hz (20ms)
- **Integral Windup Koruması**: Evet
- **Çıkış Sınırlama**: Evet

### Ana Sınıflar

#### `PIDController` (Tek Eksen)
```python
from common.pid_controller import PIDController

# PID kontrolcü oluştur
pid = PIDController(
    kp=2.5,             # Proportional gain
    ki=0.1,             # Integral gain
    kd=0.8,             # Derivative gain
    max_output=500,     # Maksimum çıkış
    integral_limit=100, # Integral windup limiti
    setpoint=0.0        # Hedef değer
)

# PID hesaplama
current_value = 5.2  # Güncel sensör değeri
output = pid.update(current_value)
print(f"PID Çıkışı: {output}")

# Hedef değer değiştir
pid.set_setpoint(10.0)

# PID parametrelerini değiştir
pid.set_gains(kp=3.0, ki=0.15, kd=1.0)

# PID durumunu sıfırla
pid.reset()
```

#### `MultiAxisPIDController` (Çoklu Eksen)
```python
from common.pid_controller import MultiAxisPIDController

# Çoklu eksen PID konfigürasyonu
pid_configs = {
    "roll": {"kp": 2.5, "ki": 0.1, "kd": 0.8, "max_output": 500, "integral_limit": 100, "setpoint": 0.0},
    "pitch": {"kp": 2.8, "ki": 0.15, "kd": 0.9, "max_output": 500, "integral_limit": 100, "setpoint": 0.0},
    "yaw": {"kp": 1.8, "ki": 0.05, "kd": 0.6, "max_output": 400, "integral_limit": 80, "setpoint": 0.0},
    "depth": {"kp": 150.0, "ki": 5.0, "kd": 25.0, "max_output": 800, "integral_limit": 200, "setpoint": 1.0}
}

multi_pid = MultiAxisPIDController(pid_configs)

# Tüm eksenleri güncelle
current_values = {
    "roll": 2.3,     # Güncel roll açısı
    "pitch": -1.5,   # Güncel pitch açısı
    "yaw": 0.8,      # Güncel yaw açısı
    "depth": 1.2     # Güncel derinlik
}

outputs = multi_pid.update_all(current_values)
print(f"Roll çıkışı: {outputs['roll']}")
print(f"Pitch çıkışı: {outputs['pitch']}")

# Hedef değerleri toplu ayarla
setpoints = {"roll": 0.0, "pitch": 5.0, "depth": 2.0}
multi_pid.set_setpoints(setpoints)

# Tüm PID'leri sıfırla
multi_pid.reset_all()
```

#### `SubmarineStabilizer` (Tam Stabilizasyon)
```python
from common.pid_controller import SubmarineStabilizer
from x_wing.hardware_pinmap import PIDConfig, FinMixingConfig

# Stabilizasyon sistemi
stabilizer = SubmarineStabilizer(
    pid_configs={
        "roll": PIDConfig.ROLL_PID,
        "pitch": PIDConfig.PITCH_PID,
        "yaw": PIDConfig.YAW_PID,
        "depth": PIDConfig.DEPTH_PID
    },
    mixing_matrix=FinMixingConfig.MIXING_MATRIX,
    fin_effectiveness=FinMixingConfig.FIN_EFFECTIVENESS
)

# Callback fonksiyonları ayarla
def servo_callback(fin_outputs):
    print(f"Fin çıkışları: {fin_outputs}")

def motor_callback(motor_output):
    print(f"Motor çıkışı: {motor_output}")

stabilizer.set_callbacks(servo_callback, motor_callback)

# Stabilizasyonu etkinleştir
stabilizer.enable_stabilization(mode=1)  # STABILIZE modu

# Sensör verisi ile güncelle
sensor_data = {"roll": 2.1, "pitch": -0.8, "yaw": 1.2, "depth": 1.5}
fin_outputs = stabilizer.update_stabilization(sensor_data)
```

### PID Parametre Rehberi

#### Roll/Pitch Kontrolü
- **Kp**: Tepki hızı (2.0-4.0 arası)
- **Ki**: Kalıcı hata düzeltimi (0.05-0.2 arası)
- **Kd**: Aşım önleme (0.5-1.5 arası)

#### Derinlik Kontrolü
- **Kp**: Yüksek değer (100-200 arası)
- **Ki**: Orta değer (2-10 arası)
- **Kd**: Yüksek değer (15-30 arası)

#### Ayarlama İpuçları
1. **Kp çok yüksek**: Sistem salınım yapar
2. **Ki çok yüksek**: Integral windup, aşım
3. **Kd çok yüksek**: Gürültüye hassasiyet artar

---

## 🔘 Button System Modülü

### Genel Özellikler
- **Buton Tipi**: Fiziksel buton (GPIO 18)
- **Debounce**: 500ms
- **Uzun Basma**: 3 saniye eşiği
- **Görev Yönetimi**: Otomatik script başlatma

### Ana Sınıf: `ButtonSystem`

#### Başlatma
```python
from common.button_system import ButtonSystem

# X Wing için buton sistemi
button_system = ButtonSystem(wing_type="x_wing")

# + Wing için buton sistemi
button_system = ButtonSystem(wing_type="+_wing")

# Sistemi başlat
button_system.start_system()
```

#### Görev Yönetimi
```python
# Mevcut görevleri listele
missions = button_system.list_missions()
for mission in missions:
    print(f"{mission['index']}: {mission['name']} ({mission['type']})")

# Sistem durumu
status = button_system.get_system_status()
print(f"Çalışıyor: {status['running']}")
print(f"Seçili Görev: {status['selected_mission']['name']}")
print(f"Aktif Görev: {status['current_mission']['name']}")
```

#### Callback Fonksiyonları
```python
def mission_started(mission):
    print(f"✅ Görev başladı: {mission.name}")
    print(f"Tahmini süre: {mission.duration_estimate} saniye")

def mission_ended(mission):
    print(f"🏁 Görev bitti: {mission.name}")

# Callback'leri ayarla
button_system.set_callbacks(mission_started, mission_ended)
```

#### Buton Kontrolleri
- **Kısa Basış**: Seçili görevi başlat/durdur
- **Uzun Basış (3s)**: Görev seçimini değiştir

#### Görev Listesi
| Wing Type | Görev | Script | Açıklama |
|-----------|-------|--------|----------|
| X Wing | Otonom Görev 1 | `autonomous_mission_1.py` | Stabilizasyon ve derinlik |
| X Wing | Otonom Görev 2 | `autonomous_mission_2.py` | Mesafe sensörü ve navigasyon |
| X Wing | Manuel Görev 1 | `manual_mission_1.py` | Manuel kontrol ve test |
| X Wing | Manuel Görev 2 | `manual_mission_2.py` | Servo kalibrasyonu |
| + Wing | Otonom Görev 1 | `autonomous_mission_1.py` | Stabilizasyon ve derinlik |
| + Wing | Otonom Görev 2 | `autonomous_mission_2.py` | Mesafe sensörü ve navigasyon |
| + Wing | Manuel Görev 1 | `manual_mission_1.py` | Manuel kontrol ve test |
| + Wing | Manuel Görev 2 | `manual_mission_2.py` | Servo kalibrasyonu |

---

## 🛠️ Kullanım Örnekleri

### Tam Sistem Entegrasyonu
```python
from common.mavlink_helper import MAVLinkController
from common.servo_controller import ServoController
from common.d300_sensor import D300Sensor
from common.gpio_helper import GPIOController
from common.pid_controller import SubmarineStabilizer

# Wing type'a göre konfigürasyon seç
wing_type = "x_wing"  # veya "+_wing"

if wing_type == "x_wing":
    from x_wing.hardware_pinmap import *
else:
    from plus_wing.hardware_pinmap import *

# Tüm sistemleri başlat
def setup_full_system():
    # GPIO ayarla
    gpio = GPIOController(
        button_pin=RaspberryPiConfig.BUTTON_PIN,
        led_pin=RaspberryPiConfig.LED_PIN,
        buzzer_pin=RaspberryPiConfig.BUZZER_PIN
    )
    gpio.setup_gpio()
    
    # MAVLink bağlantısı
    mav = MAVLinkController(
        PixhawkConfig.MAVLINK_PORT,
        PixhawkConfig.MAVLINK_BAUD
    )
    mav.connect()
    
    # Servo kontrolcüsü
    servo_controller = ServoController(mav, FinControlConfig.FINS)
    
    # D300 sensörü
    depth_sensor = D300Sensor(
        bus_number=SensorConfig.D300_BUS,
        address=SensorConfig.D300_I2C_ADDRESS
    )
    depth_sensor.connect()
    depth_sensor.calibrate_surface_level()
    depth_sensor.start_continuous_reading()
    
    # Stabilizasyon sistemi
    stabilizer = SubmarineStabilizer(
        pid_configs={
            "roll": PIDConfig.ROLL_PID,
            "pitch": PIDConfig.PITCH_PID,
            "yaw": PIDConfig.YAW_PID,
            "depth": PIDConfig.DEPTH_PID
        },
        mixing_matrix=FinMixingConfig.MIXING_MATRIX,
        fin_effectiveness=FinMixingConfig.FIN_EFFECTIVENESS
    )
    
    return gpio, mav, servo_controller, depth_sensor, stabilizer
```

### Kontrol Döngüsü Örneği
```python
def control_loop():
    gpio, mav, servo_controller, depth_sensor, stabilizer = setup_full_system()
    
    # Callback'leri ayarla
    def servo_callback(fin_outputs):
        servo_controller.set_multiple_servos(fin_outputs)
    
    def motor_callback(motor_output):
        mav.set_motor_speed(motor_output)
    
    stabilizer.set_callbacks(servo_callback, motor_callback)
    stabilizer.enable_stabilization(mode=1)
    
    try:
        while True:
            # Sensör verilerini topla
            attitude = mav.get_attitude()
            depth = depth_sensor.get_depth()
            
            # Sensör verilerini birleştir
            sensor_data = {
                "roll": attitude["roll"],
                "pitch": attitude["pitch"], 
                "yaw": attitude["yaw"],
                "depth": depth
            }
            
            # Stabilizasyonu güncelle
            stabilizer.update_stabilization(sensor_data)
            
            time.sleep(0.02)  # 50Hz kontrol döngüsü
            
    except KeyboardInterrupt:
        print("Kontrol döngüsü durduruldu")
    finally:
        # Temizlik
        stabilizer.disable_stabilization()
        servo_controller.all_servos_neutral()
        mav.emergency_stop()
        depth_sensor.disconnect()
        mav.disconnect()
        gpio.cleanup_gpio()
```

## 🔧 Hata Giderme

### Yaygın Sorunlar

#### D300 Sensörü Bağlanmıyor
```bash
# I2C cihazları tarama
i2cdetect -y 1

# Beklenen çıktı: 0x76 adresinde cihaz görünmeli
```

**Çözümler**:
1. I2C bağlantılarını kontrol edin (SDA: GPIO 2, SCL: GPIO 3)
2. I2C'nin etkin olduğunu kontrol edin: `sudo raspi-config`
3. Sensör güç beslemesini kontrol edin

#### MAVLink Bağlantısı Başarısız
**Çözümler**:
1. USB kablo bağlantısını kontrol edin
2. Pixhawk'ın açık olduğunu kontrol edin
3. Port adresini kontrol edin: `ls /dev/ttyACM*`
4. Baud rate ayarını kontrol edin (115200)

#### Servo Hareket Etmiyor
**Çözümler**:
1. MAVLink bağlantısının aktif olduğunu kontrol edin
2. AUX port bağlantılarını kontrol edin
3. PWM değerlerinin doğru aralıkta olduğunu kontrol edin (1000-2000)
4. Pixhawk'ın servo output'larının etkin olduğunu kontrol edin

#### GPIO Hataları
**Çözümler**:
1. Root yetkisiyle çalıştırın: `sudo python script.py`
2. GPIO pinlerinin başka process tarafından kullanılmadığını kontrol edin
3. Raspberry Pi GPIO'larının doğru bağlandığını kontrol edin

### Log Dosyaları
```python
# Logging ayarları
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('/tmp/submarine_system.log'),
        logging.StreamHandler()
    ]
)
```

## 📊 Performans İzleme

### Sistem Metrikleri
```python
# PID istatistikleri
pid_stats = pid_controller.get_stats()
print(f"Güncelleme sayısı: {pid_stats['update_count']}")
print(f"Ortalama hata: {pid_stats['avg_error']:.3f}")
print(f"Maksimum hata: {pid_stats['max_error']:.3f}")

# Servo durumları
servo_status = servo_controller.get_servo_status()
for servo_name, status in servo_status.items():
    print(f"{status['name']}: PWM {status['current_pwm']}")

# D300 sensör durumu
sensor_status = depth_sensor.get_status_summary()
print(f"D300 bağlı: {sensor_status['connected']}")
print(f"Son okuma: {sensor_status['last_reading']} saniye önce")
```

### Performans Optimizasyonu
1. **Thread Kullanımı**: Sensör okuma ve kontrol döngüleri ayrı thread'lerde
2. **Async İletişim**: MAVLink mesajları non-blocking
3. **Veri Filtreleme**: Sensör verilerinde noise filtreleme
4. **Memory Management**: Circular buffer'lar ile bellek optimizasyonu

## 🧪 Test Fonksiyonları

### Modül Testleri
```python
# D300 sensör testi
python Test/test_d300_sensor.py

# GPIO bileşen testi
python Test/test_gpio_components.py

# MAVLink bağlantı testi
python Test/test_mavlink_connection.py
```

### Manuel Test Komutları
```python
# Servo test
from common.servo_controller import ServoController
servo_controller.test_all_servos(test_duration=2.0)

# PID test
from common.pid_controller import test_pid_response
test_config = {"kp": 2.0, "ki": 0.1, "kd": 0.5, "max_output": 500, "integral_limit": 100, "setpoint": 0.0}
test_signal = [0, 10, 10, 0, -5, 0]
test_pid_response(test_config, test_signal)

# GPIO test
from common.gpio_helper import GPIOController
gpio = GPIOController()
gpio.setup_gpio()
gpio.startup_sequence()
```

## 📋 Modül Bağımlılıkları

### Python Paketleri
```bash
# Gerekli paketler
pip install pymavlink      # MAVLink protokolü
pip install smbus2         # I2C iletişimi
pip install RPi.GPIO       # Raspberry Pi GPIO
```

### Sistem Gereksinimleri
- **Python**: 3.8+
- **Raspberry Pi OS**: Bookworm veya üzeri
- **I2C**: Etkinleştirilmiş
- **GPIO**: Root erişimi

### Import Yapısı
```python
# Common modüller import edilirken
from common.d300_sensor import D300Sensor
from common.servo_controller import ServoController
from common.gpio_helper import GPIOController
from common.mavlink_helper import MAVLinkController
from common.pid_controller import PIDController, MultiAxisPIDController, SubmarineStabilizer
from common.button_system import ButtonSystem, MissionType, MissionConfig

# Wing-specific konfigürasyonlar
from x_wing.hardware_pinmap import *    # X Wing için
from plus_wing.hardware_pinmap import *  # + Wing için
```

## 🎯 Gelişmiş Kullanım

### Özel PID Ayarlama
```python
# Otomatik PID ayarlama
stabilizer.auto_tune_pid("roll", test_duration=30.0)

# Manuel PID gain ayarlama
multi_pid.update_gains("pitch", kp=3.0, ki=0.2, kd=1.0)
```

### Özel Servo Patterns
```python
# Özel servo hareketi tanımla
custom_pattern = {
    "upper_right": [1500, 1800, 1200, 1500],  # PWM sequence
    "upper_left": [1500, 1200, 1800, 1500]
}

# Pattern'i çalıştır
for i in range(len(custom_pattern["upper_right"])):
    positions = {fin: pattern[i] for fin, pattern in custom_pattern.items()}
    servo_controller.set_multiple_servos(positions)
    time.sleep(0.5)
```

### Veri Logging
```python
# Performans verilerini logla
import json
import datetime

def log_performance_data(sensor_data, pid_outputs, fin_outputs):
    log_entry = {
        "timestamp": datetime.datetime.now().isoformat(),
        "sensor_data": sensor_data,
        "pid_outputs": pid_outputs,
        "fin_outputs": fin_outputs
    }
    
    with open("/tmp/submarine_performance.log", "a") as f:
        f.write(json.dumps(log_entry) + "\n")
```

---

Bu dokümantasyon, common modüllerin tam işlevselliğini ve kullanımını detaylandırır. Her modül, sistem güvenilirliği ve performansı için optimize edilmiştir.
