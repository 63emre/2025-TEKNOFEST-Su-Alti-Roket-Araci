# 🚀 TEKNOFEST 2025 - Pixhawk Pasif I/O Hub Yaklaşımı

Bu döküman TEKNOFEST Su Altı Roket Aracı için **Pixhawk Pasif I/O Hub** yaklaşımını açıklar.

## 🎯 Neden Bu Yaklaşım?

### ❌ GUIDED Hibrit Yaklaşımının Sorunları:
- **GPS Bağımlılığı**: GUIDED/RTL/LOITER modları GPS gerektirir, su altında GPS yok
- **Altitude ≠ Derinlik**: ArduPlane negatif altitude'ı derinlik olarak yorumlamaz
- **Mode Geçiş Sorunları**: Su altında GPS olmadan mode geçişleri başarısız olur
- **Karmaşıklık**: Gereksiz ArduPlane özelliklerini kullanmaya çalışır

### ✅ Pixhawk Pasif I/O Hub Avantajları:
- **Su Altında Çalışır**: GPS gerektirmez, MANUAL mode güvenli
- **Basit ve Güvenilir**: Sadece PWM I/O + telemetri hub olarak kullanır
- **Tam Kontrol**: Pi'de tüm PID'ler ve navigasyon logic'i
- **Gerçekçi**: Yarış ortamında test edilebilir ve çalışır

## 🛠️ Sistem Mimarisi

### 🔧 Pixhawk Rolü (Pasif):
- **PWM Çıkışları**: DO_SET_SERVO ile Pi kontrolünde
- **ATTITUDE Telemetrisi**: 20 Hz IMU verisi
- **SCALED_PRESSURE Telemetrisi**: 10 Hz barometer verisi
- **MANUAL Mode**: Tek güvenli mod (GPS gerektirmez)

### 🧠 Raspberry Pi Rolü (Aktif):
- **Derinlik Kontrolü**: D300 + SCALED_PRESSURE fallback
- **Servo Mixing**: Plus-Wing/X-Wing matematik
- **PID Controllers**: Derinlik, heading, attitude
- **Odometri**: PWM→hız→mesafe (GPS'siz navigasyon)
- **Mission State Machine**: Tüm görev logic'i
- **Watchdog**: 0.5s telemetri timeout → emergency neutral

## 📁 Dosya Yapısı

```
Görevler/
├── config/
│   ├── arduplane_params.txt           # Pixhawk parametreleri (pasif I/O)
│   ├── mission_config_passive_io.json # Pi mission config
│   └── cal_speed.json                 # PWM→hız kalibrasyonu
├── pluswing/
│   ├── hardware_config_passive_io.py  # Servo mixing + PID config
│   └── mission_1_passive_io.py        # Mission 1 navigator
├── test/
│   ├── pixhawk_io_smoke_test.py       # I/O + telemetri testi
│   ├── test_mission_integration.py    # Entegrasyon testleri
│   └── test_physical_hardware.py      # Donanım testleri
└── README_PIXHAWK_PASSIVE_IO.md
```

## ⚙️ Pixhawk Konfigürasyonu

### 1. Servo Functions (Pasif I/O)
```
SERVO1_FUNCTION = 0    # Right fin - DISABLED (Pi kontrolü)
SERVO2_FUNCTION = 0    # Left fin - DISABLED (Pi kontrolü)
SERVO3_FUNCTION = 0    # Up fin - DISABLED (Pi kontrolü)
SERVO4_FUNCTION = 0    # Down fin - DISABLED (Pi kontrolü)
SERVO5_FUNCTION = 0    # Motor - DISABLED (Pi kontrolü)
```

### 2. PWM Limitleri
```
# Fin servolar (DS3230MG)
SERVOx_MIN = 1300
SERVOx_TRIM = 1500
SERVOx_MAX = 1700

# Motor (ESC)
SERVO5_MIN = 1100
SERVO5_TRIM = 1500
SERVO5_MAX = 1900
```

### 3. GPS ve Navigasyon (Devre Dışı)
```
GPS_TYPE = 0           # GPS yok
FLTMODE1 = 0          # MANUAL mode
FLTMODE2 = 0          # MANUAL mode
# ... tüm modlar MANUAL
```

### 4. Telemetri Stream Rates
```
SR0_ATTITUDE = 20     # 20 Hz ATTITUDE
SR0_EXTRA2 = 10       # 10 Hz SCALED_PRESSURE
```

### 5. AUX PWM Çıkışları
```
BRD_PWM_COUNT = 6     # 6 AUX çıkışını PWM yap
```

## 🚀 Çalıştırma

### Ön Gereksinimler
1. **ArduPlane Parametreleri**: `arduplane_params.txt` yüklenmiş
2. **Pixhawk MANUAL Mode**: Tek güvenli mod
3. **D300 Sensörü**: I2C 0x76 adresinde çalışıyor
4. **MAVLink Bağlantısı**: /dev/ttyACM0 115200 baud

### Mission 1 Çalıştırma
```bash
cd Görevler/pluswing/
python3 mission_1_passive_io.py
```

### I/O Smoke Test
```bash
cd Görevler/test/
python3 pixhawk_io_smoke_test.py
python3 pixhawk_io_smoke_test.py --quick
```

## 🧪 Test Sistemi

### Pixhawk I/O Smoke Test
- **MANUAL Mode**: Tek güvenli mod geçişi
- **Telemetri Streams**: ATTITUDE 20Hz + SCALED_PRESSURE 10Hz
- **D300 Sensor**: Derinlik okuma testi
- **Servo I/O**: 4 fin + motor PWM komutları
- **Watchdog**: Telemetri timeout (0.5s) testi

### Test Kapsamı
```
✅ MAVLink bağlantı + heartbeat
✅ MANUAL mode geçiş (doğrulamalı)
✅ ATTITUDE stream (20 Hz)
✅ SCALED_PRESSURE stream (10 Hz)
✅ D300 derinlik sensörü
✅ DO_SET_SERVO komutları (4 fin + motor)
✅ Watchdog + emergency neutral
✅ Su altı uygunluk değerlendirmesi
```

## 📊 Mission Flow

### Görev 1: Seyir Yapma & Geri Dönüş
1. **90s Arming**: Motor/servo neutral (donanım kuralı)
2. **DESCENT**: 2m derinlik (D300/pressure PID)
3. **STRAIGHT_COURSE**: 10m düz seyir (odometri)
4. **OFFSHORE_CRUISE**: 50m uzaklaşma (odometri)
5. **RETURN_NAVIGATION**: Başlangıça dönüş (bearing)
6. **FINAL_APPROACH**: ±2m/5s pozisyon tutma
7. **SURFACE_SHUTDOWN**: Yüzeye çıkış + enerji kesme

### Kontrol Sistemi
- **Derinlik PID**: D300 öncelik, SCALED_PRESSURE fallback
- **Heading PID**: IMU yaw tabanlı
- **Servo Mixing**: Plus-Wing matematik (Pi'de)
- **Odometri**: PWM→hız→mesafe entegrasyonu
- **Watchdog**: 0.5s timeout → latched fault

## 🛡️ Güvenlik Sistemleri

### Watchdog Sistemi
```python
# 0.5s telemetri timeout
if attitude_timeout or depth_timeout:
    trigger_latched_fault("SENSOR_TIMEOUT")
    emergency_neutral_all()
    mission_abort()
```

### Latched Faults
- `SENSOR_TIMEOUT`: Telemetri kesilmesi
- `DEPTH_EXCEEDED`: Maksimum derinlik aşımı
- `BATTERY_LOW`: Düşük battery voltajı
- `LEAK_DETECTED`: Su sızıntısı tespiti
- `WATCHDOG_TIMEOUT`: Watchdog timeout

### Emergency Procedures
1. **Emergency Neutral**: Tüm servolar 1500 PWM
2. **Mission Abort**: Görev durumu → MISSION_ABORT
3. **Telemetri Log**: Fault sebebi kaydet
4. **Clean Shutdown**: Güvenli sistem kapanışı

## 📈 Performans Metrikleri

### Görev 1 Puanlama
- **Seyir Yapma**: 0-150 puan (süre faktörü)
- **Pozisyon Tutma**: 0-90 puan (±2m tolerans)
- **Sızdırmazlık**: 0-60 puan (fault yok)
- **Toplam**: 300 puan maksimum

### Sistem Performansı
- **Derinlik Kontrolü**: ±0.2m tolerans
- **Pozisyon Tutma**: ±2m tolerans (odometri)
- **Telemetri Rate**: ATTITUDE 20Hz, PRESSURE 10Hz
- **Kontrol Frekansı**: 20 Hz ana döngü
- **Watchdog**: 0.5s timeout

## 🔧 Kalibrasyonlar

### PWM→Hız Kalibrasyonu
```json
{
  "plus_wing": {
    "1400": 0.5,
    "1500": 0.0,
    "1600": 0.8,
    "1700": 1.5,
    "1800": 2.2
  }
}
```

### Derinlik Sensörü
- **D300 Öncelik**: I2C 0x76, 10 Hz
- **SCALED_PRESSURE Fallback**: (press_abs - 1013.25) × 0.0102

### Servo Mixing (Plus-Wing)
```python
# Roll/Pitch/Yaw → 4 Fin PWM
PLUS_WING_MATRIX = [
    [ 1.0, -1.0,  0.0,  0.0],  # Roll: Right+, Left-
    [ 0.0,  0.0,  1.0, -1.0],  # Pitch: Up+, Down-
    [ 0.5,  0.5,  0.5,  0.5]   # Yaw: Tüm finler aynı yön
]
```

## 🎯 Sahada Kontrol Listesi

### ArduPlane Tarafı
- [ ] `SERVOx_FUNCTION = 0` (4 fin + motor)
- [ ] PWM limitleri: Fin 1300/1500/1700, Motor 1100/1500/1900
- [ ] `BRD_PWM_COUNT` doğru (AUX PWM için)
- [ ] `GPS_TYPE = 0` (GPS devre dışı)
- [ ] Tüm flight modlar `MANUAL`
- [ ] `ARMING_CHECK` gevşetilmiş (test için)

### Pi Tarafı
- [ ] MAVLink bağlantı: /dev/ttyACM0 115200
- [ ] D300 sensör: I2C 0x76 çalışıyor
- [ ] ATTITUDE stream: 20 Hz geliyor
- [ ] SCALED_PRESSURE stream: 10 Hz geliyor
- [ ] DO_SET_SERVO komutları: ACK alınıyor
- [ ] Watchdog: 0.5s timeout çalışıyor

### Sistem Testleri
- [ ] I/O Smoke Test: %75+ başarı
- [ ] 90s Arming: Motor/servo neutral
- [ ] Derinlik kontrolü: ±0.2m tolerans
- [ ] Odometri: 20m hat ±10% hata
- [ ] Emergency neutral: <0.5s response
- [ ] Mission flow: DESCENT → STRAIGHT → OFFSHORE → RETURN → FINAL → SURFACE

## 💡 Troubleshooting

### Servo Komutları Çalışmıyor
```bash
# SERVOx_FUNCTION kontrolü
param show SERVO1_FUNCTION  # 0 olmalı
param show SERVO2_FUNCTION  # 0 olmalı
param show SERVO3_FUNCTION  # 0 olmalı
param show SERVO4_FUNCTION  # 0 olmalı
param show SERVO5_FUNCTION  # 0 olmalı
```

### AUX Çıkışları PWM Değil
```bash
# BRD_PWM_COUNT ayarı
param show BRD_PWM_COUNT  # 6 olmalı (AUX1-6 PWM)
param set BRD_PWM_COUNT 6
```

### Telemetri Stream Yok
```python
# SET_MESSAGE_INTERVAL kullan
self.master.mav.set_message_interval_send(
    target_system, target_component,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    50000  # 50ms = 20 Hz
)
```

### D300 Sensör Okuma Hatası
```bash
# I2C bus kontrolü
i2cdetect -y 1
# 0x76 adresinde cihaz görünmeli
```

## 🎉 Sonuç

Bu **Pixhawk Pasif I/O Hub** yaklaşımı:
- ✅ **Su altında çalışır** (GPS gerektirmez)
- ✅ **Basit ve güvenilir** (kompleks ArduPlane özellikler yok)
- ✅ **Test edilebilir** (gerçekçi smoke testler)
- ✅ **Yarış uyumlu** (TEKNOFEST ortamında çalışır)
- ✅ **Maintainable** (Pi'de tüm logic, anlaşılır kod)

Bu yaklaşım ile sistem **gerçekten çalışacak** ve yarışta başarılı olacak! 🚀🌊
