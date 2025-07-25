# 🚀 TEKNOFEST Su Altı ROV - Terminal GUI [OPTIMIZED REAL DATA]

## 📋 Terminal GUI Genel Bakış

Bu terminal uygulaması, **tamamen gerçek verilerle** çalışan, **FPS optimize** edilmiş ve **real-time** kontrol sağlayan gelişmiş terminal arayüzüdür. Tüm veri stabilizasyonu kaldırılmış, direkt sensörlerden canlı veri akışı sağlanmıştır.

### 🎯 Terminal GUI Özellikleri

- **💯 %100 REAL DATA**: Hiçbir cache/stabilizasyon yok, tüm veriler sensörlerden direkt
- **⚡ FPS OPTIMIZED**: 20 FPS ekran, 50 FPS veri, 100 FPS klavye kontrolü
- **🔴 LIVE STREAM**: IMU, Depth, GPS, Vibration verileri canlı akış
- **⌨️ INSTANT CONTROL**: Servo ve motor kontrolü milisaniye tepkisi
- **🎮 REAL-TIME FEEDBACK**: Her tuş basımında anında sistem yanıtı
- **📊 PERFORMANCE MONITOR**: FPS, bağlantı durumu, veri tazeliği gösterimi
- **🖥️ CROSS-PLATFORM**: Windows, Linux, macOS curses desteği

## 🏗️ Terminal Mimarisi

```
terminal_gui.py
├── 🎮 Real-time Kontroller
│   ├── Servo Kontrol (W,A,S,D,Q,E)
│   ├── Motor Kontrol (O,L)
│   └── ARM/DISARM (Space)
├── 📊 Live Data Stream
│   ├── IMU Data (50 FPS)
│   ├── Depth Sensor (MAVLink)
│   ├── GPS Data (5 FPS)
│   └── Vibration Monitor (20 FPS)
├── 🖥️ Optimized Display
│   ├── Screen Update (20 FPS)
│   ├── Keyboard Poll (100 FPS)
│   └── Performance Monitor
└── 🔧 System Integration
    ├── MAVLink Handler
    ├── Navigation Engine
    └── GPIO Controller
```

## 🚀 Terminal GUI Kurulum ve Çalıştırma

### 1. Gereksinimler

```bash
# Python bağımlılıkları
pip install -r requirements.txt

# Windows için curses
pip install windows-curses

# Linux/macOS için (genelde varsayılan)
sudo apt install python3-dev  # Ubuntu/Debian
```

### 2. Hızlı Başlatma

```bash
cd App/
python3 terminal_gui.py
```

### 3. Sistem Kontrolü

```bash
# Config dosyalarının varlığını kontrol et
ls -la config/

# MAVLink bağlantısını test et
telnet 127.0.0.1 5777
```

## 🎮 Terminal GUI Kontrolü

### ⌨️ Klavye Kontrolleri (REAL-TIME)

#### 🎯 Servo Kontrol (Anlık Tepki):
- **W/S**: Pitch kontrolü (yukarı/aşağı) ↕
- **A/D**: Roll kontrolü (sola/sağa yatma) ↔
- **Q/E**: Yaw kontrolü (sola/sağa dönme) ↺↻
- **X**: Tüm servoları sıfırla

#### ⚙️ Motor Kontrol:
- **O**: Motor gücü artır (+10%)
- **L**: Motor gücü azalt (-10%)
- **Range**: -100% ↔ +100%

#### 🔴 Sistem Kontrol:
- **Space**: ARM/DISARM toggle
- **R**: RAW PWM modu (titreşimsiz)
- **F**: PID modu (filtrelenmiş)

#### 🧭 Navigation Modları:
- **1**: GPS Only modu
- **2**: IMU Only modu  
- **3**: Hybrid (GPS + IMU) modu

#### 📊 Debug Komutları:
- **V**: Canlı vibration verisi göster
- **G**: Canlı GPS koordinatları göster
- **I**: Canlı IMU verileri göster

#### 🚪 Çıkış:
- **ESC**: Güvenli çıkış
- **P**: Program sonlandır
- **Ctrl+C**: Acil çıkış

### 📊 Terminal Ekran Layout

```
┌─────────────────────────────────────────────────────────────────────────────────────────────────────────┐
│                    🚀 TEKNOFEST Su Altı ROV - REAL DATA Terminal [OPTIMIZED] 🚀                        │
│ MAVLink: ✅ LIVE    Durum: 🟢 DISARMED    Kontrol: RAW    Navigation: IMU                              │
├─────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│                                                                                                         │
│ 🎮 SERVO KONTROL (REAL-TIME):    ⚙️ MOTOR KONTROL:         📊 LIVE IMU VERİ:                          │
│   Roll:  +15° (A/D)                Güç: +30% (O/L)          Acc X: +2.341 m/s²                        │
│   Pitch: -10° (W/S)                Hedef Derinlik: 2.5m     Acc Y: -1.203 m/s²                        │
│   Yaw:   +05° (Q/E)                                         Acc Z: +9.801 m/s²                        │
│                                                              Gyro X: +0.123 rad/s                      │
│                                                              Gyro Y: -0.045 rad/s                      │
│                                                              Gyro Z: +0.067 rad/s                      │
│                                                              Vibration: 12.34%                         │
│                                                                                                         │
│ 🌊 LIVE DERİNLİK:                                                                                      │
│   Derinlik: 2.456m                                                                                     │
│   Sıcaklık: 18.5°C                                                                                     │
│   Basınç: 1245.6mb                                                                                     │
│                                                                                                         │
├─────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ ⌨️  KOMUTLAR (REAL-TIME):                                                                              │
│   W/S: Pitch ↕      A/D: Roll ↔         Q/E: Yaw ↺↻                                                   │
│   O/L: Motor ⚡      X: Servo Reset      Space: ARM/DISARM 🔴                                          │
│   R/F: RAW/PID      1/2/3: GPS/IMU/HYB  T: Test Scripts                                               │
│   V: Vibration 📳   G: GPS Data 🌍       ESC/P: Exit 🚪                                               │
├─────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ ⚡ PERFORMANCE (REAL-TIME):                                                                            │
│   Screen FPS: 19.8 (Target: 20)    Connection: LIVE                                                   │
│   Data FPS: 50.0 (Live stream)     Data: FRESH                                                        │
├─────────────────────────────────────────────────────────────────────────────────────────────────────────┤
│ 📝 LIVE LOG MESAJLARI:                                                                                │
│   [14:23:45.123] ✅ OPTIMIZED Terminal GUI sistem bileşenleri başlatıldı!                            │
│   [14:23:45.145] ✅ MAVLink bağlantısı kuruldu!                                                       │
│   [14:23:45.167] ✅ Navigation engine başlatıldı                                                      │
│   [14:23:46.234] 🎮 LIVE Pitch: 0 → +5                                                               │
│   [14:23:46.456] 🎮 LIVE Roll: +5 → +10                                                              │
│   [14:23:47.123] 🔴 ARMED - Sistem aktif!                                                            │
│   [14:23:47.345] 🎮 LIVE Motor: 0 → +10% (O)                                                         │
└─────────────────────────────────────────────────────────────────────────────────────────────────────────┘
```

## 📊 Veri Akışı ve FPS Optimizasyonu

### 🔄 Real-Time Data Flow

```
MAVLink Stream ──→ Terminal GUI ──→ Screen Display
      50 FPS           Processing        20 FPS
                         ↓
                   Live Data Buffer
                   (No Caching!)
                         ↓
                   Keyboard Input
                      100 FPS
```

### ⚡ Performance Metrikleri

| Bileşen | FPS Target | Açıklama |
|---------|------------|----------|
| **Keyboard Input** | 100 FPS | Anında tepki için |
| **Data Fetch** | 50 FPS | Fresh sensor data |
| **Screen Update** | 20 FPS | Flicker-free display |
| **IMU Stream** | 50 FPS | Direct from MAVLink |
| **Depth Sensor** | 20 FPS | MAVLink pressure data |
| **Vibration Monitor** | 20 FPS | Real-time analysis |

### 🎯 Optimizasyon Teknikleri

1. **Direct Data Access**: Hiçbir cache katmanı yok
2. **Minimal Sleep**: 10ms main loop
3. **Smart Screen Updates**: Sadece gerektiğinde redraw
4. **Non-blocking I/O**: Curses nodelay mode
5. **Efficient Memory**: Fixed buffer sizes
6. **Thread-safe Operations**: Lock-free data access

## 🔧 Kontrol Modu Karşılaştırması

### RAW PWM Control (Önerilen):
- ✅ **Titreşim**: Minimum (test_aux4_servo.py benzeri)
- ✅ **Response Time**: 20ms (50Hz direct PWM)
- ✅ **CPU Usage**: Düşük
- ✅ **Real-time**: %100 responsive
- ⚠️ **Precision**: Manuel kontrol gerekli

### PID Control:
- ⚠️ **Titreşim**: Orta (filtreleme nedeniyle)
- ⚠️ **Response Time**: 50ms (20Hz filtered)
- ⚠️ **CPU Usage**: Yüksek
- ✅ **Precision**: Otomatik stabilizasyon
- ⚠️ **Real-time**: Gecikme var

### 🎯 Hangi Modu Seçmeli?

**RAW Modu** şu durumlarda:
- Manuel pilotlama
- Maksimum tepki hızı gerekli
- Minimum titreşim isteniyor
- Test ve kalibrasyon

**PID Modu** şu durumlarda:
- Otomatik stabilizasyon gerekli
- Uzun süreli hover
- Hassas positioning
- Yeni pilotlar

## 🧭 Navigation Sistemleri

### 1. GPS Only Mode:
```
GPS Satellite ──→ Position Update (5Hz)
                      ↓
                 Navigation Engine
                      ↓
              Coordinate Tracking
```
- **Accuracy**: ±1.0m (4+ uydu gerekli)
- **Update Rate**: 5Hz
- **Best For**: Yüzey seyri, uzun mesafe

### 2. IMU Only Mode:
```
IMU Sensor ──→ Acceleration/Gyro (50Hz)
                      ↓
                Dead Reckoning
                      ↓
              Relative Position
```
- **Accuracy**: ±0.5m (başlangıçta, drift artar)
- **Update Rate**: 50Hz
- **Best For**: Su altı, hassas manevra

### 3. Hybrid Mode (Önerilen):
```
GPS + IMU ──→ Sensor Fusion
                   ↓
            Kalman Filtering
                   ↓
           Best Position Estimate
```
- **Logic**: GPS varsa GPS, yoksa IMU
- **Switching**: 5 saniye GPS timeout
- **Fusion**: %70 GPS + %30 IMU
- **Best For**: Tüm koşullar

## 📊 Gerçek Zamanlı Veri Formatları

### IMU Data Format:
```json
{
  "accel_x": 2.341,    // m/s² (real-time)
  "accel_y": -1.203,   // m/s² (real-time)
  "accel_z": 9.801,    // m/s² (real-time)
  "gyro_x": 0.123,     // rad/s (real-time)
  "gyro_y": -0.045,    // rad/s (real-time)
  "gyro_z": 0.067      // rad/s (real-time)
}
```

### Depth Data Format:
```json
{
  "depth_m": 2.456,         // meters (fresh)
  "temperature_c": 18.5,    // celsius (fresh)
  "pressure_mbar": 1245.6,  // millibar (fresh)
  "timestamp": 1635789123.456
}
```

### GPS Data Format:
```json
{
  "latitude": 41.0082,      // degrees (live)
  "longitude": 28.9784,     // degrees (live)
  "altitude": 15.2,         // meters (live)
  "satellites": 8           // count (live)
}
```

## 🐛 Sorun Giderme

### ❌ Terminal GUI Açılmıyor:

```bash
# Windows curses sorunu
pip install windows-curses

# Terminal boyutu kontrol
# Minimum: 120x30
resize -s 30 120
```

### ❌ MAVLink Bağlantı Sorunu:

```bash
# Port kontrolü
netstat -an | grep 5777

# Pixhawk bağlantısı
ls /dev/tty*  # Linux
ls /dev/cu.*  # macOS
```

### ❌ Veri Gelmıyor:

```bash
# MAVLink message kontrolü
python3 -c "
from pymavlink import mavutil
master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')
print(master.recv_match(timeout=5))
"
```

### ❌ FPS Düşük:

```bash
# CPU kullanımı kontrol
top -p $(pgrep -f terminal_gui.py)

# Terminal emülatörü optimize et
export TERM=xterm-256color
```

### ❌ Servo Hareket Etmiyor:

1. **ARM durumu kontrol et**: Space tuşu ile ARM et
2. **PWM değerleri kontrol et**: Log mesajlarını izle
3. **Servo kalibrasyonu**: scripts/servo_calibration.py çalıştır
4. **MAVLink bağlantısı**: Connection durumunu kontrol et

## 🎯 İleri Düzey Kullanım

### 🔧 Debug Mode:

Terminal başlatırken debug aktif et:
```bash
python3 terminal_gui.py --debug
```

### 📊 Performance Monitoring:

Her 5 saniyede FPS raporu:
```bash
python3 terminal_gui.py --fps-report
```

### 🎮 Joystick Integration:

Gelecek sürümde joystick desteği:
```bash
# Planning for next version
python3 terminal_gui.py --joystick /dev/input/js0
```

## 📞 Destek ve İletişim

### 🆘 Acil Durum:
- **Space**: Sistem DISARM
- **ESC**: Güvenli çıkış
- **Ctrl+C**: Acil çıkış

### 📝 Log Dosyaları:
- **Terminal Logs**: Ekranda canlı görüntü
- **System Logs**: /var/log/teknofest-rov/
- **Debug Logs**: ~/.teknofest/debug.log

### 🔗 Bağlantılar:
- **GitHub**: [TEKNOFEST ROV Project]
- **Dokümantasyon**: App/README.md
- **Web GUI**: App/web_gui.py (alternatif)

---

## 🚀 Kullanım Örnekleri

### Temel Pilotlama:
1. Terminal'i aç: `python3 terminal_gui.py`
2. MAVLink bağlantısını kontrol et (✅ LIVE)
3. Space ile ARM et (🔴 ARMED)
4. W,A,S,D ile servo kontrol
5. O,L ile motor kontrol
6. ESC ile güvenli çıkış

### Test Pilotlama:
1. R tuşu ile RAW modu seç
2. X tuşu ile servoları sıfırla
3. V tuşu ile vibration kontrol et
4. Düşük motor değerleri ile test
5. G tuşu ile GPS durumu kontrol et

### Profesyonel Kullanım:
1. 3 tuşu ile Hybrid navigation
2. F tuşu ile PID mod (stabilizasyon)
3. Performance panelinde FPS kontrol
4. Real-time data freshness izle
5. Log mesajları ile sistem takip

---

**🎯 Terminal GUI ile tam kontrol, gerçek veri ve maksimum performans!**

**🚀 TEKNOFEST 2025'te başarılar dileriz!** 