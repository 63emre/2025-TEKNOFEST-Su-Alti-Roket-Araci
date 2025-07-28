# TEKNOFEST Su Altı Roket Aracı - Güncellenmiş Kabiliyet Videosu

## Video Demo Özeti - 2025 Şartnamesi

### Mevcut Donanım Konfigürasyonu
- **Ana Sistem**: Raspberry Pi 5 + Pixhawk
- **Bağlantı**: TCP MAVLink (tcp:127.0.0.1:5777)
- **Servo/Motor Mapping**:
  - AUX1 → Fin Servo 1 - Ön Sol (X-Wing)
  - AUX3 → Fin Servo 2 - Ön Sağ (X-Wing)
  - AUX4 → Fin Servo 3 - Arka Sol (X-Wing)
  - AUX5 → Fin Servo 4 - Arka Sağ (X-Wing)
  - AUX6 → Ana Motor (DEGZ M5 + BLU 30A ESC)
- **GPIO**: 1 LED, 1 Buzzer, 1 Button
- **Sensör**: D300 Depth Sensor (I2C)

### Test Ortamı
- 2x2 metre havuz, 1 metre derinlik
- Gerçek zamanlı IMU verisi (HIGHRES_IMU, ATTITUDE, VFR_HUD)
- X-Wing konfigürasyonu ile asimetrik kontrol

## Video Segmentleri (Kapak Fırlatma Kaldırıldı)

### 1. Demo Waterproof Test
- **Dosya**: `demo_waterproof_test.py`
- **Süre**: 90 saniye
- **İçerik**: 
  - Statik sızdırmazlık testi (1m derinlik)
  - Dinamik hareket testi (tüm eksenlerde)
  - Payload bay açma/kapama testi (güvenli)
- **Güncelleme**: Kapak mekanizması sadece açma/kapama, fiziksel fırlatma yok

### 2. Demo Maneuver Capabilities  
- **Dosya**: `demo_maneuver_capabilities.py`
- **Süre**: 120 saniye
- **İçerik**:
  - X-Wing fin kontrol sistemi
  - 3D hareket kabiliyeti (roll, pitch, yaw)
  - Derinlik kontrolü (0-1m)
  - Hassas pozisyonlama

### 3. Demo Rocket Separation (Güncellenmiş)
- **Dosya**: `demo_rocket_separation.py` 
- **Süre**: 45 saniye
- **İçerik**:
  - Yüzeye çıkış (+30° pitch açısı)
  - Payload bay açma mekanizması (güvenli)
  - Ayrılma simülasyonu (gerçek fırlatma yok)
  - Kurtarma prosedürü

### 4. Demo Emergency Stop
- **Dosya**: `demo_emergency_stop.py`
- **Süre**: 30 saniye  
- **İçerik**:
  - Acil durdurma sistemi
  - GPIO button ile manuel stop
  - Sistem kurtarma prosedürü

### 5. Demo Full Capability
- **Dosya**: `demo_full_capability.py`
- **Süre**: 300 saniye (5 dakika)
- **İçerik**: Tüm testlerin birleştirilmiş versiyonu

## LAST Klasörü - Real-Time Kontrol Scriptleri

### 1. Raw PWM Kontrol
- **Dosya**: `App/LAST/raw_swim_control.py`
- **Özellikler**:
  - Manuel PWM kontrolü
  - X-Wing servo karışım formulü
  - İnteraktif ve otomatik modlar
  - 2x2m havuz için optimize edilmiş PWM değerleri

### 2. PID Kontrollü Sürüş
- **Dosya**: `App/LAST/pid_swim_control.py`
- **Özellikler**:
  - IMU tabanlı otomatik stabilizasyon
  - HIGHRES_IMU + ATTITUDE mesajları
  - 50Hz kontrol döngüsü
  - Roll/Pitch/Yaw PID kontrol

### 3. IMU Dead Reckoning
- **Dosya**: `App/LAST/imu_dead_reckoning.py`
- **Özellikler**:
  - GPS olmadan pozisyon tracking
  - IMU'dan 3D pozisyon tahmini
  - Trajectory kaydetme ve görselleştirme
  - Sensör kalibrasyonu

## Terminal GUI - Real-Time Yüzdürme

### Hızlı Yüzdürme Modu
- **Aktivasyon**: X tuşu
- **Kontroller**:
  - 1,2,3: Yüzdürme seviyeleri (yüzey, sığ, derin)
  - 4,5: Sol/Sağ dönüş
  - 6,7: İleri/Geri hareket
  - 0: Neutral (durma)
  - ESC: Çıkış

## Güvenlik Önlemleri

### Fiziksel Güvenlik
- ❌ Kapak fırlatma mekanizması devre dışı
- ✅ Sadece payload bay açma/kapama
- ✅ Acil durdurma sistemi (GPIO button)
- ✅ PWM limitleri korunuyor

### Yazılım Güvenliği  
- ✅ Servo PWM limit kontrolü
- ✅ Motor güvenlik aralıkları
- ✅ Bağlantı timeout koruması
- ✅ Exception handling

## Test Prosedürü

### Ön Hazırlık
1. `python3 terminal_gui.py` ile sistem kontrolü
2. TCP bağlantı doğrulama (Z tuşu ile debug)
3. IMU verisi kontrolü (HIGHRES_IMU + ATTITUDE)
4. GPIO/D300 sensör testi

### Raw Kontrol Testi
1. `cd App/LAST`
2. `python3 raw_swim_control.py`
3. İnteraktif mod ile manuel test
4. Otomatik sekans ile video kaydı

### PID Kontrol Testi
1. `python3 pid_swim_control.py`
2. Sensör kalibrasyonu (10s)
3. Stabilizasyon modu aktif
4. Hedef attitude ayarlama

### Dead Reckoning Testi
1. `python3 imu_dead_reckoning.py`
2. Sensör kalibrasyonu
3. Tracking başlatma
4. 2D trajectory görselleştirme

## Video Çekim Önerileri

### Çekim Açıları
- Suya daldırma (yandan)
- Su altı hareketler (alttan)
- Yüzeye çıkış (yandan/arkadan)
- Payload bay açılması (yakın çekim)

### Teknik Gösterim
- Terminal GUI ekranı (screen record)
- Real-time IMU verileri
- Dead reckoning trajectory
- GPIO LED/Buzzer gösterimi

## Sonuç

Sistem kapak fırlatma olmadan güvenli şekilde çalışmaya hazır. Tüm kabiliyet testleri gerçek donanım konfigürasyonu ile uyumlu ve 2x2m havuzda test edilebilir durumda. 