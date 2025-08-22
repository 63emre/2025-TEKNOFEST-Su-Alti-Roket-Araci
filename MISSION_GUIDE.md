# GÖREV REHBERİ - TEKNOFEST 2025 SU ALTI ROKET ARACI

## 🎯 Görev Genel Bakış

Bu dokümantasyon, TEKNOFEST 2025 Su Altı Roket Yarışması için tasarlanan tüm görev scriptlerini detaylandırır. Her iki kanat konfigürasyonu (X Wing ve + Wing) için ayrı görev implementasyonları mevcuttur.

## 📋 Görev Listesi

### X Wing Görevleri (`x_wing/Görevler/`)
| Görev | Dosya | Tür | Süre | Açıklama |
|-------|-------|-----|------|----------|
| Otonom Görev 1 | `autonomous_mission_1.py` | Otonom | 5 dk | Seyir yapma ve geri dönüş |
| Otonom Görev 2 | `autonomous_mission_2.py` | Otonom | 4 dk | Mesafe sensörü ve navigasyon |
| Manuel Görev 1 | `manual_mission_1.py` | Manuel | ∞ | İnteraktif kontrol ve test |
| Manuel Görev 2 | `manual_mission_2.py` | Manuel | ∞ | Servo kalibrasyonu |

### + Wing Görevleri (`+_wing/Görevler/`)
| Görev | Dosya | Tür | Süre | Açıklama |
|-------|-------|-----|------|----------|
| Otonom Görev 1 | `autonomous_mission_1.py` | Otonom | 5 dk | Seyir yapma ve geri dönüş |
| Otonom Görev 2 | `autonomous_mission_2.py` | Otonom | 4 dk | Mesafe sensörü ve roket fırlatma |
| Manuel Görev 1 | `manual_mission_1.py` | Manuel | ∞ | İnteraktif kontrol ve test |
| Manuel Görev 2 | `manual_mission_2.py` | Manuel | ∞ | Servo kalibrasyonu |

---

## 🤖 OTONOM GÖREVLER

### Otonom Görev 1: Seyir Yapma & Geri Dönüş

#### Şartname Gereksinimleri
- **Hedef Derinlik**: 2 metre
- **Düz Seyir**: 10 metre ilerleme
- **Kıyı Mesafesi**: En az 50 metre uzaklaşma
- **Geri Dönüş**: Başlangıç noktasına otonom dönüş
- **Yüzey Çıkış**: Pozitif sephiye ile çıkış
- **Süre Limiti**: 5 dakika
- **Toplam Puan**: 300 puan

#### Görev Fazları

##### Faz 1: Başlatma ve Referans Alma
```python
# Sistem başlatma
self._phase_initialization()
```
- **Süre**: 5 saniye
- **İşlemler**:
  - Tüm sensörlerin stabilizasyonu
  - Başlangıç pozisyonu kaydetme
  - Servo test ve kalibrasyon
  - Referans heading ve mesafe kaydetme

##### Faz 2: Dalış (2m Derinlik)
```python
# Kontrollü dalış
self._phase_descent()
```
- **Hedef**: 2.0m ± 0.2m derinlik
- **Kontrol**: Derinlik PID kontrolü
- **Güvenlik**: Maksimum dalış hızı sınırlaması
- **İzleme**: Sürekli derinlik kontrolü

##### Faz 3: Düz Seyir (10m İlerleme)
```python
# Düz istikamette ilerleme
self._phase_straight_course()
```
- **Mesafe**: 10 metre düz ilerleme
- **Hız**: %25 motor gücü (seyir hızı)
- **Kontrol**: Heading stabilizasyonu
- **İzleme**: Mesafe sensörü ile takip

##### Faz 4: Açık Deniz Navigasyonu (50m+)
```python
# Kıyıdan uzaklaşma
self._phase_offshore_navigation()
```
- **Minimum Mesafe**: 50 metre kıyıdan
- **Kontrol**: Mesafe sensörü tabanlı navigasyon
- **Güvenlik**: Engel algılama ve kaçınma

##### Faz 5: Geri Dönüş Navigasyonu
```python
# Başlangıç noktasına dönüş
self._phase_return_navigation()
```
- **Hedef**: Başlangıç pozisyonu ± 2m tolerans
- **Hız**: %30 motor gücü (geri dönüş hızı)
- **Kontrol**: Pozisyon PID kontrolü

##### Faz 6: Son Yaklaşım ve Yüzey Çıkış
```python
# Güvenli yüzey çıkışı
self._phase_surface_and_shutdown()
```
- **Çıkış Açısı**: Pozitif sephiye
- **Güvenlik**: Kontrollü yükselme
- **Sonlandırma**: Enerji kesimi

#### Görev Parametreleri (mission_1_params.json)
```json
{
  "target_depth": 2.0,
  "straight_distance": 10.0,
  "min_offshore_distance": 50.0,
  "cruise_speed_pwm": 1620,
  "return_speed_pwm": 1650,
  "timeout_seconds": 300,
  "position_tolerance": 2.0,
  "depth_tolerance": 0.2,
  "distance_measurement_interval": 0.5,
  "control_frequency": 10
}
```

### Otonom Görev 2: Mesafe Sensörü ve Roket Fırlatma

#### Şartname Gereksinimleri
- **Güvenli Fırlatma Mesafesi**: 30 metre
- **Fırlatma Derinliği**: 1.5 metre
- **Yüzey Yaklaşım Açısı**: 30°
- **Roket Ayrılma**: Servo mekanizması
- **Süre Limiti**: 4 dakika

#### Görev Fazları

##### Faz 1: Sistem Başlatma
- Tüm sistemlerin kontrolü ve hazırlığı
- Sensör kalibrasyonu
- Başlangıç pozisyonu kaydetme

##### Faz 2: Güvenli Bölgeye Gitme
- 30 metre güvenli mesafeye navigasyon
- Engel algılama ve kaçınma
- Pozisyon doğrulama

##### Faz 3: Fırlatma Derinliğine Dalış
- 1.5 metre derinliğe kontrollü dalış
- Derinlik stabilizasyonu
- Pozisyon sabit tutma

##### Faz 4: Yüzey Yaklaşımı (30° Açı)
- 30° pitch açısıyla yüzeye yaklaşım
- Açı kontrolü ve stabilizasyon
- Fırlatma pozisyonu hazırlığı

##### Faz 5: Roket Fırlatma
- Servo ile kapak açma mekanizması
- Roket ayrılma simülasyonu
- Güvenlik kontrolleri

##### Faz 6: Güvenli Geri Çekilme
- Fırlatma bölgesinden uzaklaşma
- Güvenli yüzey çıkışı

#### Görev Parametreleri (mission_2_params.json)
```json
{
  "safe_launch_zone_distance": 30.0,
  "launch_depth": 1.5,
  "surface_approach_angle": 30.0,
  "required_pitch_angle": 30.0,
  "timeout_seconds": 300,
  "depth_tolerance": 0.3,
  "angle_tolerance": 5.0,
  "launch_motor_speed": 35,
  "surface_motor_speed": 40,
  "control_frequency": 10
}
```

---

## 🎮 MANUEL GÖREVLER

### Manuel Görev 1: İnteraktif Kontrol ve Test

#### Özellikler
- **Kontrol Tipi**: Terminal tabanlı komut girişi
- **Stabilizasyon**: İsteğe bağlı PID kontrolü
- **Veri İzleme**: Gerçek zamanlı sensör verileri
- **Test Modu**: Tüm bileşenlerin test edilmesi

#### Kullanılabilir Komutlar

##### Sistem Komutları
| Komut | Açıklama |
|-------|----------|
| `h`, `help` | Yardım menüsünü göster |
| `q`, `quit` | Sistemden çık |
| `status` | Sistem durumunu göster |
| `monitor` | Veri monitörlemeyi başlat/durdur |

##### Servo Komutları
| Komut | Açıklama | Örnek |
|-------|----------|-------|
| `servo test` | Tüm servoları test et | `servo test` |
| `servo <fin> <pwm>` | Belirli servo kontrolü | `servo upper_right 1800` |
| `neutral` | Tüm servolar nötr | `neutral` |

##### Hareket Komutları (X Wing)
| Komut | Açıklama |
|-------|----------|
| `yukarı` | Yukarı hareket |
| `aşağı` | Aşağı hareket |
| `sola` | Sola hareket |
| `sağa` | Sağa hareket |
| `roll_sağ` | Saat yönü dönüş |
| `roll_sol` | Saat yönü tersi dönüş |

##### Hareket Komutları (+ Wing)
| Komut | Açıklama |
|-------|----------|
| `yukarı` | Yukarı hareket (üst fin) |
| `aşağı` | Aşağı hareket (alt fin) |
| `sola` | Sola hareket (sol fin) |
| `sağa` | Sağa hareket (sağ fin) |

##### Motor Komutları
| Komut | Açıklama | Örnek |
|-------|----------|-------|
| `motor <hız>` | Motor hızı ayarla (0-100%) | `motor 50` |
| `stop` | Motoru durdur | `stop` |

##### Stabilizasyon Komutları
| Komut | Açıklama |
|-------|----------|
| `stab on` | Stabilizasyonu aç |
| `stab off` | Stabilizasyonu kapat |
| `stab status` | Stabilizasyon durumu |

##### Test Komutları
| Komut | Açıklama |
|-------|----------|
| `test servo` | Servo test sequence |
| `test movement` | Hareket komutları testi |
| `test system` | Tam sistem testi |

#### Kullanım Örneği
```bash
# Manuel görev başlatma
python x_wing/Görevler/manual_mission_1.py

# Örnek komut sırası
Komut: status                    # Sistem durumunu kontrol et
Komut: servo test               # Servoları test et
Komut: stab on                  # Stabilizasyonu aç
Komut: monitor                  # Veri monitörlemeyi başlat
Komut: yukarı                   # Yukarı hareket
Komut: neutral                  # Nötr pozisyon
Komut: motor 30                 # %30 motor hızı
Komut: stop                     # Motoru durdur
Komut: q                        # Çıkış
```

### Manuel Görev 2: Servo Kalibrasyonu

#### Özellikler
- **Kalibrasyon Modu**: Her servo için ayrı kalibrasyon
- **PWM Ayarlama**: Manuel PWM değer ayarlama
- **Limit Testi**: Servo limit testleri
- **Kaydetme**: Kalibrasyon değerlerini kaydetme

#### Kalibrasyon Süreci
1. **Servo Seçimi**: Kalibre edilecek servo seçimi
2. **Limit Ayarlama**: Min/Max pozisyon ayarlama
3. **Nötr Ayarlama**: Nötr pozisyon fine-tuning
4. **Test**: Kalibrasyon sonucu test
5. **Kaydetme**: Değerleri konfigürasyona kaydetme

---

## 🚀 Görev Çalıştırma Rehberi

### Otonom Görev Çalıştırma

#### Ön Hazırlık
1. **Sistem Kontrolü**: Tüm bileşenlerin çalışır durumda olması
2. **Kalibrasyon**: D300 sensörü yüzey kalibrasyonu
3. **Parametre Kontrolü**: mission_params.json dosyalarının kontrolü
4. **Güvenlik**: Acil durdurma sisteminin hazır olması

#### Çalıştırma Adımları
```bash
# 1. Test scriptlerini çalıştır
python Test/test_d300_sensor.py
python Test/test_mavlink_connection.py
python Test/test_gpio_components.py

# 2. Otonom görevi başlat
python x_wing/Görevler/autonomous_mission_1.py
# veya
python +_wing/Görevler/autonomous_mission_1.py
```

#### Görev İzleme
- **Log Dosyaları**: `/logs/` klasöründe görev logları
- **Gerçek Zamanlı**: Terminal çıktısında görev durumu
- **LED Gösterge**: GPIO LED ile sistem durumu

### Manuel Görev Çalıştırma

#### Terminal Kontrol Modu
```bash
# Manuel görev başlat
python x_wing/Görevler/manual_mission_1.py

# Komut örnekleri
Komut: help        # Tüm komutları göster
Komut: status      # Sistem durumu
Komut: servo test  # Servo testleri
Komut: yukarı      # Hareket komutları
Komut: motor 50    # Motor kontrolü
Komut: stab on     # Stabilizasyon
Komut: monitor     # Veri izleme
```

#### Kalibrasyon Modu
```bash
# Kalibrasyon görevini başlat
python x_wing/Görevler/manual_mission_2.py

# Kalibrasyon süreci
1. Servo seçimi (upper_right, upper_left, vs.)
2. PWM değer ayarlama (1000-2000)
3. Limit testleri
4. Kaydetme işlemi
```

---

## 🔧 Görev Parametreleri Ayarlama

### Mission 1 Parametreleri

#### Temel Parametreler
```json
{
  "target_depth": 2.0,              // Hedef derinlik (metre)
  "straight_distance": 10.0,        // Düz gidiş mesafesi (metre)
  "min_offshore_distance": 50.0,    // Minimum açık deniz mesafesi (metre)
  "timeout_seconds": 300             // Görev timeout (saniye)
}
```

#### Hız Parametreleri
```json
{
  "cruise_speed_pwm": 1620,          // Seyir hızı PWM (1100-2000)
  "return_speed_pwm": 1650,          // Dönüş hızı PWM (1100-2000)
}
```

#### Tolerans Parametreleri
```json
{
  "position_tolerance": 2.0,         // Pozisyon toleransı (metre)
  "depth_tolerance": 0.2,            // Derinlik toleransı (metre)
  "distance_measurement_interval": 0.5, // Mesafe ölçüm aralığı (saniye)
  "control_frequency": 10            // Kontrol frekansı (Hz)
}
```

### Mission 2 Parametreleri

#### Fırlatma Parametreleri
```json
{
  "safe_launch_zone_distance": 30.0, // Güvenli fırlatma mesafesi (metre)
  "launch_depth": 1.5,               // Fırlatma derinliği (metre)
  "surface_approach_angle": 30.0,    // Yüzey yaklaşım açısı (derece)
  "required_pitch_angle": 30.0       // Gerekli pitch açısı (derece)
}
```

#### Kontrol Parametreleri
```json
{
  "depth_tolerance": 0.3,            // Derinlik toleransı (metre)
  "angle_tolerance": 5.0,            // Açı toleransı (derece)
  "launch_motor_speed": 35,          // Fırlatma motor hızı (%)
  "surface_motor_speed": 40,         // Yüzey motor hızı (%)
  "control_frequency": 10            // Kontrol frekansı (Hz)
}
```

---

## 📊 Görev Algoritma Detayları

### Navigasyon Algoritması

#### Pozisyon Hesaplama (GPS Yok)
```python
# Mesafe sensörü tabanlı pozisyon tahmini
def estimate_position(self):
    # Dead reckoning ile pozisyon tahmini
    distance_traveled = self.start_distance_reading - self.current_distance
    
    # Heading ile X,Y koordinatları hesapla
    heading_rad = math.radians(self.current_heading)
    delta_x = distance_traveled * math.cos(heading_rad)
    delta_y = distance_traveled * math.sin(heading_rad)
    
    self.position_estimate["x"] += delta_x
    self.position_estimate["y"] += delta_y
```

#### Waypoint Navigasyonu
```python
# Waypoint listesi (X Wing Görev 2)
waypoints = [
    {"x": 0, "y": 0, "description": "Başlangıç"},
    {"x": 2, "y": 0, "description": "İleri 2m"},
    {"x": 2, "y": 2, "description": "Sağa 2m"},
    {"x": 0, "y": 2, "description": "Geri 2m"},
    {"x": 0, "y": 0, "description": "Başlangıç"}
]

# Waypoint takip algoritması
def navigate_to_waypoint(self, target_waypoint):
    # Hedef mesafe ve açı hesapla
    dx = target_waypoint["x"] - self.position_estimate["x"]
    dy = target_waypoint["y"] - self.position_estimate["y"]
    
    target_distance = math.sqrt(dx*dx + dy*dy)
    target_heading = math.degrees(math.atan2(dy, dx))
    
    # PID kontrol ile hedefe git
    distance_output = self.distance_pid.compute(target_distance, 0)
    heading_output = self.heading_pid.compute(target_heading, self.current_heading)
```

### Stabilizasyon Algoritması

#### X Wing Stabilizasyon
```python
# Çapraz fin karıştırma
def apply_x_wing_mixing(roll_cmd, pitch_cmd, yaw_cmd):
    fin_outputs = {}
    
    # Roll kontrolü (çapraz)
    fin_outputs["upper_right"] = 1500 + roll_cmd + pitch_cmd + yaw_cmd
    fin_outputs["upper_left"] = 1500 - roll_cmd + pitch_cmd - yaw_cmd
    fin_outputs["lower_left"] = 1500 - roll_cmd - pitch_cmd + yaw_cmd
    fin_outputs["lower_right"] = 1500 + roll_cmd - pitch_cmd - yaw_cmd
    
    return fin_outputs
```

#### + Wing Stabilizasyon
```python
# Dikey/yatay fin karıştırma
def apply_plus_wing_mixing(roll_cmd, pitch_cmd, yaw_cmd):
    fin_outputs = {}
    
    # Pitch kontrolü (dikey finler)
    fin_outputs["upper"] = 1500 + pitch_cmd + (yaw_cmd * 0.5)
    fin_outputs["lower"] = 1500 - pitch_cmd - (yaw_cmd * 0.5)
    
    # Roll kontrolü (yatay finler)
    fin_outputs["left"] = 1500 - roll_cmd + (yaw_cmd * 0.5)
    fin_outputs["right"] = 1500 + roll_cmd - (yaw_cmd * 0.5)
    
    return fin_outputs
```

### Roket Fırlatma Algoritması

#### Fırlatma Sequence (+ Wing Görev 2)
```python
def rocket_deployment_sequence(self):
    # 1. Fırlatma pozisyonunu doğrula
    if abs(self.current_pitch) >= self.required_pitch_angle:
        
        # 2. Kapak açma (üst fin maksimum pozisyona)
        rocket_deploy_commands = {
            "upper": 2000,    # Kapak aç
            "lower": 1500,    # Nötr
            "left": 1500,     # Nötr
            "right": 1500     # Nötr
        }
        
        # 3. Kapağı 3 saniye açık tut
        self.servo_controller.set_multiple_servos(rocket_deploy_commands)
        time.sleep(3)
        
        # 4. Kapağı kapat
        self.servo_controller.all_servos_neutral()
        
        return True
    return False
```

---

## 🧪 Test ve Doğrulama

### Ön Test Kontrolleri

#### Sistem Hazırlık Testi
```bash
# 1. GPIO bileşenleri
python Test/test_gpio_components.py

# 2. D300 sensörü
python Test/test_d300_sensor.py

# 3. MAVLink bağlantısı
python Test/test_mavlink_connection.py
```

#### Servo Kalibrasyon Testi
```bash
# X Wing servo testi
python x_wing/Test/test_servo_control.py

# + Wing servo testi
python +_wing/Test/test_servo_control.py
```

#### Stabilizasyon Testi
```bash
# X Wing stabilizasyon
python x_wing/Test/test_stabilization.py

# + Wing stabilizasyon
python +_wing/Test/test_stabilization.py
```

### Görev Simülasyonu

#### Kuru Test (Suya Girmeden)
1. **Servo Hareketleri**: Tüm fin hareketlerinin testi
2. **Motor Kontrolü**: ESC ve motor yanıt testi
3. **Sensör Okuma**: D300 ve Pixhawk sensör testleri
4. **Algoritma Testi**: PID ve navigasyon algoritmalarının testi

#### Su Testi (Havuzda)
1. **Sızdırmazlık**: Su geçirmezlik kontrolü
2. **Yüzdürme**: Araç yüzdürme testi
3. **Derinlik Kontrolü**: D300 sensörü ile derinlik kontrolü
4. **Hareket Testi**: Fin hareketleri ile yön kontrolü

---

## 📈 Performans Optimizasyonu

### Kontrol Döngüsü Optimizasyonu

#### Frekans Ayarları
- **Ana Kontrol**: 50Hz (20ms döngü)
- **Sensör Okuma**: 10Hz (100ms)
- **MAVLink İletişim**: 20Hz (50ms)
- **Veri Logging**: 1Hz (1000ms)

#### Thread Yönetimi
```python
# Veri okuma thread'i
def sensor_reading_thread():
    while running:
        sensor_data = read_all_sensors()
        update_control_system(sensor_data)
        time.sleep(0.02)  # 50Hz

# Kontrol thread'i
def control_thread():
    while running:
        apply_control_outputs()
        time.sleep(0.05)  # 20Hz
```

### Memory Optimizasyonu
- **Circular Buffers**: Sensör verisi için
- **Data Compression**: Log dosyaları için
- **Garbage Collection**: Düzenli memory temizliği

---

## 🛡️ Güvenlik Protokolleri

### Acil Durdurma Senaryoları

#### Otomatik Durdurma Koşulları
1. **Timeout**: Görev süresi aşımı
2. **Düşük Batarya**: 18V altında voltaj
3. **Sensör Hatası**: Kritik sensör bağlantı kaybı
4. **Kontrol Kaybı**: MAVLink bağlantı kaybı
5. **Aşırı Derinlik**: Güvenlik limitini aşma

#### Manuel Durdurma
- **Fiziksel Buton**: GPIO 18 ile acil durdurma
- **Terminal Komutu**: `q` veya Ctrl+C
- **Uzaktan Durdurma**: MAVLink üzerinden

### Güvenlik Kontrolleri
```python
def safety_check(self):
    # Batarya kontrolü
    if self.battery_voltage < 18.0:
        self.emergency_stop("Düşük batarya")
        return False
    
    # Derinlik kontrolü
    if self.current_depth > 5.0:  # 5m maksimum
        self.emergency_stop("Aşırı derinlik")
        return False
    
    # Bağlantı kontrolü
    if not self.mav.is_connected():
        self.emergency_stop("MAVLink bağlantı kaybı")
        return False
    
    return True
```

---

## 📋 Görev Çıktıları ve Raporlama

### Log Dosyaları

#### Otonom Görev Logları
- **Konum**: `/logs/x_wing_autonomous_mission_1.log`
- **İçerik**: Görev fazları, sensör verileri, hata mesajları
- **Format**: Timestamp + Level + Message

#### Performance Logları
```json
{
  "timestamp": "2025-01-16T10:30:45.123",
  "mission_phase": "DESCENT",
  "sensor_data": {
    "depth": 1.85,
    "roll": 2.3,
    "pitch": -1.2,
    "yaw": 0.8,
    "distance": 45.2
  },
  "control_outputs": {
    "upper_right": 1650,
    "upper_left": 1350,
    "lower_left": 1450,
    "lower_right": 1550
  },
  "motor_speed": 25
}
```

### Görev Başarı Kriterleri

#### Görev 1 Başarı Metrikleri
- **Derinlik Doğruluğu**: ±0.2m tolerans
- **Mesafe Doğruluğu**: ±2.0m tolerans
- **Süre Performansı**: 5 dakika altında
- **Geri Dönüş**: Başlangıç noktası ±2m

#### Görev 2 Başarı Metrikleri
- **Güvenli Mesafe**: 30m+ açık deniz
- **Fırlatma Derinliği**: 1.5m ±0.3m
- **Açı Doğruluğu**: 30° ±5° pitch
- **Roket Ayrılması**: Başarılı kapak açma

---

## 🔍 Hata Giderme ve Debugging

### Yaygın Görev Hataları

#### "Sistem Başlatma Hatası"
**Çözümler**:
1. Tüm bağlantıları kontrol edin
2. Test scriptlerini çalıştırın
3. GPIO/I2C ayarlarını kontrol edin

#### "Stabilizasyon Başarısız"
**Çözümler**:
1. PID parametrelerini kontrol edin
2. Servo kalibrasyonu yapın
3. IMU verilerini kontrol edin

#### "Navigasyon Hatası"
**Çözümler**:
1. Mesafe sensörü bağlantısını kontrol edin
2. Başlangıç kalibrasyonunu tekrarlayın
3. Waypoint parametrelerini kontrol edin

### Debug Modu
```python
# Debug logging aktif et
logging.getLogger().setLevel(logging.DEBUG)

# Detaylı veri çıktısı
def debug_sensor_data(self):
    print(f"Derinlik: {self.current_depth:.2f}m")
    print(f"Roll: {self.current_roll:.1f}°")
    print(f"Pitch: {self.current_pitch:.1f}°")
    print(f"Yaw: {self.current_yaw:.1f}°")
    print(f"Mesafe: {self.current_distance:.2f}m")
    print(f"Motor: {self.current_motor_speed}%")
```

---

## 🎯 Gelişmiş Özellikler

### Adaptive PID Kontrolü
```python
# Çevresel koşullara göre PID ayarlama
def adaptive_pid_tuning(self):
    # Su akıntısı tespiti
    if self.detect_current():
        # Daha agresif PID parametreleri
        self.pid_controller.set_gains(kp=3.0, ki=0.2, kd=1.2)
    else:
        # Normal PID parametreleri
        self.pid_controller.set_gains(kp=2.5, ki=0.1, kd=0.8)
```

### Engel Kaçınma
```python
# Mesafe sensörü ile engel tespiti
def obstacle_avoidance(self):
    if self.current_distance < self.min_distance:
        # Engel tespit edildi - kaçınma manevrası
        self.logger.warning(f"Engel tespit edildi: {self.current_distance:.2f}m")
        
        # Sağa dön ve engeli geç
        self.set_fins_mixed(yaw_cmd=200)  # Sağa dön
        time.sleep(2)
        
        # İleri git
        self.mav.set_motor_speed(30)
        time.sleep(3)
        
        # Sola dön (orijinal rota)
        self.set_fins_mixed(yaw_cmd=-200)
        time.sleep(2)
```

### Veri Analizi
```python
# Görev performans analizi
def analyze_mission_performance(log_file):
    with open(log_file, 'r') as f:
        logs = f.readlines()
    
    # Başarı oranları hesapla
    depth_accuracy = calculate_depth_accuracy(logs)
    position_accuracy = calculate_position_accuracy(logs)
    time_efficiency = calculate_time_efficiency(logs)
    
    return {
        "depth_accuracy": depth_accuracy,
        "position_accuracy": position_accuracy,
        "time_efficiency": time_efficiency,
        "overall_score": (depth_accuracy + position_accuracy + time_efficiency) / 3
    }
```

---

Bu rehber, tüm görev tiplerinin detaylı kullanımını ve özelleştirilmesini kapsar. Her görev, TEKNOFEST şartnamesi gereksinimlerine uygun olarak tasarlanmıştır.
