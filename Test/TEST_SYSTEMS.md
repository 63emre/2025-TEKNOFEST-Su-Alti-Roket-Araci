# TEST SİSTEMLERİ DOKÜMANTASYONU

## 🧪 Test Sistemi Genel Bakış

Bu dokümantasyon, TEKNOFEST 2025 Su Altı Roket Aracı için geliştirilmiş tüm test sistemlerini detaylandırır. Test sistemleri, hardware bileşenlerinin doğru çalışmasını ve sistem entegrasyonunu doğrulamak için tasarlanmıştır.

## 📁 Test Dosyaları

### Genel Test Dosyaları (`Test/`)
| Test Dosyası | Hedef Bileşen | Test Türü | Süre |
|--------------|---------------|-----------|------|
| `test_d300_sensor.py` | D300 Derinlik Sensörü | I2C İletişim | 2-5 dk |
| `test_gpio_components.py` | GPIO Bileşenleri | Hardware Test | 3-7 dk |
| `test_mavlink_connection.py` | Pixhawk MAVLink | İletişim Test | 2-4 dk |

### Wing-Specific Test Dosyaları
| Konum | Test Dosyası | Hedef | Test Türü |
|-------|--------------|-------|-----------|
| `x_wing/Test/` | `test_servo_control.py` | X Wing Servolar | Servo Kalibrasyon |
| `x_wing/Test/` | `test_stabilization.py` | X Wing Stabilizasyon | PID Test |
| `+_wing/Test/` | `test_servo_control.py` | + Wing Servolar | Servo Kalibrasyon |
| `+_wing/Test/` | `test_stabilization.py` | + Wing Stabilizasyon | PID Test |

---

## 🌊 D300 Derinlik Sensörü Testi

### Test Dosyası: `Test/test_d300_sensor.py`

#### Test Kapsamı
- **I2C Bağlantı Kontrolü**: Bus tarama ve cihaz tespiti
- **Tek Okuma Testi**: Basınç ve sıcaklık verisi okuma
- **Sürekli Okuma Testi**: Thread-based veri akışı
- **Kalibrasyon Testi**: Yüzey seviyesi kalibrasyonu
- **Sensör Limit Testi**: Minimum/maksimum değer testleri

#### Test Çalıştırma
```bash
# Tam test suite'i çalıştır
python Test/test_d300_sensor.py

# Sadece bağlantı testi
python Test/test_d300_sensor.py --connection-only

# Manuel derinlik testi dahil
python Test/test_d300_sensor.py --include-manual
```

#### Test Senaryoları

##### 1. I2C Tarama Testi
```python
def test_i2c_scan(self):
    """I2C bus'ını tarayarak D300 sensörünü ara"""
    # I2C bus 1'i tara
    # Beklenen sonuç: 0x76 adresinde cihaz bulunmalı
```

##### 2. Bağlantı Testi
```python
def test_connection(self):
    """D300 sensörüne bağlantı testi"""
    # Sensöre bağlan
    # Basit veri okuma testi
    # Bağlantı durumu doğrulama
```

##### 3. Tek Okuma Testi
```python
def test_single_reading(self):
    """Tek seferlik veri okuma testi"""
    # Basınç verisi oku
    # Sıcaklık verisi oku
    # Derinlik hesaplama
    # Veri format kontrolü
```

##### 4. Sürekli Okuma Testi
```python
def test_continuous_reading(self):
    """30 saniye sürekli veri okuma"""
    # Thread başlat
    # 10Hz veri okuma
    # Veri tutarlılığı kontrolü
    # Thread güvenli durdurma
```

##### 5. Kalibrasyon Testi
```python
def test_calibration(self):
    """Yüzey seviyesi kalibrasyonu"""
    # 10 ölçüm al
    # Ortalama hesapla
    # Yüzey basıncını ayarla
    # Derinlik offset'i sıfırla
```

##### 6. Sensör Limit Testi
```python
def test_sensor_limits(self):
    """Sensör minimum/maksimum değer testleri"""
    # Minimum basınç testi
    # Maksimum basınç testi
    # Sıcaklık aralığı testi
    # Hata durumu testleri
```

#### Beklenen Çıktı Örneği
```
==================================================
D300 DERINLIK SENSÖRÜ TEST SİSTEMİ
==================================================
Test 1: I2C Bus Tarama
✅ I2C Bus 1 tarandı
✅ D300 sensörü 0x76 adresinde bulundu

Test 2: Bağlantı Testi  
✅ D300 sensörüne başarıyla bağlanıldı
✅ Sensör yanıt veriyor

Test 3: Tek Okuma Testi
✅ Basınç verisi: 1013.25 mbar
✅ Sıcaklık verisi: 22.5°C
✅ Derinlik hesaplama: 0.00m

Test 4: Sürekli Okuma Testi (30s)
✅ Thread başlatıldı
📊 Veri akışı: 10.2Hz ortalama
✅ 302 başarılı okuma
✅ Thread güvenli durduruldu

Test 5: Kalibrasyon Testi
✅ 10 ölçüm alındı
✅ Yüzey basıncı: 1013.18 mbar
✅ Kalibrasyon tamamlandı

Test 6: Sensör Limit Testi
✅ Minimum basınç: 950 mbar
✅ Maksimum basınç: 1100 mbar
✅ Sıcaklık aralığı: 15-35°C

🎉 TÜM D300 TESTLER BAŞARIYLA TAMAMLANDI!
```

---

## 🔌 GPIO Bileşenleri Testi

### Test Dosyası: `Test/test_gpio_components.py`

#### Test Kapsamı
- **GPIO Ayarlama**: Pin konfigürasyonu ve başlatma
- **LED Kontrolü**: Açma, kapama, yanıp sönme
- **Buzzer Kontrolü**: Ses çıkışı ve pattern testleri
- **Buton Kontrolü**: Input okuma ve callback testleri
- **Sequence Testleri**: Özel sequence'lar
- **Stress Testi**: Uzun süreli çalışma testi

#### Test Çalıştırma
```bash
# Tam test suite'i çalıştır
python Test/test_gpio_components.py

# Sadece LED testi
python Test/test_gpio_components.py --led-only

# Stress testi dahil
python Test/test_gpio_components.py --include-stress
```

#### Test Senaryoları

##### 1. GPIO Ayarlama Testi
```python
def test_gpio_setup(self):
    """GPIO pin ayarlama testi"""
    # BCM mod ayarlama
    # Pin konfigürasyonu (IN/OUT)
    # Pull-up/pull-down ayarları
    # Başlangıç durumları
```

##### 2. LED Kontrolü Testi
```python
def test_led(self):
    """LED kontrol testi"""
    # LED açma/kapama
    # Yanıp sönme (5s, 0.5s aralık)
    # Hızlı blink (10 kez, 0.1s)
    # Özel pattern testleri
```

##### 3. Buzzer Kontrolü Testi
```python
def test_buzzer(self):
    """Buzzer kontrol testi"""
    # Tek bip (0.5s)
    # Çoklu bip (3 kez, 0.2s)
    # Özel pattern (startup, success, error)
    # Ses seviyesi kontrolü
```

##### 4. Buton Kontrolü Testi
```python
def test_button(self):
    """Buton input testi"""
    # Buton durumu okuma
    # Callback fonksiyon testi
    # Debounce testi
    # Uzun basma testi (3s)
```

##### 5. Sequence Testleri
```python
def test_sequences(self):
    """Özel sequence testleri"""
    # Başlangıç sequence
    # Başarı sequence
    # Hata sequence
    # Kombinasyon testleri
```

##### 6. Stress Testi
```python
def test_stress_test(self):
    """Uzun süreli çalışma testi"""
    # 60 saniye sürekli blink
    # Yüksek frekanslı switching
    # Bellek kullanımı kontrolü
    # Performans ölçümü
```

#### Beklenen Çıktı Örneği
```
==================================================
GPIO BİLEŞENLERİ TEST SİSTEMİ
==================================================
Test 1: GPIO Ayarlama
✅ BCM mod ayarlandı
✅ Buton pin (GPIO 18) input olarak ayarlandı
✅ LED pin (GPIO 22) output olarak ayarlandı
✅ Buzzer pin (GPIO 23) output olarak ayarlandı

Test 2: LED Kontrolü
✅ LED açma/kapama testi başarılı
✅ 5 saniye yanıp sönme testi (0.5s aralık)
✅ Hızlı blink testi (10 kez)

Test 3: Buzzer Kontrolü
🔊 Tek bip testi (0.5s)
🔊 Çoklu bip testi (3 kez)
🔊 Startup sequence
🔊 Success sequence

Test 4: Buton Kontrolü
⏳ Buton testini başlatmak için butona basın...
✅ Buton basışı algılandı
✅ Callback fonksiyon çalıştı
✅ Debounce testi başarılı

🎉 TÜM GPIO TESTLER BAŞARIYLA TAMAMLANDI!
```

---

## 📡 MAVLink Bağlantı Testi

### Test Dosyası: `Test/test_mavlink_connection.py`

#### Test Kapsamı
- **Bağlantı Kurma**: USB Serial port üzerinden bağlantı
- **Heartbeat Testi**: Pixhawk ile iletişim doğrulama
- **Servo Komut Testi**: AUX port PWM komutları
- **Sensör Veri Testi**: IMU ve mesafe sensörü okuma
- **Motor Komut Testi**: ESC kontrol testleri

#### Test Çalıştırma
```bash
# Tam test suite'i çalıştır
python Test/test_mavlink_connection.py

# Sadece bağlantı testi
python Test/test_mavlink_connection.py --connection-only

# Motor testleri dahil
python Test/test_mavlink_connection.py --include-motor
```

#### Test Senaryoları

##### 1. Bağlantı Testi
```python
def test_connection(self):
    """Pixhawk bağlantı testi"""
    # Serial port açma
    # MAVLink handshake
    # Heartbeat alma
    # Bağlantı durumu doğrulama
```

##### 2. Heartbeat Testi
```python
def test_heartbeat(self):
    """Heartbeat mesaj testi"""
    # Sürekli heartbeat alma
    # Mesaj frekansı kontrolü
    # Bağlantı stabilitesi
    # Timeout testi
```

##### 3. Servo Komut Testi
```python
def test_servo_commands(self):
    """Servo PWM komut testi"""
    # AUX 3-6 portları test
    # PWM değer aralığı (1000-2000)
    # Komut yanıt süresi
    # Hata durumu testleri
```

##### 4. Sensör Veri Testi
```python
def test_sensor_data(self):
    """Sensör veri okuma testi"""
    # ATTITUDE mesajları (roll/pitch/yaw)
    # DISTANCE_SENSOR mesajları
    # BATTERY_STATUS mesajları
    # Veri güncellenme frekansı
```

##### 5. Motor Komut Testi
```python
def test_motor_commands(self):
    """Motor kontrol testi"""
    # Motor hız komutları (0-100%)
    # ESC yanıt testi
    # Acil durdurma testi
    # Güvenlik kontrolleri
```

#### Beklenen Çıktı Örneği
```
==================================================
MAVLINK BAĞLANTI TEST SİSTEMİ
==================================================
Test 1: Bağlantı Kurma
✅ Serial port açıldı: /dev/ttyACM0
✅ MAVLink bağlantısı kuruldu
✅ Heartbeat alındı - Sistem ID: 1

Test 2: Heartbeat Testi (10s)
✅ 47 heartbeat mesajı alındı
✅ Ortalama frekans: 4.7Hz
✅ Bağlantı stabil

Test 3: Servo Komut Testi
✅ AUX 3 test: PWM 1000-2000 aralığı
✅ AUX 4 test: PWM 1000-2000 aralığı
✅ AUX 5 test: PWM 1000-2000 aralığı
✅ AUX 6 test: PWM 1000-2000 aralığı

Test 4: Sensör Veri Testi (15s)
✅ ATTITUDE mesajları: 73 adet (4.9Hz)
✅ Roll: -2.3° ~ +1.8° aralığı
✅ Pitch: -1.1° ~ +2.4° aralığı
✅ Yaw: 0.2° ~ 359.8° aralığı
✅ DISTANCE_SENSOR: 2.45m

🎉 TÜM TESTLER BAŞARIYLA TAMAMLANDI!
```

---

## 🎛️ Servo Kontrol Testleri

### X Wing Servo Testi (`x_wing/Test/test_servo_control.py`)

#### Test Özellikler
- **Fin Konfigürasyonu**: 4 çapraz fin (upper_right, upper_left, lower_left, lower_right)
- **PWM Aralığı**: 1000-2000 μs
- **Test Modları**: Tek servo, çoklu servo, hareket komutları

#### Test Senaryoları

##### 1. Tek Servo Testi
```python
def test_individual_servos(self):
    """Her servo'yu ayrı ayrı test et"""
    for fin_name in ["upper_right", "upper_left", "lower_left", "lower_right"]:
        # Min-Max-Neutral sequence
        # 3 saniye her pozisyonda bekle
        # Servo yanıt süresini ölçü
```

##### 2. X Wing Hareket Testi
```python
def test_x_wing_movements(self):
    """X Wing özel hareket komutları"""
    movements = ["yukarı", "aşağı", "sola", "sağa", "roll_sağ", "roll_sol"]
    
    for movement in movements:
        # Hareket komutunu uygula
        # 3 saniye bekle
        # Nötr pozisyona dön
        # Sonraki harekete geç
```

##### 3. Çapraz Fin Koordinasyonu
```python
def test_diagonal_coordination(self):
    """X Wing çapraz fin koordinasyon testi"""
    # Çapraz fin çiftlerini test et
    # Senkronizasyon kontrolü
    # Timing doğruluğu
```

### + Wing Servo Testi (`+_wing/Test/test_servo_control.py`)

#### Test Özellikler
- **Fin Konfigürasyonu**: 4 dikey/yatay fin (upper, lower, left, right)
- **PWM Aralığı**: 1000-2000 μs
- **Test Modları**: Bağımsız eksen testleri

#### Test Senaryoları

##### 1. Dikey Eksen Testi
```python
def test_vertical_axis(self):
    """Üst/Alt fin koordinasyonu"""
    # Üst fin max, alt fin min
    # Alt fin max, üst fin min
    # Nötr pozisyon
    # Pitch hareket simülasyonu
```

##### 2. Yatay Eksen Testi
```python
def test_horizontal_axis(self):
    """Sol/Sağ fin koordinasyonu"""
    # Sol fin max, sağ fin min
    # Sağ fin max, sol fin min
    # Nötr pozisyon
    # Roll hareket simülasyonu
```

##### 3. + Wing Hareket Testi
```python
def test_plus_wing_movements(self):
    """+ Wing özel hareket komutları"""
    movements = ["yukarı", "aşağı", "sola", "sağa"]
    
    # Her hareket için bağımsız fin kontrolü
    # Doğrudan hareket testleri
    # Karıştırma olmadan kontrol
```

---

## ⚖️ Stabilizasyon Testleri

### X Wing Stabilizasyon (`x_wing/Test/test_stabilization.py`)

#### Test Kapsamı
- **PID Parametre Testi**: Roll, Pitch, Yaw PID'leri
- **Fin Mixing Testi**: X konfigürasyonu karıştırma matrisi
- **Çapraz Kontrol Testi**: Çapraz fin koordinasyonu
- **Stabilizasyon Modu Testi**: Farklı mod testleri

#### Test Senaryoları

##### 1. Roll Stabilizasyon Testi
```python
def test_roll_stabilization(self):
    """X Wing roll kontrolü testi"""
    # Roll setpoint: 0°
    # Simüle roll disturbance: ±15°
    # PID yanıt analizi
    # Çapraz fin koordinasyonu kontrolü
```

##### 2. Pitch Stabilizasyon Testi
```python
def test_pitch_stabilization(self):
    """X Wing pitch kontrolü testi"""
    # Pitch setpoint: 0°
    # Simüle pitch disturbance: ±10°
    # Tüm finlerin koordineli hareketi
    # Overshoot kontrolü
```

##### 3. Çapraz Fin Mixing Testi
```python
def test_diagonal_mixing(self):
    """X Wing çapraz karıştırma testi"""
    # Roll komutu -> çapraz fin hareketi
    # Pitch komutu -> tüm fin hareketi
    # Yaw komutu -> çapraz yaw hareketi
    # Kombinasyon komutları
```

### + Wing Stabilizasyon (`+_wing/Test/test_stabilization.py`)

#### Test Kapsamı
- **Bağımsız Eksen Kontrolü**: Pitch ve Roll ayrı kontrolü
- **Doğrudan Fin Kontrolü**: Karıştırmasız kontrol
- **+ Wing PID Testi**: + Wing optimize PID parametreleri

#### Test Senaryoları

##### 1. Pitch Kontrolü (Dikey Finler)
```python
def test_pitch_control(self):
    """+ Wing pitch kontrolü (üst/alt finler)"""
    # Sadece üst/alt finler aktif
    # Sol/sağ finler nötr
    # Doğrudan pitch kontrolü
```

##### 2. Roll Kontrolü (Yatay Finler)
```python
def test_roll_control(self):
    """+ Wing roll kontrolü (sol/sağ finler)"""
    # Sadece sol/sağ finler aktif
    # Üst/alt finler nötr
    # Doğrudan roll kontrolü
```

##### 3. Bağımsız Eksen Testi
```python
def test_independent_axis(self):
    """+ Wing bağımsız eksen kontrolü"""
    # Pitch ve roll aynı anda
    # Karışım olmadan kontrol
    # + Wing avantajı gösterimi
```

---

## 🔧 Test Yardımcı Araçları

### Test Konfigürasyonu

#### Test Parametreleri
```python
# Test/test_config.py
TEST_CONFIG = {
    "d300_sensor": {
        "bus_number": 1,
        "address": 0x76,
        "test_duration": 30,
        "reading_frequency": 10
    },
    "gpio_components": {
        "button_pin": 18,
        "led_pin": 22,
        "buzzer_pin": 23,
        "debounce_time": 0.5
    },
    "mavlink": {
        "port": "/dev/ttyACM0",
        "baud": 115200,
        "timeout": 5
    },
    "servo_test": {
        "test_duration": 3.0,
        "pwm_step": 100,
        "movement_delay": 2.0
    }
}
```

### Test Raporlama

#### Test Sonuç Formatı
```python
class TestResult:
    """Test sonucu sınıfı"""
    def __init__(self, test_name: str):
        self.test_name = test_name
        self.start_time = time.time()
        self.end_time = None
        self.success = False
        self.error_message = None
        self.metrics = {}
    
    def complete(self, success: bool, error_message: str = None):
        self.end_time = time.time()
        self.success = success
        self.error_message = error_message
        self.duration = self.end_time - self.start_time
```

#### Test Raporu Oluşturma
```python
def generate_test_report(test_results):
    """Test raporu oluştur"""
    report = {
        "test_date": datetime.now().isoformat(),
        "total_tests": len(test_results),
        "passed_tests": sum(1 for r in test_results if r.success),
        "failed_tests": sum(1 for r in test_results if not r.success),
        "total_duration": sum(r.duration for r in test_results),
        "results": test_results
    }
    
    # JSON rapor kaydet
    with open(f"/tmp/test_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json", "w") as f:
        json.dump(report, f, indent=2)
```

---

## 🚀 Entegrasyon Testleri

### Tam Sistem Testi

#### Test Senaryosu
```python
def full_system_integration_test():
    """Tam sistem entegrasyon testi"""
    
    # 1. Tüm bileşenleri başlat
    gpio = setup_gpio()
    mav = setup_mavlink()
    servo_controller = setup_servos(mav)
    depth_sensor = setup_d300()
    
    # 2. Bileşen testleri
    test_results = []
    test_results.append(test_gpio_functionality(gpio))
    test_results.append(test_mavlink_communication(mav))
    test_results.append(test_servo_response(servo_controller))
    test_results.append(test_d300_accuracy(depth_sensor))
    
    # 3. Entegrasyon testleri
    test_results.append(test_stabilization_loop())
    test_results.append(test_mission_sequence())
    
    # 4. Rapor oluştur
    generate_integration_report(test_results)
```

### Pre-Mission Kontrol Listesi

#### Otonom Görev Öncesi
```bash
# 1. Hardware kontrolleri
python Test/test_gpio_components.py
python Test/test_d300_sensor.py
python Test/test_mavlink_connection.py

# 2. Wing-specific testler
python x_wing/Test/test_servo_control.py
python x_wing/Test/test_stabilization.py

# 3. Kalibrasyon
python x_wing/Görevler/manual_mission_2.py  # Servo kalibrasyonu

# 4. Sistem entegrasyon
python Test/integration_test.py
```

#### Manuel Görev Öncesi
```bash
# 1. Temel bileşen testleri
python Test/test_gpio_components.py
python Test/test_mavlink_connection.py

# 2. Servo yanıt testleri
python x_wing/Test/test_servo_control.py

# 3. Manuel kontrol hazırlığı
python x_wing/Görevler/manual_mission_1.py
```

---

## 📊 Test Metrikleri ve KPI'lar

### Performans Metrikleri

#### D300 Sensör Performansı
- **Okuma Frekansı**: 10Hz ± 0.5Hz
- **Veri Doğruluğu**: ±1% basınç, ±0.5°C sıcaklık
- **Yanıt Süresi**: <100ms
- **Bağlantı Stabilitesi**: >99% uptime

#### Servo Performansı
- **Yanıt Süresi**: <50ms PWM değişikliği
- **Pozisyon Doğruluğu**: ±2° servo açısı
- **Hareket Yumuşaklığı**: <5° step değişimi
- **Koordinasyon**: <10ms fin senkronizasyonu

#### MAVLink Performansı
- **Bağlantı Süresi**: <5s ilk bağlantı
- **Mesaj Frekansı**: 20Hz ± 2Hz
- **Komut Yanıt**: <30ms servo komutları
- **Veri Güncellenme**: <50ms sensör verileri

### Başarı Kriterleri

#### Test Geçme Kriterleri
- **D300 Sensör**: Tüm testlerin %90+ başarı oranı
- **GPIO Bileşenler**: Hiç hata olmadan tamamlanma
- **MAVLink**: Kararlı bağlantı ve %95+ mesaj başarısı
- **Servo Kontrol**: Tüm servolar yanıt vermeli
- **Stabilizasyon**: PID kontrolü aktif ve stabil

#### Kritik Hata Koşulları
- **D300 Bağlantı Kaybı**: I2C 0x76 adresinde cihaz yok
- **MAVLink Timeout**: 5s+ heartbeat yok
- **Servo Yanıtsızlık**: PWM komutuna servo yanıt yok
- **GPIO Hatası**: Pin konfigürasyonu başarısız

---

## 🔍 Hata Giderme Rehberi

### D300 Sensör Sorunları

#### "I2C Cihaz Bulunamadı"
```bash
# I2C tarama
i2cdetect -y 1

# I2C etkinleştirme
sudo raspi-config
# -> Interface Options -> I2C -> Enable
```

#### "Veri Okuma Hatası"
**Çözümler**:
1. I2C bağlantılarını kontrol edin (SDA: GPIO 2, SCL: GPIO 3)
2. Güç beslemesini kontrol edin
3. I2C pull-up dirençlerini kontrol edin
4. Bus hızını düşürün (100kHz)

### MAVLink Sorunları

#### "Port Açılamıyor"
```bash
# Port durumunu kontrol et
ls -la /dev/ttyACM*

# Port yetkilerini kontrol et
sudo chmod 666 /dev/ttyACM0

# USB bağlantısını kontrol et
dmesg | grep ttyACM
```

#### "Heartbeat Alınamıyor"
**Çözümler**:
1. Pixhawk'ın açık olduğunu kontrol edin
2. USB kablo kalitesini kontrol edin
3. Baud rate ayarını kontrol edin (115200)
4. MAVLink protokol versiyonunu kontrol edin

### Servo Sorunları

#### "Servo Hareket Etmiyor"
**Çözümler**:
1. AUX port bağlantılarını kontrol edin
2. Servo güç beslemesini kontrol edin
3. PWM değerlerinin doğru aralıkta olduğunu kontrol edin
4. Pixhawk servo output ayarlarını kontrol edin

#### "Servo Titreme/Jitter"
**Çözümler**:
1. PWM frekansını ayarlayın (330Hz)
2. Güç beslemesi gürültüsünü kontrol edin
3. Servo kalitesini kontrol edin
4. PWM sinyali kalitesini ölçün

### GPIO Sorunları

#### "Permission Denied"
```bash
# Root yetkisiyle çalıştır
sudo python Test/test_gpio_components.py

# Kullanıcıyı gpio grubuna ekle
sudo usermod -a -G gpio $USER
```

#### "GPIO Already in Use"
**Çözümler**:
1. Diğer GPIO kullanan process'leri durdurun
2. GPIO cleanup yapın: `GPIO.cleanup()`
3. Sistem reboot yapın
4. Pin çakışmalarını kontrol edin

---

## 📋 Test Checklist

### Günlük Test Rutini

#### Sabah Kontrolleri
- [ ] D300 sensör bağlantısı
- [ ] Pixhawk bağlantısı
- [ ] Servo yanıt testleri
- [ ] Batarya voltaj kontrolü
- [ ] GPIO bileşen testleri

#### Görev Öncesi Kontroller
- [ ] Tüm test scriptleri başarılı
- [ ] Servo kalibrasyonu güncel
- [ ] D300 yüzey kalibrasyonu
- [ ] Acil durdurma sistemi test
- [ ] Log dosyaları temizliği

#### Görev Sonrası Kontroller
- [ ] Log dosyaları analizi
- [ ] Performans metrikleri
- [ ] Hata raporları
- [ ] Sistem durumu kayıtları
- [ ] Sonraki görev hazırlığı

### Test Döngüsü

#### Haftalık Test Döngüsü
1. **Pazartesi**: Tam sistem entegrasyon testi
2. **Salı**: D300 sensör detay testleri
3. **Çarşamba**: Servo kalibrasyon ve testleri
4. **Perşembe**: MAVLink ve stabilizasyon testleri
5. **Cuma**: Görev simülasyonu testleri
6. **Cumartesi**: Gerçek su testleri (havuz)
7. **Pazar**: Test sonuçları analizi ve iyileştirme

---

## 🛠️ Test Geliştirme

### Yeni Test Ekleme

#### Test Template
```python
"""
Test Template - Yeni Test Dosyası İçin
"""
import time
import logging
from typing import Dict, Any

class NewComponentTester:
    """Yeni bileşen test sınıfı"""
    
    def __init__(self):
        self.running = True
        self.logger = logging.getLogger(__name__)
    
    def test_basic_functionality(self):
        """Temel işlevsellik testi"""
        try:
            # Test kodları burada
            pass
        except Exception as e:
            self.logger.error(f"Test hatası: {e}")
            return False
        return True
    
    def run_full_test(self):
        """Tam test senaryosu"""
        test_results = []
        
        # Test 1
        result1 = self.test_basic_functionality()
        test_results.append(("Basic Functionality", result1))
        
        # Test sonuçları
        passed = sum(1 for _, result in test_results if result)
        total = len(test_results)
        
        print(f"\nTest Sonuçları: {passed}/{total} başarılı")
        
        if passed == total:
            print("🎉 TÜM TESTLER BAŞARIYLA TAMAMLANDI!")
        else:
            print("❌ BAZI TESTLER BAŞARISIZ!")

def main():
    """Ana test fonksiyonu"""
    tester = NewComponentTester()
    tester.run_full_test()

if __name__ == "__main__":
    main()
```

### Test Otomasyonu

#### Automated Test Runner
```bash
#!/bin/bash
# run_all_tests.sh

echo "TEKNOFEST 2025 - Otomatik Test Sistemi"
echo "======================================"

# Test sonuçları
PASSED=0
FAILED=0

# D300 Sensör Testi
echo "D300 Sensör Testi..."
if python Test/test_d300_sensor.py; then
    echo "✅ D300 Test Başarılı"
    PASSED=$((PASSED + 1))
else
    echo "❌ D300 Test Başarısız"
    FAILED=$((FAILED + 1))
fi

# GPIO Bileşen Testi
echo "GPIO Bileşen Testi..."
if python Test/test_gpio_components.py; then
    echo "✅ GPIO Test Başarılı"
    PASSED=$((PASSED + 1))
else
    echo "❌ GPIO Test Başarısız"
    FAILED=$((FAILED + 1))
fi

# MAVLink Bağlantı Testi
echo "MAVLink Bağlantı Testi..."
if python Test/test_mavlink_connection.py; then
    echo "✅ MAVLink Test Başarılı"
    PASSED=$((PASSED + 1))
else
    echo "❌ MAVLink Test Başarısız"
    FAILED=$((FAILED + 1))
fi

# Sonuçlar
echo "======================================"
echo "Test Sonuçları: $PASSED başarılı, $FAILED başarısız"

if [ $FAILED -eq 0 ]; then
    echo "🎉 TÜM TESTLER BAŞARIYLA TAMAMLANDI!"
    exit 0
else
    echo "❌ BAZI TESTLER BAŞARISIZ!"
    exit 1
fi
```

---

Bu test sistemi dokümantasyonu, tüm hardware bileşenlerinin güvenilir çalışmasını sağlamak ve görev başarısını garanti etmek için tasarlanmıştır. Düzenli test yapılması, yarışma günü başarısı için kritik öneme sahiptir.
