# X WING KONFIGÜRASYONU - PIN HARİTASI

## Genel Bilgi

Bu dokümantasyon X kanat konfigürasyonu için hardware pin haritasını detaylandırır. X Wing konfigürasyonu, çapraz yerleştirilmiş 4 fin ile daha karmaşık hareket kontrolü sağlar.

## Fin Düzeni (X Konfigürasyonu)

```
    Üst Sol (AUX 4)     Üst Sağ (AUX 3)
           \               /
            \             /
             \           /
              \         /
               \       /
                \     /
                 \   /
                  \ /
                   X  <-- Ana Motor (AUX 1)
                  / \
                 /   \
                /     \
               /       \
              /         \
             /           \
            /             \
           /               \
    Alt Sol (AUX 5)     Alt Sağ (AUX 6)
```

## PIXHAWK PIN KONFIGÜRASYONU

### MAVLink Bağlantısı
- **Port**: `/dev/ttyACM0` (USB bağlantısı)
- **Baud Rate**: 115200

### AUX Çıkışları (PWM)
| AUX Port | Bileşen | Model | Açıklama |
|----------|---------|-------|----------|
| AUX 1 | Ana Motor | DEGZ M5 Su Geçirmez Motor | İleri/geri hareket |
| AUX 2 | **BOZUK** | - | **KULLANILMAYACAK** |
| AUX 3 | Üst Sağ Fin | DS3230MG Servo | Çapraz hareket kontrolü |
| AUX 4 | Üst Sol Fin | DS3230MG Servo | Çapraz hareket kontrolü |
| AUX 5 | Alt Sol Fin | DS3230MG Servo | Çapraz hareket kontrolü |
| AUX 6 | Alt Sağ Fin | DS3230MG Servo | Çapraz hareket kontrolü |

### PWM Değerleri
- **Minimum PWM**: 1000 (servo minimum pozisyon)
- **Nötr PWM**: 1500 (servo orta pozisyon)
- **Maksimum PWM**: 2000 (servo maksimum pozisyon)
- **PWM Frekansı**: 330 Hz

### Motor PWM Değerleri (ESC için)
- **Motor Durdurma**: 1000
- **Minimum Hız**: 1100
- **Maksimum Hız**: 2000

## RASPBERRY PI GPIO KONFIGÜRASYONU

### GPIO Pin Atamaları
| GPIO Pin | Bileşen | Model | Açıklama |
|----------|---------|-------|----------|
| GPIO 18 | Kontrol Butonu | 16A P1Z-EC | Sistem kontrol butonu |
| GPIO 22 | Durum LED'i | - | Sistem durumu göstergesi |
| GPIO 23 | Buzzer | - | Ses uyarı sistemi |

### I2C Bağlantısı (D300 Sensörü)
- **I2C Bus**: 1
- **SDA Pin**: GPIO 2
- **SCL Pin**: GPIO 3
- **D300 I2C Adresi**: 0x76

## FIN KONTROL SİSTEMİ (X WING)

### Fin Tanımları
| Fin Adı | AUX Port | Açıklama |
|---------|----------|----------|
| upper_right | AUX 3 | Üst Sağ Fin - Sağ üst çapraz hareket kontrolü |
| upper_left | AUX 4 | Üst Sol Fin - Sol üst çapraz hareket kontrolü |
| lower_left | AUX 5 | Alt Sol Fin - Sol alt çapraz hareket kontrolü |
| lower_right | AUX 6 | Alt Sağ Fin - Sağ alt çapraz hareket kontrolü |

### Hareket Komutları (X Konfigürasyonu)

#### Yukarı Hareket
- Üst Sağ Fin: PWM 2000 (MAX)
- Üst Sol Fin: PWM 2000 (MAX)
- Alt Sol Fin: PWM 1000 (MIN)
- Alt Sağ Fin: PWM 1000 (MIN)

#### Aşağı Hareket
- Üst Sağ Fin: PWM 1000 (MIN)
- Üst Sol Fin: PWM 1000 (MIN)
- Alt Sol Fin: PWM 2000 (MAX)
- Alt Sağ Fin: PWM 2000 (MAX)

#### Sola Hareket
- Üst Sağ Fin: PWM 1000 (MIN)
- Üst Sol Fin: PWM 2000 (MAX)
- Alt Sol Fin: PWM 2000 (MAX)
- Alt Sağ Fin: PWM 1000 (MIN)

#### Sağa Hareket
- Üst Sağ Fin: PWM 2000 (MAX)
- Üst Sol Fin: PWM 1000 (MIN)
- Alt Sol Fin: PWM 1000 (MIN)
- Alt Sağ Fin: PWM 2000 (MAX)

#### Roll Sağ (Saat Yönü Dönüş)
- Üst Sağ Fin: PWM 2000 (MAX)
- Üst Sol Fin: PWM 1000 (MIN)
- Alt Sol Fin: PWM 1000 (MIN)
- Alt Sağ Fin: PWM 2000 (MAX)

#### Roll Sol (Saat Yönü Tersi Dönüş)
- Üst Sağ Fin: PWM 1000 (MIN)
- Üst Sol Fin: PWM 2000 (MAX)
- Alt Sol Fin: PWM 2000 (MAX)
- Alt Sağ Fin: PWM 1000 (MIN)

#### Nötr Pozisyon
- Tüm Finler: PWM 1500 (NEUTRAL)

## FIN MIXİNG MATRİSİ (X WING)

X konfigürasyonu için fin karıştırma matrisi:

### Roll Kontrolü
- **upper_right**: +1.0 (pozitif katkı)
- **upper_left**: -1.0 (negatif katkı)
- **lower_left**: -1.0 (negatif katkı)
- **lower_right**: +1.0 (pozitif katkı)

### Pitch Kontrolü
- **upper_right**: +1.0 (pozitif katkı)
- **upper_left**: +1.0 (pozitif katkı)
- **lower_left**: -1.0 (negatif katkı)
- **lower_right**: -1.0 (negatif katkı)

### Yaw Kontrolü
- **upper_right**: +0.7 (kısmi pozitif katkı)
- **upper_left**: -0.7 (kısmi negatif katkı)
- **lower_left**: +0.7 (kısmi pozitif katkı)
- **lower_right**: -0.7 (kısmi negatif katkı)

## PID KONTROL PARAMETRELERİ

### Roll PID (X Wing için çapraz fin kontrolü)
- **Kp**: 2.5 (Proportional gain)
- **Ki**: 0.1 (Integral gain)
- **Kd**: 0.8 (Derivative gain)
- **Maksimum Çıkış**: ±500 PWM (nötr pozisyondan)
- **Integral Limit**: 100
- **Hedef Açı**: 0.0°

### Pitch PID
- **Kp**: 2.8
- **Ki**: 0.15
- **Kd**: 0.9
- **Maksimum Çıkış**: ±500 PWM
- **Integral Limit**: 100
- **Hedef Açı**: 0.0°

### Yaw PID
- **Kp**: 1.8
- **Ki**: 0.05
- **Kd**: 0.6
- **Maksimum Çıkış**: ±400 PWM
- **Integral Limit**: 80
- **Hedef Açı**: 0.0°

### Derinlik PID
- **Kp**: 150.0 (yüksek gain)
- **Ki**: 5.0
- **Kd**: 25.0
- **Maksimum Çıkış**: ±800 (motor hızı değişimi)
- **Integral Limit**: 200
- **Hedef Derinlik**: 1.0m

### Hız PID (İleri/Geri)
- **Kp**: 100.0
- **Ki**: 2.0
- **Kd**: 15.0
- **Maksimum Çıkış**: ±900
- **Integral Limit**: 150
- **Hedef Hız**: 0.5 m/s

### Pozisyon PID (Mesafe Sensörü)
- **Kp**: 80.0
- **Ki**: 1.5
- **Kd**: 20.0
- **Maksimum Çıkış**: ±600
- **Integral Limit**: 120
- **Hedef Mesafe**: 2.0m

### PID Güncelleme Frekansı
- **Frekans**: 50 Hz (20ms döngü)

## STABİLİZASYON MODLARI

| Mod | Değer | Açıklama |
|-----|-------|----------|
| MANUAL | 0 | Manuel kontrol, PID kapalı |
| STABILIZE | 1 | Roll/Pitch stabilizasyonu aktif |
| DEPTH_HOLD | 2 | Derinlik sabit tutma modu |
| AUTO_PILOT | 3 | Tam otomatik pilot |
| HOVER | 4 | Sabit pozisyon hovering |

## GÜÇ VE GÜVENLİK KONFIGÜRASYONU

### Batarya Özellikleri
- **Voltaj**: 22.2V (6S LiPo)
- **Kapasite**: 1800 mAh
- **Deşarj Oranı**: 65C

### ESC Özellikleri
- **Akım Değeri**: 30A ESC

### Güvenlik Ayarları
- **Acil Durdurma**: Fiziksel güç kesimi mevcut
- **Düşük Voltaj Eşiği**: 18.0V
- **Güç Besleme**: Main çıkışlardan jumper ile
- **Power Port**: Alternatif güç besleme seçeneği

## KULLANIM ÖRNEKLERİ

### Python Kodunda Kullanım

```python
from x_wing.hardware_pinmap import *

# Konfigürasyonu yazdır
print_configuration()

# Fin konfigürasyonunu al
fins = get_fin_configuration()

# Yukarı hareket komutu al
up_command = get_movement_command("yukarı")
```

### Servo Kontrolü

```python
# Servo kontrolcüsü başlat
servo_controller = ServoController(mav_controller, FinControlConfig.FINS)

# Yukarı hareket
servo_controller.execute_movement_command(FinControlConfig.MOVEMENT_COMMANDS["yukarı"])

# Nötr pozisyon
servo_controller.all_servos_neutral()
```

## ÖNEMLİ NOTLAR

1. **AUX 2 BOZUK**: Bu port kullanılmamalıdır
2. **I2C Adresi**: D300 sensörü 0x76 adresinde çalışır
3. **PWM Sınırları**: Servo güvenliği için 1000-2000 arası sınırlandırılmıştır
4. **X Konfigürasyonu**: Çapraz fin yerleşimi sayesinde daha hassas hareket kontrolü sağlar
5. **PID Ayarları**: Su altı dinamikleri için optimize edilmiştir

## TROUBLESHOOTING

### Servo Çalışmıyor
- AUX port bağlantılarını kontrol edin
- PWM değerlerinin 1000-2000 arasında olduğunu doğrulayın
- MAVLink bağlantısının aktif olduğunu kontrol edin

### D300 Sensörü Bağlanmıyor
- I2C bağlantılarını kontrol edin (SDA: GPIO 2, SCL: GPIO 3)
- I2C adresinin 0x76 olduğunu doğrulayın
- `i2cdetect -y 1` komutu ile sensörü tarayın

### Motor Çalışmıyor
- ESC kalibrasyonunu kontrol edin
- Batarya voltajının yeterli olduğunu kontrol edin (>18V)
- PWM değerlerinin ESC için uygun olduğunu doğrulayın
