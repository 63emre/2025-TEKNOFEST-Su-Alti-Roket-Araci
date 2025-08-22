# + WING KONFIGÜRASYONU - PIN HARİTASI

## Genel Bilgi

Bu dokümantasyon + (Plus) kanat konfigürasyonu için hardware pin haritasını detaylandırır. + Wing konfigürasyonu, dikey ve yatay eksenlerde yerleştirilmiş 4 fin ile daha basit ve doğrudan hareket kontrolü sağlar.

## Fin Düzeni (+ Konfigürasyonu)

```
           Üst Fin (AUX 3)
               |
               |
               |
Sol Fin -------|------- Sağ Fin
(AUX 5)        |        (AUX 6)
               |
               | Ana Motor (AUX 1)
               |
               |
           Alt Fin (AUX 4)
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
| AUX 3 | Üst Fin | DS3230MG Servo | Yukarı yönlü hareket kontrolü |
| AUX 4 | Alt Fin | DS3230MG Servo | Aşağı yönlü hareket kontrolü |
| AUX 5 | Sol Fin | DS3230MG Servo | Sola yönlü hareket kontrolü |
| AUX 6 | Sağ Fin | DS3230MG Servo | Sağa yönlü hareket kontrolü |

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

## FIN KONTROL SİSTEMİ (+ WING)

### Fin Tanımları
| Fin Adı | AUX Port | Açıklama |
|---------|----------|----------|
| upper | AUX 3 | Üst Fin - Yukarı yönlü hareket kontrolü |
| lower | AUX 4 | Alt Fin - Aşağı yönlü hareket kontrolü |
| left | AUX 5 | Sol Fin - Sola yönlü hareket kontrolü |
| right | AUX 6 | Sağ Fin - Sağa yönlü hareket kontrolü |

### Hareket Komutları (+ Konfigürasyonu)

#### Yukarı Hareket
- Üst Fin: PWM 2000 (MAX)
- Alt Fin: PWM 1000 (MIN)
- Sol/Sağ Fin: Değişmez

#### Aşağı Hareket
- Üst Fin: PWM 1000 (MIN)
- Alt Fin: PWM 2000 (MAX)
- Sol/Sağ Fin: Değişmez

#### Sola Hareket
- Sol Fin: PWM 2000 (MAX)
- Sağ Fin: PWM 1000 (MIN)
- Üst/Alt Fin: Değişmez

#### Sağa Hareket
- Sol Fin: PWM 1000 (MIN)
- Sağ Fin: PWM 2000 (MAX)
- Üst/Alt Fin: Değişmez

#### Nötr Pozisyon
- Tüm Finler: PWM 1500 (NEUTRAL)

## FIN MIXİNG MATRİSİ (+ WING)

+ konfigürasyonu için fin karıştırma matrisi:

### Roll Kontrolü
- **upper**: 0.0 (katkı yok)
- **lower**: 0.0 (katkı yok)
- **left**: -1.0 (negatif katkı)
- **right**: +1.0 (pozitif katkı)

### Pitch Kontrolü
- **upper**: +1.0 (pozitif katkı)
- **lower**: -1.0 (negatif katkı)
- **left**: 0.0 (katkı yok)
- **right**: 0.0 (katkı yok)

### Yaw Kontrolü
- **upper**: +0.5 (kısmi pozitif katkı)
- **lower**: -0.5 (kısmi negatif katkı)
- **left**: +0.5 (kısmi pozitif katkı)
- **right**: -0.5 (kısmi negatif katkı)

## PID KONTROL PARAMETRELERİ

### Roll PID (+ Wing için dikey fin kontrolü)
- **Kp**: 3.2 (+ wing için daha yüksek gain)
- **Ki**: 0.12
- **Kd**: 1.0
- **Maksimum Çıkış**: ±500 PWM
- **Integral Limit**: 100
- **Hedef Açı**: 0.0°

### Pitch PID (+ Wing pitch kontrolü daha doğrudan)
- **Kp**: 3.5
- **Ki**: 0.18
- **Kd**: 1.1
- **Maksimum Çıkış**: ±500 PWM
- **Integral Limit**: 100
- **Hedef Açı**: 0.0°

### Yaw PID
- **Kp**: 2.2
- **Ki**: 0.08
- **Kd**: 0.7
- **Maksimum Çıkış**: ±400 PWM
- **Integral Limit**: 80
- **Hedef Açı**: 0.0°

### Derinlik PID
- **Kp**: 150.0
- **Ki**: 5.0
- **Kd**: 25.0
- **Maksimum Çıkış**: ±800 (motor hızı)
- **Integral Limit**: 200
- **Hedef Derinlik**: 1.0m

### Hız PID
- **Kp**: 100.0
- **Ki**: 2.0
- **Kd**: 15.0
- **Maksimum Çıkış**: ±900
- **Integral Limit**: 150
- **Hedef Hız**: 0.5 m/s

### Pozisyon PID
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
from plus_wing.hardware_pinmap import *

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

## X WING ile KARŞILAŞTIRMA

| Özellik | + Wing | X Wing |
|---------|--------|--------|
| Fin Sayısı | 4 (dikey/yatay) | 4 (çapraz) |
| Hareket Karmaşıklığı | Basit | Karmaşık |
| Roll PID Kp | 3.2 | 2.5 |
| Pitch PID Kp | 3.5 | 2.8 |
| Kontrol Doğruluğu | Doğrudan | Karıştırmalı |
| Yaw Kontrolü | Kısmi (0.5) | Daha güçlü (0.7) |

## ÖNEMLİ NOTLAR

1. **AUX 2 BOZUK**: Bu port kullanılmamalıdır
2. **I2C Adresi**: D300 sensörü 0x76 adresinde çalışır
3. **PWM Sınırları**: Servo güvenliği için 1000-2000 arası sınırlandırılmıştır
4. **+ Konfigürasyonu**: Dikey/yatay fin yerleşimi sayesinde daha basit kontrol sağlar
5. **PID Ayarları**: + wing'in doğrudan kontrolü için optimize edilmiştir

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
