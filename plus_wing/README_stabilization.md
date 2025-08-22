# Plus Wing Stabilizasyon Sistemi

Bu klasörde SARA'nın 4 kanatlı stabilizasyon sistemi için gerekli dosyalar bulunmaktadır.

## Dosyalar

### Manuel Test Dosyaları (Tek Eksen)

Bu dosyalar tek bir ekseni test etmek için kullanılır:

- **`manuel_roll.py`** - Sadece roll ekseni testi (sol-sağ kanatlar)
- **`manuel_pitch.py`** - Sadece pitch ekseni testi (ön-arka kanatlar)
- **`manuel_yaw.py`** - Sadece yaw ekseni testi (çapraz koordinasyon)

### Tam Stabilizasyon

- **`full_stabilization.py`** - Tüm eksenler aynı anda (ROLL + PITCH + YAW)

## Kullanım

### 1. Tek Eksen Testleri

Önce her ekseni ayrı ayrı test edin:

```bash
# Roll testi (Pixhawk'ı sola-sağa çevirin)
python3 manuel_roll.py

# Pitch testi (Pixhawk'ı öne-arkaya çevirin)
python3 manuel_pitch.py

# Yaw testi (Pixhawk'ı Z ekseni etrafında çevirin)
python3 manuel_yaw.py
```

### 2. Tam Stabilizasyon

Tüm eksenler doğru çalıştıktan sonra:

```bash
python3 full_stabilization.py
```

## Kanat Konfigürasyonu

```
Arkadan Bakış:
   UP (14) ---- RIGHT (12)
      |            |
      |     SARA   |
      |            |
   LEFT (13) -- DOWN (11)
```

## Hareket Mantığı

### Roll (Yan Yatış)

- **CCW Roll** → Sol kanatlar ↑, Sağ kanatlar ↓
- Kanatlar: UP↑ + LEFT↓ vs RIGHT↓ + DOWN↑

### Pitch (Öne-Arkaya)

- **+Pitch (Burun Yukarı)** → Ön kanatlar ↑, Arka kanatlar ↓
- Kanatlar: UP↑ + RIGHT↓ vs DOWN↓ + LEFT↑

### Yaw (Dönüş)

- **CCW Yaw** → Çapraz koordinasyon
- Grup 1: UP↑ + DOWN↑ (sol-çapraz)
- Grup 2: RIGHT↓ + LEFT↓ (sağ-çapraz)

## Ayarlar

Her dosyada aşağıdaki parametreler sahada ayarlanabilir:

```python
# Duyarlılık (rad → μs)
K_ANG_US_PER_RAD = 500.0  # 300-700 arası deneyin

# Deadband (küçük titreşimleri bastır)
DEADBAND_DEG = 1.0        # 0.5-3.0 arası

# Güvenlik sınırı
MAX_DELTA_US = 350.0      # 200-400 arası

# İşaret yönü (ters davranırsa -1)
ROLL_SENSE = +1.0
PITCH_SENSE = +1.0
YAW_SENSE = +1.0

# Mekanik yönler (PWM↑ fiziksel↑ değilse -1)
DIR_LEFT_UP = +1.0
DIR_RIGHT_DOWN = +1.0
DIR_RIGHT_UP = -1.0
DIR_LEFT_DOWN = -1.0
```

## Güvenlik

- Tüm dosyalar Ctrl+C ile güvenli çıkış yapar
- Veri kesilirse servolar otomatik nötr konuma gelir
- PWM değerleri güvenli aralıkta sınırlanır (1100-1900μs)
- Deadband ile küçük titreşimler bastırılır

## Troubleshooting

### Ters Hareket Ediyorsa

1. İlgili `_SENSE` değerini -1 yapın
2. Mekanik yön yanlışsa `DIR_*` değerlerini -1 yapın

### Çok Hassas/Az Hassas

- `K_ANG_US_PER_RAD` değerini artırın/azaltın
- `MAX_DELTA_US` ile maksimum hareketi sınırlayın

### Titreşim Varsa

- `DEADBAND_DEG` değerini artırın
- Kontrol frekansını azaltın (`time.sleep` değerini artırın)

## Test Sırası

1. ✅ `manuel_roll.py` - Roll ekseni
2. ✅ `manuel_pitch.py` - Pitch ekseni
3. ✅ `manuel_yaw.py` - Yaw ekseni
4. ✅ `full_stabilization.py` - Tam sistem
