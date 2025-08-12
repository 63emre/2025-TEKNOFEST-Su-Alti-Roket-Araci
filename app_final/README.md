# app_final - Otonom Görevler, Testler ve Terminal Kontrol (Serial MAVLink)

## İçerik

- `mission1_navigation.py`: Görev 1 (seyir + geri dönüş) otonom scripti
- `mission2_rocket_launch.py`: Görev 2 (roket fırlatma) otonom scripti + GPIO tetikleme
- `movement_basic.py`: Basit hareket komutları (düz git, sola dön, kendi ekseninde dön)
- `terminal_control.py`: Eşzamanlı terminal kontrol ve canlı PID ayarı
- `tests/`:
  - `test_servos.py`: AUX1/3/4/5 servo testleri
  - `test_motor.py`: AUX6 motor testleri

## Gereksinimler

- Serial MAVLink: `MAV_ADDRESS` (varsayılan `/dev/ttyACM0`), `MAV_BAUD` (varsayılan `115200`)
- D300 derinlik sensörü: I2C adresi `0x76`
- Roket tetikleme çıkışı: `GPIO 11` (konfigüre edilebilir)

## Kurulum

```bash
cd app_final
python3 -m pip install -r ../App/requirements.txt
```

## Kullanım

### Görev 1
```bash
python3 mission1_navigation.py --distance 50 --depth 2.0 --speed 1.0
```

### Görev 2 (Roket Fırlatma)
```bash
python3 mission2_rocket_launch.py --launch-angle 30 --arm-seconds 2 --fire-seconds 1 --no-confirm
```

### Basit Hareketler
```bash
python3 movement_basic.py forward --meters 10
python3 movement_basic.py turn-left --degrees 45
python3 movement_basic.py rotate --degrees 90
```

### Testler
```bash
python3 tests/test_servos.py
python3 tests/test_motor.py
```

### Eşzamanlı Terminal Kontrol + PID Tuning
```bash
python3 terminal_control.py
```

Komutlar örnek: `arm`, `disarm`, `roll 10`, `pitch -5`, `yaw 15`, `motor 20`, `pid roll 180 10 40`, `pid save`, `status`, `quit`.

## Güvenlik Uyarıları

- Roket tetikleme için GPIO çıkışı bir sürücü devresi üzerinden ateşleme kapsülüne bağlanmalıdır. GPIO doğrudan kapsüle bağlanmaz.
- Ateşleme sırasında çevrenin boş ve güvenli olduğundan emin olun. `--no-confirm` sadece kapalı alanda test için kullanılmamalıdır.


