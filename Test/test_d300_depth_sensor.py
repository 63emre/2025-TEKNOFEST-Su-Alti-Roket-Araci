#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TEKNOFEST 2025 - Su Altı Roket Aracı
Pixhawk (COM5) Üzerinden D300 (MS5837 sınıfı) Basınç/Sıcaklık Okuma ve Test Suite

Bu sürüm D300'ü doğrudan I2C'den okumaz; Pixhawk'ın yayımladığı MAVLink
SCALED_PRESSURE(29) / SCALED_PRESSURE2(137) / SCALED_PRESSURE3(142) mesajlarını
kullanarak basınç (mbar) ve sıcaklığı (°C) alır. Yüzey basıncı kalibre edilip
derinlik P = P0 + ρ g h bağıntısı ile hesaplanır.

Bağlantı:
- Windows: COM5 @ 115200 (gerekirse BAUD’u 57600 yapın)
- ArduSub/ArduPilot üzerinde MS5837/Bar30/D300 benzeri sensör etkin olmalı.

Menü:
1) Yüzey Basıncı Kalibrasyonu
2) Sıcaklık Offset Kalibrasyonu (referans termometre ile)
3) Doğruluk Testi
4) Yanıt Süresi Testi
5) Kararlılık Testi
6) Canlı Veri Monitörleme
7) Test Raporu
0) Çıkış
"""

import time
import json
import math
import numpy as np
from datetime import datetime
import sys
from pymavlink import mavutil

# --------- Kullanıcı Ayarları ----------
SERIAL_PORT = 'COM5'      # Pixhawk seri portu (Windows)
BAUD        = 115200      # 57600 da denenebilir
REQUEST_HZ  = 10          # İstenen yayın hızı (Hz)

# Ölçüm/sınır parametreleri
DEPTH_RESOLUTION = 0.01        # 0.01 m (raporda gösterim amaçlı)
TEMP_RESOLUTION  = 0.01        # 0.01 °C
DEPTH_RANGE_MAX  = 300.0       # 300 m (bilgilendirme)
TEMP_RANGE_MIN   = -20.0
TEMP_RANGE_MAX   = 85.0

CALIBRATION_SAMPLES       = 50
DEPTH_ACCURACY_THRESHOLD  = 0.05   # ±5 cm
TEMP_ACCURACY_THRESHOLD   = 0.5    # ±0.5 °C
SENSOR_TIMEOUT            = 3.0    # s

RHO_SEAWATER = 1025.0  # kg/m^3
G            = 9.81    # m/s^2

# MAVLink mesaj ID'leri (yayın aralığı talebi için)
MSG_ID_SCALED_PRESSURE  = 29
MSG_ID_SCALED_PRESSURE2 = 137
MSG_ID_SCALED_PRESSURE3 = 142
MSG_ID_VFR_HUD          = 74  # (opsiyonel; vertical speed/altitude için)

class PixhawkDepthSensor:
    """
    Pixhawk üstünden MAVLink ile basınç/sıcaklık alır,
    yüzey basıncını kalibre ederek derinlik hesaplar.
    """
    def __init__(self, port=SERIAL_PORT, baud=BAUD):
        self.port = port
        self.baud = baud
        self.master = None
        self.connected = False

        # Durum
        self.current_pressure = 1013.25  # mbar
        self.current_temperature = 0.0   # °C
        self.current_depth = 0.0         # m

        # Kalibrasyon
        self.surface_pressure = 1013.25  # mbar
        self.temperature_offset = 0.0    # °C
        self.calibrated = False
        self.calibration_offset = 0.0    # m (gerekirse küçük düzeltme)

        # İstatistik
        self.total_readings = 0
        self.failed_readings = 0
        self.last_reading_time = None

    # ---------- Bağlantı ----------
    def connect(self):
        print(f"🔌 Pixhawk'a bağlanılıyor: {self.port} @ {self.baud}...")
        try:
            self.master = mavutil.mavlink_connection(self.port, baud=self.baud)
            self.master.wait_heartbeat(timeout=5)
            print(f"✅ Heartbeat alındı: SYS={self.master.target_system}, COMP={self.master.target_component}")

            # Yayın hızlarını talep et (destekliyorsa)
            self._request_message_interval(MSG_ID_SCALED_PRESSURE,  REQUEST_HZ)
            self._request_message_interval(MSG_ID_SCALED_PRESSURE2, REQUEST_HZ)
            self._request_message_interval(MSG_ID_SCALED_PRESSURE3, REQUEST_HZ)
            self._request_message_interval(MSG_ID_VFR_HUD,          REQUEST_HZ)

            self.connected = True
            return True
        except Exception as e:
            print(f"❌ Bağlantı hatası: {e}")
            self.connected = False
            return False

    def disconnect(self):
        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
        self.connected = False
        print("🔌 Bağlantı kapatıldı.")

    def _request_message_interval(self, msg_id, hz):
        try:
            interval_us = int(1_000_000 / max(1, hz))
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
        except Exception as e:
            # Desteklenmeyebilir; kritik değil
            print(f"⚠️ Mesaj {msg_id} yayın aralığı talebi başarısız: {e}")

    # ---------- Okuma & Dönüşümler ----------
    def read_raw_data(self):
        """
        MAVLink'ten SCALED_PRESSURE*/VFR_HUD bekler.
        Basınç: mbar (hPa), Sıcaklık: °C, Derinlik: kalibreye göre hesap.
        """
        if not self.connected:
            return None, None, None

        deadline = time.time() + SENSOR_TIMEOUT
        press_abs = None
        temp_c = None

        while time.time() < deadline:
            msg = self.master.recv_match(type=[
                'SCALED_PRESSURE', 'SCALED_PRESSURE2', 'SCALED_PRESSURE3'
            ], blocking=False)

            if msg is None:
                # VFR_HUD vs. dinleyip geçelim (kullanmasak da buffer boşalır)
                _ = self.master.recv_match(type=['VFR_HUD'], blocking=False)
                time.sleep(0.01)
                continue

            # SCALED_PRESSURE*: press_abs (hPa), temperature (cdegC)
            try:
                if hasattr(msg, 'press_abs'):
                    press_abs = float(msg.press_abs)  # hPa = mbar
                if hasattr(msg, 'temperature'):
                    temp_c = float(msg.temperature) / 100.0  # cdegC -> °C
                break
            except Exception:
                # Beklenmedik alan/format
                pass

        if press_abs is None:
            self.failed_readings += 1
            return None, None, None

        # Varsayılan: press_abs mbar, temp °C (offset uygulanacak)
        pressure_mbar = press_abs
        temperature_c = (temp_c if temp_c is not None else self.current_temperature) + self.temperature_offset

        depth_m = self._pressure_to_depth(pressure_mbar) if self.calibrated else 0.0

        self.current_pressure = pressure_mbar
        self.current_temperature = temperature_c
        self.current_depth = max(0.0, depth_m + self.calibration_offset)
        self.last_reading_time = time.time()
        self.total_readings += 1

        return self.current_depth, self.current_temperature, self.current_pressure

    def _pressure_to_depth(self, pressure_mbar: float) -> float:
        """P = P0 + ρ g h  =>  h = (P - P0) / (ρ g) ; mbar->Pa için ×100"""
        dp = (pressure_mbar - self.surface_pressure) * 100.0  # Pa
        h = dp / (RHO_SEAWATER * G)
        return h

    # ---------- Kalibrasyon ----------
    def calibrate_surface_pressure(self, sample_count=CALIBRATION_SAMPLES):
        if not self.connected:
            print("❌ Pixhawk bağlı değil!")
            return False

        print(f"🔧 Yüzey basıncı kalibrasyonu başlıyor ({sample_count} örnek)...")
        samples = []

        for i in range(sample_count):
            d, t, p = self.read_raw_data()
            if p is not None:
                samples.append(p)
                print(f"  Örnek {i+1:02d}/{sample_count}: {p:.2f} mbar")
            else:
                print(f"  Örnek {i+1:02d}/{sample_count}: Hata!")
            time.sleep(0.1)

        if len(samples) < sample_count * 0.8:
            print("❌ Yetersiz örnek; kalibrasyon başarısız.")
            return False

        self.surface_pressure = float(np.mean(samples))
        std = float(np.std(samples))
        print("✅ Yüzey basıncı kalibrasyonu tamam.")
        print(f"   Ortalama: {self.surface_pressure:.2f} mbar | Std: {std:.2f} mbar")
        print(f"   Min: {min(samples):.2f} | Max: {max(samples):.2f}")
        self.calibrated = True
        return True

    def calibrate_temperature_offset(self, reference_temp_c: float):
        print(f"🌡️ Sıcaklık offset kalibrasyonu (Referans: {reference_temp_c:.2f} °C)")
        vals = []
        for _ in range(20):
            d, t, p = self.read_raw_data()
            if t is not None:
                vals.append(t)
            time.sleep(0.5)

        if len(vals) < 15:
            print("❌ Sıcaklık kalibrasyonu başarısız (yetersiz veri).")
            return False

        measured_avg = float(np.mean(vals))
        self.temperature_offset = reference_temp_c - measured_avg
        print(f"✅ Sıcaklık offset: {self.temperature_offset:+.2f} °C (ölçülen ort: {measured_avg:.2f} °C)")
        return True

    # ---------- Testler ----------
    def run_accuracy_test(self, known_depth=0.0, duration=30):
        if not self.calibrated:
            print("❌ Önce yüzey basıncı kalibrasyonu yapın.")
            return False

        print(f"📏 Doğruluk testi: hedef {known_depth:.3f} m, süre {duration}s")
        depth_vals, temp_vals = [], []
        t0 = time.time()
        print("📊 Veri toplanıyor...")

        while time.time() - t0 < duration:
            d, t, p = self.read_raw_data()
            if d is not None and t is not None:
                depth_vals.append(d)
                temp_vals.append(t)
                print(f"  d={d:.3f} m | T={t:.2f} °C | P={p:.1f} mbar")
            time.sleep(1.0)

        if len(depth_vals) < 10:
            print("❌ Yetersiz veri.")
            return False

        d_mean = float(np.mean(depth_vals))
        d_std  = float(np.std(depth_vals))
        d_err  = abs(d_mean - known_depth)

        t_mean = float(np.mean(temp_vals))
        t_std  = float(np.std(temp_vals))

        print("\n📊 DOĞRULUK TESTİ SONUÇLARI")
        print("=" * 48)
        print(f"Derinlik Ort.: {d_mean:.3f} ± {d_std:.3f} m")
        print(f"Bilinen Der.:  {known_depth:.3f} m")
        print(f"Mutlak Hata:   {d_err:.3f} m")
        print(f"Sıcaklık Ort.: {t_mean:.2f} ± {t_std:.2f} °C")
        print(f"Toplam Okuma:  {len(depth_vals)}")

        ok_depth = d_err <= DEPTH_ACCURACY_THRESHOLD
        ok_temp  = t_std <= TEMP_ACCURACY_THRESHOLD
        print(f"\n✅ Derinlik Doğruluğu: {'GEÇTİ' if ok_depth else 'BAŞARISIZ'}")
        print(f"✅ Sıcaklık Kararlılığı: {'GEÇTİ' if ok_temp else 'BAŞARISIZ'}")
        return ok_depth and ok_temp

    def run_response_time_test(self):
        print("⏱️ Yanıt süresi testi başlıyor...")
        times_ms = []
        for i in range(20):
            t_start = time.time()
            d, t, p = self.read_raw_data()
            t_end = time.time()
            if d is not None:
                dt_ms = (t_end - t_start) * 1000.0
                times_ms.append(dt_ms)
                print(f"  Okuma {i+1:02d}: {dt_ms:.1f} ms")
            time.sleep(0.5)

        if len(times_ms) < 15:
            print("❌ Yetersiz örnek.")
            return False

        avg = float(np.mean(times_ms))
        mn  = float(np.min(times_ms))
        mx  = float(np.max(times_ms))
        print("\n📊 YANIT SÜRESİ")
        print(f"Ortalama: {avg:.1f} ms | Min: {mn:.1f} ms | Max: {mx:.1f} ms")
        ok = avg < 100.0
        print(f"✅ Sonuç: {'GEÇTİ' if ok else 'BAŞARISIZ'} (hedef <100 ms)")
        return ok

    def run_stability_test(self, duration=60):
        print(f"🔄 Kararlılık testi ({duration}s)...")
        depth_vals, temp_vals = [], []
        t0 = time.time()

        while time.time() - t0 < duration:
            d, t, p = self.read_raw_data()
            if d is not None and t is not None:
                depth_vals.append(d)
                temp_vals.append(t)
                if len(depth_vals) % 10 == 0:
                    print(f"  {len(depth_vals)} okuma...")
            time.sleep(1.0)

        if len(depth_vals) < 30:
            print("❌ Yetersiz veri.")
            return False

        d_drift = abs(depth_vals[-1] - depth_vals[0])
        t_drift = abs(temp_vals[-1] - temp_vals[0])
        d_noise = float(np.std(depth_vals))
        t_noise = float(np.std(temp_vals))

        print("\n📊 KARARLILIK")
        print(f"Derinlik Drift:  {d_drift:.3f} m")
        print(f"Sıcaklık Drift:  {t_drift:.2f} °C")
        print(f"Derinlik Gürültü:{d_noise:.3f} m")
        print(f"Sıcaklık Gürültü:{t_noise:.2f} °C")

        ok = (d_drift < 0.10) and (t_drift < 1.0)
        print(f"✅ Sonuç: {'GEÇTİ' if ok else 'BAŞARISIZ'}")
        return ok

    # ---------- Rapor ----------
    def generate_test_report(self):
        return {
            "sensor_source": "Pixhawk MAVLink (SCALED_PRESSURE*)",
            "serial_port": self.port,
            "baud": self.baud,
            "test_timestamp": datetime.now().isoformat(),
            "connection_status": self.connected,
            "calibration_status": self.calibrated,
            "surface_pressure_mbar": self.surface_pressure,
            "temperature_offset_c": self.temperature_offset,
            "statistics": {
                "total_readings": self.total_readings,
                "failed_readings": self.failed_readings,
                "success_rate": (self.total_readings - self.failed_readings) / max(1, self.total_readings) * 100.0
            },
            "current_values": {
                "depth_m": round(self.current_depth, 3),
                "temperature_c": round(self.current_temperature, 2),
                "pressure_mbar": round(self.current_pressure, 2),
            },
            "assumptions": {
                "rho_seawater": RHO_SEAWATER,
                "gravity": G,
                "pressure_units": "mbar (hPa)",
                "temperature_units": "Celsius",
            }
        }

# ----------------- CLI -----------------
def main():
    print("🔬 TEKNOFEST 2025 - Pixhawk COM5 D300 Test Suite")
    print("=" * 60)
    sensor = PixhawkDepthSensor()

    try:
        if not sensor.connect():
            print("❌ Bağlantı sağlanamadı.")
            return 1

        while True:
            print("\n🔧 TEST MENÜSÜ")
            print("1. Yüzey Basıncı Kalibrasyonu")
            print("2. Sıcaklık Offset Kalibrasyonu")
            print("3. Doğruluk Testi")
            print("4. Yanıt Süresi Testi")
            print("5. Kararlılık Testi")
            print("6. Canlı Veri Monitörleme")
            print("7. Test Raporu Oluştur")
            print("0. Çıkış")

            choice = input("\nSeçiminiz (0-7): ").strip()

            if choice == '1':
                sensor.calibrate_surface_pressure()

            elif choice == '2':
                try:
                    ref_temp = float(input("Referans sıcaklık (°C): "))
                except Exception:
                    print("Geçersiz değer.")
                    continue
                sensor.calibrate_temperature_offset(ref_temp)

            elif choice == '3':
                try:
                    known_depth = float(input("Bilinen derinlik (m, yüzey için 0): "))
                    duration = int(input("Test süresi (sn): "))
                except Exception:
                    print("Geçersiz değer.")
                    continue
                sensor.run_accuracy_test(known_depth, duration)

            elif choice == '4':
                sensor.run_response_time_test()

            elif choice == '5':
                try:
                    duration = int(input("Test süresi (sn): "))
                except Exception:
                    print("Geçersiz değer.")
                    continue
                sensor.run_stability_test(duration)

            elif choice == '6':
                print("📡 Canlı veri (Ctrl+C ile durdur)")
                try:
                    i=0
                    while i<1000:
                        d, t, p = sensor.read_raw_data()
                        if d is not None:
                            print(f"Derinlik: {d:.3f} m | Sıcaklık: {t:.2f} °C | Basınç: {p:.1f} mbar")
                        time.sleep(1.0)
                        i=i+2
                except KeyboardInterrupt:
                    print("\n⚠️ Monitörleme durduruldu")

            elif choice == '7':
                report = sensor.generate_test_report()
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                fn = f"d300_pixhawk_report_{ts}.json"
                with open(fn, "w", encoding="utf-8") as f:
                    json.dump(report, f, ensure_ascii=False, indent=2)
                print(f"📊 Rapor kaydedildi: {fn}")
                print(json.dumps(report, ensure_ascii=False, indent=2))

            elif choice == '0':
                break

            else:
                print("❌ Geçersiz seçim.")

        return 0

    except KeyboardInterrupt:
        print("\n⚠️ Kullanıcı tarafından durduruldu.")
        return 1
    except Exception as e:
        print(f"❌ Hata: {e}")
        return 1
    finally:
        sensor.disconnect()

if __name__ == "__main__":
    sys.exit(main())
