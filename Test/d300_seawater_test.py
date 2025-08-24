#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
D300 DENİZ SUYU TEST SCRIPTI
Deniz suyu yoğunluğu (1025 kg/m³) ile havada kalibrasyon testi

Kullanım:
    python d300_seawater_test.py --calib 6
    python d300_seawater_test.py --calib 6 --port COM3
"""

import argparse
import time
import statistics
import sys
import os
from pymavlink import mavutil

# Test klasöründen config'e erişim için path ekle
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'görevlerf1', 'pluswing'))

try:
    from config import *
except ImportError:
    # Fallback değerler
    D300_SEAWATER_DENSITY = 1025.0
    D300_GRAVITY = 9.81
    D300_CALIB_DURATION_SEAWATER = 6

MSG_NAME_BY_SRC = {2: 'SCALED_PRESSURE2', 3: 'SCALED_PRESSURE3'}
MSG_ID_BY_SRC   = {2: 137,                  3: 142}

def request_interval(master, msg_id, hz=10):
    """İlgili mesaj için yayın aralığı talep et"""
    try:
        interval_us = int(1_000_000 / max(1, hz))
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, msg_id, interval_us, 0, 0, 0, 0, 0
        )
    except Exception as e:
        print(f"⚠️ Yayın aralığı talebi başarısız: {e}")

def calibrate_surface_seawater(master, msg_name, seconds, pmin, pmax):
    """Deniz suyu için havada kalibrasyon (su yüzeyinde tutmadan)"""
    if seconds <= 0:
        return None
    print(f"🌊 DENİZ SUYU kalibrasyonu: {seconds} sn – sensör havada (deniz suyu ρ=1025 kg/m³)")
    samples = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        msg = master.recv_match(type=msg_name, blocking=True, timeout=2.0)
        if msg is None:
            print("⚠️ Zaman aşımı – veri gelmedi.")
            continue
        try:
            p_mbar = float(msg.press_abs)  # hPa = mbar
        except Exception:
            continue
        if pmin <= p_mbar <= pmax:
            samples.append(p_mbar)
            print(f"  P: {p_mbar:7.2f} mbar  (örnek {len(samples)})")
    if len(samples) < max(10, seconds // 2):
        print("❌ Yetersiz örnek – kalibrasyon başarısız.")
        return None
    p0 = statistics.mean(samples)
    print(f"✅ P0 (havada kalibrasyon): {p0:.2f} mbar  | Örnek: {len(samples)}")
    print(f"ℹ️ Deniz suyu derinlik hesaplamaları için hazır (ρ=1025 kg/m³)")
    return p0

def main():
    ap = argparse.ArgumentParser(description="D300 Deniz Suyu Kalibrasyon Testi")
    ap.add_argument('--port', default='COM5', help='Seri port (Windows: COM5)')
    ap.add_argument('--baud', default=115200, type=int, help='Baud (115200/57600)')
    ap.add_argument('--src',  default=2, type=int, choices=[2,3],
                    help='D300 akış kaynağı: 2=SCALED_PRESSURE2, 3=SCALED_PRESSURE3')
    ap.add_argument('--hz',   default=10, type=int, help='İstenen yayın hızı (Hz)')

    # Deniz suyu parametreleri (sabit)
    rho = D300_SEAWATER_DENSITY  # 1025 kg/m³
    g = D300_GRAVITY             # 9.81 m/s²
    
    ap.add_argument('--p0',   type=float, help='Yüzey basıncı mbar (kalibrasyon yerine doğrudan ver)')

    # Basit filtreler
    ap.add_argument('--pmin', default=700.0, type=float, help='Geçerli min basınç (mbar)')
    ap.add_argument('--pmax', default=1200.0, type=float, help='Geçerli max basınç (mbar)')
    ap.add_argument('--tmin', default=-5.0,  type=float, help='Geçerli min sıcaklık (°C)')
    ap.add_argument('--tmax', default=60.0,  type=float, help='Geçerli max sıcaklık (°C)')
    ap.add_argument('--median', default=5, type=int, help='Basınç medyan penceresi (3/5/7)')
    ap.add_argument('--calib', default=D300_CALIB_DURATION_SEAWATER, type=int, 
                    help=f'Havada kalibrasyon süresi (sn). Varsayılan: {D300_CALIB_DURATION_SEAWATER}')
    args = ap.parse_args()

    msg_name = MSG_NAME_BY_SRC[args.src]
    msg_id   = MSG_ID_BY_SRC[args.src]

    print(f"🔌 Pixhawk bağlanıyor: {args.port} @ {args.baud} | Kaynak: {msg_name}")
    print(f"🌊 Deniz suyu modu: ρ={rho} kg/m³, g={g} m/s²")
    
    try:
        m = mavutil.mavlink_connection(args.port, baud=args.baud)
        m.wait_heartbeat(timeout=5)
        print(f"✅ Heartbeat alındı (SYS={m.target_system}, COMP={m.target_component})")
    except Exception as e:
        print(f"❌ Bağlantı hatası: {e}")
        return 1

    # Yalnızca seçili akışın hızını iste
    request_interval(m, msg_id, hz=args.hz)

    # P0 belirle (havada kalibrasyon)
    p0 = args.p0 if args.p0 is not None else None
    if p0 is None and args.calib > 0:
        p0 = calibrate_surface_seawater(m, msg_name, args.calib, args.pmin, args.pmax)
    if p0 is None:
        print("⚠️ P0 verilmedi/kalibre edilmedi; 1013.25 mbar ile devam ediliyor.")
        p0 = 1013.25

    # Medyan kuyruğu
    from collections import deque
    q = deque(maxlen=max(1, args.median))

    print("📡 Canlı derinlik - DENİZ SUYU (Ctrl+C ile durdur)")
    print(f"🌊 Su yoğunluğu: {rho} kg/m³ (deniz suyu)")
    try:
        while True:
            # Sadece seçilen mesaj tipini dinle
            msg = m.recv_match(type=msg_name, blocking=True, timeout=3.0)
            if msg is None:
                print("⚠️ Zaman aşımı: D300 verisi gelmedi.")
                continue

            # Basınç & sıcaklık oku
            try:
                p_mbar = float(msg.press_abs)  # hPa = mbar
            except Exception:
                continue

            t_c = None
            try:
                t_c = float(msg.temperature) / 100.0
            except Exception:
                pass

            # Uygunsuz değerleri ele
            if not (args.pmin <= p_mbar <= args.pmax):
                continue
            if (t_c is not None) and not (args.tmin <= t_c <= args.tmax):
                continue

            # Medyan filtre ve derinlik
            q.append(p_mbar)
            p_med = statistics.median(q)
            depth_m = ( (p_med - p0) * 100.0 ) / (rho * g)
            if depth_m < 0:
                depth_m = 0.0

            # Sıcaklık bilgisi varsa göster
            temp_str = f", Sıcaklık: {t_c:.1f}°C" if t_c is not None else ""
            print(f"Derinlik: {depth_m:6.3f} m | Basınç: {p_med:.1f} mbar{temp_str}")

    except KeyboardInterrupt:
        print("\n🛑 Durduruldu.")
    finally:
        try:
            m.close()
        except Exception:
            pass
        print("🔌 Bağlantı kapatıldı.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
