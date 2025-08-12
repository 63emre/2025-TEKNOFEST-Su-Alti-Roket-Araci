#!/usr/bin/env python3
"""
Underwater yaw rotation test at target depth using fins only.
Steps yaw by chunks and holds neutral between.
"""
import argparse
import time
from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--depth', type=float, default=2.0)
    p.add_argument('--degrees', type=float, default=90.0)
    args = p.parse_args()

    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    d300 = D300Sensor(cfg)
    d300.connect()

    try:
        # Depth acquire (same style)
        target = max(0.3, args.depth)
        start = time.time()
        while time.time() - start < 60.0:
            data = d300.get() if d300.available else None
            depth_now = (data or {}).get('depth_m', None)
            if depth_now is None:
                xwing_mix_and_send(mav, cfg, 0, +10.0, 0)
                time.sleep(1.0)
                break
            err = target - depth_now
            if abs(err) < 0.15:
                break
            xwing_mix_and_send(mav, cfg, 0, max(-12.0, min(12.0, err * 6.0)), 0)
            time.sleep(0.2)
        xwing_mix_and_send(mav, cfg, 0, 0, 0)

        # Yaw rotate in place using fins only
        sign = 1 if args.degrees >= 0 else -1
        remaining = abs(args.degrees)
        chunk = 10.0
        while remaining > 0:
            mag = min(chunk, remaining)
            xwing_mix_and_send(mav, cfg, 0, 0, sign * mag)
            time.sleep(0.25)
            xwing_mix_and_send(mav, cfg, 0, 0, 0)
            time.sleep(0.1)
            remaining -= mag
    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


if __name__ == '__main__':
    main()


