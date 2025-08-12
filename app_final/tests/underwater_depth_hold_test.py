#!/usr/bin/env python3
"""
Underwater depth hold smoke test using D300 at 0x76.
Targets a depth and maintains within ±0.2m using simple P control on pitch + small motor bias.
"""
import argparse
import time
from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--depth', type=float, default=2.0)
    p.add_argument('--duration', type=float, default=20.0)
    args = p.parse_args()

    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    d300 = D300Sensor(cfg)
    d300.connect()

    target = max(0.3, args.depth)
    end_ts = time.time() + max(5.0, args.duration)
    try:
        while time.time() < end_ts:
            data = d300.get() if d300.available else None
            depth_now = (data or {}).get('depth_m', None)
            if depth_now is None:
                # no sensor; do open loop settle
                xwing_mix_and_send(mav, cfg, 0, +8.0, 0)
                mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, 10))
                time.sleep(0.5)
                xwing_mix_and_send(mav, cfg, 0, 0, 0)
                continue
            err = target - depth_now
            pitch_cmd = max(-12.0, min(12.0, err * 6.0))
            xwing_mix_and_send(mav, cfg, 0, +pitch_cmd, 0)
            # small forward to keep water flow
            mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, 8))
            time.sleep(0.2)
        # neutral
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])
    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


if __name__ == '__main__':
    main()


