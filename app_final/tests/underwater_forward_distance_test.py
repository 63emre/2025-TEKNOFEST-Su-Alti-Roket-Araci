#!/usr/bin/env python3
"""
Underwater forward distance test at target depth using D300.
1) Go to target depth (closed-loop if D300 available)
2) Move forward for given meters at given speed estimate
3) Neutralize and stop
"""
import argparse
import time
from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--depth', type=float, default=2.0)
    p.add_argument('--meters', type=float, default=10.0)
    p.add_argument('--speed', type=float, default=0.5, help='m/s estimate')
    args = p.parse_args()

    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    d300 = D300Sensor(cfg)
    d300.connect()

    try:
        # Depth acquire
        target = max(0.3, args.depth)
        start = time.time()
        while time.time() - start < 60.0:
            data = d300.get() if d300.available else None
            depth_now = (data or {}).get('depth_m', None)
            if depth_now is None:
                # open loop descend a bit
                xwing_mix_and_send(mav, cfg, 0, +10.0, 0)
                mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, 12))
                time.sleep(2.0)
                break
            err = target - depth_now
            if abs(err) < 0.15:
                break
            xwing_mix_and_send(mav, cfg, 0, max(-12.0, min(12.0, err * 6.0)), 0)
            mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, 10))
            time.sleep(0.2)
        xwing_mix_and_send(mav, cfg, 0, 0, 0)

        # Forward
        base_speed = 0.5
        base_power = 20.0
        power = max(10.0, min(70.0, base_power * (args.speed / base_speed)))
        duration = max(2.0, args.meters / max(0.2, args.speed))
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, power))
        time.sleep(duration)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])

    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


if __name__ == '__main__':
    main()


