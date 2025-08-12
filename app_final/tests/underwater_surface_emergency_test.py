#!/usr/bin/env python3
"""
Underwater surface emergency test using D300.
From current depth, try to reach near-surface (<=0.3m) safely.
"""
import time
from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


def main():
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    d300 = D300Sensor(cfg)
    d300.connect()

    try:
        start_ts = time.time()
        timeout = 60.0
        while time.time() - start_ts < timeout:
            data = d300.get() if d300.available else None
            depth_now = (data or {}).get('depth_m', None)
            if depth_now is not None and depth_now <= 0.3:
                break
            xwing_mix_and_send(mav, cfg, 0, -10.0, 0)
            mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, -12))
            time.sleep(0.3)
        # neutral
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])
    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


if __name__ == '__main__':
    main()


