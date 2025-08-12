#!/usr/bin/env python3
import time
from common import load_config, SerialMAV


def main():
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    mav.arm()
    ch_map = cfg['pixhawk']['servo_channels']
    pwm = cfg['pixhawk']['pwm']

    try:
        for name in ['fin_front_left', 'fin_front_right', 'fin_rear_left', 'fin_rear_right']:
            ch = 8 + ch_map[name]
            print(f'Testing {name} (AUX{ch_map[name]})')
            for val in [pwm['neutral'], pwm['min'] + 100, pwm['max'] - 100, pwm['neutral']]:
                mav.set_servo_pwm(ch, val)
                time.sleep(1.0)
    finally:
        # Neutral all
        for name in ['fin_front_left', 'fin_front_right', 'fin_rear_left', 'fin_rear_right']:
            ch = 8 + ch_map[name]
            mav.set_servo_pwm(ch, pwm['neutral'])
        mav.disarm()


if __name__ == '__main__':
    main()


