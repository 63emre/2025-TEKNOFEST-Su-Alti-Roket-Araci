#!/usr/bin/env python3
import time
from common import load_config, SerialMAV, motor_percent_to_pwm


def main():
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    mav.arm()
    motor_ch = 8 + cfg['pixhawk']['servo_channels']['motor']
    neutral = cfg['pixhawk']['pwm']['neutral']

    try:
        for pct in [0, 20, 40, 20, 0, -15, 0]:
            pwm = motor_percent_to_pwm(cfg, pct)
            print(f'Motor {pct}% → {pwm}us')
            mav.set_servo_pwm(motor_ch, pwm)
            time.sleep(2.0)
    finally:
        mav.set_servo_pwm(motor_ch, neutral)
        mav.disarm()


if __name__ == '__main__':
    main()


