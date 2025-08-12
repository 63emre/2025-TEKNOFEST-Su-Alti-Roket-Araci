#!/usr/bin/env python3
"""
Computer-side MAVLink link test.
Connects, waits heartbeat, sends a neutral servo command and exits.
Use this to confirm serial link before water tests.
"""
from common import load_config, SerialMAV


def main():
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    # send a benign command to AUX1 neutral
    ch = 8 + cfg['pixhawk']['servo_channels']['fin_front_left']
    neutral = cfg['pixhawk']['pwm']['neutral']
    mav.set_servo_pwm(ch, neutral)
    print('MAVLink OK and command accepted (no error)')


if __name__ == '__main__':
    main()


