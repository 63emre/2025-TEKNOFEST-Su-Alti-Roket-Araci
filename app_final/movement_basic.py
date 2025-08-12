#!/usr/bin/env python3
import argparse
import time
from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm


def forward(mav: SerialMAV, cfg: dict, meters: float, power_percent: float = 20.0):
    pwm = motor_percent_to_pwm(cfg, power_percent)
    duration = max(0.5, meters / 0.5)  # conservative speed model: 0.5 m/s
    mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], pwm)
    time.sleep(duration)
    mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])


def turn_left(mav: SerialMAV, cfg: dict, degrees: float):
    # yaw positive → left in our convention
    step = 10.0
    left = max(1, int(abs(degrees) / step))
    for _ in range(left):
        xwing_mix_and_send(mav, cfg, roll=0, pitch=0, yaw=step)
        time.sleep(0.2)
    # neutral
    xwing_mix_and_send(mav, cfg, 0, 0, 0)


def rotate_in_place(mav: SerialMAV, cfg: dict, degrees: float):
    # Use fins only (no motor) to yaw
    sign = 1 if degrees >= 0 else -1
    remaining = abs(degrees)
    chunk = 10.0
    while remaining > 0:
        mag = min(chunk, remaining)
        xwing_mix_and_send(mav, cfg, roll=0, pitch=0, yaw=sign * mag)
        time.sleep(0.25)
        remaining -= mag
    xwing_mix_and_send(mav, cfg, 0, 0, 0)


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='cmd', required=True)

    p_fwd = sub.add_parser('forward')
    p_fwd.add_argument('--meters', type=float, required=True)
    p_fwd.add_argument('--power', type=float, default=20.0)

    p_left = sub.add_parser('turn-left')
    p_left.add_argument('--degrees', type=float, required=True)

    p_rot = sub.add_parser('rotate')
    p_rot.add_argument('--degrees', type=float, required=True)

    args = parser.parse_args()
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    mav.arm()

    try:
        if args.cmd == 'forward':
            forward(mav, cfg, args.meters, args.power)
        elif args.cmd == 'turn-left':
            turn_left(mav, cfg, args.degrees)
        elif args.cmd == 'rotate':
            rotate_in_place(mav, cfg, args.degrees)
    finally:
        # Motor stop and fins neutral
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


if __name__ == '__main__':
    main()


