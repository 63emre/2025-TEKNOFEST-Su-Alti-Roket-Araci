#!/usr/bin/env python3
import argparse
import math
import time
from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


def run_mission(distance_m: float, depth_m: float, speed_mps: float):
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    try:
        # D300 entegrasyonu
        d300 = D300Sensor(cfg)
        d300.connect()

        # 1) Derinliğe in (kapalı çevrim D300 varsa)
        target_depth = max(0.3, depth_m)
        start_ts = time.time()
        timeout = 60.0
        while time.time() - start_ts < timeout:
            data = d300.get() if d300.available else None
            current_depth = (data or {}).get('depth_m', None)

            if current_depth is None:
                # Basit açık çevrim düşüş
                xwing_mix_and_send(mav, cfg, 0, +10.0, 0)
                mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, 15))
                time.sleep(0.5)
                xwing_mix_and_send(mav, cfg, 0, 0, 0)
                break
            else:
                err = target_depth - current_depth
                if abs(err) < 0.15:
                    xwing_mix_and_send(mav, cfg, 0, 0, 0)
                    break
                pitch_cmd = max(-12.0, min(12.0, err * 6.0))  # küçük P kontrol
                xwing_mix_and_send(mav, cfg, 0, +pitch_cmd, 0)
                mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, 12))
                time.sleep(0.2)
        xwing_mix_and_send(mav, cfg, 0, 0, 0)

        # 2) Move straight distance at requested speed
        # Use motor power estimation: 0.5 m/s at 20% → scale linearly
        base_speed = 0.5
        base_power = 20.0
        power = max(10.0, min(70.0, base_power * (speed_mps / base_speed)))
        travel_time = max(2.0, distance_m / max(0.2, speed_mps))
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, power))
        time.sleep(travel_time)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])

        # 3) Turn back (approx 180°)
        for _ in range(18):
            xwing_mix_and_send(mav, cfg, 0, 0, 10.0)
            time.sleep(0.25)
        xwing_mix_and_send(mav, cfg, 0, 0, 0)

        # 4) Go back same distance
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], motor_percent_to_pwm(cfg, power))
        time.sleep(travel_time)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])

        # 5) Yüzeye çıkış (D300 ile kapalı çevrim)
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
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])

    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--distance', type=float, default=50.0)
    p.add_argument('--depth', type=float, default=2.0)
    p.add_argument('--speed', type=float, default=0.5, help='m/s')
    args = p.parse_args()
    run_mission(args.distance, args.depth, args.speed)


if __name__ == '__main__':
    main()


