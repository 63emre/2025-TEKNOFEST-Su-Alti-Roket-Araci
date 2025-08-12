#!/usr/bin/env python3
import argparse
import time
import sys
try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm, D300Sensor


def setup_gpio(trigger_pin: int):
    if not GPIO_AVAILABLE:
        print('⚠️ RPi.GPIO not available, running in SIM mode (no firing)')
        return
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(trigger_pin, GPIO.OUT, initial=GPIO.LOW)


def fire_capsule(trigger_pin: int, arm_s: float, fire_s: float):
    if not GPIO_AVAILABLE:
        print('SIM FIRE: would ARM for %.1fs then FIRE for %.1fs' % (arm_s, fire_s))
        time.sleep(arm_s + fire_s)
        return
    # ARM window (e.g., enable driver)
    GPIO.output(trigger_pin, GPIO.HIGH)
    time.sleep(arm_s)
    # Short FIRE pulse can be the same line or a separate high-current driver stage.
    # Here we keep HIGH for total (arm+fire) as a simple interface.
    time.sleep(fire_s)
    GPIO.output(trigger_pin, GPIO.LOW)


def run_mission(launch_angle_deg: float, arm_seconds: float, fire_seconds: float, confirm: bool):
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    trig_pin = cfg['rocket']['trigger_gpio_bcm']
    setup_gpio(trig_pin)

    try:
        # 1) D300 ile yüzeye yaklaş (hedef ~0.3m)
        d300 = D300Sensor(cfg)
        d300.connect()
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

        # Set launch attitude (+pitch)
        steps = int(abs(launch_angle_deg) / 5) or 1
        pitch_sign = 1 if launch_angle_deg >= 0 else -1
        for _ in range(steps):
            xwing_mix_and_send(mav, cfg, 0, -5.0 * pitch_sign, 0)
            time.sleep(0.3)
        xwing_mix_and_send(mav, cfg, 0, 0, 0)

        # 2) Fire sequence
        if confirm:
            ans = input('Roket fırlatma ONAY? (YES): ').strip()
            if ans != 'YES':
                print('İptal edildi')
                return

        print('Arming firing line...')
        fire_capsule(trig_pin, arm_seconds, fire_seconds)
        print('FIRE sequence complete')

        # 3) Safe settle (neutral controls)
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.set_servo_pwm(8 + cfg['pixhawk']['servo_channels']['motor'], cfg['pixhawk']['pwm']['neutral'])
        time.sleep(1.0)

    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()
        if GPIO_AVAILABLE:
            GPIO.cleanup()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--launch-angle', type=float, default=30.0)
    p.add_argument('--arm-seconds', type=float, default=None)
    p.add_argument('--fire-seconds', type=float, default=None)
    p.add_argument('--no-confirm', action='store_true')
    args = p.parse_args()

    cfg = load_config()
    arm_s = args.arm_seconds if args.arm_seconds is not None else cfg['rocket']['arm_seconds']
    fire_s = args.fire_seconds if args.fire_seconds is not None else cfg['rocket']['fire_seconds']

    run_mission(args.launch_angle, arm_s, fire_s, confirm=not args.no_confirm)


if __name__ == '__main__':
    main()


