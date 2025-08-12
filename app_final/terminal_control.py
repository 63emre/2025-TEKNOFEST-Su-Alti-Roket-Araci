#!/usr/bin/env python3
import threading
import time
import sys

from common import load_config, SerialMAV, xwing_mix_and_send, motor_percent_to_pwm


class PID:
    def __init__(self, kp=100.0, ki=10.0, kd=20.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i = 0.0
        self.prev_e = 0.0
        self.last_t = time.time()

    def update(self, target, current):
        t = time.time()
        dt = max(1e-3, t - self.last_t)
        self.last_t = t
        e = target - current
        self.i += e * dt
        d = (e - self.prev_e) / dt
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * d


class Controller:
    def __init__(self):
        self.cfg = load_config()
        self.mav = SerialMAV(self.cfg)
        assert self.mav.connect(), 'MAVLink connect failed'
        assert self.mav.arm(), 'ARM failed'
        p = self.cfg['pid']
        self.pid_roll = PID(**p['roll'])
        self.pid_pitch = PID(**p['pitch'])
        self.pid_yaw = PID(**p['yaw'])
        self.targets = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def loop(self):
        while self.running:
            att = self.mav.get_attitude()
            if att:
                roll_rad, pitch_rad, yaw_rad = att
                roll = roll_rad * 180.0 / 3.14159
                pitch = pitch_rad * 180.0 / 3.14159
                yaw = yaw_rad * 180.0 / 3.14159
                u_roll = self.pid_roll.update(self.targets['roll'], roll)
                u_pitch = self.pid_pitch.update(self.targets['pitch'], pitch)
                u_yaw = self.pid_yaw.update(self.targets['yaw'], yaw)
                xwing_mix_and_send(self.mav, self.cfg, u_roll, u_pitch, u_yaw)
            time.sleep(0.02)

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        xwing_mix_and_send(self.mav, self.cfg, 0, 0, 0)
        self.mav.disarm()


def main():
    ctl = Controller()
    print('Terminal Control Ready. Commands:')
    print('- arm | disarm')
    print('- roll <deg> | pitch <deg> | yaw <deg>')
    print('- motor <percent>')
    print('- pid <axis> <kp> <ki> <kd>  (axis: roll|pitch|yaw)')
    print('- pid save  (persist to config.json)')
    print('- status | quit')

    motor_ch = 8 + ctl.cfg['pixhawk']['servo_channels']['motor']
    neutral = ctl.cfg['pixhawk']['pwm']['neutral']

    try:
        while True:
            try:
                line = input('> ').strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd == 'quit':
                break
            elif cmd == 'status':
                print('Targets:', ctl.targets)
                print('PID roll:', ctl.pid_roll.kp, ctl.pid_roll.ki, ctl.pid_roll.kd)
                print('PID pitch:', ctl.pid_pitch.kp, ctl.pid_pitch.ki, ctl.pid_pitch.kd)
                print('PID yaw:', ctl.pid_yaw.kp, ctl.pid_yaw.ki, ctl.pid_yaw.kd)
            elif cmd == 'arm':
                ctl.mav.arm()
            elif cmd == 'disarm':
                ctl.mav.disarm()
            elif cmd in ('roll', 'pitch', 'yaw') and len(parts) == 2:
                try:
                    val = float(parts[1])
                    ctl.targets[cmd] = val
                    print(f'{cmd} target → {val} deg')
                except ValueError:
                    print('number required')
            elif cmd == 'motor' and len(parts) == 2:
                try:
                    pct = float(parts[1])
                    pwm = motor_percent_to_pwm(ctl.cfg, pct)
                    ctl.mav.set_servo_pwm(motor_ch, pwm)
                except ValueError:
                    print('number required')
            elif cmd == 'pid' and len(parts) >= 2:
                if parts[1] == 'save':
                    import json, os
                    with open(os.path.join(os.path.dirname(__file__), 'config.json'), 'r+', encoding='utf-8') as f:
                        data = json.load(f)
                        data['pid']['roll'] = {"kp": ctl.pid_roll.kp, "ki": ctl.pid_roll.ki, "kd": ctl.pid_roll.kd}
                        data['pid']['pitch'] = {"kp": ctl.pid_pitch.kp, "ki": ctl.pid_pitch.ki, "kd": ctl.pid_pitch.kd}
                        data['pid']['yaw'] = {"kp": ctl.pid_yaw.kp, "ki": ctl.pid_yaw.ki, "kd": ctl.pid_yaw.kd}
                        f.seek(0)
                        json.dump(data, f, indent=2)
                        f.truncate()
                    print('PID saved to config.json')
                elif len(parts) == 6:
                    axis = parts[1]
                    try:
                        kp, ki, kd = float(parts[2]), float(parts[3]), float(parts[4])
                        kd = float(parts[5])
                        if axis == 'roll':
                            ctl.pid_roll.kp, ctl.pid_roll.ki, ctl.pid_roll.kd = kp, ki, kd
                        elif axis == 'pitch':
                            ctl.pid_pitch.kp, ctl.pid_pitch.ki, ctl.pid_pitch.kd = kp, ki, kd
                        elif axis == 'yaw':
                            ctl.pid_yaw.kp, ctl.pid_yaw.ki, ctl.pid_yaw.kd = kp, ki, kd
                        else:
                            print('axis must be roll|pitch|yaw')
                            continue
                        print(f'PID {axis} updated to kp={kp} ki={ki} kd={kd}')
                    except ValueError:
                        print('numbers required')
                else:
                    print('Usage: pid <axis> <kp> <ki> <kd> | pid save')
            else:
                print('Unknown command')
    finally:
        ctl.mav.set_servo_pwm(motor_ch, neutral)
        ctl.stop()


if __name__ == '__main__':
    main()


