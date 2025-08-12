#!/usr/bin/env python3
"""
Simple online PID tuner for roll/pitch/yaw using step-response search.

Strategy:
1) For selected axis, apply a small step target (e.g., +10°), record response (overshoot, rise time).
2) Iterate candidate (kp, ki, kd) triplets from a safe grid around current values.
3) Score response (lower overshoot, faster rise, low settling error) and pick best.
4) Optionally save to config.json.

Note: This is conservative; run in a safe environment, with tether and low motor power.
"""
import argparse
import json
import os
import time

from common import load_config, SerialMAV, xwing_mix_and_send


class PID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.prev_e = 0.0
        self.last_t = time.time()

    def update(self, sp, pv):
        t = time.time()
        dt = max(1e-3, t - self.last_t)
        self.last_t = t
        e = sp - pv
        self.i += e * dt
        d = (e - self.prev_e) / dt
        self.prev_e = e
        return self.kp * e + self.ki * self.i + self.kd * d


def measure_axis_response(axis: str, step_deg: float, duration: float, pid: PID, mav: SerialMAV, cfg: dict):
    start = time.time()
    series = []  # (t, pv, u)
    target = step_deg
    while time.time() - start < duration:
        att = mav.get_attitude()
        if att:
            roll, pitch, yaw = [v * 180.0 / 3.14159 for v in att]
            pv = roll if axis == 'roll' else pitch if axis == 'pitch' else yaw
            u = pid.update(target, pv)
            # apply only selected axis
            r = u if axis == 'roll' else 0.0
            p = u if axis == 'pitch' else 0.0
            y = u if axis == 'yaw' else 0.0
            xwing_mix_and_send(mav, cfg, r, p, y)
            series.append((time.time() - start, pv, u))
        time.sleep(0.02)
    # neutral
    xwing_mix_and_send(mav, cfg, 0, 0, 0)
    return series


def score_response(series, target):
    if not series:
        return 1e9
    pv_vals = [pv for _, pv, _ in series]
    t_vals = [t for t, _, _ in series]
    # overshoot
    overshoot = max(0.0, (max(pv_vals) - target)) if target > 0 else max(0.0, (target - min(pv_vals)))
    # rise time ~ time to reach 90% of target
    thr = 0.9 * target
    rt = next((t for t, pv in zip(t_vals, pv_vals) if (pv >= thr if target > 0 else pv <= thr)), t_vals[-1])
    # final error
    final_err = abs(target - pv_vals[-1])
    # weighted sum
    return overshoot * 3.0 + rt * 1.0 + final_err * 2.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--axis', required=True, choices=['roll', 'pitch', 'yaw'])
    p.add_argument('--step', type=float, default=10.0)
    p.add_argument('--duration', type=float, default=5.0)
    p.add_argument('--grid', type=float, default=0.5, help='scale around current gains, e.g., 0.5=±50%')
    p.add_argument('--save', action='store_true', help='write best gains to config.json')
    args = p.parse_args()

    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    assert mav.arm(), 'ARM failed'

    pid_cfg = cfg['pid'][args.axis]
    base_kp, base_ki, base_kd = pid_cfg['kp'], pid_cfg['ki'], pid_cfg['kd']
    scales = [0.5 - args.grid, 0.75 - args.grid / 2, 1.0, 1.25 + args.grid / 2, 1.5 + args.grid]
    candidates = []
    for skp in scales:
        for ski in scales:
            for skd in scales:
                candidates.append((max(0.0, base_kp * skp), max(0.0, base_ki * ski), max(0.0, base_kd * skd)))

    try:
        best = None
        best_score = 1e9
        for (kp, ki, kd) in candidates:
            pid = PID(kp, ki, kd)
            series = measure_axis_response(args.axis, args.step, args.duration, pid, mav, cfg)
            s = score_response(series, args.step)
            print(f'cand kp={kp:.1f} ki={ki:.1f} kd={kd:.1f} -> score {s:.3f}')
            if s < best_score:
                best_score = s
                best = (kp, ki, kd)
        if best:
            print(f'BEST {args.axis}: kp={best[0]:.1f} ki={best[1]:.1f} kd={best[2]:.1f} (score={best_score:.3f})')
            if args.save:
                cfg['pid'][args.axis] = {"kp": best[0], "ki": best[1], "kd": best[2]}
                with open(os.path.join(os.path.dirname(__file__), 'config.json'), 'w', encoding='utf-8') as f:
                    json.dump(cfg, f, indent=2)
                print('Saved to config.json')
    finally:
        xwing_mix_and_send(mav, cfg, 0, 0, 0)
        mav.disarm()


if __name__ == '__main__':
    main()


