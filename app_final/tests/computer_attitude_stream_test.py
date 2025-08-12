#!/usr/bin/env python3
"""
Computer-side attitude stream test (no D300 dependency).
Prints ATTITUDE (roll/pitch/yaw in deg) at ~5Hz to verify data pipeline.
"""
import time
from common import load_config, SerialMAV


def main():
    cfg = load_config()
    mav = SerialMAV(cfg)
    assert mav.connect(), 'MAVLink connect failed'
    try:
        t0 = time.time()
        while time.time() - t0 < 30.0:
            att = mav.get_attitude()
            if att:
                r, p, y = att
                rd = r * 180.0 / 3.14159
                pd = p * 180.0 / 3.14159
                yd = y * 180.0 / 3.14159
                print(f'ATT: roll={rd:+6.1f} pitch={pd:+6.1f} yaw={yd:+6.1f}')
            time.sleep(0.2)
    finally:
        pass


if __name__ == '__main__':
    main()


