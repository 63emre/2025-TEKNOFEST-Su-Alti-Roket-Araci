#!/usr/bin/env python3
# Pixhawk 2.4.8 — MAIN OUT 1 (SERVO1) normal PWM ramp + doğrulama (BASİT)
# /dev/ttyACM0 üzerinden bağlanır, DO_SET_SERVO ile PWM verir.
# Her adımda SERVO_OUTPUT_RAW'ı okuyup komutun uygulandığını teyit eder.

import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_NUM = 11        # MAIN OUT 1 -> SERVO1
PWM_MIN   = 1100
PWM_MAX   = 1100
STEP      = 10       # µs
DELAY_S   = 0.1      # adımlar arası bekleme
VERIFY_TOL = 30      # µs tolerans
MSG_ID_SERVO_OUTPUT_RAW = 36  # SERVO_OUTPUT_RAW

def set_msg_interval(m, msg_id, hz):
    interval_us = int(1e6 / hz)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        float(msg_id), float(interval_us), 0,0,0,0,0
    )

def set_servo(m, servo_num, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(servo_num), float(pwm), 0,0,0,0,0
    )

def wait_ack_or_output(m, target_pwm, timeout=1.5):
    """Komutun ulaşıp ulaşmadığını anlamak için:
       - COMMAND_ACK (opsiyonel) bekle
       - SERVO_OUTPUT_RAW içinden servo1_raw'ı kontrol et
    """
    t0 = time.time()
    got_ack = False
    got_match = False
    last_pwm = None

    while time.time() - t0 < timeout:
        msg = m.recv_match(type=['COMMAND_ACK', 'SERVO_OUTPUT_RAW'], blocking=False)
        if not msg:
            time.sleep(0.01)
            continue

        if msg.get_type() == 'COMMAND_ACK':
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_SERVO:
                got_ack = True

        elif msg.get_type() == 'SERVO_OUTPUT_RAW':
            # SERVO1 ana çıkış olduğundan 'servo1_raw' alanını kontrol ediyoruz
            last_pwm = getattr(msg, 'servo1_raw', None)
            if last_pwm is not None and abs(int(last_pwm) - int(target_pwm)) <= VERIFY_TOL:
                got_match = True
                break

    return got_ack, got_match, last_pwm

def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    m.wait_heartbeat(timeout=10)

    # SERVO_OUTPUT_RAW'ı 5 Hz'te iste (doğrulama için)
    set_msg_interval(m, MSG_ID_SERVO_OUTPUT_RAW, hz=5)

    try:
        # Yukarı ramp
        for pwm in range(PWM_MIN, PWM_MAX + 1, STEP):
            print(f"SERVO{SERVO_NUM} -> {pwm}us", end="", flush=True)
            set_servo(m, SERVO_NUM, pwm)
            ack, match, last = wait_ack_or_output(m, pwm, timeout=1.0)
            if match:
                print("  [OK: output güncellendi]")
            elif ack:
                print(f"  [ACK var, output teyitsiz; son={last}]")
            else:
                print(f"  [UYARI: ACK yok, output teyitsiz; son={last}]")
            time.sleep(DELAY_S)

        time.sleep(0.5)

        # Aşağı ramp
        for pwm in range(PWM_MAX, PWM_MIN - 1, -STEP):
            print(f"SERVO{SERVO_NUM} -> {pwm}us", end="", flush=True)
            set_servo(m, SERVO_NUM, pwm)
            ack, match, last = wait_ack_or_output(m, pwm, timeout=1.0)
            if match:
                print("  [OK: output güncellendi]")
            elif ack:
                print(f"  [ACK var, output teyitsiz; son={last}]")
            else:
                print(f"  [UYARI: ACK yok, output teyitsiz; son={last}]")
            time.sleep(DELAY_S)

    except KeyboardInterrupt:
        pass
    finally:
        # Güvenli bitiş: min'e dön
        set_servo(m, SERVO_NUM, PWM_MIN)
        print("Bitti, min PWM verildi.")

if __name__ == "__main__":
    main()
