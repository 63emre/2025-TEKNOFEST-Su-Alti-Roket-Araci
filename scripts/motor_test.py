import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0"
BAUD = 115200

SERVO_NUM = 11          # MAIN OUT 1 -> SERVO1
MIN_PWM   = 1500       # alt sınır (µs)
MAX_PWM   = 1900       # üst sınır (µs)
STEP      = 10        # artış miktarı (µs)
DELAY_S   = 0.1        # adımlar arası bekleme (s)

def set_servo(master, pwm):
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(SERVO_NUM), float(pwm), 0,0,0,0,0
    )

def main():
    m = mavutil.mavlink_connection(PORT, baud=BAUD)
    m.wait_heartbeat(timeout=10)

    print(f"RAMP YUKARI: SERVO{SERVO_NUM} {MIN_PWM}->{MAX_PWM}")
    for pwm in range(MIN_PWM, MAX_PWM + 1, STEP):
        print(f"{pwm} µs")
        set_servo(m, pwm)
        time.sleep(DELAY_S)

    time.sleep(1.0)

    print(f"RAMP AŞAĞI: SERVO{SERVO_NUM} {MAX_PWM}->{MIN_PWM}")
    for pwm in range(MAX_PWM, MIN_PWM - 1, -STEP):
        print(f"{pwm} µs")
        set_servo(m, pwm)
        time.sleep(DELAY_S)

    set_servo(m, 0)

    print("Bitti.")

if __name__ == "__main__":
    main()
