import time
from pymavlink import mavutil

PORT = "/dev/ttyACM0" # raspi USB portu
BAUD = 115200
SERVOS = [11, 12, 13, 14]      # AUX3, AUX4, AUX5, AUX6
SWEEP  = [1100, 1500, 1900, 1500]
DELAY  = 0.8  

m = mavutil.mavlink_connection(PORT, baud=BAUD)
m.wait_heartbeat(timeout=10)
print(f"Heartbeat'im var! {PORT} üzerinden. \n{m}")

def set_servo(servo_num, pwm):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
        float(servo_num), float(pwm), 0,0,0,0,0
    )

for s in SERVOS:
    print(f"\n== SERVO{s} ==")
    for pwm in SWEEP:
        print(f"SERVO{s} -> {pwm}")
        set_servo(s, pwm)
        time.sleep(DELAY)

