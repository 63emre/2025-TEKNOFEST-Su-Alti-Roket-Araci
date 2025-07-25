from pymavlink import mavutil
import time

MOTOR_CH = 11  # AUX3
master = mavutil.mavlink_connection('tcp:127.0.0.1:5777')
master.wait_heartbeat()

while True:
    user_input = input("Saniye (q=çıkış): ").strip()
    if user_input.lower() == 'q':
        break
        
    try:
        duration = float(user_input)
    except:
        continue
        
    pwm_input = input("PWM (1100-1900): ").strip()
    try:
        pwm = int(pwm_input)
        pwm = max(1100, min(1900, pwm))
    except:
        continue
    
    print(f"Motor çalıyor: {duration}s, PWM: {pwm}")
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, MOTOR_CH, pwm, 0, 0, 0, 0, 0
    )
    
    time.sleep(duration)
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0, MOTOR_CH, 1500, 0, 0, 0, 0, 0
    )

master.close()
