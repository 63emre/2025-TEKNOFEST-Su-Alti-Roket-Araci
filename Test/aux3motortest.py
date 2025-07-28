from pymavlink import mavutil
import time
import os

MOTOR_CH = 11  # AUX3

# Serial MAVLink connection with environment variable support
serial_port = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
baud_rate = int(os.getenv("MAV_BAUD", "115200"))

print(f"🔌 Serial bağlantısı: {serial_port} @ {baud_rate} baud")
master = mavutil.mavlink_connection(serial_port, baud=baud_rate, autoreconnect=True)
print("💓 Heartbeat bekleniyor...")
master.wait_heartbeat(timeout=15)
print("✅ MAVLink bağlantısı kuruldu")

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
