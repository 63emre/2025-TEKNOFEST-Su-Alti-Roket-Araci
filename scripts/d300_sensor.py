from pymavlink import mavutil

CONN = "/dev/ttyACM0"   
BAUD = 115200

SEA_LEVEL_PRESSURE_PA = 101325.0
FLUID_DENSITY = 997.0   
G = 9.80665

def to_depth_m(press_hpa: float) -> float:
    pa = press_hpa * 100.0
    dp = max(0.0, pa - SEA_LEVEL_PRESSURE_PA)  
    print(pa)
    return dp / (FLUID_DENSITY * G)

def main():
    print("maine geldim")
    m = mavutil.mavlink_connection(CONN, baud=BAUD)
    print("mav connection bakiyorum")
    if not m.wait_heartbeat(timeout=10):
        print("Heartbeat yok — bağlantıyı/portu kontrol edin.")
        return

    print("heartbeatim var")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2,
        int(1e6/5), 0, 0, 0, 0, 0
    )

    print("data almak için sorgu")

    while True:
        print("while'a girdim")
        msg = m.recv_match(type="SCALED_PRESSURE2", blocking=True, timeout=2)
        if not msg:
            print("mesaj bos veya gelmiyor")
            continue
        press_hpa = float(getattr(msg, "press_abs", 0.0))   
        temp_c    = float(getattr(msg, "temperature", 0)) / 100.0
        depth_m   = to_depth_m(press_hpa)
        print(f"Derinlik: {depth_m:.2f} m | Sıcaklık: {temp_c:.2f} °C")

if __name__ == "__main__":
    main()
