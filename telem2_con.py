from pymavlink import mavutil

# TELEM2, 115200 baud
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

master.wait_heartbeat()
print(f"✓ Bağlandım: SYS={master.target_system} COMP={master.target_component}")

# Örnek komut: araç ARM
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0,0,0,0,0,0
)
