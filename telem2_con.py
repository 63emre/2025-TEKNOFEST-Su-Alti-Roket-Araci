from pymavlink import mavutil
import time

# TELEM2, 57600 baud - ÇALIŞAN BAĞLANTI
master = mavutil.mavlink_connection('/dev/serial0', baud=57600)

master.wait_heartbeat()
print(f"✓ Bağlandım: SYS={master.target_system} COMP={master.target_component}")

# Örnek komut: araç ARM
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0,0,0,0,0,0
)

print("🔍 Gelen MAVLink mesajlarını dinliyorum (10 saniye)...")
print("=" * 60)

start_time = time.time()
message_count = {}

try:
    while time.time() - start_time < 10:  # 10 saniye dinle
        msg = master.recv_match(blocking=False)
        if msg is not None:
            msg_type = msg.get_type()
            
            # Mesaj sayısını takip et
            if msg_type in message_count:
                message_count[msg_type] += 1
            else:
                message_count[msg_type] = 1
            
            # Önemli mesajları detaylı göster
            if msg_type == 'RAW_IMU':
                print(f"📊 RAW_IMU: ACC({msg.xacc/1000:.3f}, {msg.yacc/1000:.3f}, {msg.zacc/1000:.3f}) GYRO({msg.xgyro/1000:.3f}, {msg.ygyro/1000:.3f}, {msg.zgyro/1000:.3f})")
            elif msg_type == 'SCALED_IMU':
                print(f"📊 SCALED_IMU: ACC({msg.xacc/1000:.3f}, {msg.yacc/1000:.3f}, {msg.zacc/1000:.3f}) GYRO({msg.xgyro/1000:.3f}, {msg.ygyro/1000:.3f}, {msg.zgyro/1000:.3f})")
            elif msg_type == 'SCALED_PRESSURE':
                print(f"🌊 SCALED_PRESSURE: {msg.press_abs:.2f}mbar, {msg.temperature/100:.1f}°C")
            elif msg_type == 'SCALED_PRESSURE2':
                print(f"🌊 SCALED_PRESSURE2: {msg.press_abs:.2f}mbar, {msg.temperature/100:.1f}°C")
            elif msg_type == 'ATTITUDE':
                import math
                roll = math.degrees(msg.roll)
                pitch = math.degrees(msg.pitch)
                yaw = math.degrees(msg.yaw)
                print(f"🎯 ATTITUDE: Roll={roll:.1f}° Pitch={pitch:.1f}° Yaw={yaw:.1f}°")
            elif msg_type == 'GLOBAL_POSITION_INT':
                print(f"🌍 GPS: {msg.lat/1e7:.6f}, {msg.lon/1e7:.6f}, Alt={msg.alt/1000:.1f}m")
            elif msg_type == 'HEARTBEAT':
                armed = "ARMED" if (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) else "DISARMED"
                print(f"💓 HEARTBEAT: {armed}, Mode={msg.custom_mode}")
        
        time.sleep(0.01)  # Küçük delay

except KeyboardInterrupt:
    print("\n⌨️ Durduruldu")

print("\n" + "=" * 60)
print("📋 GELEN MESAJ TİPLERİ VE SAYILARI:")
for msg_type, count in sorted(message_count.items()):
    print(f"  {msg_type}: {count} adet")

print(f"\n📊 Toplam {len(message_count)} farklı mesaj tipi, {sum(message_count.values())} toplam mesaj")
print("\n💡 Bu bilgiler ile terminal_gui.py'yi düzeltebiliriz!")
