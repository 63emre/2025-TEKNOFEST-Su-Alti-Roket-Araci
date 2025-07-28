#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - Serial MAVLink Connection Test
Pixhawk PX4 PIX 2.4.8 Serial Communication Test
Environment Variable Support: MAV_ADDRESS, MAV_BAUD
"""

import os
import sys
import time
import math
from pymavlink import mavutil

# Environment variables for serial connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

print(f"🔧 Serial Configuration:")
print(f"   Port: {MAV_ADDRESS}")
print(f"   Baud: {MAV_BAUD}")
print(f"   Environment: MAV_ADDRESS={MAV_ADDRESS}, MAV_BAUD={MAV_BAUD}")

class MAVLinkTester:
    """Serial MAVLink bağlantı test sınıfı"""
    
    def __init__(self):
        """Test sınıfını başlat"""
        self.master = None
        self.connected = False
        
        # Test counters
        self.heartbeat_count = 0
        self.imu_count = 0
        self.attitude_count = 0
        
        print(f"🔧 MAVLink Tester initialized for serial: {MAV_ADDRESS}@{MAV_BAUD}")
    
    def connect(self):
        """Serial MAVLink bağlantısı kur"""
        try:
            print(f"📡 Connecting to Pixhawk serial...")
            print(f"   Port: {MAV_ADDRESS}")
            print(f"   Baud: {MAV_BAUD}")
            
            # Serial MAVLink connection
            self.master = mavutil.mavlink_connection(
                MAV_ADDRESS,
                baud=MAV_BAUD,
                autoreconnect=True
            )
            
            print("💓 Waiting for heartbeat...")
            heartbeat = self.master.wait_heartbeat(timeout=15)
            
            if heartbeat:
                self.connected = True
                print("✅ Serial MAVLink connection established!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Vehicle Type: {heartbeat.type}")
                print(f"   Base Mode: {heartbeat.base_mode}")
                return True
            else:
                print("❌ No heartbeat received!")
                return False
                
        except Exception as e:
            print(f"❌ Serial connection failed: {e}")
            print("💡 Check:")
            print(f"   • Pixhawk connected to {MAV_ADDRESS}")
            print(f"   • Correct baud rate: {MAV_BAUD}")
            print("   • ArduSub firmware running")
            return False
    
    def test_heartbeat(self, duration=10):
        """Heartbeat mesajlarını test et"""
        if not self.connected:
            return False
        
        print(f"\n🔍 Testing heartbeat messages ({duration}s)...")
        
        start_time = time.time()
        heartbeat_count = 0
        
        while time.time() - start_time < duration:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False, timeout=1.0)
            if msg:
                heartbeat_count += 1
                if heartbeat_count <= 3:  # İlk 3 heartbeat'i göster
                    print(f"   💓 Heartbeat #{heartbeat_count}: "
                          f"Type={msg.type}, Mode={msg.base_mode}, "
                          f"Armed={bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")
        
        rate = heartbeat_count / duration
        print(f"📊 Heartbeat test results:")
        print(f"   • Received: {heartbeat_count} messages")
        print(f"   • Rate: {rate:.1f} Hz")
        print(f"   • Status: {'✅ GOOD' if rate > 0.5 else '❌ LOW'}")
        
        self.heartbeat_count = heartbeat_count
        return heartbeat_count > 0
    
    def test_imu_data(self, duration=10):
        """IMU data mesajlarını test et"""
        if not self.connected:
            return False
        
        print(f"\n🧭 Testing IMU data ({duration}s)...")
        
        start_time = time.time()
        imu_count = 0
        attitude_count = 0
        
        while time.time() - start_time < duration:
            # RAW_IMU mesajı
            imu_msg = self.master.recv_match(type='RAW_IMU', blocking=False, timeout=0.1)
            if imu_msg:
                imu_count += 1
                if imu_count <= 2:  # İlk 2 IMU mesajını göster
                    print(f"   📊 RAW_IMU #{imu_count}: "
                          f"Accel=({imu_msg.xacc/1000:.2f}, {imu_msg.yacc/1000:.2f}, {imu_msg.zacc/1000:.2f}), "
                          f"Gyro=({math.degrees(imu_msg.xgyro/1000):.1f}°, {math.degrees(imu_msg.ygyro/1000):.1f}°, {math.degrees(imu_msg.zgyro/1000):.1f}°)")
            
            # ATTITUDE mesajı
            att_msg = self.master.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
            if att_msg:
                attitude_count += 1
                if attitude_count <= 2:  # İlk 2 attitude mesajını göster
                    print(f"   🎯 ATTITUDE #{attitude_count}: "
                          f"Roll={math.degrees(att_msg.roll):.1f}°, "
                          f"Pitch={math.degrees(att_msg.pitch):.1f}°, "
                          f"Yaw={math.degrees(att_msg.yaw):.1f}°")
        
        imu_rate = imu_count / duration
        att_rate = attitude_count / duration
        
        print(f"📊 IMU test results:")
        print(f"   • RAW_IMU: {imu_count} messages ({imu_rate:.1f} Hz)")
        print(f"   • ATTITUDE: {attitude_count} messages ({att_rate:.1f} Hz)")
        print(f"   • Status: {'✅ GOOD' if (imu_rate > 5 or att_rate > 5) else '❌ LOW'}")
        
        self.imu_count = imu_count
        self.attitude_count = attitude_count
        return (imu_count > 0 or attitude_count > 0)
    
    def test_system_status(self):
        """Sistem durumu test et"""
        if not self.connected:
            return False
        
        print(f"\n🔍 Testing system status...")
        
        # SYS_STATUS mesajı bekle
        sys_msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5.0)
        if sys_msg:
            voltage = sys_msg.voltage_battery / 1000.0  # mV to V
            current = sys_msg.current_battery / 100.0   # cA to A
            battery = sys_msg.battery_remaining         # %
            
            print(f"⚡ System Status:")
            print(f"   • Battery: {voltage:.1f}V, {current:.1f}A, {battery}%")
            print(f"   • Load: {sys_msg.load/10:.1f}%")
            print(f"   • Status: {'✅ HEALTHY' if voltage > 10.0 else '⚠️ LOW BATTERY'}")
            return True
        else:
            print("❌ No system status received")
            return False
    
    def test_servo_output(self):
        """Servo komut test et (test amaçlı)"""
        if not self.connected:
            return False
        
        print(f"\n🎮 Testing servo command capability...")
        
        try:
            # Test servo command (neutral position)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                9,  # servo number (AUX1)
                1500,  # PWM value (neutral)
                0, 0, 0, 0, 0  # unused parameters
            )
            
            print("✅ Servo command sent successfully")
            print("   • Command: AUX1 → 1500µs (neutral)")
            print("   • Note: Check AUX1 output for PWM signal")
            return True
            
        except Exception as e:
            print(f"❌ Servo command failed: {e}")
            return False
    
    def run_full_test(self):
        """Tam test paketi çalıştır"""
        print("🚀 TEKNOFEST ROV - Full Serial MAVLink Test")
        print("=" * 60)
        
        # Bağlantı testi
        if not self.connect():
            print("❌ Connection test failed!")
            return False
        
        # Test suite
        tests = [
            ("Heartbeat Test", lambda: self.test_heartbeat(10)),
            ("IMU Data Test", lambda: self.test_imu_data(10)),
            ("System Status Test", lambda: self.test_system_status()),
            ("Servo Command Test", lambda: self.test_servo_output())
        ]
        
        results = []
        for test_name, test_func in tests:
            try:
                result = test_func()
                results.append((test_name, result))
                print(f"{'✅' if result else '❌'} {test_name}: {'PASSED' if result else 'FAILED'}")
            except Exception as e:
                print(f"❌ {test_name}: ERROR - {e}")
                results.append((test_name, False))
        
        # Test özeti
        print(f"\n📋 TEST SUMMARY")
        print("=" * 30)
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"   {test_name}: {status}")
        
        print(f"\n🎯 Overall Result: {passed}/{total} tests passed")
        
        if passed == total:
            print("🎉 ALL TESTS PASSED! Serial MAVLink connection is working perfectly!")
        elif passed > total // 2:
            print("⚠️ PARTIAL SUCCESS. Some tests failed but basic connection works.")
        else:
            print("❌ MAJOR ISSUES. Serial connection needs troubleshooting.")
        
        return passed == total
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        if self.master:
            try:
                self.master.close()
                print("🔌 Serial connection closed")
            except:
                pass
        self.connected = False

def main():
    """Ana test fonksiyonu"""
    tester = MAVLinkTester()
    
    try:
        # Full test suite çalıştır
        success = tester.run_full_test()
        
        # Cleanup
        tester.disconnect()
        
        # Exit code
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n⚠️ Test interrupted by user")
        tester.disconnect()
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Test suite error: {e}")
        tester.disconnect()
        sys.exit(1)

if __name__ == "__main__":
    main() 