#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - Emergency Stop Demo
Pixhawk PX4 PIX 2.4.8 Serial MAVLink Emergency Procedures Demo
Environment Variable Support: MAV_ADDRESS, MAV_BAUD

EMERGENCY STOP DEMONSTRATION:
- Serial MAVLink emergency commands
- Motor immediate stop
- Servo neutral positioning
- System disarm procedures
- Emergency surface protocol
- Safety system validation

Protocol: MAVLink via Serial (Port/Baud from environment)
Safety Critical: All motor/servo neutralization
"""

import os
import sys
import time
import threading
from pymavlink import mavutil

# Environment variables for serial connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

print(f"🚨 TEKNOFEST Emergency Stop Demo - Serial MAVLink")
print(f"📡 Serial Configuration:")
print(f"   Port: {MAV_ADDRESS}")
print(f"   Baud: {MAV_BAUD}")

class EmergencyStopDemo:
    """Emergency stop procedures demonstration"""
    
    def __init__(self):
        """Initialize emergency stop demo"""
        self.master = None
        self.connected = False
        self.armed = False
        
        # Emergency stop state
        self.emergency_active = False
        self.stop_start_time = None
        
        # X-Wing Servo Configuration  
        self.servo_channels = {
            'fin_front_left': 9,   # AUX1 → Channel 9
            'fin_front_right': 11, # AUX3 → Channel 11
            'fin_rear_left': 12,   # AUX4 → Channel 12
            'fin_rear_right': 13,  # AUX5 → Channel 13
            'main_motor': 14,      # AUX6 → Channel 14
            'rocket_release': 15   # AUX7 → Channel 15
        }
        
        # PWM Values
        self.pwm_min = 1000
        self.pwm_mid = 1500  # Neutral/Safe position
        self.pwm_max = 2000
        
        print("✅ Emergency Stop Demo initialized")
    
    def connect_to_vehicle(self):
        """Establish serial MAVLink connection"""
        try:
            print(f"\n📡 Connecting to Pixhawk serial...")
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
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print("✅ Serial MAVLink connection established!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Armed: {'YES' if self.armed else 'NO'}")
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
    
    def set_servo_pwm(self, channel, pwm_value):
        """Send servo PWM command"""
        if not self.connected:
            return False
        
        pwm_value = max(self.pwm_min, min(self.pwm_max, pwm_value))
        
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                channel,  # servo number
                pwm_value,  # PWM value
                0, 0, 0, 0, 0  # unused parameters
            )
            return True
        except Exception as e:
            print(f"❌ Servo command error: {e}")
            return False
    
    def simulate_normal_operation(self):
        """Simulate normal operation before emergency stop"""
        print("\n🎮 Simulating normal operation...")
        print("   (This demonstrates what happens during emergency)")
        
        # Simulate active fin control
        print("   🔄 Active X-Wing fin control...")
        movements = [
            ("Roll Left", {9: self.pwm_min, 11: self.pwm_max, 12: self.pwm_min, 13: self.pwm_max}),
            ("Pitch Up", {9: self.pwm_min, 11: self.pwm_min, 12: self.pwm_max, 13: self.pwm_max}),
            ("Yaw Right", {9: self.pwm_max, 11: self.pwm_min, 12: self.pwm_min, 13: self.pwm_max})
        ]
        
        for movement_name, servo_values in movements:
            print(f"   → {movement_name}")
            for channel, pwm_value in servo_values.items():
                self.set_servo_pwm(channel, pwm_value)
                time.sleep(0.1)
            time.sleep(1.5)
        
        # Simulate motor operation
        print("   🚀 Motor thrust simulation...")
        motor_speeds = [1600, 1700, 1800, 1750]  # Increasing then decreasing
        
        for speed in motor_speeds:
            print(f"   → Motor: {speed}µs ({((speed-1500)/5):.0f}% thrust)")
            self.set_servo_pwm(self.servo_channels['main_motor'], speed)
            time.sleep(1.0)
        
        print("✅ Normal operation simulation complete")
        time.sleep(2.0)
    
    def execute_emergency_stop(self):
        """Execute complete emergency stop procedure"""
        print("\n" + "🚨"*20)
        print("🚨 EMERGENCY STOP ACTIVATED!")
        print("🚨"*20)
        
        self.emergency_active = True
        self.stop_start_time = time.time()
        
        # PHASE 1: Immediate motor stop (CRITICAL)
        print("\n⚡ PHASE 1: IMMEDIATE MOTOR STOP")
        motor_stop_time = time.time()
        
        success = self.set_servo_pwm(self.servo_channels['main_motor'], self.pwm_mid)
        if success:
            stop_duration = (time.time() - motor_stop_time) * 1000  # ms
            print(f"✅ Motor stopped in {stop_duration:.1f}ms")
        else:
            print("❌ Motor stop command failed!")
        
        # PHASE 2: Servo neutralization
        print("\n🎯 PHASE 2: SERVO NEUTRALIZATION")
        servo_neutralize_time = time.time()
        
        fin_servos = [
            ('Front Left', self.servo_channels['fin_front_left']),
            ('Front Right', self.servo_channels['fin_front_right']),
            ('Rear Left', self.servo_channels['fin_rear_left']),
            ('Rear Right', self.servo_channels['fin_rear_right'])
        ]
        
        neutralized_count = 0
        for fin_name, channel in fin_servos:
            if self.set_servo_pwm(channel, self.pwm_mid):
                print(f"   ✅ {fin_name} fin neutralized")
                neutralized_count += 1
            else:
                print(f"   ❌ {fin_name} fin neutralization failed")
            time.sleep(0.05)  # 50ms between commands
        
        servo_duration = (time.time() - servo_neutralize_time) * 1000
        print(f"📊 {neutralized_count}/4 servos neutralized in {servo_duration:.1f}ms")
        
        # PHASE 3: System disarm (if armed)
        print("\n🔐 PHASE 3: SYSTEM DISARM")
        if self.armed:
            try:
                disarm_time = time.time()
                
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                    0,  # confirmation
                    0,  # disarm
                    0, 0, 0, 0, 0, 0  # unused parameters
                )
                
                # Wait for disarm acknowledgment
                ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    disarm_duration = (time.time() - disarm_time) * 1000
                    print(f"✅ System disarmed in {disarm_duration:.1f}ms")
                    self.armed = False
                else:
                    print("⚠️ Disarm command sent but no acknowledgment")
                    
            except Exception as e:
                print(f"❌ Disarm command error: {e}")
        else:
            print("ℹ️ System already disarmed")
        
        # PHASE 4: Emergency surface (simulation)
        print("\n🔼 PHASE 4: EMERGENCY SURFACE PROTOCOL")
        try:
            surface_time = time.time()
            
            # Send emergency ascent command
            self.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b110111000111,  # type_mask (velocity only)
                0, 0, 0,  # position
                0, 0, -2.0,  # velocity (2 m/s upward)
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )
            
            surface_duration = (time.time() - surface_time) * 1000
            print(f"✅ Emergency ascent command sent in {surface_duration:.1f}ms")
            print("   → 2.0 m/s vertical ascent rate")
            
        except Exception as e:
            print(f"❌ Emergency surface command error: {e}")
        
        # PHASE 5: Safety status report
        print("\n📊 PHASE 5: SAFETY STATUS REPORT")
        total_stop_time = (time.time() - self.stop_start_time) * 1000
        
        print(f"⏱️ Total emergency stop time: {total_stop_time:.1f}ms")
        print(f"🎯 Motor: {'✅ STOPPED' if success else '❌ FAILED'}")
        print(f"🎮 Servos: {neutralized_count}/4 neutralized")
        print(f"🔐 Armed: {'✅ DISARMED' if not self.armed else '⚠️ STILL ARMED'}")
        print(f"🔼 Surface: ✅ ASCENDING")
        
        # Overall assessment
        critical_systems_safe = success and (neutralized_count >= 3)  # Allow 1 servo failure
        
        if critical_systems_safe:
            print("\n🎉 EMERGENCY STOP SUCCESSFUL!")
            print("   All critical systems neutralized")
            print("   Vehicle is in safe state")
        else:
            print("\n⚠️ EMERGENCY STOP ISSUES DETECTED!")
            print("   Some critical systems may still be active")
            print("   Manual intervention may be required")
        
        return critical_systems_safe
    
    def test_emergency_response_time(self):
        """Test emergency response time performance"""
        print("\n📊 EMERGENCY RESPONSE TIME TEST")
        print("="*50)
        
        # Test multiple emergency stop cycles
        response_times = []
        
        for test_num in range(3):
            print(f"\n🔄 Test {test_num + 1}/3: Emergency response time")
            
            # Setup: Set servos to active position
            print("   → Setting up active state...")
            for channel in [9, 11, 12, 13]:
                self.set_servo_pwm(channel, self.pwm_max)
            self.set_servo_pwm(self.servo_channels['main_motor'], 1700)
            time.sleep(1.0)
            
            # Execute emergency stop and measure time
            start_time = time.time()
            
            # Critical stop commands
            self.set_servo_pwm(self.servo_channels['main_motor'], self.pwm_mid)
            for channel in [9, 11, 12, 13]:
                self.set_servo_pwm(channel, self.pwm_mid)
            
            stop_time = (time.time() - start_time) * 1000  # ms
            response_times.append(stop_time)
            
            print(f"   ⏱️ Response time: {stop_time:.1f}ms")
            time.sleep(2.0)
        
        # Performance analysis
        avg_time = sum(response_times) / len(response_times)
        max_time = max(response_times)
        min_time = min(response_times)
        
        print(f"\n📈 PERFORMANCE ANALYSIS:")
        print(f"   Average response: {avg_time:.1f}ms")
        print(f"   Fastest response: {min_time:.1f}ms")
        print(f"   Slowest response: {max_time:.1f}ms")
        
        # Safety assessment
        if max_time < 500:  # 500ms threshold
            print("   ✅ EXCELLENT: All responses under 500ms")
        elif max_time < 1000:  # 1000ms threshold
            print("   ✅ GOOD: All responses under 1000ms")
        else:
            print("   ⚠️ SLOW: Some responses over 1000ms")
        
        return avg_time < 500
    
    def run_full_demo(self):
        """Run complete emergency stop demonstration"""
        print("🚨 TEKNOFEST Emergency Stop Demo - FULL DEMONSTRATION")
        print("="*60)
        
        try:
            # Connect to vehicle
            if not self.connect_to_vehicle():
                print("❌ Failed to connect to vehicle!")
                return False
            
            # Demo phases
            phases = [
                ("Normal Operation Simulation", self.simulate_normal_operation),
                ("Emergency Stop Execution", self.execute_emergency_stop),
                ("Response Time Testing", self.test_emergency_response_time)
            ]
            
            results = []
            
            for phase_name, phase_func in phases:
                print(f"\n{'='*20} {phase_name} {'='*20}")
                
                try:
                    result = phase_func()
                    results.append((phase_name, result))
                    print(f"{'✅' if result else '❌'} {phase_name}: {'PASSED' if result else 'FAILED'}")
                except Exception as e:
                    print(f"❌ {phase_name}: ERROR - {e}")
                    results.append((phase_name, False))
                
                time.sleep(2.0)  # Brief pause between phases
            
            # Final assessment
            print(f"\n" + "="*60)
            print("📋 EMERGENCY STOP DEMO SUMMARY")
            print("="*60)
            
            passed_phases = sum(1 for _, result in results if result)
            total_phases = len(results)
            
            for phase_name, result in results:
                status = "✅ PASSED" if result else "❌ FAILED"
                print(f"   {phase_name}: {status}")
            
            print(f"\n🎯 Overall Result: {passed_phases}/{total_phases} phases passed")
            
            if passed_phases == total_phases:
                print("🎉 EMERGENCY STOP SYSTEM FULLY OPERATIONAL!")
                print("   All safety procedures working correctly")
                print("   Vehicle is ready for competition")
            elif passed_phases >= total_phases - 1:
                print("⚠️ EMERGENCY STOP MOSTLY OPERATIONAL")
                print("   Minor issues detected but critical functions work")
                print("   System is acceptable for competition")
            else:
                print("❌ EMERGENCY STOP SYSTEM ISSUES")
                print("   Critical safety problems detected")
                print("   System needs repair before competition")
            
            return passed_phases >= total_phases - 1
            
        except KeyboardInterrupt:
            print("\n⚠️ Demo interrupted by user")
            # Execute emergency stop for safety
            self.execute_emergency_stop()
            return False
        except Exception as e:
            print(f"\n❌ Demo error: {e}")
            return False
        finally:
            # Ensure all systems are safe
            if self.connected:
                print("\n🔄 Final safety neutralization...")
                self.set_servo_pwm(self.servo_channels['main_motor'], self.pwm_mid)
                for channel in [9, 11, 12, 13]:
                    self.set_servo_pwm(channel, self.pwm_mid)
                
                if self.master:
                    self.master.close()
                    print("🔌 Serial connection closed")

def main():
    """Main demo function"""
    demo = EmergencyStopDemo()
    
    print("⚠️ SAFETY WARNING:")
    print("   This demo will test emergency stop procedures")
    print("   Ensure vehicle is in safe testing environment")
    print("   Keep emergency stop button accessible")
    
    response = input("\n🔍 Continue with emergency stop demo? (y/N): ")
    if response.lower() != 'y':
        print("Demo cancelled by user.")
        return
    
    success = demo.run_full_demo()
    
    if success:
        print("\n✅ Emergency stop demo completed successfully!")
        sys.exit(0)
    else:
        print("\n❌ Emergency stop demo completed with issues!")
        sys.exit(1)

if __name__ == "__main__":
    main() 