#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - Full Capability Demo
Pixhawk PX4 PIX 2.4.8 Serial MAVLink Comprehensive Demo
Environment Variable Support: MAV_ADDRESS, MAV_BAUD

COMPLETE SYSTEM DEMONSTRATION:
- Serial MAVLink communication
- X-Wing servo control matrix
- Motor speed control
- Emergency stop procedures
- Waterproof testing simulation
- Rocket separation simulation
- Live orientation feedback

Protocol: MAVLink via Serial (Port/Baud from environment)
Hardware: X-Configuration ROV + Rocket Payload + Emergency Systems
"""

import os
import sys
import time
import math
import json
import threading
from datetime import datetime
from pymavlink import mavutil

# Environment variables for serial connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

print(f"🚀 TEKNOFEST Full Capability Demo - Serial MAVLink")
print(f"📡 Serial Configuration:")
print(f"   Port: {MAV_ADDRESS}")
print(f"   Baud: {MAV_BAUD}")

class FullCapabilityDemo:
    """Complete system capability demonstration"""
    
    def __init__(self):
        """Initialize demo system"""
        self.master = None
        self.connected = False
        self.armed = False
        
        # Demo state
        self.demo_active = False
        self.current_phase = "Initialization"
        
        # Telemetry data
        self.current_position = {'lat': 0, 'lon': 0, 'alt': 0}
        self.current_attitude = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.current_depth = 0.0
        self.system_status = {}
        
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
        self.pwm_mid = 1500
        self.pwm_max = 2000
        
        # Demo phases
        self.demo_phases = [
            ("Connection Test", self.demo_connection_test),
            ("System Check", self.demo_system_check),
            ("X-Wing Servo Demo", self.demo_x_wing_servos),
            ("Motor Control Demo", self.demo_motor_control),
            ("Maneuver Capabilities", self.demo_maneuver_capabilities),
            ("Waterproof Testing", self.demo_waterproof_test),
            ("Rocket Separation", self.demo_rocket_separation),
            ("Emergency Procedures", self.demo_emergency_procedures),
            ("Performance Summary", self.demo_performance_summary)
        ]
        
        # Demo results
        self.demo_results = {}
        
        print("✅ Full Capability Demo initialized")
    
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
                print(f"   Vehicle Type: {heartbeat.type}")
                print(f"   Armed: {'YES' if self.armed else 'NO'}")
                
                # Start telemetry monitoring
                self.start_telemetry_monitoring()
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
    
    def start_telemetry_monitoring(self):
        """Start background telemetry monitoring"""
        def telemetry_thread():
            while self.connected:
                try:
                    # Position data
                    pos_msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if pos_msg:
                        self.current_position = {
                            'lat': pos_msg.lat / 1e7,
                            'lon': pos_msg.lon / 1e7,
                            'alt': pos_msg.alt / 1000.0
                        }
                    
                    # Attitude data
                    att_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                    if att_msg:
                        self.current_attitude = {
                            'roll': math.degrees(att_msg.roll),
                            'pitch': math.degrees(att_msg.pitch),
                            'yaw': math.degrees(att_msg.yaw)
                        }
                    
                    # System status
                    sys_msg = self.master.recv_match(type='SYS_STATUS', blocking=False)
                    if sys_msg:
                        self.system_status = {
                            'voltage': sys_msg.voltage_battery / 1000.0,
                            'current': sys_msg.current_battery / 100.0,
                            'battery_remaining': sys_msg.battery_remaining,
                            'load': sys_msg.load / 10.0
                        }
                    
                    # Depth data (from pressure sensor)
                    depth_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                    if depth_msg:
                        depth_m = max(0.0, (depth_msg.press_abs - 1013.25) / 100.0)
                        self.current_depth = depth_m
                    
                    time.sleep(0.1)  # 10Hz monitoring
                    
                except Exception as e:
                    print(f"⚠️ Telemetry error: {e}")
                    time.sleep(1.0)
        
        monitor_thread = threading.Thread(target=telemetry_thread, daemon=True)
        monitor_thread.start()
        print("🔄 Telemetry monitoring started")
    
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
    
    def demo_connection_test(self):
        """Demo Phase 1: Connection and Communication Test"""
        print("\n" + "="*60)
        print("📡 PHASE 1: Serial MAVLink Connection Test")
        print("="*60)
        
        results = {'phase': 'Connection Test', 'tests': []}
        
        # Test 1: Heartbeat monitoring
        print("🔍 Test 1: Heartbeat monitoring (10 seconds)...")
        heartbeat_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 10:
            msg = self.master.recv_match(type='HEARTBEAT', blocking=False, timeout=1.0)
            if msg:
                heartbeat_count += 1
            time.sleep(1.0)
        
        heartbeat_rate = heartbeat_count / 10.0
        heartbeat_success = heartbeat_rate >= 0.8  # At least 0.8 Hz
        
        results['tests'].append({
            'name': 'Heartbeat Rate',
            'result': f"{heartbeat_rate:.1f} Hz",
            'success': heartbeat_success,
            'expected': '≥0.8 Hz'
        })
        
        print(f"   Result: {heartbeat_rate:.1f} Hz ({'✅ PASS' if heartbeat_success else '❌ FAIL'})")
        
        # Test 2: Parameter communication
        print("\n🔍 Test 2: Parameter communication...")
        try:
            self.master.mav.param_request_read_send(
                self.master.target_system,
                self.master.target_component,
                b'SYSID_THISMAV',
                -1
            )
            
            param_msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            param_success = param_msg is not None
            
            results['tests'].append({
                'name': 'Parameter Communication',  
                'result': f"System ID: {param_msg.param_value if param_msg else 'N/A'}",
                'success': param_success,
                'expected': 'Valid parameter response'
            })
            
            print(f"   Result: {'✅ PASS' if param_success else '❌ FAIL'}")
            
        except Exception as e:
            results['tests'].append({
                'name': 'Parameter Communication',
                'result': f"Error: {e}",
                'success': False,
                'expected': 'Valid parameter response'
            })
            print(f"   Result: ❌ FAIL - {e}")
        
        # Test 3: Command acknowledgment
        print("\n🔍 Test 3: Command acknowledgment...")
        try:
            # Send a harmless command (request system info)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            
            ack_msg = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
            ack_success = ack_msg is not None and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            
            results['tests'].append({
                'name': 'Command Acknowledgment',
                'result': f"ACK: {ack_msg.result if ack_msg else 'TIMEOUT'}",
                'success': ack_success,
                'expected': 'MAV_RESULT_ACCEPTED'
            })
            
            print(f"   Result: {'✅ PASS' if ack_success else '❌ FAIL'}")
            
        except Exception as e:
            results['tests'].append({
                'name': 'Command Acknowledgment',
                'result': f"Error: {e}",
                'success': False,
                'expected': 'MAV_RESULT_ACCEPTED'
            })
            print(f"   Result: ❌ FAIL - {e}")
        
        # Phase summary
        passed_tests = sum(1 for test in results['tests'] if test['success'])
        total_tests = len(results['tests'])
        results['overall_success'] = passed_tests == total_tests
        
        print(f"\n📊 Connection Test Summary: {passed_tests}/{total_tests} tests passed")
        self.demo_results['connection_test'] = results
        
        return results['overall_success']
    
    def demo_system_check(self):
        """Demo Phase 2: Complete System Check"""
        print("\n" + "="*60)
        print("🔍 PHASE 2: Complete System Check")
        print("="*60)
        
        results = {'phase': 'System Check', 'tests': []}
        
        # Check 1: Battery status
        print("⚡ Check 1: Battery and power system...")
        if self.system_status:
            voltage = self.system_status['voltage']
            current = self.system_status['current']
            battery_pct = self.system_status['battery_remaining']
            
            battery_ok = voltage > 20.0  # 6S LiPo minimum
            
            results['tests'].append({
                'name': 'Battery Status',
                'result': f"{voltage:.1f}V, {current:.1f}A, {battery_pct}%",
                'success': battery_ok,
                'expected': '>20.0V (6S LiPo)'
            })
            
            print(f"   Voltage: {voltage:.1f}V")
            print(f"   Current: {current:.1f}A") 
            print(f"   Battery: {battery_pct}%")
            print(f"   Status: {'✅ HEALTHY' if battery_ok else '⚠️ LOW'}")
        else:
            results['tests'].append({
                'name': 'Battery Status',
                'result': 'No telemetry data',
                'success': False,
                'expected': '>20.0V (6S LiPo)'
            })
            print("   Status: ❌ NO DATA")
        
        # Check 2: GPS status
        print("\n🛰️ Check 2: GPS system...")
        try:
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
            if gps_msg:
                gps_ok = gps_msg.fix_type >= 3 and gps_msg.satellites_visible >= 6
                
                results['tests'].append({
                    'name': 'GPS Status',
                    'result': f"Fix: {gps_msg.fix_type}, Sats: {gps_msg.satellites_visible}",
                    'success': gps_ok,
                    'expected': 'Fix ≥3, Satellites ≥6'
                })
                
                print(f"   Fix Type: {gps_msg.fix_type}")
                print(f"   Satellites: {gps_msg.satellites_visible}")
                print(f"   Position: ({gps_msg.lat/1e7:.6f}, {gps_msg.lon/1e7:.6f})")
                print(f"   Status: {'✅ GOOD' if gps_ok else '⚠️ WEAK'}")
            else:
                results['tests'].append({
                    'name': 'GPS Status',
                    'result': 'Timeout',
                    'success': False,
                    'expected': 'Fix ≥3, Satellites ≥6'
                })
                print("   Status: ❌ TIMEOUT")
        except Exception as e:
            results['tests'].append({
                'name': 'GPS Status',
                'result': f"Error: {e}",
                'success': False,
                'expected': 'Fix ≥3, Satellites ≥6'
            })
            print(f"   Status: ❌ ERROR - {e}")
        
        # Check 3: IMU system
        print("\n🧭 Check 3: IMU system...")
        try:
            imu_msg = self.master.recv_match(type='RAW_IMU', blocking=True, timeout=5)
            att_msg = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
            
            imu_ok = imu_msg is not None and att_msg is not None
            
            if imu_ok:
                results['tests'].append({
                    'name': 'IMU System',
                    'result': f"Roll: {self.current_attitude['roll']:.1f}°, Pitch: {self.current_attitude['pitch']:.1f}°, Yaw: {self.current_attitude['yaw']:.1f}°",
                    'success': True,
                    'expected': 'Valid IMU data'
                })
                
                print(f"   Roll: {self.current_attitude['roll']:.1f}°")
                print(f"   Pitch: {self.current_attitude['pitch']:.1f}°")
                print(f"   Yaw: {self.current_attitude['yaw']:.1f}°")
                print("   Status: ✅ ACTIVE")
            else:
                results['tests'].append({
                    'name': 'IMU System',
                    'result': 'No IMU data',
                    'success': False,
                    'expected': 'Valid IMU data'
                })
                print("   Status: ❌ NO DATA")
                
        except Exception as e:
            results['tests'].append({
                'name': 'IMU System',
                'result': f"Error: {e}",
                'success': False,
                'expected': 'Valid IMU data'
            })
            print(f"   Status: ❌ ERROR - {e}")
        
        # Check 4: Depth sensor
        print("\n📏 Check 4: Depth sensor...")
        try:
            depth_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=5)
            depth_ok = depth_msg is not None
            
            if depth_ok:
                results['tests'].append({
                    'name': 'Depth Sensor',
                    'result': f"Depth: {self.current_depth:.2f}m, Pressure: {depth_msg.press_abs:.1f} hPa",
                    'success': True,
                    'expected': 'Valid pressure data'
                })
                
                print(f"   Depth: {self.current_depth:.2f}m")
                print(f"   Pressure: {depth_msg.press_abs:.1f} hPa")
                print(f"   Temperature: {depth_msg.temperature/100:.1f}°C")
                print("   Status: ✅ ACTIVE")
            else:
                results['tests'].append({
                    'name': 'Depth Sensor',
                    'result': 'No pressure data',
                    'success': False,
                    'expected': 'Valid pressure data'
                })
                print("   Status: ❌ NO DATA")
                
        except Exception as e:
            results['tests'].append({
                'name': 'Depth Sensor',
                'result': f"Error: {e}",
                'success': False,
                'expected': 'Valid pressure data'
            })
            print(f"   Status: ❌ ERROR - {e}")
        
        # Phase summary
        passed_checks = sum(1 for test in results['tests'] if test['success'])
        total_checks = len(results['tests'])
        results['overall_success'] = passed_checks >= total_checks - 1  # Allow 1 failure
        
        print(f"\n📊 System Check Summary: {passed_checks}/{total_checks} checks passed")
        self.demo_results['system_check'] = results
        
        return results['overall_success']
    
    def demo_x_wing_servos(self):
        """Demo Phase 3: X-Wing Servo Control Matrix"""
        print("\n" + "="*60)
        print("🎮 PHASE 3: X-Wing Servo Control Matrix Demo")
        print("="*60)
        
        results = {'phase': 'X-Wing Servos', 'movements': []}
        
        # X-Wing control movements
        movements = [
            ("Neutral Position", {9: self.pwm_mid, 11: self.pwm_mid, 12: self.pwm_mid, 13: self.pwm_mid}),
            ("Roll Left", {9: self.pwm_min, 11: self.pwm_max, 12: self.pwm_min, 13: self.pwm_max}),
            ("Roll Right", {9: self.pwm_max, 11: self.pwm_min, 12: self.pwm_max, 13: self.pwm_min}),
            ("Pitch Up", {9: self.pwm_min, 11: self.pwm_min, 12: self.pwm_max, 13: self.pwm_max}),
            ("Pitch Down", {9: self.pwm_max, 11: self.pwm_max, 12: self.pwm_min, 13: self.pwm_min}),
            ("Yaw Left", {9: self.pwm_min, 11: self.pwm_max, 12: self.pwm_max, 13: self.pwm_min}),
            ("Yaw Right", {9: self.pwm_max, 11: self.pwm_min, 12: self.pwm_min, 13: self.pwm_max}),
            ("Return to Neutral", {9: self.pwm_mid, 11: self.pwm_mid, 12: self.pwm_mid, 13: self.pwm_mid})
        ]
        
        for movement_name, servo_values in movements:
            print(f"\n🎯 Executing: {movement_name}")
            
            # Display fin positions
            fin_names = ['Front Left (Ch9)', 'Front Right (Ch11)', 'Rear Left (Ch12)', 'Rear Right (Ch13)']
            channels = [9, 11, 12, 13]
            
            for fin_name, channel in zip(fin_names, channels):
                pwm_value = servo_values[channel]
                direction = "MIN" if pwm_value == self.pwm_min else "MAX" if pwm_value == self.pwm_max else "MID"
                print(f"   {fin_name}: {pwm_value}µs ({direction})")
            
            # Send servo commands
            success_count = 0
            for channel, pwm_value in servo_values.items():
                if self.set_servo_pwm(channel, pwm_value):
                    success_count += 1
                time.sleep(0.1)  # Small delay between commands
            
            movement_success = success_count == len(servo_values)
            
            results['movements'].append({
                'name': movement_name,
                'commands_sent': success_count,
                'total_commands': len(servo_values),
                'success': movement_success
            })
            
            print(f"   Result: {success_count}/{len(servo_values)} commands sent {'✅' if movement_success else '❌'}")
            
            # Hold position for demonstration
            time.sleep(2.5)
        
        # Phase summary
        successful_movements = sum(1 for mov in results['movements'] if mov['success'])
        total_movements = len(results['movements'])
        results['overall_success'] = successful_movements >= total_movements - 1
        
        print(f"\n📊 X-Wing Servo Demo Summary: {successful_movements}/{total_movements} movements executed")
        self.demo_results['x_wing_servos'] = results
        
        return results['overall_success'] 