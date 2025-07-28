#!/usr/bin/env python3
"""
TEKNOFEST 2025 Su Altı Roket Aracı - Mission Manager
Pixhawk PX4 PIX 2.4.8 Serial MAVLink Mission Control System
Environment Variable Support: MAV_ADDRESS, MAV_BAUD

AUTONOMOUS MISSION EXECUTION SYSTEM
- Navigation waypoint missions  
- Rocket launch sequences
- Emergency protocols
- Real-time mission monitoring

Protocol: MAVLink via Serial (Port/Baud from environment)
Hardware: X-Configuration ROV + Rocket Payload
"""

import os
import sys
import time
import json
import math
import threading
from datetime import datetime
from pymavlink import mavutil

# Environment variables for serial connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

class MissionManager:
    """Mission execution and monitoring system"""
    
    def __init__(self):
        """Initialize mission manager"""
        print(f"🚀 TEKNOFEST Mission Manager - Serial MAVLink")
        print(f"📡 Serial Configuration:")
        print(f"   Port: {MAV_ADDRESS}")
        print(f"   Baud: {MAV_BAUD}")
        
        # Serial MAVLink connection
        self.master = None
        self.connected = False
        self.armed = False
        
        # Mission state
        self.current_mission = None
        self.mission_active = False
        self.mission_thread = None
        
        # Navigation data
        self.current_position = {'lat': 0, 'lon': 0, 'alt': 0}
        self.current_attitude = {'roll': 0, 'pitch': 0, 'yaw': 0}
        self.current_depth = 0.0
        
        # Mission types
        self.mission_types = {
            'waypoint_navigation': self.execute_waypoint_mission,
            'rocket_launch': self.execute_rocket_launch,
            'emergency_surface': self.execute_emergency_surface,
            'pattern_search': self.execute_pattern_search
        }
        
        # Safety limits
        self.max_depth = 10.0  # meters
        self.max_speed = 2.0   # m/s
        self.max_distance = 100.0  # meters from start
        
        print("✅ Mission Manager initialized")
    
    def connect_mavlink(self):
        """Establish serial MAVLink connection"""
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
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print("✅ Serial MAVLink connection established!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Armed: {'YES' if self.armed else 'NO'}")
                
                # Start monitoring thread
                self.start_monitoring()
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
    
    def start_monitoring(self):
        """Start telemetry monitoring thread"""
        def monitor_thread():
            while self.connected:
                try:
                    # Get position data
                    pos_msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
                    if pos_msg:
                        self.current_position = {
                            'lat': pos_msg.lat / 1e7,
                            'lon': pos_msg.lon / 1e7,
                            'alt': pos_msg.alt / 1000.0
                        }
                    
                    # Get attitude data
                    att_msg = self.master.recv_match(type='ATTITUDE', blocking=False)
                    if att_msg:
                        self.current_attitude = {
                            'roll': math.degrees(att_msg.roll),
                            'pitch': math.degrees(att_msg.pitch),
                            'yaw': math.degrees(att_msg.yaw)
                        }
                    
                    # Get depth data (from pressure sensor)
                    depth_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=False)
                    if depth_msg:
                        # Convert pressure to depth (rough approximation)
                        depth_m = max(0.0, (depth_msg.press_abs - 1013.25) / 100.0)
                        self.current_depth = depth_m
                    
                    time.sleep(0.1)  # 10Hz monitoring
                    
                except Exception as e:
                    print(f"⚠️ Monitoring error: {e}")
                    time.sleep(1.0)
        
        monitor_thread = threading.Thread(target=monitor_thread, daemon=True)
        monitor_thread.start()
        print("🔄 Telemetry monitoring started")
    
    def perform_system_check(self):
        """Perform pre-mission system check"""
        print("\n🔍 Performing system check...")
        
        checks = {
            'serial_connection': False,
            'armed_status': False,
            'gps_fix': False,
            'depth_sensor': False,
            'battery_level': False
        }
        
        if self.connected:
            checks['serial_connection'] = True
            print("   ✅ Serial MAVLink connection: OK")
        else:
            print("   ❌ Serial MAVLink connection: FAILED")
            return checks
        
        if self.armed:
            checks['armed_status'] = True
            print("   ✅ Armed status: ARMED")
        else:
            print("   ⚠️ Armed status: DISARMED")
        
        # Check GPS
        try:
            gps_msg = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=3)
            if gps_msg and gps_msg.fix_type >= 3:
                checks['gps_fix'] = True
                print(f"   ✅ GPS fix: {gps_msg.fix_type} ({gps_msg.satellites_visible} sats)")
            else:
                print("   ❌ GPS fix: NO FIX")
        except:
            print("   ❌ GPS fix: TIMEOUT")
        
        # Check depth sensor
        try:
            depth_msg = self.master.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=3)
            if depth_msg:
                checks['depth_sensor'] = True
                print(f"   ✅ Depth sensor: {self.current_depth:.1f}m")
            else:
                print("   ❌ Depth sensor: NO DATA")
        except:
            print("   ❌ Depth sensor: TIMEOUT")
        
        # Check battery
        try:
            sys_msg = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=3)
            if sys_msg:
                voltage = sys_msg.voltage_battery / 1000.0
                if voltage > 20.0:  # 6S LiPo minimum
                    checks['battery_level'] = True
                    print(f"   ✅ Battery: {voltage:.1f}V")
                else:
                    print(f"   ⚠️ Battery: {voltage:.1f}V (LOW)")
            else:
                print("   ❌ Battery: NO DATA")
        except:
            print("   ❌ Battery: TIMEOUT")
        
        # System check summary
        passed = sum(checks.values())
        total = len(checks)
        print(f"\n📊 System Check: {passed}/{total} checks passed")
        
        if passed == total:
            print("🎉 ALL SYSTEMS GO! Mission ready to execute.")
            return True
        elif passed >= total - 1:
            print("⚠️ MINOR ISSUES. Mission can proceed with caution.")
            return True
        else:
            print("❌ MAJOR ISSUES. Mission should not proceed.")
            return False
    
    def execute_waypoint_mission(self, waypoints):
        """Execute waypoint navigation mission"""
        print(f"\n🗺️ Executing waypoint mission ({len(waypoints)} points)")
        
        for i, waypoint in enumerate(waypoints):
            if not self.mission_active:
                break
            
            print(f"   → Waypoint {i+1}/{len(waypoints)}: "
                  f"({waypoint['lat']:.6f}, {waypoint['lon']:.6f}, {waypoint['depth']:.1f}m)")
            
            # Navigate to waypoint
            success = self.navigate_to_waypoint(waypoint)
            
            if success:
                print(f"   ✅ Waypoint {i+1} reached")
                # Hold position for specified time
                hold_time = waypoint.get('hold_time', 5.0)
                print(f"   ⏳ Holding position for {hold_time}s...")
                time.sleep(hold_time)
            else:
                print(f"   ❌ Waypoint {i+1} failed")
                return False
        
        print("✅ Waypoint mission completed successfully!")
        return True
    
    def navigate_to_waypoint(self, waypoint):
        """Navigate to a specific waypoint"""
        target_lat = waypoint['lat']
        target_lon = waypoint['lon']
        target_depth = waypoint['depth']
        
        # Calculate distance and bearing
        distance = self.calculate_distance(
            self.current_position['lat'], self.current_position['lon'],
            target_lat, target_lon
        )
        
        bearing = self.calculate_bearing(
            self.current_position['lat'], self.current_position['lon'],
            target_lat, target_lon
        )
        
        print(f"      Distance: {distance:.1f}m, Bearing: {bearing:.1f}°")
        
        # Navigation timeout
        timeout = max(30.0, distance / self.max_speed * 2)  # 2x expected time
        start_time = time.time()
        
        while time.time() - start_time < timeout and self.mission_active:
            # Check if we've reached the waypoint
            current_distance = self.calculate_distance(
                self.current_position['lat'], self.current_position['lon'],
                target_lat, target_lon
            )
            
            depth_error = abs(self.current_depth - target_depth)
            
            # Waypoint reached criteria
            if current_distance < 2.0 and depth_error < 0.5:  # 2m horizontal, 0.5m depth
                print(f"      ✅ Waypoint reached! (Distance: {current_distance:.1f}m)")
                return True
            
            # Send navigation commands
            self.send_navigation_commands(target_lat, target_lon, target_depth)
            
            time.sleep(1.0)  # 1Hz navigation updates
        
        print(f"      ❌ Waypoint timeout ({timeout:.0f}s)")
        return False
    
    def send_navigation_commands(self, target_lat, target_lon, target_depth):
        """Send navigation commands to autopilot"""
        try:
            # Send position target
            self.master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                0b110111111000,  # type_mask (position only)
                int(target_lat * 1e7),  # lat
                int(target_lon * 1e7),  # lon
                -target_depth,  # alt (negative for depth)
                0, 0, 0,  # velocity
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )
            
        except Exception as e:
            print(f"      ❌ Navigation command error: {e}")
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        """Calculate distance between two GPS points (Haversine formula)"""
        R = 6371000  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lat = math.radians(lat2 - lat1)
        delta_lon = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_lat/2)**2 + 
             math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(delta_lon/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c
    
    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        """Calculate bearing between two GPS points"""
        lat1_rad = math.radians(lat1)
        lat2_rad = math.radians(lat2)
        delta_lon = math.radians(lon2 - lon1)
        
        y = math.sin(delta_lon) * math.cos(lat2_rad)
        x = (math.cos(lat1_rad) * math.sin(lat2_rad) - 
             math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon))
        
        bearing = math.degrees(math.atan2(y, x))
        return (bearing + 360) % 360  # Normalize to 0-360°
    
    def execute_rocket_launch(self, launch_params):
        """Execute rocket launch sequence"""
        print(f"\n🚀 Executing rocket launch sequence")
        print(f"   Target depth: {launch_params['depth']}m")
        print(f"   Launch angle: {launch_params['angle']}°")
        
        # Pre-launch checks
        if not self.perform_pre_launch_checks():
            return False
        
        # Navigate to launch position
        if 'position' in launch_params:
            success = self.navigate_to_waypoint(launch_params['position'])
            if not success:
                print("❌ Failed to reach launch position")
                return False
        
        # Stabilize attitude
        print("   🎯 Stabilizing launch attitude...")
        if not self.stabilize_attitude(launch_params['angle']):
            return False
        
        # Final countdown
        for i in range(5, 0, -1):
            print(f"   🔢 Launch countdown: {i}")
            time.sleep(1.0)
        
        # LAUNCH!
        print("   🚀 ROCKET LAUNCH!")
        success = self.trigger_rocket_launch()
        
        if success:
            print("✅ Rocket launch sequence completed!")
            return True
        else:
            print("❌ Rocket launch failed!")
            return False
    
    def perform_pre_launch_checks(self):
        """Perform pre-launch safety checks"""
        print("   🔍 Pre-launch safety checks...")
        
        # Check depth limits
        if self.current_depth > self.max_depth:
            print(f"   ❌ Depth too deep: {self.current_depth:.1f}m > {self.max_depth}m")
            return False
        
        # Check system status
        if not self.armed:
            print("   ❌ System not armed")
            return False
        
        print("   ✅ Pre-launch checks passed")
        return True
    
    def stabilize_attitude(self, target_angle):
        """Stabilize vehicle attitude for launch"""
        print(f"   🎯 Stabilizing to {target_angle}° pitch...")
        
        timeout = 30.0  # 30 second timeout
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Check attitude error
            pitch_error = abs(self.current_attitude['pitch'] - target_angle)
            roll_error = abs(self.current_attitude['roll'])
            
            if pitch_error < 2.0 and roll_error < 2.0:  # 2° tolerance
                print(f"   ✅ Attitude stabilized (P:{self.current_attitude['pitch']:.1f}°)")
                return True
            
            # Send attitude commands
            self.send_attitude_commands(0, target_angle, self.current_attitude['yaw'])
            
            time.sleep(0.1)  # 10Hz attitude control
        
        print("   ❌ Attitude stabilization timeout")
        return False
    
    def send_attitude_commands(self, roll, pitch, yaw):
        """Send attitude commands"""
        try:
            # Convert to radians
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(yaw)
            
            # Send attitude target
            self.master.mav.set_attitude_target_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                0b00000111,  # type_mask (attitude only)
                [math.cos(roll_rad/2)*math.cos(pitch_rad/2)*math.cos(yaw_rad/2) + 
                 math.sin(roll_rad/2)*math.sin(pitch_rad/2)*math.sin(yaw_rad/2),
                 math.sin(roll_rad/2)*math.cos(pitch_rad/2)*math.cos(yaw_rad/2) - 
                 math.cos(roll_rad/2)*math.sin(pitch_rad/2)*math.sin(yaw_rad/2),
                 math.cos(roll_rad/2)*math.sin(pitch_rad/2)*math.cos(yaw_rad/2) + 
                 math.sin(roll_rad/2)*math.cos(pitch_rad/2)*math.sin(yaw_rad/2),
                 math.cos(roll_rad/2)*math.cos(pitch_rad/2)*math.sin(yaw_rad/2) - 
                 math.sin(roll_rad/2)*math.sin(pitch_rad/2)*math.cos(yaw_rad/2)],  # quaternion
                0, 0, 0,  # body roll rate, pitch rate, yaw rate
                0  # thrust
            )
            
        except Exception as e:
            print(f"   ❌ Attitude command error: {e}")
    
    def trigger_rocket_launch(self):
        """Trigger rocket launch mechanism"""
        try:
            # Send servo command to release rocket (example: AUX7)
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                15,  # servo number (AUX7)
                2000,  # PWM value (release position)
                0, 0, 0, 0, 0  # unused parameters
            )
            
            print("   ✅ Rocket release command sent")
            time.sleep(2.0)  # Hold release for 2 seconds
            
            # Return servo to lock position
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,  # confirmation
                15,  # servo number (AUX7)
                1000,  # PWM value (lock position)
                0, 0, 0, 0, 0  # unused parameters
            )
            
            return True
            
        except Exception as e:
            print(f"   ❌ Rocket launch error: {e}")
            return False
    
    def execute_emergency_surface(self, params=None):
        """Execute emergency surface protocol"""
        print("\n🚨 EMERGENCY SURFACE PROTOCOL ACTIVATED!")
        
        try:
            # Stop all horizontal movement
            self.master.mav.set_position_target_local_ned_send(
                0,  # time_boot_ms
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b110111000111,  # type_mask (velocity only)
                0, 0, 0,  # position
                0, 0, -1.0,  # velocity (1 m/s upward)
                0, 0, 0,  # acceleration
                0, 0  # yaw, yaw_rate
            )
            
            print("   🔼 Emergency ascent command sent")
            
            # Monitor ascent
            start_depth = self.current_depth
            timeout = 60.0  # 1 minute timeout
            start_time = time.time()
            
            while time.time() - start_time < timeout and self.current_depth > 0.5:
                ascent_rate = (start_depth - self.current_depth) / (time.time() - start_time)
                print(f"   📈 Ascending: {self.current_depth:.1f}m (Rate: {ascent_rate:.2f} m/s)")
                time.sleep(2.0)
            
            if self.current_depth <= 0.5:
                print("✅ Emergency surface completed!")
                return True
            else:
                print("❌ Emergency surface timeout!")
                return False
                
        except Exception as e:
            print(f"❌ Emergency surface error: {e}")
            return False
    
    def execute_pattern_search(self, search_params):
        """Execute pattern search mission"""
        print(f"\n🔍 Executing pattern search")
        print(f"   Pattern: {search_params['pattern']}")
        print(f"   Area: {search_params['width']}m x {search_params['height']}m")
        
        # Generate search pattern waypoints
        waypoints = self.generate_search_pattern(search_params)
        
        # Execute waypoint mission
        return self.execute_waypoint_mission(waypoints)
    
    def generate_search_pattern(self, params):
        """Generate search pattern waypoints"""
        pattern = params['pattern']
        width = params['width']
        height = params['height']
        spacing = params.get('spacing', 10.0)  # 10m default spacing
        
        waypoints = []
        center_lat = self.current_position['lat']
        center_lon = self.current_position['lon']
        depth = params.get('depth', 2.0)
        
        if pattern == 'lawnmower':
            # Lawnmower pattern
            num_lines = int(width / spacing)
            for i in range(num_lines):
                # Calculate offset
                y_offset = (i - num_lines/2) * spacing
                
                if i % 2 == 0:  # Even lines: left to right
                    x_start, x_end = -height/2, height/2
                else:  # Odd lines: right to left
                    x_start, x_end = height/2, -height/2
                
                # Add waypoints for this line
                num_points = max(2, int(height / spacing))
                for j in range(num_points):
                    x_offset = x_start + (x_end - x_start) * j / (num_points - 1)
                    
                    # Convert offsets to lat/lon
                    lat_offset = x_offset / 111320.0  # Rough conversion
                    lon_offset = y_offset / (111320.0 * math.cos(math.radians(center_lat)))
                    
                    waypoints.append({
                        'lat': center_lat + lat_offset,
                        'lon': center_lon + lon_offset,
                        'depth': depth,
                        'hold_time': 2.0
                    })
        
        elif pattern == 'spiral':
            # Spiral pattern
            radius = min(width, height) / 2
            num_points = int(2 * math.pi * radius / spacing)
            
            for i in range(num_points):
                angle = 2 * math.pi * i / num_points
                r = radius * i / num_points
                
                x_offset = r * math.cos(angle)
                y_offset = r * math.sin(angle)
                
                # Convert offsets to lat/lon
                lat_offset = x_offset / 111320.0
                lon_offset = y_offset / (111320.0 * math.cos(math.radians(center_lat)))
                
                waypoints.append({
                    'lat': center_lat + lat_offset,
                    'lon': center_lon + lon_offset,
                    'depth': depth,
                    'hold_time': 1.0
                })
        
        print(f"   Generated {len(waypoints)} waypoints for {pattern} pattern")
        return waypoints
    
    def start_mission(self, mission_type, mission_params):
        """Start mission execution"""
        if self.mission_active:
            print("❌ Mission already active!")
            return False
        
        if mission_type not in self.mission_types:
            print(f"❌ Unknown mission type: {mission_type}")
            return False
        
        if not self.connected:
            print("❌ Not connected to vehicle!")
            return False
        
        # Perform system check
        if not self.perform_system_check():
            print("❌ System check failed - mission aborted")
            return False
        
        # Start mission
        self.current_mission = mission_type
        self.mission_active = True
        
        def mission_worker():
            try:
                print(f"\n🚀 Starting mission: {mission_type}")
                mission_func = self.mission_types[mission_type]
                success = mission_func(mission_params)
                
                if success:
                    print(f"✅ Mission '{mission_type}' completed successfully!")
                else:
                    print(f"❌ Mission '{mission_type}' failed!")
                    
            except Exception as e:
                print(f"❌ Mission error: {e}")
            finally:
                self.mission_active = False
                self.current_mission = None
        
        self.mission_thread = threading.Thread(target=mission_worker, daemon=True)
        self.mission_thread.start()
        
        return True
    
    def stop_mission(self):
        """Stop current mission"""
        if self.mission_active:
            print("🛑 Stopping mission...")
            self.mission_active = False
            
            # Wait for mission thread to finish
            if self.mission_thread and self.mission_thread.is_alive():
                self.mission_thread.join(timeout=5.0)
            
            print("✅ Mission stopped")
            return True
        else:
            print("⚠️ No active mission to stop")
            return False
    
    def get_mission_status(self):
        """Get current mission status"""
        return {
            'connected': self.connected,
            'armed': self.armed,
            'mission_active': self.mission_active,
            'current_mission': self.current_mission,
            'position': self.current_position,
            'attitude': self.current_attitude,
            'depth': self.current_depth,
            'timestamp': datetime.now().isoformat()
        }
    
    def disconnect(self):
        """Disconnect from vehicle"""
        self.stop_mission()
        
        if self.master:
            try:
                self.master.close()
                print("🔌 Serial connection closed")
            except:
                pass
        
        self.connected = False

def main():
    """Test mission manager"""
    print("🚀 TEKNOFEST Mission Manager - Test Mode")
    print("=" * 50)
    
    manager = MissionManager()
    
    try:
        # Connect to vehicle
        if manager.connect_mavlink():
            print("\n📋 Mission Manager ready!")
            print("Available missions:")
            print("  • waypoint_navigation")
            print("  • rocket_launch")
            print("  • emergency_surface")
            print("  • pattern_search")
            
            # Example waypoint mission
            waypoints = [
                {'lat': 41.0123, 'lon': 29.0456, 'depth': 2.0, 'hold_time': 5.0},
                {'lat': 41.0124, 'lon': 29.0457, 'depth': 3.0, 'hold_time': 5.0},
                {'lat': 41.0125, 'lon': 29.0458, 'depth': 2.0, 'hold_time': 5.0}
            ]
            
            # Start test mission
            print(f"\n🎯 Starting test waypoint mission...")
            success = manager.start_mission('waypoint_navigation', waypoints)
            
            if success:
                # Monitor mission
                while manager.mission_active:
                    status = manager.get_mission_status()
                    print(f"📍 Position: ({status['position']['lat']:.6f}, {status['position']['lon']:.6f})")
                    print(f"📏 Depth: {status['depth']:.1f}m")
                    time.sleep(5.0)
            
        else:
            print("❌ Failed to connect to vehicle")
    
    except KeyboardInterrupt:
        print("\n⚠️ Mission manager interrupted")
    
    finally:
        manager.disconnect()

if __name__ == "__main__":
    main() 