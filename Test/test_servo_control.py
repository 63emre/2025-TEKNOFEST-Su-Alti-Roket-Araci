#!/usr/bin/env python3
"""
TEKNOFEST Su Altı ROV - X-Fin Servo Control Test
Pixhawk PX4 PIX 2.4.8 Serial MAVLink Servo Test
Environment Variable Support: MAV_ADDRESS, MAV_BAUD
"""

import os
import sys
import time
import threading
from pymavlink import mavutil

# Environment variables for serial connection
MAV_ADDRESS = os.getenv("MAV_ADDRESS", "/dev/ttyACM0")
MAV_BAUD = int(os.getenv("MAV_BAUD", "115200"))

print(f"🔧 Serial Configuration:")
print(f"   Port: {MAV_ADDRESS}")
print(f"   Baud: {MAV_BAUD}")

# X-Fin Servo Mapping (AUX → MAVLink Channel)
SERVO_CHANNELS = {
    'fin_front_left': 9,   # AUX1 → Channel 9
    'fin_front_right': 11, # AUX3 → Channel 11
    'fin_rear_left': 12,   # AUX4 → Channel 12
    'fin_rear_right': 13   # AUX5 → Channel 13
}

# PWM Values
PWM_MIN = 1000
PWM_MID = 1500  
PWM_MAX = 2000

class ServoControlTester:
    """Serial MAVLink servo control test sınıfı"""
    
    def __init__(self):
        """Test sınıfını başlat"""
        self.master = None
        self.connected = False
        self.armed = False
        
        print(f"🔧 Servo Control Tester for serial: {MAV_ADDRESS}@{MAV_BAUD}")
        print(f"🎮 X-Fin Configuration: {SERVO_CHANNELS}")
    
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
                self.armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print("✅ Serial MAVLink connection established!")
                print(f"   System ID: {self.master.target_system}")
                print(f"   Component ID: {self.master.target_component}")
                print(f"   Armed Status: {'ARMED' if self.armed else 'DISARMED'}")
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
        """Servo PWM değeri ayarla"""
        if not self.connected:
            return False
        
        # PWM limitlerini kontrol et
        pwm_value = max(PWM_MIN, min(PWM_MAX, pwm_value))
        
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
            print(f"❌ Servo PWM command failed: {e}")
            return False
    
    def test_individual_servos(self):
        """Bireysel servo testi"""
        print("\n🎮 Testing individual servos...")
        
        test_sequence = [
            ('Neutral', PWM_MID),
            ('Min Position', PWM_MIN),
            ('Max Position', PWM_MAX),
            ('Neutral', PWM_MID)
        ]
        
        for servo_name, channel in SERVO_CHANNELS.items():
            print(f"\n🔧 Testing {servo_name} (Channel {channel}):")
            
            for position_name, pwm_value in test_sequence:
                print(f"   → {position_name}: {pwm_value}µs")
                
                if self.set_servo_pwm(channel, pwm_value):
                    print(f"   ✅ Command sent successfully")
                else:
                    print(f"   ❌ Command failed")
                
                time.sleep(2.0)  # 2 saniye bekle
        
        print("✅ Individual servo test completed")
    
    def test_synchronized_servos(self):
        """Senkron servo testi"""
        print("\n🎮 Testing synchronized servo movement...")
        
        test_patterns = [
            ("All Neutral", {ch: PWM_MID for ch in SERVO_CHANNELS.values()}),
            ("All Min", {ch: PWM_MIN for ch in SERVO_CHANNELS.values()}),
            ("All Max", {ch: PWM_MAX for ch in SERVO_CHANNELS.values()}),
            ("X-Pattern 1", {9: PWM_MIN, 11: PWM_MAX, 12: PWM_MIN, 13: PWM_MAX}),
            ("X-Pattern 2", {9: PWM_MAX, 11: PWM_MIN, 12: PWM_MAX, 13: PWM_MIN}),
            ("Front-Rear 1", {9: PWM_MIN, 11: PWM_MIN, 12: PWM_MAX, 13: PWM_MAX}),
            ("Front-Rear 2", {9: PWM_MAX, 11: PWM_MAX, 12: PWM_MIN, 13: PWM_MIN}),
            ("All Neutral", {ch: PWM_MID for ch in SERVO_CHANNELS.values()})
        ]
        
        for pattern_name, servo_values in test_patterns:
            print(f"\n🎯 Pattern: {pattern_name}")
            
            # Tüm servo'ları aynı anda ayarla
            success_count = 0
            for channel, pwm_value in servo_values.items():
                if self.set_servo_pwm(channel, pwm_value):
                    success_count += 1
                    print(f"   Channel {channel}: {pwm_value}µs ✅")
                else:
                    print(f"   Channel {channel}: {pwm_value}µs ❌")
            
            print(f"   Result: {success_count}/{len(servo_values)} commands successful")
            time.sleep(3.0)  # 3 saniye bekle
        
        print("✅ Synchronized servo test completed")
    
    def test_smooth_movement(self):
        """Yumuşak hareket testi"""
        print("\n🎮 Testing smooth servo movement...")
        
        # Tüm servo'ları yumuşak bir şekilde hareket ettir
        steps = 20
        duration = 10  # 10 saniye
        
        print(f"   • Steps: {steps}")
        print(f"   • Duration: {duration}s")
        print(f"   • Update rate: {steps/duration:.1f} Hz")
        
        for step in range(steps + 1):
            # PWM değerini hesapla (MIN'den MAX'e)
            progress = step / steps
            pwm_value = int(PWM_MIN + (PWM_MAX - PWM_MIN) * progress)
            
            print(f"   Step {step+1}/{steps+1}: {pwm_value}µs ({progress*100:.0f}%)")
            
            # Tüm servo'ları aynı değere ayarla
            success_count = 0
            for channel in SERVO_CHANNELS.values():
                if self.set_servo_pwm(channel, pwm_value):
                    success_count += 1
            
            print(f"     → {success_count}/{len(SERVO_CHANNELS)} servos updated")
            time.sleep(duration / steps)
        
        # Neutral'a geri dön
        print("   → Returning to neutral...")
        for channel in SERVO_CHANNELS.values():
            self.set_servo_pwm(channel, PWM_MID)
        
        print("✅ Smooth movement test completed")
    
    def test_x_wing_control_matrix(self):
        """X-Wing kontrol matrisi testi"""
        print("\n🎮 Testing X-Wing control matrix...")
        
        # X-Wing control movements
        movements = [
            ("Roll Left", {9: PWM_MIN, 11: PWM_MAX, 12: PWM_MIN, 13: PWM_MAX}),
            ("Roll Right", {9: PWM_MAX, 11: PWM_MIN, 12: PWM_MAX, 13: PWM_MIN}),
            ("Pitch Up", {9: PWM_MIN, 11: PWM_MIN, 12: PWM_MAX, 13: PWM_MAX}),
            ("Pitch Down", {9: PWM_MAX, 11: PWM_MAX, 12: PWM_MIN, 13: PWM_MIN}),
            ("Yaw Left", {9: PWM_MIN, 11: PWM_MAX, 12: PWM_MAX, 13: PWM_MIN}),
            ("Yaw Right", {9: PWM_MAX, 11: PWM_MIN, 12: PWM_MIN, 13: PWM_MAX}),
            ("Neutral", {ch: PWM_MID for ch in SERVO_CHANNELS.values()})
        ]
        
        for movement_name, servo_values in movements:
            print(f"\n🎯 X-Wing Movement: {movement_name}")
            
            # Servo değerlerini göster
            fin_names = ['Front Left', 'Front Right', 'Rear Left', 'Rear Right']
            channels = [9, 11, 12, 13]
            
            for fin_name, channel in zip(fin_names, channels):
                pwm_value = servo_values[channel]
                direction = "MIN" if pwm_value == PWM_MIN else "MAX" if pwm_value == PWM_MAX else "MID"
                print(f"   {fin_name} (Ch{channel}): {pwm_value}µs ({direction})")
            
            # Servo komutlarını gönder
            success_count = 0
            for channel, pwm_value in servo_values.items():
                if self.set_servo_pwm(channel, pwm_value):
                    success_count += 1
            
            print(f"   Result: {success_count}/{len(servo_values)} commands successful")
            time.sleep(3.0)  # 3 saniye bekle
        
        print("✅ X-Wing control matrix test completed")
    
    def run_full_test(self):
        """Tam servo test paketi"""
        print("🚀 TEKNOFEST ROV - Full X-Fin Servo Test")
        print("=" * 60)
        
        # Bağlantı testi
        if not self.connect():
            print("❌ Connection test failed!")
            return False
        
        # ARM durumu kontrolü
        if not self.armed:
            print("⚠️ System is DISARMED. Servo commands may not work.")
            print("💡 Consider arming the system for full servo functionality.")
        
        # Test suite
        tests = [
            ("Individual Servo Test", self.test_individual_servos),
            ("Synchronized Servo Test", self.test_synchronized_servos),
            ("Smooth Movement Test", self.test_smooth_movement),
            ("X-Wing Control Matrix Test", self.test_x_wing_control_matrix)
        ]
        
        results = []
        for test_name, test_func in tests:
            try:
                print(f"\n{'='*20} {test_name} {'='*20}")
                test_func()
                results.append((test_name, True))
                print(f"✅ {test_name}: COMPLETED")
            except Exception as e:
                print(f"❌ {test_name}: ERROR - {e}")
                results.append((test_name, False))
        
        # Test özeti
        print(f"\n📋 SERVO TEST SUMMARY")
        print("=" * 40)
        completed = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = "✅ COMPLETED" if result else "❌ FAILED"
            print(f"   {test_name}: {status}")
        
        print(f"\n🎯 Overall Result: {completed}/{total} tests completed")
        
        if completed == total:
            print("🎉 ALL SERVO TESTS COMPLETED! X-Fin control system is ready!")
        elif completed > total // 2:
            print("⚠️ PARTIAL SUCCESS. Some tests failed but basic servo control works.")
        else:
            print("❌ MAJOR ISSUES. Servo control system needs troubleshooting.")
        
        return completed == total
    
    def disconnect(self):
        """Bağlantıyı kapat"""
        # Önce tüm servo'ları neutral'a getir
        if self.connected:
            print("🔄 Setting all servos to neutral position...")
            for channel in SERVO_CHANNELS.values():
                self.set_servo_pwm(channel, PWM_MID)
            time.sleep(1.0)
        
        if self.master:
            try:
                self.master.close()
                print("🔌 Serial connection closed")
            except:
                pass
        self.connected = False

def main():
    """Ana test fonksiyonu"""
    tester = ServoControlTester()
    
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