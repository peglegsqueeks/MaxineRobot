#!/usr/bin/env python3
"""
Test script for PID-controlled head servo system
Tests initialization, centering, and smooth movement
Run this to verify your servo system is working properly before using with main robot code

Usage: python3 servo_test.py
"""

import time
import sys
import signal

# Add the path to your project modules
sys.path.append('/home/jetson/maxine/MiniMax')

from src.action_managers.ServoController import ServoController
from src.action_managers.HeadMoveManager import HeadVelocityManager


class ServoTest:
    def __init__(self):
        self.servo_controller = None
        self.head_manager = None
        self.running = True
        
        # Set up signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\n🛑 Shutdown signal received...")
        self.running = False
        self.cleanup()
        sys.exit(0)
    
    def initialize_servo_system(self):
        """Initialize the servo controller and head manager"""
        print("=" * 60)
        print("🤖 MAXINE ROBOT HEAD SERVO TEST")
        print("=" * 60)
        print("Testing PID-controlled head movement system")
        print("Servo specs: 30kg servo with 4kg head assembly")
        print("I2C Address: 11 (0x0b)")
        print()
        
        try:
            # Initialize servo controller
            print("🔧 Initializing servo controller...")
            self.servo_controller = ServoController(i2c_address=11)
            
            if not self.servo_controller.initialize():
                print("❌ Failed to initialize servo controller")
                return False
            
            print("✅ Servo controller initialized successfully")
            
            # Initialize head manager
            print("🔧 Initializing head movement manager...")
            self.head_manager = HeadVelocityManager(self.servo_controller)
            print("✅ Head movement manager initialized")
            
            return True
            
        except Exception as e:
            print(f"❌ Error initializing servo system: {e}")
            return False
    
    def test_basic_info(self):
        """Test basic servo information retrieval"""
        print("\n" + "=" * 40)
        print("📊 BASIC SERVO INFORMATION")
        print("=" * 40)
        
        try:
            status = self.servo_controller.get_status()
            print(f"Current position: {status['current_position']:.3f}")
            print(f"Target position: {status['target_position']:.3f}")
            print(f"Current angle: {status['current_angle_deg']:.1f}°")
            print(f"Target angle: {status['target_angle_deg']:.1f}°")
            print(f"Movement active: {status['movement_active']}")
            print(f"Position error: {status['position_error']:.3f}")
            
        except Exception as e:
            print(f"❌ Error getting servo status: {e}")
            return False
        
        return True
    
    def test_centering(self):
        """Test head centering functionality"""
        print("\n" + "=" * 40)
        print("🎯 TESTING HEAD CENTERING")
        print("=" * 40)
        
        try:
            print("Testing servo controller centering...")
            success = self.servo_controller.center()
            
            if success:
                print("⏱️ Waiting for centering to complete...")
                success = self.servo_controller.wait_for_position(timeout=10.0)
                
                if success:
                    final_pos = self.servo_controller.get_position()
                    final_angle = self.servo_controller.get_angle_degrees()
                    print(f"✅ Centering completed!")
                    print(f"   Final position: {final_pos:.3f}")
                    print(f"   Final angle: {final_angle:.1f}°")
                    print(f"   Centering accuracy: ±{abs(final_pos):.3f}")
                else:
                    print("⚠️ Centering timeout - movement may still be in progress")
            else:
                print("❌ Failed to start centering")
                return False
            
            # Test head manager centering
            print("\nTesting head manager centering...")
            time.sleep(1)
            success = self.head_manager.center_head()
            
            if success:
                print("✅ Head manager centering completed!")
            else:
                print("⚠️ Head manager centering had issues")
            
        except Exception as e:
            print(f"❌ Error during centering test: {e}")
            return False
        
        return True
    
    def test_smooth_movements(self):
        """Test smooth PID-controlled movements"""
        print("\n" + "=" * 40)
        print("🎯 TESTING SMOOTH MOVEMENTS")
        print("=" * 40)
        
        test_angles = [20, -30, 45, -45, 0]  # Test sequence
        
        try:
            for angle in test_angles:
                print(f"\n🎯 Moving to {angle:.1f}°...")
                
                # Start movement
                success = self.servo_controller.move_to_angle(angle)
                if not success:
                    print(f"❌ Failed to start movement to {angle:.1f}°")
                    continue
                
                # Monitor movement progress
                start_time = time.time()
                timeout = 8.0
                
                while time.time() - start_time < timeout:
                    current_angle = self.servo_controller.get_angle_degrees()
                    target_angle = self.servo_controller.get_target_angle_degrees()
                    error = abs(target_angle - current_angle)
                    
                    print(f"   Progress: {current_angle:6.1f}° → {target_angle:6.1f}° (error: {error:.2f}°)")
                    
                    if error <= 1.0:  # Close enough
                        break
                    
                    time.sleep(0.5)
                
                # Check final position
                final_angle = self.servo_controller.get_angle_degrees()
                error = abs(angle - final_angle)
                
                if error <= 2.0:
                    print(f"✅ Reached {final_angle:.1f}° (target: {angle:.1f}°, error: {error:.1f}°)")
                else:
                    print(f"⚠️ Position error: {final_angle:.1f}° (target: {angle:.1f}°, error: {error:.1f}°)")
                
                time.sleep(1.0)  # Pause between movements
        
        except Exception as e:
            print(f"❌ Error during movement test: {e}")
            return False
        
        return True
    
    def test_step_movements(self):
        """Test incremental step movements"""
        print("\n" + "=" * 40)
        print("🎯 TESTING STEP MOVEMENTS")
        print("=" * 40)
        
        try:
            # Center first
            self.servo_controller.center()
            self.servo_controller.wait_for_position(timeout=5.0)
            
            print("Testing left steps...")
            for i in range(3):
                print(f"   Step {i+1} left...")
                success = self.head_manager.move_left()
                if success:
                    time.sleep(1.5)  # Allow movement
                    angle = self.head_manager.get_head_angle_degrees()
                    print(f"   Position: {angle:.1f}°")
                else:
                    print("   ❌ Left step failed")
            
            time.sleep(2)
            
            print("Testing right steps...")
            for i in range(6):  # Go past center to right
                print(f"   Step {i+1} right...")
                success = self.head_manager.move_right()
                if success:
                    time.sleep(1.5)  # Allow movement
                    angle = self.head_manager.get_head_angle_degrees()
                    print(f"   Position: {angle:.1f}°")
                else:
                    print("   ❌ Right step failed")
            
            # Return to center
            print("Returning to center...")
            self.head_manager.center_head()
            
        except Exception as e:
            print(f"❌ Error during step movement test: {e}")
            return False
        
        return True
    
    def test_scan_pattern(self):
        """Test scanning pattern"""
        print("\n" + "=" * 40)
        print("🔄 TESTING SCAN PATTERN")
        print("=" * 40)
        
        try:
            success = self.head_manager.scan_left_right(
                angle_range=60.0,  # ±30 degrees
                steps=5,
                dwell_time=1.0
            )
            
            if success:
                print("✅ Scan pattern completed successfully!")
            else:
                print("❌ Scan pattern failed")
                return False
            
        except Exception as e:
            print(f"❌ Error during scan test: {e}")
            return False
        
        return True
    
    def interactive_test(self):
        """Interactive test mode"""
        print("\n" + "=" * 40)
        print("🎮 INTERACTIVE TEST MODE")
        print("=" * 40)
        print("Commands:")
        print("  l - Move left")
        print("  r - Move right") 
        print("  c - Center")
        print("  s - Status")
        print("  scan - Scan pattern")
        print("  <angle> - Move to specific angle (e.g., 45, -30)")
        print("  q - Quit")
        print()
        
        while self.running:
            try:
                cmd = input("Enter command: ").strip().lower()
                
                if cmd == 'q':
                    break
                elif cmd == 'l':
                    print("Moving left...")
                    self.head_manager.move_left()
                elif cmd == 'r':
                    print("Moving right...")
                    self.head_manager.move_right()
                elif cmd == 'c':
                    print("Centering...")
                    self.head_manager.center_head()
                elif cmd == 's':
                    status = self.servo_controller.get_status()
                    print(f"Position: {status['current_position']:.3f} ({status['current_angle_deg']:.1f}°)")
                    print(f"Target: {status['target_position']:.3f} ({status['target_angle_deg']:.1f}°)")
                    print(f"Moving: {status['movement_active']}")
                elif cmd == 'scan':
                    print("Starting scan...")
                    self.head_manager.scan_left_right(60.0, 5, 1.0)
                else:
                    # Try to parse as angle
                    try:
                        angle = float(cmd)
                        if -90 <= angle <= 90:
                            print(f"Moving to {angle:.1f}°...")
                            self.servo_controller.move_to_angle(angle)
                        else:
                            print("Angle must be between -90 and +90 degrees")
                    except ValueError:
                        print("Unknown command")
                
                time.sleep(0.1)  # Small delay
                
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(f"Error: {e}")
    
    def cleanup(self):
        """Clean up resources"""
        if self.servo_controller:
            print("🔧 Shutting down servo controller...")
            self.servo_controller.shutdown()
    
    def run_all_tests(self):
        """Run all automated tests"""
        print("🚀 Starting automated test sequence...")
        
        tests = [
            ("Basic Info", self.test_basic_info),
            ("Centering", self.test_centering),
            ("Smooth Movements", self.test_smooth_movements),
            ("Step Movements", self.test_step_movements),
            ("Scan Pattern", self.test_scan_pattern),
        ]
        
        passed = 0
        total = len(tests)
        
        for test_name, test_func in tests:
            print(f"\n🧪 Running {test_name} test...")
            try:
                if test_func():
                    print(f"✅ {test_name} test PASSED")
                    passed += 1
                else:
                    print(f"❌ {test_name} test FAILED")
            except Exception as e:
                print(f"❌ {test_name} test FAILED with exception: {e}")
        
        print(f"\n" + "=" * 60)
        print(f"🏁 TEST SUMMARY: {passed}/{total} tests passed")
        print("=" * 60)
        
        if passed == total:
            print("🎉 All tests passed! Your servo system is working perfectly.")
        else:
            print("⚠️ Some tests failed. Please check the output above.")
        
        return passed == total


def main():
    test = ServoTest()
    
    try:
        if not test.initialize_servo_system():
            print("❌ Failed to initialize servo system")
            return
        
        # Ask user what they want to do
        while test.running:
            print("\n" + "=" * 40)
            print("🎮 SERVO TEST MENU")
            print("=" * 40)
            print("1. Run all automated tests")
            print("2. Interactive test mode")
            print("3. Exit")
            
            choice = input("\nSelect option (1-3): ").strip()
            
            if choice == '1':
                test.run_all_tests()
            elif choice == '2':
                test.interactive_test()
            elif choice == '3':
                break
            else:
                print("Invalid choice")
    
    except KeyboardInterrupt:
        print("\n🛑 Test interrupted by user")
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
    finally:
        test.cleanup()
        print("👋 Test complete!")


if __name__ == "__main__":
    main()