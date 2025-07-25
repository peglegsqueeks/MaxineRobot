#!/usr/bin/env python3
"""
LiDAR Chase Mode Test Script
Run this script to test LiDAR chase mode independently
"""

import sys
import time
import pygame
import signal

# Add the src directory to the Python path
sys.path.append('/home/jetson/Maxine_Robot/src')

from sensors.LidarSensor import LidarSensor
from sensors.CameraSensor import CameraSensor
from types.CameraMode import CameraMode
from types.RobotModes import RobotMode


class LidarChaseTest:
    """
    Test class for LiDAR chase functionality
    """
    
    def __init__(self):
        self.lidar_sensor = None
        self.camera_sensor = None
        self.running = False
        self.test_results = {}
        
    def setup_signal_handler(self):
        """Setup signal handler for graceful shutdown"""
        signal.signal(signal.SIGINT, self._signal_handler)
        
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        print("\n🛑 Shutdown signal received...")
        self.running = False
        
    def test_lidar_initialization(self):
        """Test 1: LiDAR sensor initialization"""
        print("🔧 Test 1: LiDAR Sensor Initialization")
        print("-" * 50)
        
        try:
            # Create LiDAR sensor
            self.lidar_sensor = LidarSensor()
            print("✅ LiDAR sensor object created")
            
            # Initialize LiDAR sensor
            print("🚀 Initializing LiDAR sensor...")
            success = self.lidar_sensor.initialize()
            
            if success:
                print("✅ LiDAR sensor initialized successfully")
                print("🔄 LiDAR motor should be spinning now")
                
                # Wait for stabilization
                print("⏳ Waiting for LiDAR to stabilize...")
                time.sleep(5.0)
                
                # Check status
                status = self.lidar_sensor.get_status()
                print(f"📊 LiDAR Status: {status}")
                
                self.test_results['lidar_init'] = True
                return True
            else:
                print("❌ Failed to initialize LiDAR sensor")
                self.test_results['lidar_init'] = False
                return False
                
        except Exception as e:
            print(f"❌ LiDAR initialization error: {e}")
            self.test_results['lidar_init'] = False
            return False
    
    def test_lidar_scanning(self):
        """Test 2: LiDAR scanning functionality"""
        print("\n🔧 Test 2: LiDAR Scanning")
        print("-" * 50)
        
        if not self.lidar_sensor:
            print("❌ LiDAR sensor not available")
            return False
            
        try:
            # Get scan readings
            print("📡 Getting LiDAR scan readings...")
            
            for i in range(5):
                readings = self.lidar_sensor.get_reading()
                
                if readings:
                    print(f"📊 Scan {i+1}: {len(readings)} obstacles detected")
                    
                    # Show some sample obstacles
                    if len(readings) > 0:
                        sample_obstacles = readings[:3]  # First 3 obstacles
                        for j, obs in enumerate(sample_obstacles):
                            print(f"   Obstacle {j+1}: {obs}")
                else:
                    print(f"⚠️ Scan {i+1}: No readings available")
                
                time.sleep(1.0)
            
            self.test_results['lidar_scan'] = True
            return True
            
        except Exception as e:
            print(f"❌ LiDAR scanning error: {e}")
            self.test_results['lidar_scan'] = False
            return False
    
    def test_camera_initialization(self):
        """Test 3: Camera sensor initialization"""
        print("\n🔧 Test 3: Camera Sensor Initialization")
        print("-" * 50)
        
        try:
            # Create camera sensor
            self.camera_sensor = CameraSensor()
            print("✅ Camera sensor object created")
            
            # Switch to object detection mode
            print("📷 Switching to object detection mode...")
            self.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
            
            # Wait for initialization
            time.sleep(3.0)
            
            # Check camera info
            camera_info = self.camera_sensor.get_camera_info()
            print(f"📊 Camera Info: {camera_info}")
            
            self.test_results['camera_init'] = True
            return True
            
        except Exception as e:
            print(f"❌ Camera initialization error: {e}")
            self.test_results['camera_init'] = False
            return False
    
    def test_pygame_display(self):
        """Test 4: Pygame display system"""
        print("\n🔧 Test 4: Pygame Display System")
        print("-" * 50)
        
        try:
            # Initialize pygame
            if not pygame.get_init():
                pygame.init()
                print("✅ Pygame initialized")
            else:
                print("✅ Pygame already initialized")
            
            # Create test display
            display_info = pygame.display.Info()
            print(f"📺 Display info: {display_info.current_w}x{display_info.current_h}")
            
            # Create a test surface
            screen = pygame.display.set_mode((800, 600))
            pygame.display.set_caption("LiDAR Chase Test")
            
            # Draw test pattern
            screen.fill((0, 0, 0))  # Black background
            pygame.draw.circle(screen, (255, 0, 0), (400, 300), 50)  # Red circle
            pygame.display.flip()
            
            print("✅ Test display created - you should see a red circle")
            print("⏳ Displaying for 3 seconds...")
            
            # Keep display for 3 seconds
            start_time = time.time()
            while time.time() - start_time < 3.0:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        break
                time.sleep(0.1)
            
            # Clean up display
            pygame.display.quit()
            print("✅ Display test completed")
            
            self.test_results['pygame_display'] = True
            return True
            
        except Exception as e:
            print(f"❌ Pygame display error: {e}")
            self.test_results['pygame_display'] = False
            return False
    
    def test_integrated_system(self):
        """Test 5: Integrated LiDAR + Camera system"""
        print("\n🔧 Test 5: Integrated System Test")
        print("-" * 50)
        
        if not self.lidar_sensor or not self.camera_sensor:
            print("❌ Required sensors not available")
            return False
            
        try:
            print("🔄 Running integrated system test...")
            print("Press Ctrl+C to stop")
            
            self.running = True
            update_count = 0
            
            while self.running and update_count < 50:  # Max 50 updates
                try:
                    # Get LiDAR readings
                    lidar_readings = self.lidar_sensor.get_reading()
                    lidar_count = len(lidar_readings) if lidar_readings else 0
                    
                    # Get camera readings
                    camera_reading = self.camera_sensor.get_reading()
                    camera_active = camera_reading is not None
                    
                    # Status update every 10 cycles
                    if update_count % 10 == 0:
                        print(f"📊 Update {update_count}: LiDAR={lidar_count} obstacles, Camera={'Active' if camera_active else 'Inactive'}")
                    
                    update_count += 1
                    time.sleep(0.5)
                    
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"⚠️ Error in integrated test: {e}")
                    continue
            
            print("✅ Integrated system test completed")
            self.test_results['integrated_system'] = True
            return True
            
        except Exception as e:
            print(f"❌ Integrated system test error: {e}")
            self.test_results['integrated_system'] = False
            return False
    
    def cleanup(self):
        """Clean up all resources"""
        print("\n🧹 Cleaning up resources...")
        
        try:
            # Shutdown LiDAR sensor
            if self.lidar_sensor:
                print("🛑 Shutting down LiDAR sensor...")
                self.lidar_sensor.shutdown()
                print("✅ LiDAR sensor shutdown complete")
            
            # Shutdown camera sensor
            if self.camera_sensor:
                print("🛑 Shutting down camera sensor...")
                self.camera_sensor.stop_camera()
                print("✅ Camera sensor shutdown complete")
            
            # Cleanup pygame
            if pygame.get_init():
                pygame.quit()
                print("✅ Pygame cleaned up")
            
            print("✅ All resources cleaned up")
            
        except Exception as e:
            print(f"⚠️ Cleanup error: {e}")
    
    def print_test_summary(self):
        """Print summary of all tests"""
        print("\n" + "="*60)
        print("🎯 LiDAR CHASE TEST SUMMARY")
        print("="*60)
        
        total_tests = len(self.test_results)
        passed_tests = sum(self.test_results.values())
        
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"{test_name.replace('_', ' ').title():.<30} {status}")
        
        print("-" * 60)
        print(f"Total Tests: {total_tests}")
        print(f"Passed: {passed_tests}")
        print(f"Failed: {total_tests - passed_tests}")
        print(f"Success Rate: {(passed_tests/total_tests*100):.1f}%")
        
        if passed_tests == total_tests:
            print("\n🎉 ALL TESTS PASSED! LiDAR Chase should work.")
        else:
            print("\n⚠️ Some tests failed. Check the errors above.")
        
        print("="*60)
    
    def run_all_tests(self):
        """Run all tests in sequence"""
        print("🚀 Starting LiDAR Chase Test Suite")
        print("="*60)
        
        # Setup signal handler
        self.setup_signal_handler()
        
        try:
            # Run all tests
            self.test_lidar_initialization()
            self.test_lidar_scanning()
            self.test_camera_initialization()
            self.test_pygame_display()
            self.test_integrated_system()
            
        except KeyboardInterrupt:
            print("\n🛑 Tests interrupted by user")
        except Exception as e:
            print(f"\n❌ Test suite error: {e}")
        finally:
            # Always cleanup
            self.cleanup()
            self.print_test_summary()


def main():
    """Main test function"""
    print("🤖 Maxine Robot - LiDAR Chase Mode Test")
    print("This script tests the LiDAR chase components independently")
    print("Make sure the robot is powered on and LiDAR is connected")
    print("")
    
    # Create and run test
    test = LidarChaseTest()
    test.run_all_tests()


if __name__ == "__main__":
    main()