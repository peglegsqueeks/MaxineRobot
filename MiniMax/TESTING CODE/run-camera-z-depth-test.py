#!/usr/bin/env python3
"""
Quick Camera Depth Test Runner
Run this file to test Oak-d pro W camera z depth vs y coordinate method
"""
import sys
import os
import time
import py_trees
from py_trees.common import Status

# Add your project path - adjust this to match your setup
sys.path.append('/home/maxine/')  # Change this to your actual path

try:
    # Import your robot initialization - adjust these imports to match your setup
    from src.robot.RobotFactory import RobotFactory
    from src.z_depth_calibration_test import CameraDepthTestBehavior
    from src.types.CameraMode import CameraMode
except ImportError as e:
    print(f"Import error: {e}")
    print("Please adjust the import paths to match your project structure")
    sys.exit(1)


class QuickCameraDepthTester:
    """Quick test runner for camera depth analysis"""
    
    def __init__(self):
        self.robot = None
        self.test_behavior = None
        
    def initialize_robot(self):
        """Initialize robot with camera - adjust this to match your robot initialization"""
        try:
            # Method 1: If you have a specific robot initialization function
            # from your_main_file import initialize_robot
            # self.robot = initialize_robot()
            
            # Method 2: Using RobotFactory (adjust config path as needed)
            factory = RobotFactory()
            # You may need to provide a config file path here
            # config_path = "/path/to/your/robot_config.json"
            # self.robot = factory.build_robot(config_path)
            
            # Method 3: If you have an existing robot instance running
            # self.robot = your_existing_robot_instance
            
            # For now, let's try the factory approach
            self.robot = factory.build_robot()
            
            # Ensure camera is available and working
            if not hasattr(self.robot, 'camera_sensor') or not self.robot.camera_sensor:
                raise Exception("Robot does not have a working camera sensor")
            
            # Switch camera to object detection mode
            self.robot.camera_sensor.switch_mode(CameraMode.OBJECT_DETECTION)
            time.sleep(2)  # Allow camera to initialize
            
            print("✅ Robot initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize robot: {e}")
            print("\nTroubleshooting:")
            print("1. Check that your robot configuration is correct")
            print("2. Ensure Oak-d pro camera is connected")
            print("3. Verify camera permissions and drivers")
            print("4. Make sure no other processes are using the camera")
            return False
    
    def create_mock_robot_reference(self):
        """Create a mock robot reference for the behavior"""
        class MockRobotReference:
            def __init__(self, robot):
                self._robot = robot
            
            def get_robot(self):
                return self._robot
        
        return MockRobotReference(self.robot)
    
    def run_test(self):
        """Run the camera depth test"""
        print("🎥 Starting Camera Depth Analysis Test")
        print("=" * 50)
        
        # Initialize robot
        if not self.initialize_robot():
            return False
        
        # Create test behavior
        self.test_behavior = CameraDepthTestBehavior()
        
        # Set up robot reference (behaviors expect to inherit from MaxineBehavior)
        mock_ref = self.create_mock_robot_reference()
        self.test_behavior.get_robot = mock_ref.get_robot
        
        try:
            # Setup behavior
            setup_result = self.test_behavior.setup()
            if not setup_result:
                print("❌ Failed to setup test behavior")
                return False
            
            print("✅ Test behavior initialized")
            print("\n📋 TEST INSTRUCTIONS:")
            print("1. Stand in front of the camera")
            print("2. Move slowly from left to right")
            print("3. Pay special attention to the far edges")
            print("4. Watch for red 'FLICKER!' warnings")
            print("5. Press SPACE to pause, ESC to exit")
            print("\n🚀 Starting test in 3 seconds...")
            
            time.sleep(3)
            
            # Run the test loop
            frame_count = 0
            while True:
                try:
                    status = self.test_behavior.update()
                    
                    if status == Status.SUCCESS:
                        print("✅ Test completed successfully")
                        break
                    elif status == Status.FAILURE:
                        print("❌ Test failed")
                        break
                    
                    frame_count += 1
                    
                    # Periodic status update
                    if frame_count % 100 == 0:
                        detection_count = len(self.test_behavior.detection_history)
                        flicker_count = len(self.test_behavior.flicker_events)
                        print(f"📊 Status: {detection_count} detections, {flicker_count} flicker events")
                    
                    # Small delay to prevent overwhelming the system
                    time.sleep(0.1)
                    
                except KeyboardInterrupt:
                    print("\n⏹️ Test interrupted by user")
                    break
                except Exception as e:
                    print(f"❌ Error during test: {e}")
                    break
            
            # Generate final report
            self.generate_final_report()
            
            return True
            
        except Exception as e:
            print(f"❌ Test failed with error: {e}")
            return False
        
        finally:
            # Cleanup
            self.cleanup()
    
    def generate_final_report(self):
        """Generate and display final test report"""
        if not self.test_behavior:
            return
        
        try:
            flicker_count = len(self.test_behavior.flicker_events)
            detection_count = len(self.test_behavior.detection_history)
            
            edge_flickers = sum(1 for event in self.test_behavior.flicker_events 
                              if event.get('is_at_edge', False))
            center_flickers = flicker_count - edge_flickers
            
            print("\n" + "=" * 50)
            print("📊 CAMERA DEPTH TEST SUMMARY")
            print("=" * 50)
            print(f"📈 Total Detections: {detection_count}")
            print(f"⚡ Total Flicker Events: {flicker_count}")
            print(f"🔄 Flickers at Edge: {edge_flickers}")
            print(f"🎯 Flickers at Center: {center_flickers}")
            print(f"📁 Data saved to: {self.test_behavior.log_filename}")
            
            if flicker_count > 0:
                avg_z_change = sum(event.get('z_change', 0) for event in self.test_behavior.flicker_events) / flicker_count
                print(f"📏 Average Z Change During Flicker: {avg_z_change:.0f}mm")
                
                # Analyze edge vs center flicker ratio
                if detection_count > 0:
                    edge_flicker_rate = (edge_flickers / detection_count) * 100
                    center_flicker_rate = (center_flickers / detection_count) * 100
                    print(f"📊 Edge Flicker Rate: {edge_flicker_rate:.1f}%")
                    print(f"📊 Center Flicker Rate: {center_flicker_rate:.1f}%")
            
            print("\n💡 ANALYSIS RECOMMENDATIONS:")
            
            if edge_flickers > center_flickers * 2:
                print("✅ Edge zones show significantly more flickering")
                print("   → Use Y-coordinate method for edge detection")
                print("   → Use Z-depth method for center detection")
            elif flicker_count < detection_count * 0.1:
                print("✅ Z-depth method is very stable")
                print("   → Primary reliance on Z-depth is recommended")
            else:
                print("⚠️ Mixed results - need weighted averaging approach")
            
            print(f"\n📋 Next Steps:")
            print(f"1. Analyze {self.test_behavior.log_filename} for detailed patterns")
            print(f"2. Determine optimal switching thresholds")
            print(f"3. Implement dual-method distance estimation")
            
        except Exception as e:
            print(f"❌ Error generating report: {e}")
    
    def cleanup(self):
        """Cleanup resources"""
        try:
            if self.test_behavior:
                self.test_behavior.terminate(Status.SUCCESS)
            
            if self.robot and hasattr(self.robot, 'camera_sensor'):
                # Reset camera to disabled mode
                self.robot.camera_sensor.switch_mode(CameraMode.DISABLED)
            
            print("🧹 Cleanup completed")
            
        except Exception as e:
            print(f"⚠️ Cleanup error (non-critical): {e}")


def main():
    """Main function"""
    print("🎥 Maxine Robot - Camera Depth Analysis Test")
    print("=" * 50)
    print("This test analyzes Oak-d pro W camera depth perception")
    print("vs y-coordinate distance method for person detection.")
    print("Will help calibrate dual-method distance estimation.")
    print()
    
    # Check if running with proper permissions
    if os.geteuid() != 0:
        print("⚠️  Note: You may need to run as root for camera access")
        print("   Try: sudo python3 run_camera_depth_test.py")
        print()
    
    tester = QuickCameraDepthTester()
    
    try:
        success = tester.run_test()
        if success:
            print("🎉 Test completed successfully!")
        else:
            print("❌ Test failed to complete")
            
    except KeyboardInterrupt:
        print("\n👋 Test interrupted. Goodbye!")
    except Exception as e:
        print(f"💥 Unexpected error: {e}")
    finally:
        tester.cleanup()


if __name__ == "__main__":
    main()