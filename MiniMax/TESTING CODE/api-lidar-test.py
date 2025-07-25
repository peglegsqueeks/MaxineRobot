#!/usr/bin/env python3
"""
RPlidar A3 Complete Test Script for Maxine the Robot
Tests all functionality and provides working scan examples
"""

import time
import sys
from pyrplidar import PyRPlidar

class RPLidarA3Tester:
    def __init__(self, port='/dev/ttyUSB0', baudrate=256000):
        self.port = port
        self.baudrate = baudrate
        self.lidar = PyRPlidar()
        self.connected = False
        
    def connect(self):
        """Connect to the RPlidar A3"""
        print("=== Connecting to RPlidar A3 ===")
        try:
            self.lidar.connect(port=self.port, baudrate=self.baudrate, timeout=3)
            if self.lidar.lidar_serial is not None:
                print(f"✓ Connected successfully to {self.port}")
                self.connected = True
                return True
            else:
                print("✗ Connection failed - no serial connection")
                return False
        except Exception as e:
            print(f"✗ Connection failed: {e}")
            return False
    
    def get_device_info(self):
        """Get detailed device information"""
        if not self.connected:
            return
            
        print("\n=== Device Information ===")
        try:
            info = self.lidar.get_info()
            print(f"Device Info: {info}")
        except Exception as e:
            print(f"✗ Failed to get info: {e}")
        
        try:
            health = self.lidar.get_health()
            print(f"Device Health: {health}")
        except Exception as e:
            print(f"✗ Failed to get health: {e}")
            
        try:
            conf = self.lidar.get_lidar_conf()
            print(f"Lidar Config: {conf}")
        except Exception as e:
            print(f"✗ Failed to get config: {e}")
    
    def test_scan_modes(self):
        """Test and display available scan modes"""
        if not self.connected:
            return
            
        print("\n=== Scan Mode Information ===")
        try:
            mode_count = self.lidar.get_scan_mode_count()
            print(f"Available scan modes: {mode_count}")
            
            typical_mode = self.lidar.get_scan_mode_typical()
            print(f"Typical scan mode: {typical_mode}")
            
            scan_modes = self.lidar.get_scan_modes()
            print(f"All scan modes: {scan_modes}")
            
            return scan_modes, typical_mode
        except Exception as e:
            print(f"✗ Failed to get scan modes: {e}")
            return None, None
    
    def test_motor_control(self):
        """Test motor PWM control"""
        if not self.connected:
            return
            
        print("\n=== Motor Control Test ===")
        try:
            # Start motor at default speed
            print("Starting motor...")
            self.lidar.set_motor_pwm(600)  # Default PWM for A3
            time.sleep(2)
            print("✓ Motor started")
            
            # Test different speeds
            for pwm in [400, 600, 800]:
                print(f"Setting motor PWM to {pwm}")
                self.lidar.set_motor_pwm(pwm)
                time.sleep(1)
                
        except Exception as e:
            print(f"✗ Motor control failed: {e}")
    
    def test_basic_scan(self):
        """Test basic scanning functionality"""
        if not self.connected:
            return
            
        print("\n=== Basic Scan Test ===")
        try:
            # Start motor first
            self.lidar.set_motor_pwm(600)
            time.sleep(2)
            
            print("Starting basic scan...")
            scan_gen = self.lidar.start_scan()
            
            print("Collecting 50 measurements...")
            measurements = []
            count = 0
            
            for measurement in scan_gen:
                if measurement[0] > 0:  # Valid measurement
                    measurements.append(measurement)
                    if count % 10 == 0:
                        angle, distance, quality = measurement
                        print(f"  Angle: {angle:6.2f}°, Distance: {distance:6.1f}mm, Quality: {quality}")
                    count += 1
                    
                if count >= 50:
                    break
                    
            print(f"✓ Basic scan successful - collected {len(measurements)} valid measurements")
            return measurements
            
        except Exception as e:
            print(f"✗ Basic scan failed: {e}")
            return None
        finally:
            try:
                self.lidar.stop()
            except:
                pass
    
    def test_express_scan(self, mode=None):
        """Test express scan mode"""
        if not self.connected:
            return
            
        print(f"\n=== Express Scan Test (Mode: {mode}) ===")
        try:
            # Start motor first
            self.lidar.set_motor_pwm(600)
            time.sleep(2)
            
            if mode is None:
                mode = 2  # Try mode 2 for A3 (typically 10Hz)
                
            print(f"Starting express scan with mode {mode}...")
            scan_gen = self.lidar.start_scan_express(mode)
            
            print("Collecting 50 measurements...")
            measurements = []
            count = 0
            
            for measurement in scan_gen:
                if measurement[0] > 0:  # Valid measurement
                    measurements.append(measurement)
                    if count % 10 == 0:
                        angle, distance, quality = measurement
                        print(f"  Angle: {angle:6.2f}°, Distance: {distance:6.1f}mm, Quality: {quality}")
                    count += 1
                    
                if count >= 50:
                    break
                    
            print(f"✓ Express scan successful - collected {len(measurements)} valid measurements")
            return measurements
            
        except Exception as e:
            print(f"✗ Express scan failed: {e}")
            return None
        finally:
            try:
                self.lidar.stop()
            except:
                pass
    
    def test_force_scan(self):
        """Test force scan functionality"""
        if not self.connected:
            return
            
        print("\n=== Force Scan Test ===")
        try:
            print("Performing force scan (single 360° sweep)...")
            measurements = self.lidar.force_scan()
            
            if measurements:
                valid_measurements = [m for m in measurements if m[0] > 0]
                print(f"✓ Force scan successful - {len(valid_measurements)} valid measurements")
                
                # Show some sample measurements
                for i in range(0, min(len(valid_measurements), 10)):
                    angle, distance, quality = valid_measurements[i]
                    print(f"  Angle: {angle:6.2f}°, Distance: {distance:6.1f}mm, Quality: {quality}")
                    
                return valid_measurements
            else:
                print("✗ Force scan returned no measurements")
                return None
                
        except Exception as e:
            print(f"✗ Force scan failed: {e}")
            return None
    
    def run_comprehensive_test(self):
        """Run all tests in sequence"""
        print("RPlidar A3 Comprehensive Test for Maxine the Robot")
        print("=" * 50)
        
        # Connect
        if not self.connect():
            print("Cannot proceed without connection. Check:")
            print("- USB cable connection")
            print("- Port permissions (try: sudo chmod 666 /dev/ttyUSB0)")
            print("- Correct port (try: ls /dev/ttyUSB*)")
            return False
        
        # Get device info
        self.get_device_info()
        
        # Test scan modes
        scan_modes, typical_mode = self.test_scan_modes()
        
        # Test motor control
        self.test_motor_control()
        
        # Test force scan (doesn't require motor)
        force_measurements = self.test_force_scan()
        
        # Test basic scan
        basic_measurements = self.test_basic_scan()
        
        # Test express scan with different modes
        if scan_modes:
            for mode_id in [0, 1, 2, 3]:  # Try common modes
                express_measurements = self.test_express_scan(mode_id)
                if express_measurements:
                    break  # Stop at first working mode
        
        # Summary
        print("\n=== Test Summary ===")
        print(f"✓ Connection: {'Success' if self.connected else 'Failed'}")
        print(f"✓ Force Scan: {'Success' if force_measurements else 'Failed'}")
        print(f"✓ Basic Scan: {'Success' if basic_measurements else 'Failed'}")
        
        # Disconnect
        try:
            self.lidar.disconnect()
            print("✓ Disconnected successfully")
        except Exception as e:
            print(f"Disconnect warning: {e}")
        
        return True
    
    def minimal_working_example(self):
        """Minimal example for integration into Maxine"""
        print("\n=== Minimal Working Example ===")
        print("Here's the minimal code for Maxine integration:")
        print("""
# Minimal RPlidar A3 integration for Maxine
from pyrplidar import PyRPlidar
import time

lidar = PyRPlidar()
lidar.connect(port='/dev/ttyUSB0', baudrate=256000, timeout=3)

# Method 1: Force scan (single sweep, no motor control needed)
measurements = lidar.force_scan()
for angle, distance, quality in measurements[:10]:
    print(f"Angle: {angle:6.2f}°, Distance: {distance:6.1f}mm")

# Method 2: Continuous scan (requires motor control)
lidar.set_motor_pwm(600)  # Start motor
time.sleep(2)  # Wait for motor to stabilize

scan_gen = lidar.start_scan()
for i, (angle, distance, quality) in enumerate(scan_gen):
    if i >= 100:  # Get 100 measurements
        break
    if distance > 0:  # Valid measurement
        print(f"Angle: {angle:6.2f}°, Distance: {distance:6.1f}mm")

lidar.stop()
lidar.disconnect()
        """)

def main():
    # You may need to adjust the port
    tester = RPLidarA3Tester(port='/dev/ttyUSB0', baudrate=256000)
    
    try:
        success = tester.run_comprehensive_test()
        if success:
            tester.minimal_working_example()
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Test failed with error: {e}")
    finally:
        try:
            tester.lidar.disconnect()
        except:
            pass

if __name__ == "__main__":
    main()