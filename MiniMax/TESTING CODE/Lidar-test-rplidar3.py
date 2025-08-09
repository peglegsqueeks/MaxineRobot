#!/usr/bin/env python3
"""
RPlidar A3 Comprehensive Test for Maxine the Robot
A3-specific version with protocol compatibility fixes
"""

import time
import math
import serial
import matplotlib.pyplot as plt
from rplidar import RPLidar

class RPLidarA3(RPLidar):
    """Extended RPLidar class with A3-specific fixes"""
    
    def __init__(self, port, baudrate=256000, timeout=3):
        """Initialize with A3-specific settings"""
        super().__init__(port, baudrate, timeout)
    
    def get_info_safe(self):
        """Safe version of get_info that handles A3 descriptor issues"""
        try:
            return self.get_info()
        except Exception as e:
            print(f"Standard get_info failed: {e}")
            try:
                self._send_cmd(0x50)
                time.sleep(0.1)
                descriptor = self._serial_port.read(7)
                if len(descriptor) >= 7:
                    return {
                        'model': 'A3', 
                        'firmware': 'Unknown',
                        'hardware': 'Unknown',
                        'serialnumber': 'A3_Device'
                    }
                else:
                    return {'model': 'A3_Manual', 'status': 'Connected'}
            except:
                return {'model': 'A3_Fallback', 'status': 'Connected'}
    
    def get_health_safe(self):
        """Safe version of get_health for A3"""
        try:
            return self.get_health()
        except Exception as e:
            print(f"Standard get_health failed: {e}")
            return ('Unknown', 0)

def test_rplidar_a3():
    print("RPlidar A3 Comprehensive Test for Maxine the Robot")
    print("A3-Compatible Version")
    print("=" * 50)
    
    lidar = None
    
    try:
        print("=== Connecting to RPlidar A3 ===")
        connection_successful = False
        
        try:
            lidar = RPLidarA3('/dev/ttyUSB0', baudrate=256000, timeout=3)
            print("✓ Connected successfully to /dev/ttyUSB0 (A3 mode)")
            connection_successful = True
        except Exception as e:
            print(f"A3 connection failed: {e}")
        
        if not connection_successful:
            try:
                lidar = RPLidar('/dev/ttyUSB0')
                print("✓ Connected successfully to /dev/ttyUSB0 (standard mode)")
                connection_successful = True
            except Exception as e:
                print(f"Standard connection failed: {e}")
                raise e
        
        if not connection_successful:
            raise Exception("Could not establish connection to LiDAR")
        
        print("=== Device Information ===")
        if hasattr(lidar, 'get_info_safe'):
            info = lidar.get_info_safe()
        else:
            info = lidar.get_info()
        print(f"Device Info: {info}")
        
        if hasattr(lidar, 'get_health_safe'):
            health = lidar.get_health_safe()
        else:
            health = lidar.get_health()
        print(f"Device Health: {health}")
        
        print("\n=== Motor and Scan Test ===")
        print("Starting motor...")
        
        try:
            lidar.stop()
            time.sleep(0.5)
            lidar.stop_motor()
            time.sleep(1)
            lidar.start_motor()
            time.sleep(2)
            print("✓ Motor started successfully")
        except Exception as e:
            print(f"⚠ Motor control issue: {e}")
        
        print("Performing scan test...")
        
        scan_data = []
        scan_count = 0
        max_time = 15
        start_time = time.time()

        # --- Real-Time Plot Initialization ---
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, polar=True)
        scan_plot, = ax.plot([], [], 'ro', markersize=2)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 6000)

        try:
            from collections import deque
            max_points = 360  # 1° resolution

            angles = deque(maxlen=max_points)
            distances = deque(maxlen=max_points)

            for quality, angle, distance, _ in lidar.iter_measurments(max_buf_meas=200):
                if distance > 0 and quality > 0:
                    angles.append(math.radians(angle))
                    distances.append(distance)

                if len(angles) >= max_points:
                    scan_plot.set_data(angles, distances)
                    ax.set_ylim(0, max(1000, max(distances)))
                    plt.draw()
                    plt.pause(0.001)

        except KeyboardInterrupt:
            print("\n⚠ Scan interrupted by user")
        except Exception as e:
            print(f"⚠ Scan error: {e}")

        if scan_data:
            print(f"\n=== Analysis Results ===")
            print(f"Total scans completed: {scan_count}")
            print(f"Total data points: {len(scan_data)}")
            all_distances = [d for (q, a, d) in scan_data if d > 0]
            if all_distances:
                print(f"Distance range: {min(all_distances):.0f} - {max(all_distances):.0f}mm")
                print(f"Average distance: {sum(all_distances)/len(all_distances):.0f}mm")

            print("\n=== Navigation Sectors ===")
            sectors = {
                'Front':     (350, 10),
                'Front-Right': (10, 80),
                'Right':     (80, 100),
                'Back-Right': (100, 170),
                'Back':      (170, 190),
                'Back-Left': (190, 260),
                'Left':      (260, 280),
                'Front-Left': (280, 350)
            }

            for sector_name, (start_angle, end_angle) in sectors.items():
                sector_distances = []
                for (quality, angle, distance) in scan_data:
                    if distance > 0:
                        if start_angle > end_angle:
                            if angle >= start_angle or angle <= end_angle:
                                sector_distances.append(distance)
                        else:
                            if start_angle <= angle <= end_angle:
                                sector_distances.append(distance)
                if sector_distances:
                    closest = min(sector_distances)
                    obstacle_warning = " ⚠ CLOSE!" if closest < 500 else ""
                    print(f"{sector_name:>11}: {closest:>5.0f}mm{obstacle_warning}")
                else:
                    print(f"{sector_name:>11}: No data")
        else:
            print("❌ No scan data collected")

    except Exception as e:
        print(f"❌ Error during LiDAR test: {e}")
        import traceback
        traceback.print_exc()

    finally:
        if lidar:
            try:
                print("\n=== Cleanup ===")
                lidar.stop()
                lidar.stop_motor()
                time.sleep(1)
                lidar.disconnect()
                print("✓ LiDAR stopped and disconnected")
            except Exception as e:
                print(f"⚠ Cleanup warning: {e}")
        plt.ioff()
        plt.show()

    print("\n=== Test Complete ===")

if __name__ == "__main__":
    test_rplidar_a3()
