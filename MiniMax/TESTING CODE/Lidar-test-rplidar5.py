#!/usr/bin/env python3
"""
RPlidar A3 Comprehensive Test for Maxine the Robot
A3-specific version with real-time plotting using iter_measurments
"""

import time
import math
import matplotlib.pyplot as plt
from collections import deque
from rplidar import RPLidar

class RPLidarA3(RPLidar):
    def __init__(self, port, baudrate=256000, timeout=3):
        super().__init__(port, baudrate, timeout)
    
    def get_info_safe(self):
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
        try:
            lidar = RPLidarA3('/dev/ttyUSB0', baudrate=256000, timeout=3)
            print("✓ Connected successfully to /dev/ttyUSB0 (A3 mode)")
        except Exception as e:
            print(f"Connection failed: {e}")
            raise e

        print("=== Device Information ===")
        info = lidar.get_info_safe()
        print(f"Device Info: {info}")
        health = lidar.get_health_safe()
        print(f"Device Health: {health}")

        print("\n=== Motor and Scan Test ===")
        print("Starting motor...")
        lidar.stop()
        time.sleep(0.5)
        lidar.stop_motor()
        time.sleep(1)
        lidar.start_motor()
        time.sleep(2)
        print("✓ Motor started successfully")
        print("Performing scan test...")

        # --- Real-Time Plot Setup ---
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111, polar=True)
        scan_plot, = ax.plot([], [], 'ro', markersize=2)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 6000)

        angles = deque(maxlen=360)
        distances = deque(maxlen=360)

        for quality, angle, distance, _ in lidar.iter_measurments(max_buf_meas=200):
            if distance > 0 and quality > 0:
                angles.append(math.radians(angle))
                distances.append(distance)

            if len(angles) >= 360:
                scan_plot.set_data(angles, distances)
                ax.set_ylim(0, max(1000, max(distances)))
                fig.canvas.draw()
                fig.canvas.flush_events()

    except KeyboardInterrupt:
        print("\n⚠ Scan interrupted by user")
    except Exception as e:
        print(f"❌ Error during LiDAR test: {e}")
    finally:
        print("\n=== Cleanup ===")
        if lidar:
            lidar.stop()
            lidar.stop_motor()
            time.sleep(1)
            lidar.disconnect()
            print("✓ LiDAR stopped and disconnected")
        plt.ioff()
        plt.close()

    print("\n=== Test Complete ===")

if __name__ == "__main__":
    test_rplidar_a3()
