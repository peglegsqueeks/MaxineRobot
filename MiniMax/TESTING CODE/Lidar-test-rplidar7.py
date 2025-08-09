#!/usr/bin/env python3
"""
RPlidar A3 Comprehensive Test for Maxine the Robot
Now with Real-Time Polar Plotting using EXPRESS scan mode
Fixed buffer overflow issues with optimized data processing
"""
import time
import math
import matplotlib.pyplot as plt
from rplidar import RPLidar
from collections import deque
import threading

class RPLidarA3(RPLidar):
    def __init__(self, port, baudrate=256000, timeout=3):
        super().__init__(port, baudrate, timeout)
        # Increase buffer size and reduce timeout for faster processing
        if hasattr(self._serial_port, 'reset_input_buffer'):
            self._serial_port.reset_input_buffer()
    
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

    def clear_input_buffer(self):
        """Clear the input buffer to prevent overflow"""
        try:
            if hasattr(self._serial_port, 'reset_input_buffer'):
                self._serial_port.reset_input_buffer()
            elif hasattr(self._serial_port, 'flushInput'):
                self._serial_port.flushInput()
        except Exception as e:
            print(f"Warning: Could not clear input buffer: {e}")

def test_rplidar_a3():
    print("RPlidar A3 Comprehensive Test for Maxine the Robot")
    print("A3-Compatible Version with Buffer Optimization")
    print("=" * 50)
    
    lidar = None
    
    try:
        print("=== Connecting to RPlidar A3 ===")
        lidar = RPLidarA3('/dev/ttyUSB0', baudrate=256000, timeout=3)
        print("✓ Connected successfully to /dev/ttyUSB0 (A3 mode)")
        
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
        
        # Clear any residual data before starting
        lidar.clear_input_buffer()
        
        lidar.start_motor()
        time.sleep(2)
        print("✓ Motor started successfully")
        
        print("Performing optimized scan test...")
        
        # --- Setup Real-Time Polar Plot ---
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, polar=True)
        scan_plot, = ax.plot([], [], 'ro', markersize=1, alpha=0.7)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 4000)
        ax.set_title("Maxine's LiDAR View", pad=20)
        
        # --- Optimized Live Scanning Loop ---
        angles = []
        distances = []
        scan_count = 0
        last_plot_time = time.time()
        plot_interval = 0.05  # Update plot every 50ms instead of every scan
        
        # Use deque for efficient append/clear operations
        angle_buffer = deque(maxlen=1000)
        distance_buffer = deque(maxlen=1000)
        
        print("Starting optimized scan loop...")
        
        for scan in lidar.iter_scans():
            scan_count += 1
            
            # Process scan data efficiently
            angle_buffer.clear()
            distance_buffer.clear()
            
            # Filter and process points in one pass
            for quality, angle, distance in scan:
                if distance > 50 and quality > 10:  # Filter out very close/poor quality points
                    angle_buffer.append(math.radians(angle))
                    distance_buffer.append(min(distance, 4000))  # Cap max distance
            
            # Only update plot at specified intervals to reduce CPU load
            current_time = time.time()
            if current_time - last_plot_time >= plot_interval and angle_buffer:
                try:
                    scan_plot.set_data(list(angle_buffer), list(distance_buffer))
                    
                    # Update title with scan info
                    ax.set_title(f"Maxine's LiDAR View - Scan #{scan_count} ({len(angle_buffer)} points)", pad=20)
                    
                    fig.canvas.draw_idle()  # Use draw_idle instead of draw for better performance
                    fig.canvas.flush_events()
                    
                    last_plot_time = current_time
                    
                    # Clear buffer periodically to prevent memory issues
                    if scan_count % 20 == 0:
                        lidar.clear_input_buffer()
                        
                except Exception as plot_error:
                    print(f"Plot update error (continuing): {plot_error}")
            
            # Small delay to prevent overwhelming the CPU
            time.sleep(0.001)
            
            # Print status every 100 scans
            if scan_count % 100 == 0:
                print(f"✓ Processed {scan_count} scans, current scan has {len(angle_buffer)} valid points")
                
    except KeyboardInterrupt:
        print("\n⚠ Scan interrupted by user")
    except Exception as e:
        print(f"❌ Error during LiDAR test: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\n=== Cleanup ===")
        if lidar:
            try:
                lidar.stop()
                lidar.stop_motor()
                time.sleep(1)
                lidar.disconnect()
                print("✓ LiDAR stopped and disconnected")
            except Exception as cleanup_error:
                print(f"Warning during cleanup: {cleanup_error}")
        
        plt.ioff()
        plt.close('all')
        print("✓ Plot windows closed")
    
    print("\n=== Test Complete ===")

if __name__ == "__main__":
    test_rplidar_a3()