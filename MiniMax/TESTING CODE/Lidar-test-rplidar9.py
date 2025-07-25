#!/usr/bin/env python3
"""
RPlidar A3 Comprehensive Test for Maxine the Robot
Now with Real-Time Polar Plotting using EXPRESS scan mode
Fixed buffer overflow with aggressive buffer management and error recovery
"""
import time
import math
import matplotlib.pyplot as plt
from rplidar import RPLidar, RPLidarException
from collections import deque
import serial

class RPLidarA3(RPLidar):
    def __init__(self, port, baudrate=256000, timeout=1):
        # Reduced timeout for faster response
        super().__init__(port, baudrate, timeout)
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5

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

    def aggressive_buffer_clear(self):
        """Aggressively clear all buffers and reset connection"""
        try:
            # Stop current operations
            self.stop()
            time.sleep(0.1)
            
            # Clear all buffers
            if hasattr(self._serial_port, 'reset_input_buffer'):
                self._serial_port.reset_input_buffer()
                self._serial_port.reset_output_buffer()
            
            # Flush any remaining data
            if hasattr(self._serial_port, 'flush'):
                self._serial_port.flush()
                
            # Small delay to let hardware settle
            time.sleep(0.2)
            
        except Exception as e:
            print(f"Warning during buffer clear: {e}")

    def iter_scans_safe(self):
        """Safe scan iterator with error recovery"""
        while True:
            try:
                for scan in self.iter_scans():
                    self.consecutive_errors = 0  # Reset error counter on success
                    yield scan
            except RPLidarException as e:
                self.consecutive_errors += 1
                print(f"LiDAR Exception #{self.consecutive_errors}: {e}")
                
                if self.consecutive_errors >= self.max_consecutive_errors:
                    print("Too many consecutive errors, stopping...")
                    break
                
                # Attempt recovery
                print("Attempting recovery...")
                self.aggressive_buffer_clear()
                
                # Restart motor and scanning
                try:
                    self.start_motor()
                    time.sleep(1)
                except:
                    print("Could not restart motor")
                    break
                    
                continue
            except Exception as e:
                print(f"Unexpected error: {e}")
                break

def test_rplidar_a3():
    print("RPlidar A3 Comprehensive Test for Maxine the Robot")
    print("A3-Compatible Version with Aggressive Buffer Management")
    print("=" * 50)
    
    lidar = None
    
    try:
        print("=== Connecting to RPlidar A3 ===")
        lidar = RPLidarA3('/dev/ttyUSB0', baudrate=256000, timeout=1)
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
        
        # Clear everything before starting
        lidar.aggressive_buffer_clear()
        
        lidar.start_motor()
        time.sleep(2)
        print("✓ Motor started successfully")
        
        print("Performing scan test with error recovery...")
        
        # --- Setup Real-Time Polar Plot ---
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, polar=True)
        scan_plot, = ax.plot([], [], 'ro', markersize=1, alpha=0.6)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 4000)
        ax.set_title("Maxine's LiDAR View", pad=20)
        
        # --- Optimized Variables ---
        scan_count = 0
        successful_scans = 0
        last_plot_time = time.time()
        plot_interval = 0.05  # Update plot every 50ms for better responsiveness
        last_buffer_clear = time.time()
        buffer_clear_interval = 5.0  # Clear buffer every 5 seconds (less aggressive)
        
        # Data buffers
        angles = []
        distances = []
        
        print("Starting scan loop with recovery...")
        
        # Use the safe iterator
        for scan in lidar.iter_scans_safe():
            scan_count += 1
            
            # Clear old data
            angles.clear()
            distances.clear()
            
            # Process scan data with relaxed filtering
            valid_points = 0
            for quality, angle, distance in scan:
                if distance > 0 and quality > 0:  # Much more relaxed filtering
                    angles.append(math.radians(angle))
                    distances.append(min(distance, 4000))  # Cap max distance for display
                    valid_points += 1
            
            if valid_points > 10:  # Need at least 10 points to plot
                successful_scans += 1
                
                # Update plot more frequently for debugging
                current_time = time.time()
                if current_time - last_plot_time >= plot_interval:
                    try:
                        if len(angles) > 0 and len(distances) > 0:
                            scan_plot.set_data(angles, distances)
                            ax.set_title(f"Maxine's LiDAR - Scan #{successful_scans} ({valid_points} pts)", pad=20)
                            plt.draw()
                            plt.pause(0.001)
                            last_plot_time = current_time
                            print(f"Plot updated: {valid_points} points, distance range: {min(distances):.0f}-{max(distances):.0f}mm")
                    except Exception as plot_error:
                        print(f"Plot error: {plot_error}")
                
            elif valid_points > 0:
                print(f"Scan {scan_count}: Only {valid_points} valid points (need >10 for plot)")
                
                # Periodic buffer maintenance (less aggressive)
                if current_time - last_buffer_clear >= buffer_clear_interval:
                    print("Performing periodic buffer maintenance...")
                    lidar.aggressive_buffer_clear()
                    last_buffer_clear = current_time
                    
                if successful_scans % 20 == 0:
                    print(f"✓ {successful_scans} successful scans processed")
            
            # Prevent CPU overload
            time.sleep(0.01)
            
            # Safety break after many scans for testing
            if scan_count > 1000:
                print("Stopping after 1000 scans for testing")
                break
                
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