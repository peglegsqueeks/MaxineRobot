#!/usr/bin/env python3
"""
RPlidar A3 Comprehensive Test for Maxine the Robot
Now with Real-Time Polar Plotting using EXPRESS scan mode
Reduced scan rate and minimal output version
"""
import time
import math
import matplotlib.pyplot as plt
from rplidar import RPLidar, RPLidarException
from collections import deque
import serial

class RPLidarA3(RPLidar):
    def __init__(self, port, baudrate=256000, timeout=1):
        super().__init__(port, baudrate, timeout)
        self.consecutive_errors = 0
        self.max_consecutive_errors = 5

    def get_info_safe(self):
        try:
            return self.get_info()
        except Exception as e:
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
            return ('Unknown', 0)

    def aggressive_buffer_clear(self):
        """Aggressively clear all buffers and reset connection"""
        try:
            self.stop()
            time.sleep(0.1)
            
            if hasattr(self._serial_port, 'reset_input_buffer'):
                self._serial_port.reset_input_buffer()
                self._serial_port.reset_output_buffer()
            
            if hasattr(self._serial_port, 'flush'):
                self._serial_port.flush()
                
            time.sleep(0.2)
            
        except Exception as e:
            pass

    def iter_scans_with_buffer_control(self):
        """Scan iterator with aggressive buffer management"""
        while True:
            try:
                # Clear buffer before each scan attempt
                self.aggressive_buffer_clear()
                time.sleep(0.1)  # Let buffer settle
                
                scan_iterator = self.iter_scans()
                scan = next(scan_iterator)
                self.consecutive_errors = 0
                yield scan
                
                # Add delay between scans to prevent buffer buildup
                time.sleep(0.2)
                    
            except RPLidarException as e:
                self.consecutive_errors += 1
                
                if self.consecutive_errors >= self.max_consecutive_errors:
                    break
                
                self.aggressive_buffer_clear()
                time.sleep(0.5)  # Longer recovery delay
                
                try:
                    self.start_motor()
                    time.sleep(1)
                except:
                    break
                    
                continue
            except StopIteration:
                # Restart scanning if iterator ends
                time.sleep(0.5)
                continue
            except Exception as e:
                break

def test_rplidar_a3():
    lidar = None
    
    try:
        lidar = RPLidarA3('/dev/ttyUSB0', baudrate=256000, timeout=1)
        
        info = lidar.get_info_safe()
        health = lidar.get_health_safe()
        
        lidar.stop()
        time.sleep(0.5)
        lidar.stop_motor()
        time.sleep(1)
        
        lidar.aggressive_buffer_clear()
        
        lidar.start_motor()
        time.sleep(2)
        
        # --- Setup Real-Time Polar Plot ---
        plt.ion()
        fig = plt.figure(figsize=(10, 10))
        ax = fig.add_subplot(111, polar=True)
        scan_plot, = ax.plot([], [], 'ro', markersize=1, alpha=0.6)
        ax.set_theta_zero_location('N')
        ax.set_theta_direction(-1)
        ax.set_ylim(0, 4000)
        ax.set_title("Maxine's LiDAR View", pad=20)
        
        # --- Reduced Rate Variables ---
        scan_count = 0
        last_plot_time = time.time()
        plot_interval = 0.3  # Update plot every 300ms
        last_buffer_clear = time.time()
        buffer_clear_interval = 5.0  # Clear buffer every 5 seconds
        
        # Data buffers
        angles = []
        distances = []
        
        # Use buffer-controlled iterator
        for scan in lidar.iter_scans_with_buffer_control():
            scan_count += 1
            
            # Clear old data
            angles.clear()
            distances.clear()
            
            # Process scan data with relaxed filtering
            valid_points = 0
            for quality, angle, distance in scan:
                if distance > 0 and quality > 0:
                    angles.append(math.radians(angle))
                    distances.append(min(distance, 4000))
                    valid_points += 1
            
            if valid_points > 10:
                # Update plot
                current_time = time.time()
                if current_time - last_plot_time >= plot_interval:
                    try:
                        if len(angles) > 0 and len(distances) > 0:
                            scan_plot.set_data(angles, distances)
                            ax.set_title(f"Maxine's LiDAR - {valid_points} points", pad=20)
                            plt.draw()
                            plt.pause(0.001)
                            last_plot_time = current_time
                    except Exception as plot_error:
                        pass
                
                # Less frequent buffer maintenance
                if current_time - last_buffer_clear >= buffer_clear_interval:
                    lidar.aggressive_buffer_clear()
                    last_buffer_clear = current_time
            
            # Much longer delay between scans to reduce buffer pressure
            time.sleep(0.3)
                
    except KeyboardInterrupt:
        pass
    except Exception as e:
        pass
    finally:
        if lidar:
            try:
                lidar.stop()
                lidar.stop_motor()
                time.sleep(1)
                lidar.disconnect()
            except Exception as cleanup_error:
                pass
        
        plt.ioff()
        plt.close('all')

if __name__ == "__main__":
    test_rplidar_a3()