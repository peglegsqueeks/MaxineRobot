#!/usr/bin/env python3
"""
Simplified RPlidar A3 Real-time Obstacle Plotting
Fixed for GUI thread safety and robust LiDAR acquisition
"""
import time
import math
import threading
import queue
import multiprocessing
import matplotlib.pyplot as plt
from rplidar import RPLidar, RPLidarException

class SimpleRPLidar(RPLidar):
    def aggressive_buffer_clear(self):
        try:
            self.stop()
            time.sleep(0.05)
            self._serial_port.reset_input_buffer()
            self._serial_port.reset_output_buffer()
            self._serial_port.flush()
            time.sleep(0.1)
        except:
            pass

def data_acquisition(lidar, scan_queue, running_flag):
    print("[Thread] LiDAR data acquisition started")
    buffer_clear_counter = 0
    try:
        for scan in lidar.iter_scans():
            if not running_flag[0]:
                break
            try:
                obstacles = [(math.radians(((angle + 180) % 360) - 180), distance)
                             for quality, angle, distance in scan if quality > 0 and distance > 0 and distance < 4000]

                if not scan_queue.empty():
                    try:
                        scan_queue.get_nowait()
                    except queue.Empty:
                        pass
                scan_queue.put_nowait(obstacles)

                buffer_clear_counter += 1
                if buffer_clear_counter >= 100:
                    lidar.aggressive_buffer_clear()
                    buffer_clear_counter = 0
                    time.sleep(0.01)

            except Exception as e:
                print(f"Scan error: {e}")
    except RPLidarException as e:
        print(f"LIDAR exception: {e}")
    finally:
        lidar.stop()
        lidar.stop_motor()
        lidar.disconnect()
        print("[Thread] LiDAR acquisition stopped")

def main():
    port = '/dev/ttyUSB0'
    scan_queue = queue.Queue(maxsize=5)
    running_flag = [True]

    lidar = SimpleRPLidar(port, baudrate=256000, timeout=1)
    lidar.stop()
    lidar.stop_motor()
    lidar.aggressive_buffer_clear()
    lidar.start_motor()
    time.sleep(2.5)  # Allow motor to reach full speed

    # Start acquisition thread only
    threading.Thread(target=data_acquisition, args=(lidar, scan_queue, running_flag), daemon=True).start()

    print("[Main] Starting visualization...")
    plt.ion()
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, polar=True)
    scatter, = ax.plot([], [], 'b.', markersize=1.5)
    ax.set_theta_zero_location('N')
    ax.set_theta_direction(-1)
    ax.set_ylim(0, 4000)
    ax.set_title("RPLidar A3 - Obstacle Plot")

    try:
        print("[Main] Press Ctrl+C to stop")
        while True:
            try:
                obstacles = scan_queue.get_nowait()
                if obstacles:
                    angles, distances = zip(*obstacles)
                    scatter.set_data(angles, distances)
                    ax.set_title(f"Obstacle Points: {len(obstacles)}")
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
            except queue.Empty:
                pass
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n[Main] Shutting down...")
    finally:
        running_flag[0] = False
        time.sleep(2)
        plt.ioff()
        plt.close(fig)
        print("[Main] Visualization stopped")

if __name__ == "__main__":
    multiprocessing.set_start_method('spawn', force=True)
    main()
