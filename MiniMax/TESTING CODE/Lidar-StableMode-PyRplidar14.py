#!/usr/bin/env python3
"""
RPLidar A3 Obstacle Plotting using pyrplidar (v0.1.2)
Cleaned to use only valid public API methods.
"""
import time
import math
import threading
import queue
import multiprocessing
import traceback
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar, PyRPlidarProtocolError

class StableRPLidar:
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = PyRPlidar()
        self.port = port
        self.running = False
        self.scan_generator = None

    def start(self):
        print("PyRPlidar Info : device is connecting...")
        try:
            self.lidar.connect(self.port, 256000)
            self.lidar.reset()
            time.sleep(1.5)  # Allow motor to stabilize
            self.scan_generator = self.lidar.start_scan()
            self.running = True
            print("PyRPlidar Info : device is connected and scanning")
        except PyRPlidarProtocolError as e:
            print("[LIDAR START ERROR] Protocol error, full reconnect...")
            self.lidar.stop()
            self.lidar.disconnect()
            time.sleep(1)
            try:
                self.lidar.connect(self.port, 256000)
                self.lidar.reset()
                time.sleep(1.5)
                self.scan_generator = self.lidar.start_scan()
                self.running = True
                print("PyRPlidar Info : reconnected and scanning")
            except Exception as e2:
                print("[LIDAR RECONNECT FAILED]", e2)
                traceback.print_exc()
                raise

    def stop(self):
        self.running = False
        self.lidar.stop()
        self.lidar.disconnect()

    def grab_scan(self):
        scan = []
        try:
            while True:
                point = next(self.scan_generator)
                scan.append((point.distance, point.angle))
                if point.start_bit:
                    break
        except StopIteration:
            pass
        except Exception as e:
            print("[grab_scan ERROR]", e)
            traceback.print_exc()
        return scan

def data_acquisition(lidar, scan_queue, running_flag):
    print("[Thread] LiDAR data acquisition started")
    try:
        while running_flag[0]:
            scan_data = lidar.grab_scan()
            obstacles = [(math.radians(angle), distance) for distance, angle in scan_data if distance > 0 and distance < 4000]

            if not scan_queue.empty():
                try:
                    scan_queue.get_nowait()
                except queue.Empty:
                    pass
            scan_queue.put_nowait(obstacles)

            time.sleep(0.01)
    except Exception as e:
        print(f"[LIDAR ERROR] {e}")
        traceback.print_exc()
    finally:
        lidar.stop()
        print("[Thread] LiDAR acquisition stopped")

def main():
    port = '/dev/ttyUSB0'
    scan_queue = queue.Queue(maxsize=5)
    running_flag = [True]

    lidar = StableRPLidar(port)
    lidar.start()

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
