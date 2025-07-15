#!/usr/bin/env python3
"""
RPLidar A3 Obstacle Plotting using pyrplidar in Stability Mode
"""
import time
import math
import threading
import queue
import multiprocessing
import matplotlib.pyplot as plt
from pyrplidar import PyRPlidar

class StableRPLidar:
    def __init__(self, port='/dev/ttyUSB0'):
        self.lidar = PyRPlidar()
        self.port = port
        self.running = False

    def start(self):
        self.lidar.connect(self.port, 256000)
        print("PyRPlidar Info : device is connected")

        self.lidar.setMotorPWM(660)
        time.sleep(1.5)

        # Enter stability scan mode (typical for A3)
        scan_modes = self.lidar.getAllSupportedScanModes()
        stability_mode = next((m for m in scan_modes if "stability" in m['scan_mode'].lower()), scan_modes[0])
        print(f"Using scan mode: {stability_mode['scan_mode']}")
        self.lidar.startScanExpress(force=False, scanMode=stability_mode['id'])
        self.running = True

    def stop(self):
        self.running = False
        self.lidar.stop()
        self.lidar.disconnect()

    def grab_scan(self):
        return self.lidar.getScanData()

def data_acquisition(lidar, scan_queue, running_flag):
    print("[Thread] LiDAR data acquisition started")
    try:
        while running_flag[0]:
            scan_data = lidar.grab_scan()
            obstacles = [(math.radians(d[1]), d[0]) for d in scan_data if d[0] > 0 and d[0] < 4000]

            if not scan_queue.empty():
                try:
                    scan_queue.get_nowait()
                except queue.Empty:
                    pass
            scan_queue.put_nowait(obstacles)

            time.sleep(0.01)
    except Exception as e:
        print(f"[LIDAR ERROR] {e}")
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
    ax.set_title("RPLidar A3 - Stability Mode Obstacle Plot")

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
