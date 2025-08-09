from .Position import Position


import numpy as np
from matplotlib import pyplot as plt


from typing import List


class LidarPlot:
    def __init__(self, range_mm: int, debug_algo_path=False) -> None:
        self.range = range_mm
        self.debug_algo_path = debug_algo_path

        self.init_plot(debug_algo_path)

    def init_plot(self, debug_algo_path):
        fig = plt.figure()
        # Only create the polar plot, no right subplot
        ax = plt.subplot(111, projection='polar')

        # Remove the debug_ax creation since we're only showing left half
        if debug_algo_path:
            # If you still need debug plotting, you could create a separate window
            # or overlay it on the polar plot, but for now removing it
            pass
        
        # angle
        ax.set_theta_zero_location("N")
        ax.set_theta_direction(-1)
        ax.set_xticks(np.radians(np.arange(0, 360, 45)))

        # distance
        range_points = [(i + 1) * self.range / 4 for i in range(4)]
        ax.set_yticks(range_points)
        ax.set_rmax(self.range)
        ax.set_rticks(range_points)

        ax.grid(True)

        self.obstacles = ax.scatter([], [], s=10, color="blue")
        self.destination = ax.scatter([], [], 100, marker="*", color="red")
        (self.path,) = ax.plot([], [], "r-")

        # Remove debug plotting elements since we're not showing the right subplot
        if debug_algo_path:            
            # These would need to be handled differently if you still want debug info
            # For now, setting them to None to avoid errors
            self.obstacles_debug = None
            self.obstacles_debug_original = None
            self.destination_debug = None
            self.path_debug = None

        # Adjust figure size - you might want to make it more square now
        fig.set_size_inches((10, 10))  # Changed from (20, 10) to (10, 10)
        
        # Store figure reference
        self.fig = fig
        
        plt.ion()
        
        # Show the plot without stealing focus from pygame window
        plt.show(block=False)
        
        # Additional focus management - send focus back to pygame
        try:
            # Try to minimize the matplotlib window or send it to background
            mng = fig.canvas.manager
            if hasattr(mng, 'window'):
                if hasattr(mng.window, 'wm_attributes'):
                    # For tkinter backend - keep window but don't steal focus
                    mng.window.wm_attributes('-topmost', False)
                elif hasattr(mng.window, 'setWindowFlags'):
                    # For Qt backend
                    from matplotlib.backends.qt_compat import QtCore
                    mng.window.setWindowFlags(QtCore.Qt.WindowStaysOnBottomHint)
        except:
            # If focus management fails, continue anyway
            pass

    def plot_obstacles(self, readings: List[Position]):
        angles = [reading.angle for reading in readings]
        distances = [reading.distance for reading in readings]

        self.obstacles.set_offsets(np.c_[angles, distances])

    def plot_obstacles_debug(self, readings, original_obs_debug):
        # Skip debug plotting since we removed the debug subplot
        if self.debug_algo_path and self.obstacles_debug is not None:
            readings = np.array(readings)
            self.obstacles_debug.set_offsets(readings[:, [1, 0]])
            original = np.array(original_obs_debug)
            self.obstacles_debug_original.set_offsets(original[:, [1, 0]])

        
    def plot_destination(self, destination: Position):
        self.destination.set_offsets(np.c_[[destination.angle], [destination.distance]])

    def plot_destination_debug(self, destination):
        # Skip debug plotting since we removed the debug subplot
        if self.debug_algo_path and self.destination_debug is not None:
            self.destination_debug.set_offsets(np.c_[[destination[1]], [destination[0]]])
    
    def plot_path(self, path: List[Position]):
        angles = [reading.angle for reading in path]
        distances = [reading.distance for reading in path]
        gg = np.c_[angles, distances]
        self.path.set_data(gg[:, 0], gg[:, 1])

    def plot_path_debug(self, path):
        # Skip debug plotting since we removed the debug subplot
        if self.debug_algo_path and self.path_debug is not None:
            path = np.array(path)[:, [1, 0]]
            self.path_debug.set_data(path.swapaxes(0, 1))

    def update_plot(self):
        # plt.draw()
        plt.pause(0.1)

    def close(self):
        plt.cla()
        plt.clf()
        plt.close()
