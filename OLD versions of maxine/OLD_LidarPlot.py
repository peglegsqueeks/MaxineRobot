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
        ax = plt.subplot(121, projection='polar')
        debug_ax = plt.subplot(122)
        debug_ax.set_xlim(-self.range // 50, self.range// 50)
        debug_ax.set_ylim(-self.range // 50, self.range// 50)
        # debug_ax.invert_yaxis()
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

        if debug_algo_path:            
            self.obstacles_debug = debug_ax.scatter([], [], s=10, color="blue")
            self.obstacles_debug_original = debug_ax.scatter([], [], s=10, color="black")
            self.destination_debug = debug_ax.scatter([], [], 100, marker="*", color="red")
            (self.path_debug,) = debug_ax.plot([], [], "r-")


        fig.set_size_inches((20, 10)) 
        plt.ion()
        plt.show()

    def plot_obstacles(self, readings: List[Position]):
        angles = [reading.angle for reading in readings]
        distances = [reading.distance for reading in readings]

        self.obstacles.set_offsets(np.c_[angles, distances])

    def plot_obstacles_debug(self, readings, original_obs_debug):
        readings = np.array(readings)
        self.obstacles_debug.set_offsets(readings[:, [1, 0]])
        original = np.array(original_obs_debug)

        self.obstacles_debug_original.set_offsets(original[:, [1, 0]])

        
    def plot_destination(self, destination: Position):
        self.destination.set_offsets(np.c_[[destination.angle], [destination.distance]])

    def plot_destination_debug(self, destination):
        self.destination_debug.set_offsets(np.c_[[destination[1]], [destination[0]]])
    
    def plot_path(self, path: List[Position]):
        angles = [reading.angle for reading in path]
        distances = [reading.distance for reading in path]
        gg = np.c_[angles, distances]
        self.path.set_data(gg[:, 0], gg[:, 1])

    def plot_path_debug(self, path):
        path = np.array(path)[:, [1, 0]]
        self.path_debug.set_data(path.swapaxes(0, 1))

    def update_plot(self):
        # plt.draw()
        plt.pause(0.1)

    def close(self):
        plt.cla()
        plt.clf()
        plt.close()
