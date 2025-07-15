from math import radians
import math
import time

from matplotlib import pyplot as plt
import numpy as np
from src.path_finding.LidarPlot import LidarPlot
from src.path_finding.PolarAStarSearchVector import (
    a_star_search,
    prebuild_obstacles,
    convert_from_search_coord,
)
from src.path_finding.Position import Position
import cProfile
import pstats


def normalize_radians(angle):
    return angle % (2 * math.pi)


def get_obstacles_1():
    obstacles = []

    for angle in range(-128, 200, 1):
        obstacles.append(Position(normalize_radians(math.radians(angle)), 1000))

    for d in range(1000, 2500):
        obstacles.append(Position(math.radians(45), d))

    for d in range(1000, 1500):
        obstacles.append(Position(normalize_radians(math.radians(-55)), d))

    for angle in range(320, 70, -1):
        obstacles.append(Position(math.radians(angle), 1500))

    for angle in range(45, 300):
        obstacles.append(Position(math.radians(angle), 2275))

    return obstacles


def get_obstacles_2():
    obstacles = []

    for angle in range(-128, 200, 1):
        obstacles.append(Position(normalize_radians(math.radians(angle)), 1000))

    for d in range(1000, 2500):
        obstacles.append(Position(math.radians(45), d))

    for d in range(1000, 1500):
        obstacles.append(Position(normalize_radians(math.radians(-55)), d))

    return obstacles


def get_obstacles_2():
    obstacles = []

    for angle in range(-128, 200, 1):
        obstacles.append(Position(normalize_radians(math.radians(angle)), 1000))

    for d in range(1000, 2500):
        obstacles.append(Position(math.radians(45), d))

    for d in range(1000, 3000):
        obstacles.append(Position(math.radians(-45), d))

    for d in range(1000, 1500):
        obstacles.append(Position(normalize_radians(math.radians(-55)), d))

    return obstacles


def plot_res(plot, goal, path, obstacles):
    # obstacles = [
    #     convert_from_search_coord(*obs) for obs in list(prebuild_obstacles(obstacles))
    # ]

    plot.plot_destination(goal)
    plot.plot_obstacles(obstacles)
    plot.plot_path(path)
    plot.update_plot()


OBS = [get_obstacles_1(), get_obstacles_2()]

if __name__ == "__main__":

    plot = LidarPlot(3000)
    goal = Position(radians(22), 2000)

    obstacles = get_obstacles_1()
    while True:
        path = a_star_search(goal, obstacles)
        plot_res(plot, goal, path, obstacles)
        if np.random.random() < 0.5:
            idx = np.random.randint(len(OBS))
            obstacles = OBS[idx]
        time.sleep(1)
