from math import radians
import time
from typing import List
import heapq
import math
from typing import List, Tuple

from matplotlib import pyplot as plt
import numpy as np

from src.path_finding.LidarPlot import LidarPlot
from src.path_finding.Position import Position
import math
import heapq
from typing import List

RESOLUTION = 50
PAD_ROUNDS = 4
DIRECTIONS = [
    (0, 0),
    (0, 1),
    (0, -1),
    (1, 0),
    (1, 1),
    (1, -1),
    (-1, 0),
    (-1, 1),
    (-1, -1)
]


def test_plt(padded_obstacles,obstacles,  goal_cart, path):
    plt.scatter(
        [y[0] for y in padded_obstacles], [y[1] for y in padded_obstacles], color="r"
    )
    plt.scatter(
        [y[0] for y in obstacles], [y[1] for y in obstacles], color="black"
    )
    plt.scatter([y[0] for y in path], [y[1] for y in path], color="b")
    plt.scatter([goal_cart[0]], [goal_cart[1]], color="g")
    plt.show()


def polar_to_cartesian(pos: Position) -> tuple:
    if pos.distance == 0:
        return (0.0, 0.0)
    angle_rad = pos.angle
    x = pos.distance * math.cos(angle_rad)
    y = pos.distance * math.sin(angle_rad)
    return (x, y)


def cartesian_to_polar(x: float, y: float) -> Position:
    distance = math.hypot(x, y)
    if distance == 0:
        return Position(distance=0.0, angle=0.0)
    angle_rad = math.atan2(y, x)
    angle_deg = angle_rad
    return Position(angle=angle_deg, distance=distance*RESOLUTION)


def snap(pos: Position):
    cart = polar_to_cartesian(pos)
    return (int(cart[0] / RESOLUTION),int(cart[1] / RESOLUTION))


def remove_duplicates(coords):
    seen = set()
    unique_coords = []
    
    for coord in coords:
        if coord not in seen:
            seen.add(coord)
            unique_coords.append(coord)
    
    return unique_coords


def pad_obstacles(obstacles):
    padded = []
    for x, y in obstacles:
        pads = [(x_p+x, y_p+y) for x_p, y_p in DIRECTIONS]
        padded.extend(pads)
    return remove_duplicates(padded)        


def pre_process_obstacles(obstacles: List[Position]):
    obstacles = remove_duplicates([snap(obs) for obs in obstacles])
    for _ in range(PAD_ROUNDS):
        obstacles = pad_obstacles(obstacles)
    return obstacles


def a_star(obstacles: List[Position], goal: Position):
    padded_obstacles = pre_process_obstacles(obstacles)
    goal = snap(goal)


    start = (0, 0)
    obstacles_set = set(padded_obstacles)
    
    open_set = []  # Priority queue
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: np.linalg.norm(np.array(start) - np.array(goal), ord=1)}
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            # test_plt(padded_obstacles, [snap(obs) for obs in obstacles],  goal, path)
            return [cartesian_to_polar(step[0], step[1]) for step in path], (padded_obstacles, [polar_to_cartesian(obs) for obs in obstacles], goal, path)
        
        for dx, dy in DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in obstacles_set:
                continue
            
            tentative_g_score = g_score[current] + 1
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + np.linalg.norm(np.array(neighbor) - np.array(goal), ord=1)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
    return []  # No path found


import numpy as np
import heapq
from math import hypot

positions = []
for angle in range(200, 360):
    positions.append(Position(radians(angle), 1000))

for angle in range(0, 100):
    positions.append(Position(radians(angle), 1000))


for distance in range(1000, 1500):
    positions.append(Position(radians(100), distance))


for distance in range(1000, 1600):
    positions.append(Position(radians(200), distance))

    

for distance in range(1700, 3000):
    positions.append(Position(radians(60), distance))


for distance in range(1700, 3000):
    positions.append(Position(radians(270), distance))

    
for distance in range(1000, 3000):
    positions.append(Position(radians(30), distance))


    
for angle in range(160, 200):
    positions.append(Position(radians(angle), 1000))

goal = Position(0, 1500)
plot = LidarPlot(range_mm=4000, debug_algo_path=True)

while True:
    path, (padded_obstacles_debug, original_obs_debug, goal_debug, path_debug) = a_star( positions, goal)
    
    plot.plot_path(path)
    plot.plot_path_debug(path_debug)
    plot.plot_destination_debug(goal_debug)
    plot.plot_obstacles_debug(padded_obstacles_debug,original_obs_debug)

    plot.plot_obstacles(positions)
    plot.plot_destination(goal)
    plot.update_plot()
    time.sleep(1)
