
import numpy as np
import heapq
from math import hypot, radians

from src.path_finding.LidarPlot import LidarPlot
from src.path_finding.new_a_star import a_star
from src.path_finding.Position import Position

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
import pstats
import io
import cProfile




goal = Position(0, 1500)
plot = LidarPlot(range_mm=4000, debug_algo_path=True)
profiler = cProfile.Profile()
profiler.enable()


path, (padded_obstacles_debug, original_obs_debug, goal_debug, path_debug) = a_star( positions, goal)

profiler.disable()
s = io.StringIO()
sortby = 'cumulative'  # Sort by cumulative time
ps = pstats.Stats(profiler, stream=s).sort_stats(sortby)
ps.print_stats()
print(s.getvalue())  # Display profiling results


plot.plot_path(path)
plot.plot_path_debug(path_debug)
plot.plot_destination_debug(goal_debug)
plot.plot_obstacles_debug(padded_obstacles_debug,original_obs_debug)

plot.plot_obstacles(positions)
plot.plot_destination(goal)
plot.update_plot()

while True:
    continue