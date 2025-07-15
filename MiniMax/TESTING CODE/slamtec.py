# from pyrplidar import PyRPlidar
# import time
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# from math import radians
# import numpy as np
# from math import radians, sin, cos
# import numpy as np
# from scipy.ndimage import binary_dilation


# class Search:
#     def __init__(self, angles, distances, world_size=200) -> None:
#         self.size = world_size

#         self.robot_x = self.size / 2
#         self.robot_y = self.size / 2

#         self.grid = self.build_grid(angles, distances)
#         self.goal = None
#         self.start = (self.robot_x, self.robot_y)

#     def build_grid(self, angles, distances):
#         grid = np.zeros((self.size, self.size))
#         for a, d in zip(angles, distances):
#             coord = self.lidar_to_grid(a, d)
#             if coord is None:
#                 continue
#             x, y = coord
#             grid[x, y] = 1

#         # plt.imshow(grid, cmap="gray", origin="lower")
#         # plt.show()

#         structure = np.ones((7, 7))

#         # Perform binary dilation to expand the obstacles
#         dilated_grid = binary_dilation(grid, structure=structure)
#         dilated_grid = dilated_grid.astype(int)

#         # plt.imshow(dilated_grid, cmap="gray", origin="lower")
#         # plt.show()

#         return dilated_grid

#     def lidar_to_grid(self, angle, distance):
#         # Ignore points with zero distance (no reading)
#         if distance == 0:
#             return None

#         # Calculate the world coordinates of the point
#         point_x = self.robot_x + (distance / 10) * cos(angle)
#         point_y = self.robot_y + (distance / 10) * sin(angle)

#         # Convert world coordinates to grid coordinates
#         grid_x = int(point_x)
#         grid_y = int(point_y)

#         # Ensure coordinates are within grid bounds
#         if 0 <= grid_x < self.size and 0 <= grid_y < self.size:
#             return grid_x, grid_y
#         else:
#             return None

#     def heuristic(self, a):
#         return np.linalg.norm(np.array(a) - np.array(self.goal))  # Euclidean distance

#     def a_star_search(self):
#         frontier = PriorityQueue()
#         frontier.put(self.start, 0)

#         came_from = {}  # Dictionary to store the path
#         cost_so_far = {}  # Dictionary to store the cost
#         came_from[self.start] = None
#         cost_so_far[self.start] = 0

#         while not frontier.empty():
#             current = frontier.get()

#             if current == self.goal:
#                 print("foudn goal")
#                 break  # Exit once the goal is reached

#             # Explore neighbors (up, down, left, right)
#             for dx, dy in [
#                 (-1, 0),
#                 (1, 0),
#                 (0, -1),
#                 (0, 1),
#                 (1, 1),
#                 (-1, -1),
#                 (1, -1),
#                 (-1, 1),
#             ]:  # Optionally add diagonals
#                 neighbor = (int(current[0] + dx), int(current[1] + dy))

#                 # Check if the neighbor is within bounds and not an obstacle
#                 if (
#                     0 <= neighbor[0] < self.grid.shape[0]
#                     and 0 <= neighbor[1] < self.grid.shape[1]
#                 ):
#                     if self.grid[neighbor[0], neighbor[1]] == 0:  # Free space
#                         new_cost = (
#                             cost_so_far[current] + 1
#                         )  # Cost of moving to a neighbor

#                         if (
#                             neighbor not in cost_so_far
#                             or new_cost < cost_so_far[neighbor]
#                         ):
#                             cost_so_far[neighbor] = new_cost
#                             priority = new_cost + self.heuristic(neighbor)
#                             frontier.put(neighbor, priority)
#                             came_from[neighbor] = current

#         return came_from, cost_so_far

#     def reconstruct_path(self, came_from):
#         current = self.goal
#         path = []

#         while current != self.start:
#             path.append(current)
#             current = came_from[current]

#         path.append(self.start)  # Optional: include start in the path
#         path.reverse()  # Reverse the path to get it from start to goal
#         return path

#     def find_path(self, goal):
#         self.goal = goal
#         came_from, cost_so_far = self.a_star_search()
#         path = self.reconstruct_path(came_from)
#         self.path = np.array([self.cartesian_to_polar(*coord) for coord in path])

#     def cartesian_to_polar(self, x, y):
#         # Translate coordinates so that the origin is the robot's position
#         dx = x - self.robot_x
#         dy = y - self.robot_y
#         dx *= 10
#         dy *= 10

#         # Calculate r (distance) and theta (angle in radians)
#         r = np.sqrt(dx**2 + dy**2)
#         theta = np.arctan2(dy, dx)  # Gives angle in radians

#         return theta, r


# class LidarPlot:
#     def __init__(self) -> None:
#         self.scatter, self.goal, self.path = self.init_polar()

#     def init_polar(self):
#         # Initialize the plot
#         fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
#         ax.set_theta_zero_location("N")
#         ax.set_theta_direction(-1)
#         ax.set_rmax(1000)  # Set the maximum range for the LIDAR
#         ax.set_rticks([250, 500, 750, 1000])  # Example of range ticks
#         ax.grid(True)

#         # Mark every 45 degrees
#         ax.set_xticks(np.radians(np.arange(0, 360, 45)))
#         ax.set_yticks([250, 500, 750, 1000])  # Inner circles for range

#         scatter = ax.scatter([], [], s=10, color="blue")
#         goal = ax.scatter([], [], 100, marker="*", color="red")
#         (path,) = ax.plot([], [], "r-")

#         plt.ion()
#         plt.show()

#         return scatter, goal, path

#     def update_points(self, angles, distances):
#         self.scatter.set_offsets(np.c_[angles, distances])

#     def update_plot(self):
#         plt.draw()
#         plt.pause(0.01)

#     def plot_search(self, search: Search):
#         # plot goal star
#         goal = search.goal[0], search.goal[1]
#         angle, distance = search.cartesian_to_polar(*goal)
#         self.goal.set_offsets(np.c_[[angle], [distance]])

#         # plot path
#         angles = search.path[:, 0]
#         distances = search.path[:, 1]
#         gg = np.c_[angles, distances]
#         self.path.set_data(gg[:, 0], gg[:, 1])


# def plot_lidar_loop(scan_generator):
#     plot = LidarPlot()

#     while True:
#         angles = []
#         distances = []

#         for count, scan in enumerate(scan_generator()):
#             if scan.quality > 0 and scan.distance > 0:
#                 angle = radians(scan.angle)  # Convert to radians for polar plot
#                 distance = scan.distance

#                 angles.append(angle)
#                 distances.append(distance)

#             # Limit the number of points to avoid overloading the plot
#             if count > 750:
#                 break

#         plot.update_points(angles, distances)

#         search = Search(angles, distances)
#         search.find_path((100 + 20, 100 - 80))
#         plot.plot_search(search)

#         plot.update_plot()


# def plot_occupancy_loop(scan_generator, robot_x, robot_y):
#     occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE))
#     plt.imshow(occupancy_grid, cmap="gray", origin="lower")
#     plt.ion()
#     plt.show()

#     while True:
#         occupancy_grid = np.zeros((GRID_SIZE, GRID_SIZE))

#         # Fetch next scan data
#         for count, scan in enumerate(scan_generator()):
#             if scan.quality > 0 and scan.distance > 0:
#                 angle = radians(scan.angle)  # Convert to radians for polar plot
#                 distance = scan.distance / 1000
#                 # print(distance)

#                 grid_coords = lidar_to_grid(angle, distance, robot_x, robot_y)
#                 if grid_coords is None:
#                     continue

#                 # print(grid_coords, angle, distance)
#                 grid_x, grid_y = grid_coords
#                 occupancy_grid[grid_x, grid_y] = 1

#             # Limit the number of points to avoid overloading the plot
#             if count > 700:
#                 break

#         print(occupancy_grid.sum())
#         plt.imshow(occupancy_grid, cmap="gray", origin="lower")
#         # plt.title("Occupancy Grid")
#         # plt.show()
#         plt.draw()
#         plt.pause(0.01)


# lidar = PyRPlidar()
# lidar.connect(port="COM9", baudrate=256000, timeout=5)
# lidar.set_motor_pwm(500)
# time.sleep(2)

# try:

#     scan_generator = lidar.force_scan()
#     plot_lidar_loop(scan_generator)
#     # plot_occupancy_loop(scan_generator, int(WORLD_SIZE / 2), int(WORLD_SIZE / 2))


# finally:
#     lidar.stop()
#     lidar.set_motor_pwm(0)
#     lidar.disconnect()
import time
from typing import List

from matplotlib import pyplot as plt
from src.path_finding.LidarPlot import LidarPlot
from src.path_finding.OccupancyGrid import OccupancyGrid
from src.path_finding.Position import Position
from src.multithreading import graceful_thread_exit
from src.multithreading.LidarSensorThread import LidarSensorThread
from src.path_finding.AStarSearch import AStarSearch


@graceful_thread_exit
def start():
    positions = []

    thread = LidarSensorThread(positions)
    thread.start()

    range = 1000
    resolution = 25
    scale_fact = resolution / range

    def scale(positions: List[Position]):
        scaled = []
        for pos in positions:
            clone = pos.copy()
            clone.dilate(scale_fact)
            clone.move(resolution, resolution)
            scaled.append(clone)
        return scaled

    def unscale(positions: List[Position]):
        unscaled = []
        for pos in positions:
            clone = pos.copy()
            clone.move(-resolution, -resolution)
            clone.dilate(1 / scale_fact)
            unscaled.append(clone)
            # print(pos.x, pos.y, pos.angle, pos.distance)
            # print(clone.x, clone.y, clone.angle, clone.distance)
            # print("\n\n")
        return unscaled

    plot = LidarPlot(range_mm=range)
    goal = Position(x=200, y=-800)
    time.sleep(2)

    count = 0
    while True:
        if count % 50 == 0:
            grid = OccupancyGrid(scale(positions), resolution * 2)

            # # plt.imshow(grid.grid, cmap="gray", origin="lower")
            # # plt.show()
            search = AStarSearch(
                grid,
                Position(x=resolution, y=resolution),
                scale([goal])[0],
            )
            path = search.find_path()
            path = unscale(path)
            plot.plot_path(path)
            
            pass
        plot.plot_obstacles(positions)
        plot.plot_destination(goal)
        plot.update_plot()
        time.sleep(0.1)
        count += 1


if __name__ == "__main__":
    start()
