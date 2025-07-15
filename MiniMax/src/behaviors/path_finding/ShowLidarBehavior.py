from MiniMax.src.behaviors.MaxineBehavior import MaxineBehavior
from MiniMax.src.path_finding.Position import Position


import cv2
import numpy as np
import py_trees
from matplotlib import pyplot as plt
from py_trees.common import Status


from math import radians


class ShowLidarBehavior(MaxineBehavior):
    def __init__(self):
        super().__init__("Show Lidar Behavior")
        self.blackboard.register_key(
            "TARGET_PERSON", access=py_trees.common.Access.WRITE
        )

    def plot_target(self, ax) -> Position:
        target_object = self.blackboard.get("TARGET_PERSON")

        fov = 110
        angle = target_object.spatialCoordinates.x / fov
        angle = radians(angle)
        depth = target_object.spatialCoordinates.z

        target_plot = ax.scatter([], [], 100, marker="*", color="red")
        target_plot.set_offsets(np.c_[[angle], [depth]])

    def plot_obstacles(self, ax):
        lidar_sensor = self.get_robot().lidar_sensor
        obstacles = lidar_sensor.get_reading()

        angles = [reading.angle for reading in obstacles]
        distances = [reading.distance for reading in obstacles]
        obstacle_plot = ax.scatter([], [], s=10, color="blue")
        obstacle_plot.set_offsets(np.c_[angles, distances])

    def update(self) -> Status:
        plt.close()
        fig, ax = plt.subplots(subplot_kw={"projection": "polar"})

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

        self.plot_obstacles(ax)
        self.plot_target(ax)

        fig.canvas.draw()
        img_plot = np.array(fig.canvas.renderer.buffer_rgba())

        cv2.imshow("Lidar", cv2.cvtColor(img_plot, cv2.COLOR_RGBA2BGR))
        cv2.waitKey(1)

        # behavior always passes
        return Status.SUCCESS


class PathfindToTargetBehavior(MaxineBehavior):
    def __init__(self):
        super().__init__("Path find to target")
        self.blackboard.register_key("PATH", access=py_trees.common.Access.WRITE)

    def update(self) -> Status:
        return super().update()
