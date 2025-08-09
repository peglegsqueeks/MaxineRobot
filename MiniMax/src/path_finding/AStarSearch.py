from queue import PriorityQueue
from typing import List

import numpy as np


from .OccupancyGrid import OccupancyGrid
from .Position import Position


class AStarSearch:
    def __init__(self, grid: OccupancyGrid, start: Position, goal: Position) -> None:
        self.grid = grid
        self.start = start
        self.goal = goal

    def find_path(self) -> List[Position]:
        try:
            came_from = self.a_star()
            return self.reconstruct_path(came_from)
        except:
            return []

    def heuristic(self, a):
        return np.linalg.norm(np.array(a) - np.array(self.goal.coordinates()))

    def a_star(self):
        frontier = PriorityQueue()
        frontier.put(self.start.coordinates(), 0)

        came_from = {}
        cost_so_far = {}
        came_from[self.start.coordinates()] = None
        cost_so_far[self.start.coordinates()] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == self.goal.coordinates():
                print("found solution")
                break

            for dx, dy in [
                (-1, 0),
                (1, 0),
                (0, -1),
                (0, 1),
                (1, 1),
                (-1, -1),
                (1, -1),
                (-1, 1),
            ]:
                neighbor = (int(current[0] + dx), int(current[1] + dy))

                if (
                    0 <= neighbor[0] < self.grid.shape[0]
                    and 0 <= neighbor[1] < self.grid.shape[1]
                ):
                    if self.grid.grid[neighbor[0], neighbor[1]] == 0:
                        new_cost = cost_so_far[current] + 1

                        if (
                            neighbor not in cost_so_far
                            or new_cost < cost_so_far[neighbor]
                        ):
                            cost_so_far[neighbor] = new_cost
                            priority = new_cost + self.heuristic(neighbor)
                            frontier.put(neighbor, priority)
                            came_from[neighbor] = current

        return came_from

    def reconstruct_path(self, came_from) -> List[Position]:
        current = self.goal.coordinates()
        path = []
        while current != self.start.coordinates():
            x, y = current
            path.append(Position(x=x, y=y))
            current = came_from[current]

        path.append(self.start)
        path.reverse()

        return path
