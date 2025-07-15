from typing import List
from src.path_finding.PolarAStarSearch import a_star_search
from src.multithreading.LoopingThread import LoopingThread
from queue import LifoQueue
from src.path_finding.AStarSearch import AStarSearch
from src.path_finding.OccupancyGrid import OccupancyGrid
from src.path_finding.Position import Position


class PathFindingThread(LoopingThread):
    def __init__(
        self,
        target_pos_queue: LifoQueue,
        obstacle_queue: LifoQueue,
        path_queue: LifoQueue,
    ):
        super().__init__(0)
        self.target_pos_queue = target_pos_queue
        self.obstacle_queue = obstacle_queue
        self.path_queue = path_queue

    def on_start(self):
        pass

    def on_finish(self):
        pass

    def tick(self):
        if self.target_pos_queue.empty():
            return

        if self.obstacle_queue.empty():
            return

        target_person = self.target_pos_queue.get()
        obstacles = self.obstacle_queue.get()

        path = a_star_search(target_person, obstacles)
        self.path_queue.put(path)
