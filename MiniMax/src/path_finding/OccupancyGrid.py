from .Position import Position


import numpy as np
from scipy.ndimage import binary_dilation


from typing import List


class OccupancyGrid:
    def __init__(
        self,
        readings: List[Position],
        size: int,
    ) -> None:
        self.size = size
        grid = self.build_grid(readings)
        self.grid = self.post_process_grid(grid)

        self.shape = self.grid.shape

    def build_grid(self, readings: List[Position]):
        grid = np.zeros((self.size, self.size))

        for reading in readings:
            x, y = reading.coordinates()

            if x < 0 or x >= self.size:
                continue

            if y < 0 or y >= self.size:
                continue

            grid[x, y] = 1

        return grid

    def post_process_grid(self, grid):
        structure = np.ones((1, 1))
        dilated_grid = binary_dilation(grid, structure=structure)
        dilated_grid = dilated_grid.astype(int)

        return dilated_grid
