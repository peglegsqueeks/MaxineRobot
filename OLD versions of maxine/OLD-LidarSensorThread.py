from typing import List
from .LoopingThread import LoopingThread
from pyrplidar import PyRPlidar
import time
from math import radians
from ..path_finding.Position import Position


class LidarSensorThread(LoopingThread):
    def __init__(
        self,
        obstacles: List[Position],
        ms_delay: int = 0,
        scan_count: int = 750,
    ) -> None:
        super().__init__(ms_delay)
        self.scan_count = scan_count
        self.obstacles = obstacles

    def tick(self):
        obstacles = []

        #`for angle in range(270, 360, 3):
        #`    obstacles.append(Position(angle=radians(angle), distance=1000))
        #`
        #`for angle in range(0, 60, 3):
        #`    obstacles.append(Position(angle=radians(angle), distance=1000))
#`
#`
        #`for distance in range(1000, 3000):
        #`    obstacles.append(Position(angle=radians(30), distance=distance))
        #`    obstacles.append(Position(angle=radians(300), distance=distance))
#`
        #`for angle in range(95, 180, 3):
        #`    obstacles.append(Position(angle=radians(angle), distance=1000))
        for count, scan in enumerate(self.scan_generator()):
             if scan.quality < 10:
                 continue

             if scan.distance <= 40:
                 continue


             obstacles.append(
                 Position(angle=radians(scan.angle), distance=scan.distance)
             )

             if count >= self.scan_count:
                 break

        self.obstacles.clear()
        self.obstacles.extend(obstacles)

    def on_start(self):
        self.lidar = PyRPlidar()
        self.lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)
        self.lidar.set_motor_pwm(400)
        time.sleep(1)
        self.scan_generator = self.lidar.force_scan()
        
    def on_finish(self):
        self.lidar.stop()
        self.lidar.set_motor_pwm(0)
        self.lidar.disconnect()
        