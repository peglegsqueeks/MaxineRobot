from src.path_finding.new_a_star import a_star
import multiprocessing.connection
import time
import math


def run_search(
    receive_connection: multiprocessing.connection.Connection,
    send_connection: multiprocessing.connection.Connection,
) -> None:
    try:
        while True:
            if receive_connection.poll():
                (goal, obstacles) = receive_connection.recv()
                try:
                    # Minimal output - removed verbose debug
                    start_time = time.time()
                    path, *_ = a_star(obstacles, goal)
                    end_time = time.time()
                    
                    send_connection.send(path)
                        
                except Exception as e:
                    send_connection.send(None)

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass