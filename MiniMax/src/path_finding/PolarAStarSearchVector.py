from heapq import heappop, heappush
import math

import numpy as np

from src.path_finding.Position import Position

STEP_DISTANCE = 50
STEP_ANGLE = math.radians(4)
N_ANGLES = 2 * math.pi // STEP_ANGLE
N_PADDING_ROUNDS = 3


# build neighbour array
neighbours = []
for distance_offset in [-STEP_DISTANCE, 0, STEP_DISTANCE]:
    for angle_offset in [-1, 0, 1]:
        if angle_offset == 0 and distance_offset == 0:
            continue
        neighbours.append((angle_offset, distance_offset))
NEIGHBOURS = np.array(neighbours)


def snap_to_valid_position(coord: np.ndarray):
    if len(coord.shape) == 1:
        coord[0] = coord[0] % N_ANGLES
        coord[1] = (coord[1] / STEP_DISTANCE).round() * STEP_DISTANCE
        return coord

    coord[:, 0] = coord[:, 0] % N_ANGLES
    coord[:, 1] = (coord[:, 1] / STEP_DISTANCE).round() * STEP_DISTANCE

    return coord


def convert_to_search_coord(positions):
    if not isinstance(positions, list):
        positions = [positions]
    positions = np.array([(pos.angle // STEP_ANGLE, pos.distance) for pos in positions])

    snapped_positions = snap_to_valid_position(positions)

    return snapped_positions if snapped_positions.shape[0] > 1 else snapped_positions[0]


def convert_from_search_coord(angle, distance):
    return Position(angle * STEP_ANGLE, distance)


def pad_obstacles(obstacles):
    padded = obstacles[None, :, :]
    for _ in range(N_PADDING_ROUNDS):
        padded = NEIGHBOURS[:, None, :] + padded
    padded = padded.reshape((-1, 2))
    padded = np.unique(padded, axis=0)

    return snap_to_valid_position(padded)


def prebuild_obstacles(obstacles):
    obstacle_list = convert_to_search_coord(obstacles)

    obstacle_set = set()
    for obstacle in pad_obstacles(obstacle_list):
        obstacle_set.add(tuple(obstacle))
    return obstacle_set


def polar_distance(c1: np.ndarray, c2: np.ndarray):
    diff = np.abs(c2 - c1)
    diff[0] *= STEP_ANGLE
    return diff.sum()


def get_neighbors(coord):
    neighbourns = NEIGHBOURS + coord
    neighbourns[:, 0] = neighbourns[:, 0] % N_ANGLES
    return neighbourns


def reconstruct_path(came_from, current):

    path = [convert_from_search_coord(*current)]

    while current in came_from:
        current = tuple(came_from[current])
        path.append(convert_from_search_coord(*current))
    return path[::-1]


def get_distance(from_tuple, from_array, dest_tuple, dest_array, distances):
    dist_tuple = (*from_tuple, *dest_tuple)
    if dist_tuple in distances:
        distance = distances[dist_tuple]
    else:
        distance = polar_distance(from_array, dest_array)
        distances[dist_tuple] = distance

    return distance, distances


def a_star_search(goal: Position, obstacle_set: list[Position]):
    start = np.array((0, 0))
    start_tuple = tuple(start)

    goal = convert_to_search_coord(goal)
    goal_tuple = tuple(goal)
    obstacle_set = prebuild_obstacles(obstacle_set)

    open_set = []
    heappush(open_set, (0, start_tuple))

    came_from = {}
    g_score = {start_tuple: 0}
    start_dist = polar_distance(start, goal)
    distances = {}
    distances[(*start_tuple, *goal_tuple)] = start_dist
    f_score = {start_tuple: start_dist}

    while open_set:
        _, current = heappop(open_set)
        tuple_current = tuple(current)

        if np.abs(current - goal).sum() < 0.1:
            return reconstruct_path(came_from, tuple_current)

        for neighbor in get_neighbors(current):
            tuple_neighbor = tuple(neighbor.round(3))

            if tuple_neighbor in obstacle_set:
                continue

            distance, distances = get_distance(
                tuple_current, current, tuple_neighbor, neighbor, distances
            )
            tentative_g_score = g_score[tuple_current] + distance

            if (
                tuple_neighbor not in g_score
                or tentative_g_score < g_score[tuple_neighbor]
            ):

                came_from[tuple_neighbor] = current
                g_score[tuple_neighbor] = tentative_g_score

                distance, distances = get_distance(
                    tuple_neighbor, neighbor, goal_tuple, goal, distances
                )

                f_score[tuple_neighbor] = tentative_g_score + distance
                heappush(open_set, (f_score[tuple_neighbor], tuple_neighbor))

    return None
