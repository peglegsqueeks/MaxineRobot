from heapq import heappop, heappush
import math

from src.path_finding.Position import Position

STEP_DISTANCE = 100
STEP_ANGLE = math.radians(20)


def euclidean_distance(c1, c2):
    """Calculate the Euclidean distance between two polar coordinates."""
    x1, y1 = c1.to_cartesian()
    x2, y2 = c2.to_cartesian()
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def snap_to_valid_position(coord, step_angle, step_distance):
    rounded_angle = round(coord.angle / step_angle) * step_angle
    rounded_distance = round(coord.distance / step_distance) * step_distance
    return Position(rounded_angle, rounded_distance)


def is_obstacle(position, obstacles, tolerance=0.1):
    for obstacle in obstacles:
        if math.isclose(
            position.angle, obstacle.angle, abs_tol=tolerance
        ) and math.isclose(position.distance, obstacle.distance, abs_tol=tolerance):
            return True
    return False


def prebuild_obstacles(obstacles, step_angle, step_distance):
    """Prebuild obstacles into a set for fast lookup."""
    obstacle_set = set()

    for obstacle in obstacles:
        # Snap each obstacle to the nearest valid position
        snapped_obstacle = snap_to_valid_position(obstacle, step_angle, step_distance)
        # Add the snapped obstacle to the set
        obstacle_set.add((snapped_obstacle.angle, snapped_obstacle.distance))

    return obstacle_set


def is_obstacle(position, obstacle_set, tolerance=1):
    """Check if the position is an obstacle by looking it up in the prebuilt set."""
    # Snap the current position to the nearest valid position
    snapped_position = snap_to_valid_position(position, STEP_ANGLE, STEP_DISTANCE)
    # Check if the snapped position is in the obstacle set
    return (snapped_position.angle, snapped_position.distance) in obstacle_set


def a_star_search(goal: Position, obstacles: list[Position]):
    """Perform A* search in polar coordinates."""
    open_set = []
    start = Position(0, 0)
    goal = snap_to_valid_position(goal, STEP_ANGLE, STEP_DISTANCE)
    obstacles = prebuild_obstacles(obstacles, STEP_ANGLE, STEP_DISTANCE)
    heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0}
    f_score = {start: euclidean_distance(start, goal)}

    while open_set:
        _, current = heappop(open_set)

        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in get_neighbors(current):
            if is_obstacle(neighbor, obstacles):
                continue

            tentative_g_score = g_score[current] + euclidean_distance(current, neighbor)

            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + euclidean_distance(
                    neighbor, goal
                )
                heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found


def get_neighbors(coord):
    """Generate neighboring coordinates in polar space."""
    step_angle = STEP_ANGLE  # Adjust step size as needed
    step_distance = STEP_DISTANCE  # Adjust step size as needed

    neighbors = []
    for angle_offset in [-step_angle, 0, step_angle]:
        for distance_offset in [-step_distance, 0, step_distance]:
            if angle_offset == 0 and distance_offset == 0:
                continue

            new_angle = coord.angle + angle_offset
            new_distance = coord.distance + distance_offset

            if new_distance > 0:  # Ensure the distance is positive
                neighbors.append(Position(new_angle, new_distance))

    return neighbors


def reconstruct_path(came_from, current):
    """Reconstruct the path from the start to the goal."""
    path = [current]

    while current in came_from:
        print(str(current))
        current = came_from[current]
        path.append(current)
    return path[::-1]
