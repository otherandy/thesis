from shapely.geometry import Point, Polygon
import numpy as np
import enum


class Direction(enum.Enum):
    UP = np.pi / 2
    DOWN = 3 * np.pi / 2
    LEFT = np.pi
    RIGHT = 0


ENVIRONMENT = Polygon(
    [(0, 0), (8, 0), (8, 6), (12, 6), (12, 12), (4, 12), (4, 6), (0, 6)]
)

LIDAR_RADIUS = 1.5
START_POSITION = Point(1.0, 1.0)
START_DIRECTION = Direction.UP

visited = set()
explored = Polygon()


def sample_lidar(position: Point, num_samples=360) -> list[Point]:
    angles = np.linspace(0, 2 * np.pi, num_samples, endpoint=False)
    points = []
    for angle in angles:
        for step in np.linspace(0, LIDAR_RADIUS, 100):
            sample_point = Point(
                position.x + step * np.cos(angle), position.y + step * np.sin(angle)
            )
            if not ENVIRONMENT.contains(sample_point):
                break
        points.append(sample_point)
    return points


def lidar_polygon(lidar_points: list[Point]) -> Polygon:
    coords = [(p.x, p.y) for p in lidar_points]
    return Polygon(coords)


def add_explored_area(lidar_area: Polygon) -> None:
    global explored
    explored = explored.union(lidar_area)


def on_explored_edge(lidar_area: Polygon) -> bool:
    intersection = explored.intersection(lidar_area)
    return not intersection.is_empty and not intersection.equals(lidar_area)


def get_progress() -> float:
    progress_area = explored.intersection(ENVIRONMENT).area
    total_area = ENVIRONMENT.area
    return progress_area / total_area if total_area > 0 else 0.0


def add_visited(position: Point) -> None:
    visited.add((position.x, position.y))


def visited_to_file(filename: str) -> None:
    with open(filename, "w") as f:
        for vx, vy in visited:
            f.write(f"{vx},{vy}\n")
