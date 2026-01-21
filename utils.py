from shapely.geometry import Point, Polygon
import numpy as np
import enum
from collections import defaultdict


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


class OccupancyGrid:
    """Grid-based occupancy map with FREE, OCCUPIED, and UNKNOWN cells."""

    def __init__(self, resolution=0.1):
        self.resolution = resolution
        self.grid = defaultdict(lambda: "UNKNOWN")  # Dictionary for sparse grid

    def world_to_grid(self, point: Point) -> tuple[int, int]:
        """Convert world coordinates to grid indices."""
        x = int(np.round(point.x / self.resolution))
        y = int(np.round(point.y / self.resolution))
        return (x, y)

    def grid_to_world(self, grid_x: int, grid_y: int) -> Point:
        """Convert grid indices to world coordinates."""
        x = grid_x * self.resolution
        y = grid_y * self.resolution
        return Point(x, y)

    def mark_free(self, point: Point) -> None:
        x, y = self.world_to_grid(point)
        self.grid[(x, y)] = "FREE"

    def mark_occupied(self, point: Point) -> None:
        x, y = self.world_to_grid(point)
        self.grid[(x, y)] = "OCCUPIED"

    def mark_line_free(self, p1: Point, p2: Point) -> None:
        """Mark all cells along a line as free."""
        num_steps = max(int(p1.distance(p2) / self.resolution) + 1, 2)
        for i in range(num_steps):
            t = i / (num_steps - 1)
            x = p1.x + t * (p2.x - p1.x)
            y = p1.y + t * (p2.y - p1.y)
            self.mark_free(Point(x, y))

    def get_cell(self, point: Point) -> str:
        x, y = self.world_to_grid(point)
        return self.grid[(x, y)]

    def get_frontier_cells(self) -> list[Point]:
        """Find all UNKNOWN cells adjacent to FREE cells."""
        frontier = set()
        free_cells = {k for k, v in self.grid.items() if v == "FREE"}

        for x, y in free_cells:
            # Check 8 neighbors
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if self.grid[(nx, ny)] == "UNKNOWN":
                        frontier.add((nx, ny))

        return [self.grid_to_world(x, y) for x, y in frontier]


class BoundaryNode:
    def __init__(self, position: Point):
        self.position = position
        self.edges = []  # Connected nodes

    def __repr__(self):
        return f"BoundaryNode(pos=({self.position.x:.2f},{self.position.y:.2f}))"


class BoundaryGraph:
    def __init__(self):
        self.nodes = []
        self.edges = []  # List of (node1_idx, node2_idx, distance)

    def add_node(self, node: BoundaryNode) -> int:
        self.nodes.append(node)
        return len(self.nodes) - 1

    def add_edge(self, idx1: int, idx2: int) -> None:
        if (
            idx1 != idx2
            and (idx1, idx2) not in self.edges
            and (idx2, idx1) not in self.edges
        ):
            dist = self.nodes[idx1].position.distance(self.nodes[idx2].position)
            self.edges.append((idx1, idx2, dist))

    def get_unvisited_regions(self, visited_set: set) -> list[BoundaryNode]:
        """Find boundary nodes not yet explored."""
        return [node for i, node in enumerate(self.nodes) if i not in visited_set]


def extract_wall_points(
    lidar_points: list[Point], distance_threshold=0.1
) -> list[Point]:
    if len(lidar_points) < 2:
        return lidar_points

    # Cluster points that are close together to find distinct wall segments
    wall_points = []
    skip_until = 0

    for i, point in enumerate(lidar_points):
        if i < skip_until:
            continue

        # Find consecutive points in same cluster
        cluster = [point]
        j = i + 1
        while j < len(lidar_points):
            if point.distance(lidar_points[j]) < distance_threshold:
                cluster.append(lidar_points[j])
                j += 1
            else:
                break

        # Add centroid of cluster
        if cluster:
            cx = np.mean([p.x for p in cluster])
            cy = np.mean([p.y for p in cluster])
            wall_points.append(Point(float(cx), float(cy)))
            skip_until = j

    return wall_points


def compute_angle_to_point(from_pos: Point, to_point: Point) -> float:
    dy = to_point.y - from_pos.y
    dx = to_point.x - from_pos.x
    return np.arctan2(dy, dx)


def normalize_angle(angle: float) -> float:
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle < -np.pi:
        angle += 2 * np.pi
    return angle


def angle_difference(angle1: float, angle2: float) -> float:
    diff = normalize_angle(angle2 - angle1)
    return diff


def move_forward(position: Point, heading: float, distance: float) -> Point:
    new_x = position.x + distance * np.cos(heading)
    new_y = position.y + distance * np.sin(heading)
    return Point(new_x, new_y)


def get_nearest_frontier(current_pos: Point, frontiers: list[Point]) -> Point | None:
    if not frontiers:
        return None
    return min(frontiers, key=lambda p: current_pos.distance(p))


def sample_lidar(position: Point, num_samples=360) -> list[Point]:
    angles = np.linspace(0, 2 * np.pi, num_samples, endpoint=False)
    points = []
    sample_point = position  # Default in case no obstacle is found
    for angle in angles:
        for step in np.linspace(0, LIDAR_RADIUS, 100):
            sample_point = Point(
                position.x + step * np.cos(angle), position.y + step * np.sin(angle)
            )
            if not ENVIRONMENT.contains(sample_point):
                break
        points.append(sample_point)
    return points


def get_min_lidar_distance(position: Point, heading: float, num_angles=5) -> float:
    min_dist = LIDAR_RADIUS
    half_width = np.pi / 4  # 45 degree cone

    for i in range(num_angles):
        angle = heading + (i - num_angles // 2) * half_width / (num_angles // 2)
        # Cast ray and find distance to obstacle
        for step in np.linspace(0, LIDAR_RADIUS, 100):
            sample_point = Point(
                position.x + step * np.cos(angle), position.y + step * np.sin(angle)
            )
            if not ENVIRONMENT.contains(sample_point):
                min_dist = min(min_dist, step)
                break

    return min_dist


def has_obstacle_ahead(
    position: Point, heading: float, threshold: float = LIDAR_RADIUS * 0.9
) -> bool:
    return get_min_lidar_distance(position, heading) < threshold


def update_occupancy_from_lidar(
    occupancy_grid: OccupancyGrid, position: Point, lidar_points: list[Point]
) -> None:
    occupancy_grid.mark_free(position)

    # Mark line from robot to each detected obstacle as free
    for lidar_point in lidar_points:
        occupancy_grid.mark_line_free(position, lidar_point)
        # Mark the obstacle point itself as occupied
        occupancy_grid.mark_occupied(lidar_point)


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


def grid_to_file(occupancy_grid: OccupancyGrid, filename: str) -> None:
    with open(filename, "w") as f:
        for (gx, gy), status in occupancy_grid.grid.items():
            wx = gx * occupancy_grid.resolution
            wy = gy * occupancy_grid.resolution
            f.write(f"{wx},{wy},{status}\n")


def graph_to_file(boundary_graph: BoundaryGraph, filename: str) -> None:
    with open(filename, "w") as f:
        for i, node in enumerate(boundary_graph.nodes):
            f.write(f"NODE,{i},{node.position.x},{node.position.y}\n")
        for idx1, idx2, dist in boundary_graph.edges:
            f.write(f"EDGE,{idx1},{idx2},{dist}\n")
