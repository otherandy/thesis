import numpy as np
from shapely.geometry import Point, Polygon
from utils import (
    LIDAR_RADIUS,
    START_POSITION,
    START_DIRECTION,
    sample_lidar,
    extract_wall_points,
    compute_angle_to_point,
    normalize_angle,
    angle_difference,
    move_forward,
    get_nearest_frontier,
    update_occupancy_from_lidar,
    has_obstacle_ahead,
    OccupancyGrid,
    BoundaryNode,
    BoundaryGraph,
    add_visited,
    visited_to_file,
)


class ExplorationBot:
    def __init__(self, start_pos: Point, start_heading: float, grid_resolution=0.1):
        self.position = start_pos
        self.heading = start_heading
        self.occupancy_grid = OccupancyGrid(resolution=grid_resolution)
        self.boundary_graph = BoundaryGraph()
        self.wall_points = []
        self.visited_regions = set()

    def take_lidar_reading(self, num_samples=360) -> list[Point]:
        return sample_lidar(self.position, num_samples)

    def phase1_initial_wall_discovery(self):
        """
        Phase 1: Initial wall discovery.
        - Choose a random direction
        - Move until detecting a wall/obstacle
        - Mark the position of first contact
        """
        print("=== Phase 1: Initial Wall Discovery ===")

        # Random initial direction
        initial_heading = np.random.uniform(0, 2 * np.pi)
        self.heading = initial_heading
        step_distance = 0.1
        max_steps = int(LIDAR_RADIUS / step_distance) * 10

        # Move forward until hitting obstacle
        for _ in range(max_steps):
            # Check if there's obstacle ahead
            if has_obstacle_ahead(
                self.position, self.heading, threshold=LIDAR_RADIUS * 0.8
            ):
                print(
                    f"Wall detected at position ({self.position.x:.2f}, {self.position.y:.2f})"
                )
                # Take lidar reading and extract wall points
                lidar_points = self.take_lidar_reading()
                wall_points = extract_wall_points(lidar_points, distance_threshold=0.15)

                # Update occupancy grid
                update_occupancy_from_lidar(
                    self.occupancy_grid, self.position, lidar_points
                )

                # Store first wall contact
                self.first_contact_pos = self.position
                self.first_contact_heading = self.heading
                self.wall_points = wall_points

                # Create initial boundary node
                node_idx = self.boundary_graph.add_node(
                    BoundaryNode(self.position, self.heading)
                )
                self.wall_point_indices = [node_idx]

                add_visited(self.position)
                return True

            # Move forward
            self.position = move_forward(self.position, self.heading, step_distance)
            add_visited(self.position)

        print("No wall found in exploration radius")
        return False

    def phase2_boundary_tracking(self):
        """
        Phase 2: Boundary tracking (wall following).
        - Use wall-following behavior with lidar
        - Update occupancy grid as exploring
        - Detect when returning to start point
        """
        print("\n=== Phase 2: Boundary Tracking ===")

        step_distance = 0.05
        min_return_distance = 0.2  # Distance threshold to consider as "returned"
        max_steps_tracking = 500
        tracking_steps = 0

        # Wall-following parameters
        target_wall_distance = LIDAR_RADIUS * 0.6

        while tracking_steps < max_steps_tracking:
            tracking_steps += 1

            # Take lidar reading
            lidar_points = self.take_lidar_reading()
            wall_points = extract_wall_points(lidar_points, distance_threshold=0.15)

            # Update occupancy grid
            update_occupancy_from_lidar(
                self.occupancy_grid, self.position, lidar_points
            )

            # Store wall points
            self.wall_points.extend(wall_points)

            # Detect if we've returned to start
            if tracking_steps > 50:  # Wait for some tracking before checking
                dist_to_start = self.position.distance(self.first_contact_pos)
                if dist_to_start < min_return_distance:
                    print(f"Returned to start after {tracking_steps} steps")
                    break

            # Simple wall-following: find wall direction and follow it
            if wall_points:
                # Get angle to nearest wall point
                wall_center = Point(
                    np.mean([p.x for p in wall_points]),
                    np.mean([p.y for p in wall_points]),
                )
                target_heading = compute_angle_to_point(self.position, wall_center)

                # Smoothly turn toward wall
                heading_diff = angle_difference(self.heading, target_heading)
                turn_amount = np.clip(heading_diff, -0.2, 0.2)  # Limit turning rate
                self.heading = normalize_angle(self.heading + turn_amount)

            # Move forward
            self.position = move_forward(self.position, self.heading, step_distance)
            add_visited(self.position)

        print(
            f"Boundary tracking complete. Collected {len(self.wall_points)} wall points"
        )
        return True

    def phase3_boundary_graph_construction(self):
        """
        Phase 3: Construct boundary graph.
        - Convert wall points to nodes (at corners/turning points)
        - Create edges representing wall connections
        """
        print("\n=== Phase 3: Boundary Graph Construction ===")

        if len(self.wall_points) < 3:
            print("Not enough wall points to construct graph")
            return False

        # Cluster wall points to find distinct corners
        corner_threshold = 0.3
        nodes_added = []

        for i, point in enumerate(self.wall_points):
            is_corner = True

            # Check if this point is a corner (significant change in direction)
            if i > 0 and i < len(self.wall_points) - 1:
                prev_point = self.wall_points[i - 1]
                next_point = self.wall_points[i + 1]

                angle1 = compute_angle_to_point(prev_point, point)
                angle2 = compute_angle_to_point(point, next_point)
                angle_change = abs(angle_difference(angle1, angle2))

                # Only add as node if significant direction change
                if angle_change < 0.3:  # Small change, likely collinear
                    is_corner = False

            if is_corner:
                # Check if far enough from existing nodes
                too_close = False
                for existing_node in nodes_added:
                    if point.distance(existing_node.position) < corner_threshold:
                        too_close = True
                        break

                if not too_close:
                    heading = compute_angle_to_point(self.position, point)
                    node = BoundaryNode(point, heading)
                    idx = self.boundary_graph.add_node(node)
                    nodes_added.append(node)

        # Connect nearby nodes as edges
        for i in range(len(self.boundary_graph.nodes)):
            for j in range(i + 1, len(self.boundary_graph.nodes)):
                dist = self.boundary_graph.nodes[i].position.distance(
                    self.boundary_graph.nodes[j].position
                )
                # Connect if reasonably close
                if dist < 1.0:
                    self.boundary_graph.add_edge(i, j)

        print(
            f"Graph constructed: {len(self.boundary_graph.nodes)} nodes, "
            f"{len(self.boundary_graph.edges)} edges"
        )
        return True

    def phase4_explore_frontiers(self):
        """
        Phase 4: Systematic frontier exploration.
        - Query occupancy grid for unknown cells
        - Navigate to frontier regions
        - "Circle" regions to discover their boundaries
        - Repeat until no frontiers remain
        """
        print("\n=== Phase 4: Frontier Exploration ===")

        exploration_iterations = 0
        max_iterations = 20

        while exploration_iterations < max_iterations:
            exploration_iterations += 1

            # Get frontier cells
            frontiers = self.occupancy_grid.get_frontier_cells()

            if not frontiers:
                print("No more frontiers to explore!")
                break

            print(
                f"Iteration {exploration_iterations}: Found {len(frontiers)} frontier cells"
            )

            # Find nearest frontier
            target = get_nearest_frontier(self.position, frontiers)

            if target is None:
                break

            # Navigate toward frontier
            target_heading = compute_angle_to_point(self.position, target)
            step_distance = 0.1
            max_nav_steps = 100
            nav_steps = 0

            while nav_steps < max_nav_steps:
                nav_steps += 1

                # Take lidar reading and update grid
                lidar_points = self.take_lidar_reading()
                update_occupancy_from_lidar(
                    self.occupancy_grid, self.position, lidar_points
                )

                # Check if reached frontier or hit obstacle
                dist_to_target = self.position.distance(target)
                if dist_to_target < LIDAR_RADIUS * 0.5:
                    print(f"  Reached frontier area")
                    break

                if has_obstacle_ahead(
                    self.position, target_heading, threshold=LIDAR_RADIUS * 0.7
                ):
                    print(f"  Obstacle ahead, adjusting direction")
                    # Try to go around obstacle
                    alternative_heading = normalize_angle(target_heading + np.pi / 6)
                    if not has_obstacle_ahead(
                        self.position, alternative_heading, threshold=LIDAR_RADIUS * 0.7
                    ):
                        target_heading = alternative_heading
                    else:
                        target_heading = normalize_angle(target_heading - np.pi / 6)

                # Move toward target
                self.heading = normalize_angle(
                    self.heading
                    + np.clip(angle_difference(self.heading, target_heading), -0.3, 0.3)
                )
                self.position = move_forward(self.position, self.heading, step_distance)
                add_visited(self.position)

        print(
            f"Frontier exploration complete after {exploration_iterations} iterations"
        )
        return True

    def run_exploration(self):
        print("Starting exploration algorithm...")

        if not self.phase1_initial_wall_discovery():
            print("Failed at phase 1")
            return False

        if not self.phase2_boundary_tracking():
            print("Failed at phase 2")
            return False

        if not self.phase3_boundary_graph_construction():
            print("Failed at phase 3")
            return False

        if not self.phase4_explore_frontiers():
            print("Failed at phase 4")
            return False

        print("\n=== Exploration Complete ===")
        return True


def main():
    bot = ExplorationBot(START_POSITION, START_DIRECTION.value, grid_resolution=0.1)

    success = bot.run_exploration()

    visited_to_file("out/visited_positions.csv")
    print(f"Visited positions saved to visited_positions.csv")

    if success:
        print("Exploration successful!")
    else:
        print("Exploration encountered issues")


if __name__ == "__main__":
    main()
