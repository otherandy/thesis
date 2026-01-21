import numpy as np
from shapely.geometry import Point
from utils import (
    LIDAR_RADIUS,
    START_POSITION,
    START_DIRECTION,
    graph_to_file,
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
    grid_to_file,
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
                node_idx = self.boundary_graph.add_node(BoundaryNode(self.position))
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
        - Follow the wall using right-hand rule (keep wall on right side)
        - Update occupancy grid as exploring
        - Detect when returning to start point
        """
        print("\n=== Phase 2: Boundary Tracking ===")

        step_distance = 0.05
        min_return_distance = 0.25  # Distance threshold to consider as "returned"
        tracking_steps = 0
        max_steps = 5000  # Safety limit
        min_steps_before_return = 100  # Minimum steps before checking return

        # Wall-following parameters (right-hand rule)
        target_wall_distance = LIDAR_RADIUS * 0.6  # Desired distance to wall

        while tracking_steps < max_steps:
            tracking_steps += 1

            # Take lidar reading
            lidar_points = self.take_lidar_reading()
            num_samples = len(lidar_points)

            # Update occupancy grid
            update_occupancy_from_lidar(
                self.occupancy_grid, self.position, lidar_points
            )

            # Store wall points
            wall_points = extract_wall_points(lidar_points, distance_threshold=0.15)
            self.wall_points.extend(wall_points)

            # Detect if we've returned to start (only after substantial movement)
            if tracking_steps > min_steps_before_return:
                dist_to_start = self.position.distance(self.first_contact_pos)
                if dist_to_start < min_return_distance:
                    print(f"Returned to start after {tracking_steps} steps")
                    return True

            # Right-hand wall following: get distance to nearest obstacle from robot position
            # in key directions relative to current heading

            def get_min_distance_in_direction(angle_offset, cone_width=np.pi / 8):
                """Get minimum distance in a cone around angle_offset from heading."""
                min_dist = LIDAR_RADIUS
                target_angle = normalize_angle(self.heading + angle_offset)

                for i, point in enumerate(lidar_points):
                    # Calculate angle from robot to this lidar point
                    point_angle = compute_angle_to_point(self.position, point)
                    angle_diff = abs(angle_difference(point_angle, target_angle))

                    if angle_diff < cone_width:
                        dist = self.position.distance(point)
                        min_dist = min(min_dist, dist)

                return min_dist

            # Get distances in key directions relative to robot's heading
            dist_front = get_min_distance_in_direction(0)
            dist_right = get_min_distance_in_direction(-np.pi / 2)
            dist_front_right = get_min_distance_in_direction(-np.pi / 4)

            # Simplified wall-following decision logic (right-hand rule)
            # Priority: avoid collision, then maintain wall distance

            if dist_front < target_wall_distance * 0.8:
                # Wall ahead - turn left away from it
                self.heading = normalize_angle(self.heading + 0.35)
            elif dist_front_right < target_wall_distance * 0.7:
                # Corner approaching - turn left
                self.heading = normalize_angle(self.heading + 0.25)
            elif dist_right > target_wall_distance * 1.5:
                # Lost the wall - turn right to find it
                self.heading = normalize_angle(self.heading - 0.25)
            elif dist_right < target_wall_distance * 0.5:
                # Too close to wall - turn left slightly
                self.heading = normalize_angle(self.heading + 0.15)
            elif dist_right > target_wall_distance * 1.1:
                # Drifting from wall - turn right gently
                self.heading = normalize_angle(self.heading - 0.1)
            # else: maintain current heading

            # Move forward
            self.position = move_forward(self.position, self.heading, step_distance)
            add_visited(self.position)

        # Failed to return to start within max steps
        print(f"Failed to return to start after {max_steps} steps")
        return False

    def phase3_boundary_graph_construction(self):
        """
        Phase 3: Construct boundary graph.
        - Identify all frontiers: boundaries between known free cells and unknown cells
        - Divide them into disjoint groups (connected components)
        - Construct graph where each node represents a distinct frontier region
        """
        print("\n=== Phase 3: Boundary Graph Construction ===")

        # Get all frontier cells (unknown cells adjacent to free cells)
        frontier_cells = self.occupancy_grid.get_frontier_cells()

        if not frontier_cells:
            print("No frontier cells found")
            return False

        print(f"Found {len(frontier_cells)} frontier cells")

        # Cluster frontier cells into connected regions
        # Two frontier cells are in the same region if they are adjacent
        visited_frontiers = set()
        regions = []

        def get_connected_region(start_point: Point) -> list[Point]:
            """BFS to find all frontier cells connected to start_point."""
            region = []
            queue = [start_point]
            visited_frontiers.add((start_point.x, start_point.y))

            while queue:
                current = queue.pop(0)
                region.append(current)

                # Check all frontier cells for adjacency
                for frontier in frontier_cells:
                    coord = (frontier.x, frontier.y)
                    if coord in visited_frontiers:
                        continue

                    # Check if adjacent (within grid resolution distance)
                    dist = current.distance(frontier)
                    if dist < self.occupancy_grid.resolution * 1.5:  # Adjacent cells
                        visited_frontiers.add(coord)
                        queue.append(frontier)

            return region

        # Find all connected regions
        for frontier in frontier_cells:
            coord = (frontier.x, frontier.y)
            if coord not in visited_frontiers:
                region = get_connected_region(frontier)
                regions.append(region)

        print(f"Identified {len(regions)} distinct frontier regions")

        # Create a graph node for each region
        # Use centroid of region as representative position
        for region in regions:
            cx = float(np.mean([p.x for p in region]))
            cy = float(np.mean([p.y for p in region]))
            centroid = Point(cx, cy)

            # Use heading toward centroid from current position
            heading = compute_angle_to_point(self.position, centroid)
            node = BoundaryNode(centroid)
            self.boundary_graph.add_node(node)

        # Optionally connect regions that are close to each other
        # This helps with navigation planning between regions
        for i in range(len(self.boundary_graph.nodes)):
            for j in range(i + 1, len(self.boundary_graph.nodes)):
                dist = self.boundary_graph.nodes[i].position.distance(
                    self.boundary_graph.nodes[j].position
                )
                # Connect if reasonably close (within 2x lidar range)
                if dist < LIDAR_RADIUS * 2:
                    self.boundary_graph.add_edge(i, j)

        print(
            f"Graph constructed: {len(self.boundary_graph.nodes)} regions, "
            f"{len(self.boundary_graph.edges)} connections"
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

    try:
        success = bot.run_exploration()

        if success:
            print("Exploration successful!")
        else:
            print("Exploration encountered issues")
    except KeyboardInterrupt:
        print("Exploration interrupted by user")

    finally:
        # Save outputs
        visited_to_file("out/visited_positions.csv")
        print(f"Visited positions saved to visited_positions.csv")

        grid_to_file(bot.occupancy_grid, "out/occupancy_grid.csv")
        print(f"Occupancy grid saved to occupancy_grid.csv")

        graph_to_file(bot.boundary_graph, "out/boundary_graph.csv")
        print(f"Boundary graph saved to boundary_graph.csv")


if __name__ == "__main__":
    main()
