import numpy as np

from utils import (
    Direction,
    LIDAR_RADIUS,
    START_POSITION,
    START_DIRECTION,
    get_progress,
    sample_lidar,
    lidar_polygon,
    add_explored_area,
    add_visited,
    visited_to_file,
)


def main():
    position = START_POSITION
    direction = START_DIRECTION
    last_sweep = START_DIRECTION

    unexplored = True

    try:
        while unexplored:
            progress = get_progress()
            print(f"Exploration progress: {progress*100:.2f}%")
            if progress >= 0.99:
                unexplored = False
                break

            sample_points = sample_lidar(position)
            index = np.rad2deg(direction.value) % 360
            print(
                f"Current position: ({position.x:.2f}, {position.y:.2f}), moving {direction.name}"
            )
            lidar_point = sample_points[int(index)]
            lidar_area = lidar_polygon(sample_points)
            add_explored_area(lidar_area)

            if position.distance(lidar_point) >= LIDAR_RADIUS:
                add_visited(position)
                position = lidar_point

                if direction == Direction.RIGHT:
                    direction = (
                        Direction.UP if last_sweep != Direction.UP else Direction.DOWN
                    )
                    last_sweep = direction
            else:
                direction = Direction.RIGHT

                index = np.rad2deg(direction.value) % 360
                lidar_point = sample_points[int(index)]

                add_visited(position)
                position = lidar_point

    finally:
        visited_to_file("out/visited.txt")


if __name__ == "__main__":
    main()
