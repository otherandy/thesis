#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"

constexpr double CELL_SIZE = 0.1;
constexpr int MAP_WIDTH = (ENV_WIDTH * 2.0 / CELL_SIZE);
constexpr int MAP_HEIGHT = (ENV_HEIGHT * 2.0 / CELL_SIZE);
constexpr std::size_t MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;

enum class CellState
{
  Unknown,
  Free,
  Occupied
};

class OccupationGrid
{
private:
  std::array<CellState, MAP_SIZE> grid;
  Point origin;

  const double inv_cell_size = 1.0 / CELL_SIZE;
  const int radius_cells = static_cast<int>(std::ceil(LIDAR_RADIUS *
                                                      inv_cell_size));

  const double lidar_radius_sq = LIDAR_RADIUS * LIDAR_RADIUS;

public:
  OccupationGrid(Point origin) : origin(origin)
  {
    grid.fill(CellState::Unknown);
  }

  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings,
                  const Point &real_position)
  {
    const double pos_x = relative_position.x();
    const double pos_y = relative_position.y();

    // Convert relative position to grid cell coordinates
    const int pos_cell_x = static_cast<int>(std::floor((pos_x + ENV_WIDTH) * inv_cell_size));
    const int pos_cell_y = static_cast<int>(std::floor((pos_y + ENV_HEIGHT) * inv_cell_size));

    // For each LIDAR reading, trace the ray and mark cells
    for (const auto &r : readings)
    {
      // Calculate hit position in world space
      const double hit_x_world = real_position.x() + r.distance * cos(r.angle);
      const double hit_y_world = real_position.y() + r.distance * sin(r.angle);

      // Convert to robot-relative coordinates
      const double hit_x_rel = hit_x_world - origin.x();
      const double hit_y_rel = hit_y_world - origin.y();

      const int hit_cell_x = static_cast<int>(std::floor((hit_x_rel + ENV_WIDTH) * inv_cell_size));
      const int hit_cell_y = static_cast<int>(std::floor((hit_y_rel + ENV_HEIGHT) * inv_cell_size));

      // Trace from robot to hit point, marking cells as free
      double step_x = (hit_x_rel - pos_x) / (r.distance / CELL_SIZE);
      double step_y = (hit_y_rel - pos_y) / (r.distance / CELL_SIZE);

      double curr_x = pos_x;
      double curr_y = pos_y;
      int steps = static_cast<int>(r.distance / CELL_SIZE) + 1;

      for (int i = 0; i < steps; ++i)
      {
        int cell_x = static_cast<int>(std::floor((curr_x + ENV_WIDTH) * inv_cell_size));
        int cell_y = static_cast<int>(std::floor((curr_y + ENV_HEIGHT) * inv_cell_size));

        if (cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT)
        {
          const int cell_index = cell_y * MAP_WIDTH + cell_x;
          if (grid[cell_index] == CellState::Unknown)
          {
            grid[cell_index] = CellState::Free;
          }
        }

        curr_x += step_x;
        curr_y += step_y;
      }

      // Mark the hit cell as occupied
      if (hit_cell_x >= 0 && hit_cell_x < MAP_WIDTH &&
          hit_cell_y >= 0 && hit_cell_y < MAP_HEIGHT)
      {
        const int cell_index = hit_cell_y * MAP_WIDTH + hit_cell_x;
        if (grid[cell_index] == CellState::Unknown)
        {
          grid[cell_index] = CellState::Occupied;
        }
      }
    }
  }

  void draw(float scale_factor, const raylib::Vector2 &offset) const
  {
    for (int y = 0; y < MAP_HEIGHT; ++y)
    {
      for (int x = 0; x < MAP_WIDTH; ++x)
      {
        const int cell_index = y * MAP_WIDTH + x;
        const CellState state = grid[cell_index];

        // Convert grid indices back to world coordinates
        const double world_x = (x * CELL_SIZE) - ENV_WIDTH;
        const double world_y = (y * CELL_SIZE) - ENV_HEIGHT;

        // Offset by origin (starting position) and apply screen scale
        const float screen_x = (origin.x() + world_x) * scale_factor + offset.x;
        const float screen_y = (origin.y() + world_y) * scale_factor + offset.y;

        if (state == CellState::Free)
        {
          DrawRectangle(screen_x,
                        screen_y,
                        CELL_SIZE * scale_factor,
                        CELL_SIZE * scale_factor,
                        LIGHTGRAY);
        }
        else if (state == CellState::Occupied)
        {
          DrawRectangle(screen_x,
                        screen_y,
                        CELL_SIZE * scale_factor,
                        CELL_SIZE * scale_factor,
                        DARKGRAY);
        }
      }
    }
  }
};

#endif