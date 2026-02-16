#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"

constexpr double CELL_SIZE = 0.1;
const double INV_CELL_SIZE = 1.0 / CELL_SIZE;
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

  inline int coord_to_cell_x(double x) const
  {
    return static_cast<int>(std::floor((x + ENV_WIDTH) * INV_CELL_SIZE));
  }

  inline int coord_to_cell_y(double y) const
  {
    return static_cast<int>(std::floor((y + ENV_HEIGHT) * INV_CELL_SIZE));
  }

  inline bool is_cell_valid(int cell_x, int cell_y) const
  {
    return cell_x >= 0 && cell_x < MAP_WIDTH && cell_y >= 0 && cell_y < MAP_HEIGHT;
  }

  inline int get_cell_index(int cell_x, int cell_y) const
  {
    return cell_y * MAP_WIDTH + cell_x;
  }

  void mark_cell_if_unknown(int cell_x, int cell_y, CellState state)
  {
    if (is_cell_valid(cell_x, cell_y))
    {
      int idx = get_cell_index(cell_x, cell_y);
      if (grid[idx] == CellState::Unknown)
      {
        grid[idx] = state;
      }
    }
  }

  void draw_cell(int x, int y, float scale_factor, float offset_x, float offset_y) const
  {
    const CellState state = grid[get_cell_index(x, y)];
    const double relative_x = (x * CELL_SIZE) - ENV_WIDTH;
    const double relative_y = (y * CELL_SIZE) - ENV_HEIGHT;

    float screen_x = (origin.x() + relative_x) * scale_factor + offset_x;
    float screen_y = (origin.y() + relative_y) * scale_factor + offset_y;
    float cell_size_scaled = CELL_SIZE * scale_factor;

    Color color = (state == CellState::Free) ? LIGHTGRAY : DARKGRAY;
    if (state != CellState::Unknown)
    {
      DrawRectangle(screen_x, screen_y, cell_size_scaled, cell_size_scaled, color);
    }
  }

public:
  OccupationGrid(Point origin) : origin(origin)
  {
    grid.fill(CellState::Unknown);
  }

  void mark_cells(const Point &real_position,
                  const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings)
  {
    for (const auto &r : readings)
    {
      double hit_x_world = real_position.x() + r.distance * cos(r.angle);
      double hit_y_world = real_position.y() + r.distance * sin(r.angle);

      double hit_x_rel = hit_x_world - origin.x();
      double hit_y_rel = hit_y_world - origin.y();

      int hit_cell_x = coord_to_cell_x(hit_x_rel);
      int hit_cell_y = coord_to_cell_y(hit_y_rel);

      double steps_count = r.distance / CELL_SIZE;
      double step_x = (hit_x_rel - relative_position.x()) / steps_count;
      double step_y = (hit_y_rel - relative_position.y()) / steps_count;

      double curr_x = relative_position.x();
      double curr_y = relative_position.y();

      for (int i = 0; i <= static_cast<int>(steps_count); ++i)
      {
        mark_cell_if_unknown(coord_to_cell_x(curr_x), coord_to_cell_y(curr_y), CellState::Free);
        curr_x += step_x;
        curr_y += step_y;
      }

      mark_cell_if_unknown(hit_cell_x, hit_cell_y, CellState::Occupied);
    }
  }

  void draw(float scale_factor, float offset_x, float offset_y) const
  {
    for (int y = 0; y < MAP_HEIGHT; ++y)
    {
      for (int x = 0; x < MAP_WIDTH; ++x)
      {
        draw_cell(x, y, scale_factor, offset_x, offset_y);
      }
    }
  }

  void save_to_file(const std::string &filename) const
  {
    ensure_parent_dir_exists(filename);
    std::ofstream f(filename);
    for (int y = 0; y < MAP_HEIGHT; ++y)
    {
      for (int x = 0; x < MAP_WIDTH; ++x)
      {
        f << static_cast<int>(grid[get_cell_index(x, y)]) << " ";
      }
      f << "\n";
    }
    f.close();
    std::cout << "BOT: Occupation grid saved to " << filename << "\n";
  }
};

#endif