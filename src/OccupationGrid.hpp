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
  Occupied,
  Visited,
  Frontier
};

const std::map<CellState, Color> CellColors = {
    {CellState::Unknown, GRAY},
    {CellState::Free, YELLOW},
    {CellState::Occupied, BLACK},
    {CellState::Visited, RED},
    {CellState::Frontier, BLUE}};

class OccupationGrid
{
private:
  std::array<CellState, MAP_SIZE> grid;
  Point origin;
  bool frontier_was_added = false;

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
    return cell_x >= 0 && cell_x < MAP_WIDTH &&
           cell_y >= 0 && cell_y < MAP_HEIGHT;
  }

  inline int get_cell_index(int cell_x, int cell_y) const
  {
    return cell_y * MAP_WIDTH + cell_x;
  }

  inline CellState get_cell_state(int cell_x, int cell_y) const
  {
    if (!is_cell_valid(cell_x, cell_y))
      return CellState::Unknown;
    return grid[get_cell_index(cell_x, cell_y)];
  }

  void verify_and_mark_cell(int cell_x, int cell_y, CellState new_state)
  {
    if (is_cell_valid(cell_x, cell_y))
    {
      int idx = get_cell_index(cell_x, cell_y);

      // Always overwrite cells to Visited
      if (new_state == CellState::Visited)
      {
        grid[idx] = CellState::Visited;
        return;
      }

      // Don't overwrite Occupied or Visited states
      if (grid[idx] == CellState::Occupied || grid[idx] == CellState::Visited)
        return;

      // Don't mark Free cells as Frontier
      if (grid[idx] == CellState::Free && new_state == CellState::Frontier)
        return;

      if (new_state == CellState::Frontier)
        frontier_was_added = true;

      grid[idx] = new_state;
    }
  }

  void draw_cell(int x, int y, float scale_factor, float offset_x, float offset_y) const
  {
    const double relative_x = (x * CELL_SIZE) - ENV_WIDTH;
    const double relative_y = (y * CELL_SIZE) - ENV_HEIGHT;

    const float screen_x = (origin.x() + relative_x) * scale_factor + offset_x;
    const float screen_y = (origin.y() + relative_y) * scale_factor + offset_y;
    const float cell_size_scaled = CELL_SIZE * scale_factor;

    const CellState state = grid[get_cell_index(x, y)];
    Color color = CellColors.at(state);

    if (state != CellState::Unknown)
      DrawRectangleLines(screen_x, screen_y,
                         cell_size_scaled, cell_size_scaled,
                         color);
  }

public:
  OccupationGrid(Point origin) : origin(origin)
  {
    grid.fill(CellState::Unknown);
  }

  bool was_frontier_added() const
  {
    return frontier_was_added;
  }

  void mark_cells(const Point &real_position,
                  const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings)
  {
    const double rel_pos_x = relative_position.x();
    const double rel_pos_y = relative_position.y();

    const int pos_cell_x = coord_to_cell_x(rel_pos_x);
    const int pos_cell_y = coord_to_cell_y(rel_pos_y);

    verify_and_mark_cell(pos_cell_x, pos_cell_y, CellState::Visited);

    frontier_was_added = false;

    for (const auto &r : readings)
    {
      const double hit_x_world = real_position.x() + r.distance * cos(r.angle);
      const double hit_y_world = real_position.y() + r.distance * sin(r.angle);

      const double hit_x_rel = hit_x_world - origin.x();
      const double hit_y_rel = hit_y_world - origin.y();

      const int hit_cell_x = coord_to_cell_x(hit_x_rel);
      const int hit_cell_y = coord_to_cell_y(hit_y_rel);

      const double steps_count = r.distance / CELL_SIZE;
      const double step_x = (hit_x_rel - rel_pos_x) / steps_count;
      const double step_y = (hit_y_rel - rel_pos_y) / steps_count;

      double curr_x = rel_pos_x;
      double curr_y = rel_pos_y;

      for (int i = 0; i < steps_count; ++i)
      {
        const int cell_x = coord_to_cell_x(curr_x);
        const int cell_y = coord_to_cell_y(curr_y);
        verify_and_mark_cell(cell_x, cell_y, CellState::Free);
        curr_x += step_x;
        curr_y += step_y;
      }

      if (r.distance < LIDAR_RADIUS)
      {
        verify_and_mark_cell(hit_cell_x, hit_cell_y, CellState::Occupied);
      }
      else
      {
        verify_and_mark_cell(hit_cell_x, hit_cell_y, CellState::Frontier);
      }
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
    if (!f.is_open())
    {
      std::cerr << "BOT: Failed to open " << filename << " for writing" << std::endl;
      return;
    }
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