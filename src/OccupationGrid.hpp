#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include <queue>
#include "Bot.hpp"

constexpr double CELL_SIZE = 0.1;
const double INV_CELL_SIZE = 1.0 / CELL_SIZE;
constexpr std::size_t MAP_WIDTH = (ENV_WIDTH * 2.0 / CELL_SIZE);
constexpr std::size_t MAP_HEIGHT = (ENV_HEIGHT * 2.0 / CELL_SIZE);
constexpr std::size_t MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;

enum class CellState
{
  Unknown,
  Free,
  Occupied,
  Visited,
  Frontier
};

struct Cell
{
  CellState state;
  int frontier_id;
};

const std::map<CellState, Color> CellColors = {
    {CellState::Unknown, GRAY},
    {CellState::Free, YELLOW},
    {CellState::Occupied, BLACK},
    {CellState::Visited, RED},
    {CellState::Frontier, BLUE}};

const std::array<Color, 9> FrontierColors = {
    BLUE,
    LIME,
    VIOLET,
    DARKBLUE,
    DARKGREEN,
    DARKPURPLE,
    SKYBLUE,
    GREEN,
    PURPLE,
};

class OccupationGrid
{
private:
  Point origin;
  std::array<Cell, MAP_SIZE> grid;

  bool frontier_cell_was_added = false;
  std::size_t frontier_count = 0;

  // Track bounding box of frontier cells for optimization
  int min_frontier_idx = std::numeric_limits<int>::max();
  int max_frontier_idx = std::numeric_limits<int>::min();

  inline int coord_to_cell_x(double x) const
  {
    return std::floor((x + ENV_WIDTH) * INV_CELL_SIZE);
  }

  inline int coord_to_cell_y(double y) const
  {
    return std::floor((y + ENV_HEIGHT) * INV_CELL_SIZE);
  }

  inline bool is_valid_cell(int cell_x, int cell_y) const
  {
    return cell_x >= 0 && cell_x < MAP_WIDTH &&
           cell_y >= 0 && cell_y < MAP_HEIGHT;
  }

  inline int get_cell_index(int cell_x, int cell_y) const
  {
    return cell_y * MAP_WIDTH + cell_x;
  }

  inline std::pair<int, int> get_cell_coordinates(int idx) const
  {
    int cell_x = idx % MAP_WIDTH;
    int cell_y = idx / MAP_WIDTH;
    return {cell_x, cell_y};
  }

  bool verify_and_mark_cell(int cell_x, int cell_y, CellState new_state)
  {
    if (is_valid_cell(cell_x, cell_y))
    {
      int idx = get_cell_index(cell_x, cell_y);
      Cell &cell = grid[idx];

      // Always overwrite cells to Visited
      if (new_state == CellState::Visited)
      {
        cell.state = CellState::Visited;
        return true;
      }

      // Don't overwrite Occupied or Visited states
      if (cell.state == CellState::Occupied ||
          cell.state == CellState::Visited)
      {
        return false;
      }

      // Don't mark known cells as Frontier
      if (cell.state == CellState::Free &&
          new_state == CellState::Frontier)
      {
        return false;
      }

      if (new_state == CellState::Frontier)
      {
        frontier_cell_was_added = true;
        min_frontier_idx = std::min(min_frontier_idx, idx);
        max_frontier_idx = std::max(max_frontier_idx, idx);
      }

      cell.state = new_state;
      return true;
    }

    return false;
  }

  void draw_cell(int x, int y,
                 float scale_factor, float offset_x, float offset_y) const
  {
    const int idx = get_cell_index(x, y);
    const CellState state = grid[idx].state;

    if (state != CellState::Unknown)
    {
      const double relative_x = (x * CELL_SIZE) - ENV_WIDTH;
      const double relative_y = (y * CELL_SIZE) - ENV_HEIGHT;

      const float screen_x = (origin.x() + relative_x) * scale_factor + offset_x;
      const float screen_y = (origin.y() + relative_y) * scale_factor + offset_y;
      const float cell_size_scaled = CELL_SIZE * scale_factor;

      if (state == CellState::Frontier)
      {
        const int frontier_id = grid[idx].frontier_id;
        const int color_idx = (frontier_id + 1) % FrontierColors.size();
        const Color color = FrontierColors[color_idx];

        DrawRectangleLines(screen_x, screen_y,
                           cell_size_scaled, cell_size_scaled,
                           color);
      }
      else
      {
        Color color = CellColors.at(state);

        DrawRectangleLines(screen_x, screen_y,
                           cell_size_scaled, cell_size_scaled,
                           color);
      }
    }
  }

public:
  OccupationGrid(Point origin) : origin(origin)
  {
    grid.fill({CellState::Unknown, -1});
  }

  bool was_frontier_cell_added() const
  {
    return frontier_cell_was_added;
  }

  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings)
  {
    const double rel_pos_x = relative_position.x();
    const double rel_pos_y = relative_position.y();

    const int pos_cell_x = coord_to_cell_x(rel_pos_x);
    const int pos_cell_y = coord_to_cell_y(rel_pos_y);

    verify_and_mark_cell(pos_cell_x, pos_cell_y, CellState::Visited);

    frontier_cell_was_added = false;
    std::vector<int> frontier_cells_to_update;

    for (const auto &r : readings)
    {
      const double hit_x_rel = rel_pos_x + r.distance * cos(r.angle);
      const double hit_y_rel = rel_pos_y + r.distance * sin(r.angle);

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

  void compute_frontier_regions()
  {
    frontier_count = 0;

    int current_region_id = 0;

    for (int idx = min_frontier_idx; idx <= max_frontier_idx; ++idx)
    {
      if (grid[idx].state == CellState::Frontier && grid[idx].frontier_id == -1)
      {
        std::queue<int> q;
        q.push(idx);
        grid[idx].frontier_id = current_region_id;

        while (!q.empty())
        {
          const int current_idx = q.front();
          q.pop();

          const auto [cx, cy] = get_cell_coordinates(current_idx);

          for (int dy = -1; dy <= 1; ++dy)
          {
            for (int dx = -1; dx <= 1; ++dx)
            {
              if (dx == 0 && dy == 0)
              {
                continue;
              }

              const int nx = cx + dx;
              const int ny = cy + dy;

              if (is_valid_cell(nx, ny))
              {
                const int neighbor_idx = get_cell_index(nx, ny);
                if (grid[neighbor_idx].state == CellState::Frontier &&
                    grid[neighbor_idx].frontier_id == -1)
                {
                  grid[neighbor_idx].frontier_id = current_region_id;
                  q.push(neighbor_idx);
                }
              }
            }
          }
        }

        current_region_id++;
        frontier_count++;
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

  void draw_frontier_count() const
  {
    const std::string text = "Frontiers: " + std::to_string(frontier_count);
    DrawText(text.c_str(), 10, GetScreenHeight() - 30, 20, BLACK);
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
        const int idx = get_cell_index(x, y);
        f << static_cast<int>(grid[idx].state) << " ";
      }
      f << "\n";
    }

    f.close();
    std::cout << "BOT: Occupation grid saved to " << filename << std::endl;
  }
};

#endif