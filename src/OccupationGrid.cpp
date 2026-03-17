#include "OccupationGrid.hpp"
#include "Utils.hpp"
#include <queue>

bool OccupationGrid::verify_and_mark_cell(Index2D index,
                                          CellState new_state)
{
  if (!is_valid_index(index))
  {
    return false;
  }

  Cell &cell = grid[index.first][index.second];

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
  }

  if (cell.state != CellState::Frontier &&
      new_state == CellState::Frontier)
  {
    number_of_frontier_cells++;
  }

  if (cell.state == CellState::Frontier &&
      new_state != CellState::Frontier)
  {
    number_of_frontier_cells--;
  }

  cell.state = new_state;
  return true;
}

void OccupationGrid::draw_cell(std::pair<int, int> cell_index,
                               float scale_factor,
                               float offset_x, float offset_y) const
{
  const Cell cell = grid[cell_index.first][cell_index.second];

  if (cell.state == CellState::Unknown)
  {
    return;
  }

  const double relative_x = (cell_index.first * CELL_SIZE) - ENV_WIDTH;
  const double relative_y = (cell_index.second * CELL_SIZE) - ENV_HEIGHT;

  const float screen_x = (origin.x() + relative_x) * scale_factor + offset_x;
  const float screen_y = (origin.y() + relative_y) * scale_factor + offset_y;
  const float cell_size_scaled = CELL_SIZE * scale_factor;

  if (cell.state == CellState::Frontier)
  {
    const int frontier_id = cell.frontier_id;
    const int color_idx = frontier_id % FrontierColors.size();
    const Color color = FrontierColors[color_idx];

    DrawRectangleLines(screen_x, screen_y,
                       cell_size_scaled, cell_size_scaled,
                       color);
    return;
  }

  Color color = CellColors.at(cell.state);

  DrawRectangleLines(screen_x, screen_y,
                     cell_size_scaled, cell_size_scaled,
                     color);
}

OccupationGrid::OccupationGrid(Point origin) : origin(origin)
{
  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      const double cell_center_x = (x + 0.5) * CELL_SIZE - ENV_WIDTH;
      const double cell_center_y = (y + 0.5) * CELL_SIZE - ENV_HEIGHT;

      grid[y][x] = {Point(cell_center_x, cell_center_y),
                    CellState::Unknown,
                    -1};
    }
  }
}

bool OccupationGrid::was_frontier_cell_added() const
{
  return frontier_cell_was_added;
}

int OccupationGrid::get_frontier_cell_count() const
{
  return number_of_frontier_cells;
}

void OccupationGrid::mark_cells(const Point &relative_position,
                                const std::array<Reading, MAX_LIDAR_SAMPLES> &readings)
{
  const double rel_pos_x = relative_position.x();
  const double rel_pos_y = relative_position.y();

  const auto relative_cell_index = get_cell_index_from(rel_pos_x, rel_pos_y);

  verify_and_mark_cell(relative_cell_index, CellState::Visited);

  frontier_cell_was_added = false;
  std::vector<int> frontier_cells_to_update;

  for (const auto &r : readings)
  {
    const double hit_x_rel = rel_pos_x + r.distance * cos(r.angle);
    const double hit_y_rel = rel_pos_y + r.distance * sin(r.angle);

    const auto hit_cell_index = get_cell_index_from(hit_x_rel, hit_y_rel);

    const double steps_count = r.distance / CELL_SIZE;
    const double step_x = (hit_x_rel - rel_pos_x) / steps_count;
    const double step_y = (hit_y_rel - rel_pos_y) / steps_count;

    double curr_x = rel_pos_x;
    double curr_y = rel_pos_y;

    for (int i = 0; i < steps_count; ++i)
    {
      const auto cell = get_cell_index_from(curr_x, curr_y);
      verify_and_mark_cell(cell, CellState::Free);
      curr_x += step_x;
      curr_y += step_y;
    }

    if (r.distance < LIDAR_RADIUS)
    {
      verify_and_mark_cell(hit_cell_index, CellState::Occupied);
    }
    else
    {
      verify_and_mark_cell(hit_cell_index, CellState::Frontier);
    }
  }
}

Cell &OccupationGrid::get_cell_from_position(const Point &position)
{
  const double rel_x = position.x() - origin.x();
  const double rel_y = position.y() - origin.y();

  const auto cell_index = get_cell_index_from(rel_x, rel_y);

  if (!is_valid_index(cell_index))
  {
    throw std::out_of_range("Position is out of grid bounds");
  }

  return grid[cell_index.second][cell_index.first];
}

void OccupationGrid::compute_frontier_regions()
{
  frontier_regions.clear();

  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      Cell &cell = grid[y][x];

      if (cell.state == CellState::Frontier && cell.frontier_id == -1)
      {
        std::queue<Index2D> q;
        q.push({x, y});
        cell.frontier_id = current_frontier_id;
        frontier_regions.push_back({current_frontier_id, {cell}});

        while (!q.empty())
        {
          const Index2D current_idx = q.front();
          q.pop();

          for (int dy = -1; dy <= 1; ++dy)
          {
            for (int dx = -1; dx <= 1; ++dx)
            {
              if (dx == 0 && dy == 0)
              {
                continue;
              }

              const int nx = current_idx.first + dx;
              const int ny = current_idx.second + dy;

              if (is_valid_index({nx, ny}))
              {
                Cell &neighbor_cell = grid[ny][nx];
                if (neighbor_cell.state == CellState::Frontier &&
                    neighbor_cell.frontier_id == -1)
                {
                  neighbor_cell.frontier_id = current_frontier_id;
                  frontier_regions.back().cells.push_back(neighbor_cell);
                  q.push({nx, ny});
                }
              }
            }
          }
        }

        current_frontier_id++;
      }
    }
  }
}

const FrontierRegion *OccupationGrid::get_nearest_frontier_region(const Point &position) const
{
  const FrontierRegion *nearest_region = nullptr;
  double nearest_distance = std::numeric_limits<double>::max();

  for (const auto &region : frontier_regions)
  {
    const double distance = std::sqrt(CGAL::squared_distance(position, region.get_closest_from(position)));

    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_region = &region;
    }
  }

  return nearest_region;
}

void OccupationGrid::draw(float scale_factor,
                          float offset_x, float offset_y) const
{
  draw_cell_centers(scale_factor, offset_x, offset_y);
  // for (int y = 0; y < MAP_HEIGHT; ++y)
  // {
  //   for (int x = 0; x < MAP_WIDTH; ++x)
  //   {
  //     draw_cell({x, y}, scale_factor, offset_x, offset_y);
  //   }
  // }
}

void OccupationGrid::draw_cell_centers(float scale_factor,
                                       float offset_x, float offset_y) const
{
  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      const Cell &cell = grid[y][x];
      if (cell.state != CellState::Unknown)
      {
        const float screen_x = (cell.center.x() + origin.x()) * scale_factor + offset_x;
        const float screen_y = (cell.center.y() + origin.y()) * scale_factor + offset_y;

        DrawCircle(screen_x, screen_y, 2, CellColors.at(cell.state));
      }
    }
  }
}

void OccupationGrid::draw_frontier_count() const
{
  const std::string text = "Frontiers: " +
                           std::to_string(frontier_regions.size());
  DrawText(text.c_str(), 10, GetScreenHeight() - 30, 20, BLACK);
}

void OccupationGrid::save_to_file(const std::string &filename) const
{
  ensure_parent_dir_exists(filename);
  std::ofstream f(filename);

  if (!f.is_open())
  {
    std::cerr << "GRID: Failed to open " << filename << " for writing" << std::endl;
    return;
  }

  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      f << static_cast<int>(grid[y][x].state) << " ";
    }
    f << "\n";
  }

  f.close();
  std::cout << "GRID: Occupation grid saved to " << filename << std::endl;
}