#include "OccupationGrid.hpp"
#include "FrontierRegion.hpp"
#include "Utils.hpp"
#include <limits>

bool OccupationGrid::mark_cell(Index2D index, CellState new_state) {
  Cell &cell = grid[index.first][index.second];

  // Always overwrite cells to Visited
  if (new_state == CellState::Visited) {
    cell.state = CellState::Visited;
    return true;
  }

  // Don't overwrite Occupied or Visited states
  if (cell.state == CellState::Occupied || cell.state == CellState::Visited) {
    return false;
  }

  // Don't mark known cells as Frontier
  if (cell.state == CellState::Free && new_state == CellState::Frontier) {
    return false;
  }

  // Indicate that a frontier cell was added this update
  if (new_state == CellState::Frontier) {
    frontier_cell_was_added = true;
  }

  // Update frontier cell count
  if (cell.state != CellState::Frontier && new_state == CellState::Frontier) {
    number_of_frontier_cells++;
  }

  if (cell.state == CellState::Frontier && new_state != CellState::Frontier) {
    number_of_frontier_cells--;
  }

  cell.state = new_state;
  return true;
}

void OccupationGrid::draw_cell(Index2D cell_index, DrawData draw_data) const {
  const Cell cell = grid[cell_index.first][cell_index.second];

  if (cell.state == CellState::Unknown) {
    return;
  }

  const double relative_x = (cell_index.second * CELL_SIZE) - ENV_WIDTH;
  const double relative_y = (cell_index.first * CELL_SIZE) - ENV_HEIGHT;

  const float screen_x =
      (origin.x() + relative_x) * draw_data.scale_factor + draw_data.offset_x;
  const float screen_y =
      (origin.y() + relative_y) * draw_data.scale_factor + draw_data.offset_y;
  const float cell_size_scaled = CELL_SIZE * draw_data.scale_factor;

  if (cell.state == CellState::Frontier) {
    const int frontier_id = cell.frontier_id.value();
    const int color_idx = frontier_id % FrontierColors.size();
    const Color color = FrontierColors[color_idx];

    DrawRectangleLines(screen_x, screen_y, cell_size_scaled, cell_size_scaled,
                       color);
    return;
  }

  Color color = CellColors.at(cell.state);

  DrawRectangleLines(screen_x, screen_y, cell_size_scaled, cell_size_scaled,
                     color);
}

void OccupationGrid::draw_cell_center(Index2D index, DrawData draw_data) const {
  const Cell &cell = grid[index.first][index.second];

  if (cell.state == CellState::Unknown) {
    return;
  }

  const float screen_x =
      (cell.center.x() + origin.x()) * draw_data.scale_factor +
      draw_data.offset_x;
  const float screen_y =
      (cell.center.y() + origin.y()) * draw_data.scale_factor +
      draw_data.offset_y;

  if (cell.state == CellState::Frontier) {
    Color color = CellColors.at(CellState::Frontier);

    if (cell.frontier_id) {
      const int frontier_id = cell.frontier_id.value();
      const int color_idx = frontier_id % FrontierColors.size();
      color = FrontierColors[color_idx];
    }

    DrawCircle(screen_x, screen_y, 2, color);
    return;
  }

  DrawCircle(screen_x, screen_y, 2, CellColors.at(cell.state));
}

OccupationGrid::OccupationGrid() {
  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      const double cell_center_x = (x + 0.5) * CELL_SIZE - ENV_WIDTH;
      const double cell_center_y = (y + 0.5) * CELL_SIZE - ENV_HEIGHT;

      grid[y][x] = {Point(cell_center_x, cell_center_y), CellState::Unknown,
                    std::nullopt};
    }
  }
}

void OccupationGrid::mark_cells(
    const Point &relative_position,
    const std::array<Reading, MAX_LIDAR_SAMPLES> &readings) {
  const double rel_pos_x = relative_position.x();
  const double rel_pos_y = relative_position.y();

  const Index2D relative_cell_index = get_cell_index_from(rel_pos_x, rel_pos_y);

  mark_cell(relative_cell_index, CellState::Visited);

  frontier_cell_was_added = false;
  std::vector<int> frontier_cells_to_update;

  for (const Reading &r : readings) {
    const double hit_x_rel = rel_pos_x + r.distance * cos(r.angle);
    const double hit_y_rel = rel_pos_y + r.distance * sin(r.angle);

    const Index2D hit_cell_index = get_cell_index_from(hit_x_rel, hit_y_rel);

    const double steps_count = r.distance / CELL_SIZE;
    const double step_x = (hit_x_rel - rel_pos_x) / steps_count;
    const double step_y = (hit_y_rel - rel_pos_y) / steps_count;

    double curr_x = rel_pos_x;
    double curr_y = rel_pos_y;

    for (int i = 0; i < steps_count; ++i) {
      const Index2D cell = get_cell_index_from(curr_x, curr_y);

      mark_cell(cell, CellState::Free);
      curr_x += step_x;
      curr_y += step_y;
    }

    if (r.distance < LIDAR_RADIUS) {
      mark_cell(hit_cell_index, CellState::Occupied);
      continue;
    }

    mark_cell(hit_cell_index, CellState::Frontier);
  }
}

Cell &OccupationGrid::get_cell_from_position(const Point &position) {
  const double rel_x = position.x() - origin.x();
  const double rel_y = position.y() - origin.y();

  const Index2D cell_index = get_cell_index_from(rel_x, rel_y);

  return grid[cell_index.first][cell_index.second];
}

bool OccupationGrid::there_is_obstacle_between(const Point &from,
                                               const Point &to) const {
  const double dist2 = CGAL::squared_distance(from, to);

  if (dist2 <= 1e-12) {
    const Index2D idx = get_cell_index_from(from.x(), from.y());
    return grid[idx.first][idx.second].state == CellState::Occupied;
  }

  const double gx0 = (from.x() + ENV_WIDTH) * INV_CELL_SIZE;
  const double gy0 = (from.y() + ENV_HEIGHT) * INV_CELL_SIZE;
  const double gx1 = (to.x() + ENV_WIDTH) * INV_CELL_SIZE;
  const double gy1 = (to.y() + ENV_HEIGHT) * INV_CELL_SIZE;

  const double dx = gx1 - gx0;
  const double dy = gy1 - gy0;

  const Index2D start = get_cell_index_from(from.x(), from.y());
  const Index2D goal = get_cell_index_from(to.x(), to.y());

  int x = static_cast<int>(start.second);
  int y = static_cast<int>(start.first);
  const int end_x = static_cast<int>(goal.second);
  const int end_y = static_cast<int>(goal.first);

  auto in_bounds = [](int yy, int xx) {
    return (yy >= 0 && xx >= 0 && yy < static_cast<int>(MAP_HEIGHT) &&
            xx < static_cast<int>(MAP_WIDTH));
  };

  auto is_occupied = [&](int yy, int xx) {
    if (!in_bounds(yy, xx)) {
      return true;
    }
    return grid[yy][xx].state == CellState::Occupied;
  };

  const int step_x = (dx > 0) ? 1 : (dx < 0) ? -1 : 0;
  const int step_y = (dy > 0) ? 1 : (dy < 0) ? -1 : 0;

  const double inf = std::numeric_limits<double>::infinity();
  const double t_delta_x = (step_x != 0) ? (1.0 / std::abs(dx)) : inf;
  const double t_delta_y = (step_y != 0) ? (1.0 / std::abs(dy)) : inf;

  // tMax = "t" until the first grid boundary crossing on each axis.
  double t_max_x = inf;
  if (step_x != 0) {
    const double next_vert_grid =
        (step_x > 0) ? (std::floor(gx0) + 1.0) : std::floor(gx0);
    t_max_x = (next_vert_grid - gx0) / dx; // positive
  }

  double t_max_y = inf;
  if (step_y != 0) {
    const double next_horiz_grid =
        (step_y > 0) ? (std::floor(gy0) + 1.0) : std::floor(gy0);
    t_max_y = (next_horiz_grid - gy0) / dy; // positive
  }

  while (true) {
    if (is_occupied(y, x)) {
      return true;
    }

    if (x == end_x && y == end_y) {
      break;
    }

    if (std::abs(t_max_x - t_max_y) < 1e-12) {
      x += step_x;
      y += step_y;
      t_max_x += t_delta_x;
      t_max_y += t_delta_y;
    } else if (t_max_x < t_max_y) {
      x += step_x;
      t_max_x += t_delta_x;
    } else {
      y += step_y;
      t_max_y += t_delta_y;
    }
  }

  return false;
}

void OccupationGrid::draw(DrawData draw_data) const {
  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      draw_cell_center({y, x}, draw_data);
    }
  }
}

void OccupationGrid::save_to_file(const std::string &filename) const {
  ensure_parent_dir_exists(filename);
  std::ofstream f(filename);

  if (!f.is_open()) {
    std::cerr << "GRID: Failed to open " << filename << " for writing"
              << std::endl;
    return;
  }

  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      f << static_cast<int>(grid[y][x].state) << " ";
    }
    f << "\n";
  }

  f.close();
  std::cout << "GRID: Occupation grid saved to " << filename << std::endl;
}
