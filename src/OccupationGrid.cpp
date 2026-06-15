#include "OccupationGrid.hpp"
#include "FrontierRegion.hpp"
#include "Grid.hpp"
#include "Utils.hpp"
#include <memory>
#include <queue>
#include <utility>

bool OccupationGrid::mark_cell(Index2D index, CellState new_state) {
  if (index.first < grid_min_y) {
    grid_min_y = index.first;
  }
  if (index.first > grid_max_y) {
    grid_max_y = index.first + INV_CELL_SIZE;
  }
  if (index.second < grid_min_x) {
    grid_min_x = index.second;
  }
  if (index.second > grid_max_x) {
    grid_max_x = index.second + INV_CELL_SIZE;
  }

  Cell *cell = grid[index.first][index.second].get();

  // Always overwrite cells to Visited
  if (new_state == CellState::Visited) {
    cell->state = CellState::Visited;
    return true;
  }

  // Don't overwrite Occupied or Visited states
  if (cell->state == CellState::Occupied || cell->state == CellState::Visited) {
    return false;
  }

  // Don't mark known cells as Frontier
  if (cell->state == CellState::Free && new_state == CellState::Frontier) {
    return false;
  }

  cell->state = new_state;
  return true;
}

void OccupationGrid::draw_cell(Index2D index, const DrawData &draw_data) const {
  const Cell *cell = grid[index.first][index.second].get();

  if (cell->state == CellState::Unknown) {
    return;
  }

  Color color;

  switch (cell->state) {
  case CellState::Free:
    color = raylib::YELLOW;
    break;
  case CellState::Occupied:
    color = raylib::BLACK;
    break;
  case CellState::Visited:
    color = raylib::RED;
    break;
  case CellState::Frontier:
    color = raylib::BLUE;
    break;
  default:
    return;
  }

  const float screen_x =
      (cell->center.x() + origin.x()) * draw_data.scale_factor +
      draw_data.offset_x;
  const float screen_y =
      (cell->center.y() + origin.y()) * draw_data.scale_factor +
      draw_data.offset_y;

  DrawRectangle(screen_x, screen_y, 2, 2, color);
}

OccupationGrid::OccupationGrid() {
  for (std::size_t y = 0; y < MAP_HEIGHT; ++y) {
    for (std::size_t x = 0; x < MAP_WIDTH; ++x) {
      const double cell_center_x = (x + 0.5) * CELL_SIZE - ENV_WIDTH;
      const double cell_center_y = (y + 0.5) * CELL_SIZE - ENV_HEIGHT;

      grid[y][x] =
          std::make_shared<Cell>(std::make_pair(y, x), CellState::Unknown,
                                 Robot::Point(cell_center_x, cell_center_y));
    }
  }
}

void OccupationGrid::mark_cells(
    const Robot::Point &relative_position,
    const std::array<Reading, MAX_LIDAR_SAMPLES> &readings) {

  const double rel_pos_x = relative_position.x();
  const double rel_pos_y = relative_position.y();

  const Index2D relative_cell_index = get_cell_index_from(rel_pos_x, rel_pos_y);

  mark_cell(relative_cell_index, CellState::Visited);

  std::vector<std::size_t> frontier_cells_to_update;

  for (const Reading &r : readings) {
    const double hit_x_rel = rel_pos_x + r.distance * cos(r.angle);
    const double hit_y_rel = rel_pos_y + r.distance * sin(r.angle);

    const Index2D hit_cell_index = get_cell_index_from(hit_x_rel, hit_y_rel);

    const double steps_count = r.distance / CELL_SIZE;
    const double step_x = (hit_x_rel - rel_pos_x) / steps_count;
    const double step_y = (hit_y_rel - rel_pos_y) / steps_count;

    double curr_x = rel_pos_x;
    double curr_y = rel_pos_y;

    for (std::size_t i = 0; i < steps_count; ++i) {
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

void OccupationGrid::compute_frontier_regions(DynamicScheduler *sched) {
  std::vector<FrontierRegion> regions;
  std::unordered_set<Cell *> visited;

  for (std::size_t y = grid_min_y; y < grid_max_y; ++y) {
    for (std::size_t x = grid_min_x; x < grid_max_x; ++x) {
      auto cell = grid[y][x];

      if (visited.count(cell.get())) {
        continue;
      }

      if (cell->state != CellState::Frontier) {
        continue;
      }

      FrontierRegion new_region;
      Index2D idx = std::make_pair(y, x);
      new_region.min = idx;
      new_region.max = idx;

      std::queue<std::shared_ptr<Cell>> to_visit;
      to_visit.push(cell);
      visited.insert(cell.get());

      while (!to_visit.empty()) {
        auto current_cell = to_visit.front();
        to_visit.pop();

        new_region.cells.push_back(current_cell);

        if (current_cell->index < new_region.min) {
          new_region.min = current_cell->index;
        }

        if (current_cell->index > new_region.max) {
          new_region.max = current_cell->index;
        }

        auto neighbors = current_cell->get_neighbors();

        for (Index2D n_idx : neighbors) {
          auto neighbor = grid[n_idx.first][n_idx.second];

          if (neighbor->state == CellState::Frontier &&
              !visited.count(neighbor.get())) {
            to_visit.push(neighbor);
            visited.insert(neighbor.get());
          }
        }
      }

      regions.push_back(std::move(new_region));
    }
  }
}

void OccupationGrid::draw(const DrawData &draw_data) const {
  for (std::size_t y = grid_min_y; y < grid_max_y; ++y) {
    for (std::size_t x = grid_min_x; x < grid_max_x; ++x) {
      draw_cell({y, x}, draw_data);
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

  for (std::size_t y = 0; y < MAP_HEIGHT; ++y) {
    for (std::size_t x = 0; x < MAP_WIDTH; ++x) {
      f << static_cast<int>(grid[y][x].get()->state) << " ";
    }
    f << "\n";
  }

  f.close();
  std::cout << "GRID: Occupation grid saved to " << filename << std::endl;
}
