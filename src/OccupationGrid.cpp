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
    color = cell->frontier_id.has_value() ? raylib::VIOLET : raylib::BLUE;
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

      grid[y][x] = std::make_unique<Cell>(
          std::make_pair(y, x), Robot::Point(cell_center_x, cell_center_y));
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
  std::vector<std::shared_ptr<FrontierRegion>> regions;
  std::unordered_set<Cell *> visited;

  for (std::size_t y = grid_min_y; y < grid_max_y; ++y) {
    for (std::size_t x = grid_min_x; x < grid_max_x; ++x) {
      Cell *cell = grid[y][x].get();

      if (visited.count(cell)) {
        continue;
      }

      if (cell->state != CellState::Frontier) {
        continue;
      }

      if (cell->frontier_id.has_value()) {
        continue;
      }

      std::vector<Cell *> region_cells;
      bool is_loop = true;

      Index2D min = std::make_pair(y, x);
      Index2D max = std::make_pair(y, x);

      std::queue<Cell *> to_visit;
      to_visit.push(cell);
      visited.insert(cell);

      while (!to_visit.empty()) {
        auto current_cell = to_visit.front();
        to_visit.pop();

        region_cells.push_back(current_cell);

        if (current_cell->index < min) {
          min = current_cell->index;
        }

        if (current_cell->index > max) {
          max = current_cell->index;
        }

        auto neighbors = current_cell->get_neighbors();
        int frontier_neighbors = 0;

        for (Index2D idx : neighbors) {
          Cell *neighbor = grid[idx.first][idx.second].get();

          if (neighbor->state != CellState::Frontier) {
            continue;
          }

          if (!neighbor->frontier_id.has_value()) {
            frontier_neighbors++;
          }

          if (visited.count(neighbor)) {
            continue;
          }

          visited.insert(neighbor);

          if (!neighbor->frontier_id.has_value()) {
            to_visit.push(neighbor);
          }
        }

        if (frontier_neighbors < 2) {
          is_loop = false;
        }
      }

      if (is_loop) {
        auto new_region = std::make_shared<FrontierRegion>();

        for (auto c : region_cells) {
          new_region->cells.push_back(c->index);
        }

        new_region->min = min;
        new_region->max = max;
        regions.push_back(std::move(new_region));
      }
    }
  }

  auto contains = [](const FrontierRegion &outer,
                     const FrontierRegion &inner) -> bool {
    return outer.min.first <= inner.min.first &&
           outer.min.second <= inner.min.second &&
           outer.max.first >= inner.max.first &&
           outer.max.second >= inner.max.second;
  };

  for (auto child : regions) {
    vertex_t id = sched->add_vertex(child);

    for (auto idx : child->cells) {
      Cell *cell = grid[idx.first][idx.second].get();
      cell->frontier_id = id;
    }

    vertex_t parent_id = 0;
    FrontierRegion *best_parent = nullptr;

    for (auto v : sched->get_all_vertices()) {
      if (v == id) {
        continue;
      }

      auto vd = sched->get_vertex_data(v);
      auto candidate = vd.region;

      if (!contains(*candidate, *child)) {
        continue;
      }

      if (best_parent == nullptr) {
        best_parent = candidate.get();
        parent_id = v;
        continue;
      }

      if (candidate->cells.size() < best_parent->cells.size()) {
        best_parent = candidate.get();
        parent_id = v;
      }
    }

    sched->add_edge(parent_id, id);
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
      f << static_cast<int>(grid[y][x]->state) << " ";
    }
    f << "\n";
  }

  f.close();
  std::cout << "GRID: Occupation grid saved to " << filename << std::endl;
}
