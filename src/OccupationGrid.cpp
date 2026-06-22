#include "OccupationGrid.hpp"
#include "Cell.hpp"
#include "FrontierRegion.hpp"
#include "Grid.hpp"
#include "Utils.hpp"
#include <memory>
#include <queue>
#include <unordered_set>
#include <utility>

bool OccupationGrid::mark_cell(Index2D index, CellState new_state) {
  if (index.first < grid_min.first) {
    grid_min.first = index.first;
  }
  if (index.first > grid_max.first) {
    grid_max.first = index.first + 1;
  }
  if (index.second < grid_min.second) {
    grid_min.second = index.second;
  }
  if (index.second > grid_max.second) {
    grid_max.second = index.second + 1;
  }

  Cell *cell = grid[index.first][index.second].get();

  // Always overwrite cells to Visited
  if (new_state == CellState::Visited) {
    if (cell->state == CellState::Frontier) {
      frontier_cell_count--;
    }

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

  if (cell->state != CellState::Frontier && new_state == CellState::Frontier) {
    frontier_cell_count++;
  } else if (cell->state == CellState::Frontier && new_state != CellState::Frontier) {
    frontier_cell_count--;
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

  static constexpr double steps_count = LIDAR_RADIUS / CELL_SIZE;

  const double rel_pos_x = relative_position.x();
  const double rel_pos_y = relative_position.y();

  const Index2D relative_cell_index = get_cell_index_from(rel_pos_x, rel_pos_y);

  mark_cell(relative_cell_index, CellState::Visited);

  for (const Reading &r : readings) {
    const double hit_x_rel = rel_pos_x + r.distance * cos(r.angle);
    const double hit_y_rel = rel_pos_y + r.distance * sin(r.angle);

    const Index2D hit_cell_index = get_cell_index_from(hit_x_rel, hit_y_rel);

    const double step_x = (hit_x_rel - rel_pos_x) / steps_count;
    const double step_y = (hit_y_rel - rel_pos_y) / steps_count;

    double curr_x = rel_pos_x;
    double curr_y = rel_pos_y;

    for (std::size_t i = 0; i < steps_count; ++i) {
      const Index2D curr_cell_index = get_cell_index_from(curr_x, curr_y);

      if (curr_cell_index == hit_cell_index) {
        break;
      }

      mark_cell(curr_cell_index, CellState::Free);
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
  std::unordered_set<Cell *> global_visited;

  auto is_closed = [&](std::vector<Cell *> region, Index2D min,
                       Index2D max) -> bool {
    auto key = [](int y, int x) {
      return (static_cast<uint64_t>(y) << 32) | static_cast<uint32_t>(x);
    };

    std::unordered_set<Cell *> region_set(region.begin(), region.end());

    int min_y = min.first - 1;
    int min_x = min.second - 1;
    int max_y = max.first + 1;
    int max_x = max.second + 1;

    std::queue<Index2D> q;
    std::unordered_set<uint64_t> visited;

    q.push({max_y, max_x});
    visited.insert(key(max_y, max_x));

    q.push({min_y, min_x});
    visited.insert(key(min_y, min_x));

    static constexpr std::array<std::pair<int, int>, 4> directions{{
        {-1, 0}, // N
        {1, 0},  // S
        {0, -1}, // W
        {0, 1}   // E
    }};

    while (!q.empty()) {
      auto [y, x] = q.front();
      q.pop();

      Cell *c = grid[y][x].get();

      if (c->state == CellState::Unknown) {
        return false;
      }

      for (auto [dy, dx] : directions) {
        int ny = y + dy;
        int nx = x + dx;

        if (ny < min_y || ny > max_y || nx < min_x || nx > max_x) {
          continue;
        }

        Cell *neighbor = grid[ny][nx].get();

        if (region_set.count(neighbor)) {
          continue;
        }

        if (neighbor->state == CellState::Occupied) {
          continue;
        }

        auto k = key(ny, nx);

        if (visited.insert(k).second) {
          q.push({ny, nx});
        }
      }
    }

    return true;
  };

  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      Cell *cell = grid[y][x].get();

      if (cell->state != CellState::Frontier) {
        continue;
      }

      if (cell->frontier_id.has_value()) {
        continue;
      }

      if (global_visited.count(cell)) {
        continue;
      }

      global_visited.insert(cell);

      std::vector<Cell *> region_cells;

      Index2D min = std::make_pair(y, x);
      Index2D max = std::make_pair(y, x);

      std::queue<Cell *> to_visit;
      to_visit.push(cell);

      while (!to_visit.empty()) {
        auto current_cell = to_visit.front();
        to_visit.pop();

        region_cells.push_back(current_cell);

        if (current_cell->index < min) {
          min = current_cell->index;
        }

        if (current_cell->index.first > max.first) {
          max.first = current_cell->index.first;
        }

        if (current_cell->index.second > max.second) {
          max.second = current_cell->index.second;
        }

        auto neighbors = current_cell->get_neighbors(&grid);

        for (Cell *neighbor : neighbors) {
          if (neighbor->state != CellState::Frontier) {
            continue;
          }

          if (global_visited.count(neighbor)) {
            continue;
          }

          if (global_visited.insert(neighbor).second &&
              !neighbor->frontier_id.has_value()) {
            to_visit.push(neighbor);
          }
        }
      }

      if (is_closed(region_cells, min, max)) {
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
  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
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
      f << static_cast<int>(grid[y][x]->state) << ",";
    }
    f << "\n";
  }

  f.close();
  std::cout << "GRID: Occupation grid saved to " << filename << std::endl;
}
