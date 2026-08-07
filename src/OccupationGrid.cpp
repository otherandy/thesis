#include "OccupationGrid.hpp"
#include "Cell.hpp"
#include "FrontierRegion.hpp"
#include "Grid.hpp"
#include "Utils.hpp"
#include <memory>
#include <queue>
#include <unordered_set>
#include <utility>

constexpr std::array<std::pair<int, int>, 4> directions{{
    {-1, 0}, // N
    {1, 0},  // S
    {0, -1}, // W
    {0, 1}   // E
}};

OccupationGrid::OccupationGrid(const Robot::Point &origin) : origin(origin) {
  for (std::size_t y = 0; y < MAP_HEIGHT; ++y) {
    for (std::size_t x = 0; x < MAP_WIDTH; ++x) {
      const double cell_center_x = (x + 0.5) * CELL_SIZE - ENV_WIDTH;
      const double cell_center_y = (y + 0.5) * CELL_SIZE - ENV_HEIGHT;

      grid[y][x] = std::make_unique<Cell>(
          std::make_pair(y, x), Robot::Point(cell_center_x, cell_center_y));
    }
  }
}

void OccupationGrid::mark_cell(Index2D index, CellState new_state,
                               bool force_change) {
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
    return;
  }

  if (force_change) {
    if (cell->state != CellState::Frontier &&
        new_state == CellState::Frontier) {
      frontier_cell_count++;
    } else if (cell->state == CellState::Frontier &&
               new_state != CellState::Frontier) {
      frontier_cell_count--;
    }

    cell->state = new_state;
    return;
  }

  // Don't overwrite Occupied or Visited states
  if (cell->state == CellState::Occupied || cell->state == CellState::Visited) {
    return;
  }

  // Don't mark known cells as Frontier
  if (cell->state == CellState::Free && new_state == CellState::Frontier) {
    return;
  }

  if (cell->state != CellState::Frontier && new_state == CellState::Frontier) {
    frontier_cell_count++;
  } else if (cell->state == CellState::Frontier &&
             new_state != CellState::Frontier) {
    frontier_cell_count--;
  }

  cell->state = new_state;
  return;
}

void OccupationGrid::mark_cells(
    const Robot::Point &relative_position,
    const std::array<Reading, LIDAR_SAMPLES> &readings) {
  const double rel_pos_x = relative_position.x();
  const double rel_pos_y = relative_position.y();

  const Index2D relative_cell_index = get_cell_index_from(rel_pos_x, rel_pos_y);
  mark_cell(relative_cell_index, CellState::Visited);

  std::vector<Index2D> demoted_frontier_cells;

  for (const Reading &r : readings) {
    const double distance = std::min(r.distance, LIDAR_RADIUS);
    const double hit_x_rel = rel_pos_x + distance * std::cos(r.angle);
    const double hit_y_rel = rel_pos_y + distance * std::sin(r.angle);
    const Index2D hit_cell_index = get_cell_index_from(hit_x_rel, hit_y_rel);

    mark_free_along_ray(rel_pos_x, rel_pos_y, hit_x_rel, hit_y_rel,
                        hit_cell_index, demoted_frontier_cells);

    mark_cell(hit_cell_index, r.distance < LIDAR_RADIUS ? CellState::Occupied
                                                        : CellState::Frontier);
  }

  for (const Index2D &idx : demoted_frontier_cells) {
    if (has_unknown_neighbor(idx)) {
      mark_cell(idx, CellState::Frontier, true);
    }
  }
}

void OccupationGrid::mark_free_along_ray(
    double start_x, double start_y, double end_x, double end_y,
    const Index2D &end_cell_index,
    std::vector<Index2D> &demoted_frontier_cells) {
  const double dx = end_x - start_x;
  const double dy = end_y - start_y;
  const double ray_length = std::hypot(dx, dy);

  if (ray_length < std::numeric_limits<double>::epsilon()) {
    return;
  }

  const auto steps_count = static_cast<std::size_t>(ray_length / CELL_SIZE) + 1;
  const double step_x = dx / static_cast<double>(steps_count);
  const double step_y = dy / static_cast<double>(steps_count);

  double curr_x = start_x;
  double curr_y = start_y;
  for (std::size_t i = 0; i < steps_count; ++i) {
    const Index2D curr_cell_index = get_cell_index_from(curr_x, curr_y);

    if (curr_cell_index == end_cell_index) {
      break;
    }

    const auto c = grid[curr_cell_index.first][curr_cell_index.second].get();

    if (c->state == CellState::Frontier) {
      demoted_frontier_cells.push_back(curr_cell_index);
    }

    mark_cell(curr_cell_index, CellState::Free);
    curr_x += step_x;
    curr_y += step_y;
  }
}

bool OccupationGrid::has_unknown_neighbor(const Index2D &idx) {
  for (auto [dy, dx] : directions) {
    int ny = idx.first + dy;
    int nx = idx.second + dx;

    Cell *neighbor = grid[ny][nx].get();

    if (neighbor->state == CellState::Unknown) {
      return true;
    }
  }

  return false;
}

void OccupationGrid::remove_dead_frontier_cells() {
  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      Cell *c = grid[y][x].get();

      if (c->state != CellState::Frontier) {
        continue;
      }

      if (c->frontier_id != std::nullopt) {
        continue;
      }

      int frontier_neighbors = 0;
      for (auto n : c->get_neighbors(&grid)) {
        if (n->state == CellState::Frontier) {
          frontier_neighbors++;
        }
      }

      if (frontier_neighbors <= 1) {
        c->state = CellState::Free;
        frontier_cell_count--;
      }
    }
  }
}

void OccupationGrid::compute_frontier_regions(DynamicScheduler *sched) {
  std::vector<std::shared_ptr<FrontierRegion>> regions;
  std::unordered_set<Cell *> global_visited;

  remove_dead_frontier_cells();

  auto is_closed = [&](std::vector<Cell *> region, Index2D min,
                       Index2D max) -> bool {
    auto key = [](int y, int x) {
      return (static_cast<uint64_t>(y) << 32) | static_cast<uint32_t>(x);
    };

    std::unordered_set<Cell *> region_set(region.begin(), region.end());

    const int min_y = min.first - 1;
    const int min_x = min.second - 1;
    const int max_y = max.first + 1;
    const int max_x = max.second + 1;

    std::queue<Index2D> q;
    std::unordered_set<uint64_t> visited;

    if (grid[min_y][min_x].get()->state == CellState::Free) {
      q.push({min_y, min_x});
      visited.insert(key(min_y, min_x));
    }

    if (grid[min_y][max_x].get()->state == CellState::Free) {
      q.push({min_y, max_x});
      visited.insert(key(min_y, max_x));
    }

    if (grid[max_y][min_x].get()->state == CellState::Free) {
      q.push({max_y, min_x});
      visited.insert(key(max_y, min_x));
    }

    if (grid[max_y][max_x].get()->state == CellState::Free) {
      q.push({max_y, max_x});
      visited.insert(key(max_y, max_x));
    }

    while (!q.empty()) {
      auto [y, x] = q.front();
      q.pop();

      Cell *c = grid[y][x].get();

      if (c->state == CellState::Unknown) {
        return false;
      }

      c->debug_color = raylib::PINK;

      for (auto [dy, dx] : directions) {
        int ny = y + dy;
        int nx = x + dx;

        if (ny < min_y || ny > max_y || nx < min_x || nx > max_x) {
          continue;
        }

        Cell *neighbor = grid[ny][nx].get();

        if (region_set.count(neighbor) ||
            neighbor->state == CellState::Occupied ||
            neighbor->state == CellState::Frontier) {
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

        current_cell->debug_color = raylib::GREEN;

        region_cells.push_back(current_cell);

        if (current_cell->index.first < min.first) {
          min.first = current_cell->index.first;
        }

        if (current_cell->index.second < min.second) {
          min.second = current_cell->index.second;
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

  const auto parents = sched->get_all_vertices();

  for (auto child : regions) {
    vertex_t id = sched->add_vertex(child);

    for (auto idx : child->cells) {
      Cell *cell = grid[idx.first][idx.second].get();
      cell->frontier_id = id;
    }

    vertex_t parent_id = 0;
    FrontierRegion *best_parent =
        sched->get_vertex_data(parent_id).region.get();

    for (auto v : parents) {
      if (v == id) {
        continue;
      }

      auto vd = sched->get_vertex_data(v);
      auto candidate = vd.region;

      if (!contains(*candidate, *child)) {
        continue;
      }

      if (parent_id == 0 ||
          candidate->cells.size() < best_parent->cells.size()) {
        best_parent = candidate.get();
        parent_id = v;
      }
    }

    sched->add_edge(parent_id, id);
  }
}

void OccupationGrid::remove_dead_free_cells() {
  std::unordered_set<Cell *> global_visited;

  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      Cell *cell = grid[y][x].get();

      if (cell->state != CellState::Free && cell->state != CellState::Visited) {
        continue;
      }

      if (global_visited.count(cell)) {
        continue;
      }

      global_visited.insert(cell);

      std::vector<Cell *> region_cells;
      bool should_remove = true;

      std::queue<Cell *> to_visit;
      to_visit.push(cell);

      while (!to_visit.empty()) {
        auto current_cell = to_visit.front();
        to_visit.pop();

        region_cells.push_back(current_cell);

        auto neighbors = current_cell->get_neighbors(&grid);

        for (Cell *neighbor : neighbors) {
          if (neighbor->state != CellState::Free &&
              neighbor->state != CellState::Visited) {
            continue;
          }

          if (neighbor->state == CellState::Visited) {
            should_remove = false;
          }

          if (global_visited.count(neighbor)) {
            continue;
          }

          if (global_visited.insert(neighbor).second) {
            to_visit.push(neighbor);
          }
        }
      }

      if (should_remove) {
        for (auto c : region_cells) {
          c->state = CellState::Unknown;
        }
      }
    }
  }
}

void OccupationGrid::compute_physical_obstacles(DynamicScheduler *sched) {
  std::vector<std::shared_ptr<FrontierRegion>> regions;
  std::unordered_set<Cell *> global_visited;

  remove_dead_free_cells();

  auto is_closed = [&](std::vector<Cell *> region, Index2D min,
                       Index2D max) -> bool {
    auto key = [](int y, int x) {
      return (static_cast<uint64_t>(y) << 32) | static_cast<uint32_t>(x);
    };

    std::unordered_set<Cell *> region_set(region.begin(), region.end());

    const int min_y = min.first - 1;
    const int min_x = min.second - 1;
    const int max_y = max.first + 1;
    const int max_x = max.second + 1;

    std::queue<Index2D> q;
    std::unordered_set<uint64_t> visited;

    q.push({min_y, min_x});
    visited.insert(key(min_y, min_x));

    CellState first_state = grid[min_y][min_x].get()->state;

    auto is_not_first_state = [first_state](CellState state) {
      if (first_state != CellState::Unknown) {
        return state != CellState::Free && state != CellState::Visited;
      }

      return state != CellState::Unknown;
    };

    while (!q.empty()) {
      auto [y, x] = q.front();
      q.pop();

      Cell *c = grid[y][x].get();

      if (c->state != CellState::Occupied && is_not_first_state(c->state)) {
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

        auto k = key(ny, nx);

        if (visited.insert(k).second) {
          q.push({ny, nx});
        }
      }
    }

    if (!found_exterior) {
      found_exterior = first_state == CellState::Unknown;
    }

    return true;
  };

  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      Cell *cell = grid[y][x].get();

      if (cell->state != CellState::Occupied) {
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

        if (current_cell->index.first < min.first) {
          min.first = current_cell->index.first;
        }

        if (current_cell->index.second < min.second) {
          min.second = current_cell->index.second;
        }

        if (current_cell->index.first > max.first) {
          max.first = current_cell->index.first;
        }

        if (current_cell->index.second > max.second) {
          max.second = current_cell->index.second;
        }

        auto neighbors = current_cell->get_neighbors(&grid);

        for (Cell *neighbor : neighbors) {
          if (neighbor->state != CellState::Occupied) {
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

  const auto parents = sched->get_all_vertices();

  for (auto child : regions) {
    vertex_t id = sched->add_vertex(child);

    for (auto idx : child->cells) {
      Cell *cell = grid[idx.first][idx.second].get();
      cell->frontier_id = id;
    }
  }
}

void OccupationGrid::draw_cell(Index2D index, const DrawData &draw_data) const {
  const Cell *cell = grid[index.first][index.second].get();

  if (cell->state == CellState::Unknown) {
    return;
  }

  raylib::Color color;

  switch (cell->state) {
  case CellState::Free:
    color = raylib::YELLOW;
    break;
  case CellState::Occupied:
    color = cell->frontier_id.has_value() ? raylib::BLACK : raylib::PURPLE;
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

  if (debug && cell->debug_color.has_value()) {
    color = *cell->debug_color;
  }

  const float screen_x =
      (cell->center.x() + origin.x()) * draw_data.scale_factor +
      draw_data.offset_x;
  const float screen_y =
      (cell->center.y() + origin.y()) * draw_data.scale_factor +
      draw_data.offset_y;

  DrawRectangle(screen_x, screen_y, 2, 2, color);
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
    std::cerr << "ERROR: Failed to open " << filename << " for writing"
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
  std::cout << "INFO: Occupation grid saved to " << filename << std::endl;
}
