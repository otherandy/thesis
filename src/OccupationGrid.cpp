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

  // Don't overwrite Occupied cells
  if (cell->state == CellState::Occupied) {
    return;
  }

  if (new_state == CellState::Occupied &&
      !has_neighbor_state(cell->index, CellState::Free)) {
    return;
  }

  cell->times_viewed++;

  // Don't overrite Visited cells with states other than Occupied
  if (cell->state == CellState::Visited && new_state != CellState::Occupied) {
    return;
  }

  // Don't mark known cells as Frontier except when forced
  if (cell->state == CellState::Free && new_state == CellState::Frontier &&
      !force_change) {
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
    const std::array<Reading, LIDAR_SAMPLES> &readings, double radius) {
  const double rel_pos_x = relative_position.x();
  const double rel_pos_y = relative_position.y();

  const Index2D relative_cell_index = get_cell_index_from(rel_pos_x, rel_pos_y);
  mark_cell(relative_cell_index, CellState::Visited);

  std::vector<Index2D> possible_frontier_cells;

  for (const Reading &r : readings) {
    const double hit_x_rel = rel_pos_x + r.distance * std::cos(r.angle);
    const double hit_y_rel = rel_pos_y + r.distance * std::sin(r.angle);
    const Index2D hit_cell_index = get_cell_index_from(hit_x_rel, hit_y_rel);

    mark_free_along_ray(rel_pos_x, rel_pos_y, hit_x_rel, hit_y_rel,
                        hit_cell_index);

    if (r.distance < radius) {
      mark_cell(hit_cell_index, CellState::Occupied);
    } else {
      possible_frontier_cells.push_back(hit_cell_index);
    }
  }

  for (const Index2D &idx : possible_frontier_cells) {
    if (has_neighbor_state(idx, CellState::Unknown, false)) {
      mark_cell(idx, CellState::Frontier, true);
    }
  }
}

void OccupationGrid::mark_free_along_ray(double start_x, double start_y,
                                         double end_x, double end_y,
                                         const Index2D &end_cell_index) {
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

    mark_cell(curr_cell_index, CellState::Free);
    curr_x += step_x;
    curr_y += step_y;
  }
}

bool OccupationGrid::has_neighbor_state(const Index2D &idx, CellState state,
                                        bool include_corners) {
  Cell *start_cell = grid[idx.first][idx.second].get();

  if (!include_corners) {
    for (const auto &[dy, dx] : directions) {
      int ny = idx.first + dy;
      int nx = idx.second + dx;

      const auto neighbor = grid[ny][nx].get();

      if (neighbor->state == state) {
        return true;
      }
    }

    return false;
  }

  auto neighbors = start_cell->get_neighbors(&grid);

  for (const auto n : neighbors) {
    if (n->state == state) {
      return true;
    }
  }

  return false;
}

Cell *OccupationGrid::find_reference(const Index2D &idx, CellState state) {
  Cell *start_cell = grid[idx.first][idx.second].get();
  auto neighbors = start_cell->get_neighbors(&grid);

  for (const auto n : neighbors) {
    if (n->state == state) {
      return n;
    }
  }

  return nullptr;
}

void OccupationGrid::clean_cells() {
  remove_dead_free_cells();
  remove_dead_frontier_cells();
}

void OccupationGrid::remove_dead_frontier_cells() {
  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      Cell *c = grid[y][x].get();

      if (c->state != CellState::Frontier) {
        continue;
      }

      if (!has_neighbor_state(Index2D{y, x}, CellState::Unknown)) {
        c->state = CellState::Free;
        frontier_cell_count--;
      }
    }
  }
}

void OccupationGrid::compute_frontier_regions(DynamicScheduler *sched) {
  std::vector<std::shared_ptr<FrontierRegion>> regions;
  std::unordered_set<Cell *> global_visited;

  auto is_closed = [&](std::vector<Cell *> region, Index2D min,
                       Index2D max) -> bool {
    for (Cell *cell : region) {
      auto neighbors = cell->get_neighbors(&grid);

      for (const auto n : neighbors) {
        if (n->state == CellState::Frontier && n->frontier_id.has_value()) {
          return false;
        }

        if (n->state == CellState::Occupied) {
          return false;
        }
      }
    }

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

    const Cell *minyminx = grid[min_y][min_x].get();
    if (minyminx->state == CellState::Free) {
      q.push({min_y, min_x});
      visited.insert(key(min_y, min_x));
    }

    const Cell *minymaxx = grid[min_y][max_x].get();
    if (minymaxx->state == CellState::Free) {
      q.push({min_y, max_x});
      visited.insert(key(min_y, max_x));
    }

    const Cell *maxyminx = grid[max_y][min_x].get();
    if (maxyminx->state == CellState::Free) {
      q.push({max_y, min_x});
      visited.insert(key(max_y, min_x));
    }

    const Cell *maxymaxx = grid[max_y][max_x].get();
    if (maxymaxx->state == CellState::Free) {
      q.push({max_y, max_x});
      visited.insert(key(max_y, max_x));
    }

    std::size_t unknown_found = 0;

    while (!q.empty()) {
      auto [y, x] = q.front();
      q.pop();

      Cell *c = grid[y][x].get();

      if (c->state == CellState::Unknown) {
        unknown_found++;

        if (unknown_found > 1) {
          return false;
        }
      }

      c->debug_color = raylib::PINK;

      for (auto [dy, dx] : directions) {
        int ny = y + dy;
        int nx = x + dx;

        if (ny < min_y || ny > max_y || nx < min_x || nx > max_x) {
          continue;
        }

        Cell *neighbor = grid[ny][nx].get();

        if (neighbor->state == CellState::Frontier ||
            neighbor->state == CellState::Occupied ||
            neighbor->state == CellState::Visited) {
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
    vertex_t id = sched->add_vertex(child, *get_data());

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

  auto is_closed = [&](std::vector<Cell *> region, Index2D min,
                       Index2D max) -> bool {
    bool result = true;

    for (Cell *cell : region) {
      auto neighbors = cell->get_neighbors(&grid);

      for (const auto n : neighbors) {
        if (n->state == CellState::Frontier) {
          result = false;
        }
      }
    }

    if (found_exterior) {
      return result;
    }

    auto key = [](int y, int x) {
      return (static_cast<uint64_t>(y) << 32) | static_cast<uint32_t>(x);
    };

    const int min_y = min.first - 1;
    const int min_x = min.second - 1;
    const int max_y = max.first + 1;
    const int max_x = max.second + 1;

    std::queue<Index2D> q;
    std::unordered_set<uint64_t> visited;

    q.push({min_y, min_x});
    visited.insert(key(min_y, min_x));

    while (!q.empty()) {
      auto [y, x] = q.front();
      q.pop();

      Cell *c = grid[y][x].get();

      if (c->state == CellState::Free) {
        return result;
      }

      for (auto [dy, dx] : directions) {
        int ny = y + dy;
        int nx = x + dx;

        if (ny < min_y || ny > max_y || nx < min_x || nx > max_x) {
          continue;
        }

        Cell *neighbor = grid[ny][nx].get();

        if (neighbor->state == CellState::Occupied) {
          continue;
        }

        auto k = key(ny, nx);

        if (visited.insert(k).second) {
          q.push({ny, nx});
        }
      }
    }

    found_exterior = true;

    return result;
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
        new_region->physical = true;
        regions.push_back(std::move(new_region));
      }
    }
  }

  for (auto child : regions) {
    vertex_t id = sched->add_vertex(child, grid);

    for (auto idx : child->cells) {
      Cell *cell = grid[idx.first][idx.second].get();
      cell->frontier_id = id;
    }
  }
}

Cell *OccupationGrid::unmarked_obstacles_exist() {
  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      Cell *cell = grid[y][x].get();
      if (cell->state == CellState::Occupied &&
          cell->frontier_id == std::nullopt) {
        return cell;
      }
    }
  }
  return nullptr;
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

void OccupationGrid::draw_info(const DrawData &draw_data) const {
  const std::string info = "FE: " + std::to_string(found_exterior);
  raylib::DrawText(info.c_str(), 10, 10, 20, raylib::BLACK);
}

void OccupationGrid::save_to_file() const {
  const std::string filename = append_timestamp("data/", "_grid.csv");
  ensure_parent_dir_exists(filename);
  std::ofstream f(filename);

  if (!f.is_open()) {
    std::cerr << "ERROR: Failed to open " << filename << " for writing"
              << std::endl;
    return;
  }

  for (std::size_t y = grid_min.first; y <= grid_max.first; ++y) {
    for (std::size_t x = grid_min.second; x <= grid_max.second; ++x) {
      f << static_cast<int>(grid[y][x]->times_viewed) << ",";
    }
    f << "\n";
  }

  f.close();
  std::cout << "INFO: Occupation grid saved to " << filename << std::endl;
}
