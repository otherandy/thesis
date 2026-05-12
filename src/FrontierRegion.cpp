#include "FrontierRegion.hpp"
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <vector>

FrontierRegion *get_frontier_region_by_id(std::vector<FrontierRegion> &regions,
                                          std::size_t id) {
  auto it =
      std::find_if(regions.begin(), regions.end(),
                   [id](FrontierRegion &region) { return region.id == id; });

  if (it != regions.end()) {
    return &(*it);
  } else {
    return nullptr;
  }
}

Point FrontierRegion::get_closest_point(const Point &pos) const {
  Point closest_point;
  double closest_distance = std::numeric_limits<double>::max();

  for (const Cell *cell : cells) {
    const double distance = CGAL::squared_distance(pos, cell->center);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_point = cell->center;
    }
  }

  return closest_point;
}

std::optional<Point>
FrontierRegion::get_closest_unexplored(const Point &pos) const {
  Point closest_point;
  double closest_distance = std::numeric_limits<double>::max();
  bool found_unexplored = false;

  for (const Cell *cell : cells) {
    if (cell->state != CellState::Frontier) {
      continue;
    }

    const double distance = CGAL::squared_distance(pos, cell->center);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_point = cell->center;
      found_unexplored = true;
    }
  }

  if (found_unexplored) {
    return closest_point;
  } else {
    return std::nullopt;
  }
}

void compute_frontier_regions(std::vector<FrontierRegion> &frontier_regions,
                              Grid2D<Cell> &grid,
                              std::shared_ptr<StepTraversal> traversal_graph,
                              std::size_t current_parent_region_id) {
  for (FrontierRegion &region : frontier_regions) {
    if (region.explored) {
      continue;
    }

    auto is_still_frontier = [](Cell *cell) {
      if (cell->state != CellState::Frontier) {
        cell->frontier_id = std::nullopt;
        return true;
      }
      return false;
    };

    region.cells.erase(std::remove_if(region.cells.begin(), region.cells.end(),
                                      is_still_frontier),
                       region.cells.end());

    if (region.id == current_parent_region_id) {
      for (Cell *cell : region.cells) {
        cell->frontier_id = std::nullopt;
      }
      region.cells.clear();
    }

    if (region.cells.empty()) {
      region.explored = true;
    }
  }

  std::unordered_set<Cell *> visited;

  for (int y = 0; y < MAP_HEIGHT; ++y) {
    for (int x = 0; x < MAP_WIDTH; ++x) {
      Cell &cell = grid[y][x];

      if (visited.count(&cell)) {
        continue;
      }

      if (cell.state != CellState::Frontier) {
        continue;
      }

      if (cell.frontier_id.has_value()) {
        continue;
      }

      std::vector<Cell *> region_cells;
      std::optional<std::size_t> found_id;

      std::queue<Cell *> to_visit;
      to_visit.push(&cell);
      visited.insert(&cell);

      while (!to_visit.empty()) {
        Cell *current_cell = to_visit.front();
        to_visit.pop();

        region_cells.push_back(current_cell);

        auto neighbors = current_cell->get_neighbors();

        for (Index2D neighbor_cell : neighbors) {
          Cell &neighbor = grid[neighbor_cell.first][neighbor_cell.second];

          if (neighbor.state == CellState::Frontier &&
              !visited.count(&neighbor)) {
            if (!neighbor.frontier_id.has_value()) {
              to_visit.push(&neighbor);
              visited.insert(&neighbor);
            } else {
              found_id = neighbor.frontier_id.value();
            }
          }
        }
      }

      if (found_id.has_value()) {
        auto region =
            get_frontier_region_by_id(frontier_regions, found_id.value());
        for (Cell *region_cell : region_cells) {
          region_cell->frontier_id = found_id;
          region->cells.push_back(region_cell);
        }
      } else {
        FrontierRegion new_region;
        new_region.cells = std::move(region_cells);
        new_region.id =
            traversal_graph->add_vertex_and_edge(current_parent_region_id);
        for (Cell *region_cell : new_region.cells) {
          region_cell->frontier_id = new_region.id;
        }
        frontier_regions.push_back(new_region);
      }
    }
  }

  traversal_graph->post_update();
}

std::size_t
get_nearest_frontier_region_id(const std::vector<FrontierRegion> &regions,
                               const Point &position) {
  std::size_t nearest_region_id = 0;
  double nearest_distance = std::numeric_limits<double>::max();

  for (const FrontierRegion &region : regions) {
    if (region.explored) {
      continue;
    }

    const double distance =
        CGAL::squared_distance(position, region.get_closest_point(position));

    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_region_id = region.id;
    }
  }

  return nearest_region_id;
}
