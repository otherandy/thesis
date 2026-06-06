#include "FrontierRegion.hpp"
#include <algorithm>
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
