#include "FrontierRegion.hpp"

const bool FrontierRegion::explored() const {
  for (const Cell *cell : cells) {
    if (cell->state == CellState::Frontier) {
      return false;
    }
  }
  return true;
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
