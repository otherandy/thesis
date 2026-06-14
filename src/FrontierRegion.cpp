#include "FrontierRegion.hpp"

const bool FrontierRegion::explored() const {
  for (const auto cell : cells) {
    if (cell->state == CellState::Frontier) {
      return false;
    }
  }
  return true;
}

std::optional<Robot::Point>
FrontierRegion::get_closest_unexplored(const Robot::Point &pos) const {
  std::optional<Robot::Point> closest_point = std::nullopt;
  double closest_distance = std::numeric_limits<double>::max();

  for (const auto cell : cells) {
    if (cell->state != CellState::Frontier) {
      continue;
    }

    const double distance = CGAL::squared_distance(pos, cell->center);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_point = cell->center;
    }
  }

  return closest_point;
}
