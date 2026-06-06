#ifndef FRONTIER_REGION_HPP
#define FRONTIER_REGION_HPP

#include "Cell.hpp"
#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

const std::array<Color, 9> FrontierColors = {
    BLUE, LIME, VIOLET, DARKBLUE, DARKGREEN, DARKPURPLE, SKYBLUE, GREEN, PURPLE,
};

struct FrontierRegion {
  std::size_t id;
  std::vector<Cell *> cells;
  bool explored = false;

  Point get_closest_point(const Point &pos) const;
  std::optional<Point> get_closest_unexplored(const Point &pos) const;
};

#endif
