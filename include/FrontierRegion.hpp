#ifndef FRONTIER_REGION_HPP
#define FRONTIER_REGION_HPP

#include "Cell.hpp"
#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

const std::array<Color, 9> FrontierColors = {
    BLUE,
    LIME,
    VIOLET,
    DARKBLUE,
    DARKGREEN,
    DARKPURPLE,
    SKYBLUE,
    GREEN,
    PURPLE,
};

struct FrontierRegion
{
  int id;
  std::vector<Cell> cells;
  bool explored = false;

  std::vector<Point> get_points() const;
  Polygon to_polygon() const;
  Point get_closest_from(const Point &pos) const;
  std::vector<Point> calculate_path_from(const Point &start) const;
};

#endif