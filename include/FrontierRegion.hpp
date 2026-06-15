#ifndef FRONTIER_REGION_HPP
#define FRONTIER_REGION_HPP

#include "Cell.hpp"
#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

const std::array<Color, 9> FrontierColors = {
    BLUE, LIME, VIOLET, DARKBLUE, DARKGREEN, DARKPURPLE, SKYBLUE, GREEN, PURPLE,
};

struct FrontierRegion {
  std::vector<std::shared_ptr<Cell>> cells;

  const bool explored() const;

  std::optional<Robot::Point>
  get_closest_unexplored(const Robot::Point &pos) const;
};

#endif
