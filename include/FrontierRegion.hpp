#ifndef FRONTIER_REGION_HPP
#define FRONTIER_REGION_HPP

#include "Cell.hpp"
#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

const std::array<Color, 9> FrontierColors = {
    BLUE, LIME, VIOLET, DARKBLUE, DARKGREEN, DARKPURPLE, SKYBLUE, GREEN, PURPLE,
};

struct FrontierRegion {
  std::optional<std::size_t> id = std::nullopt;
  std::optional<std::size_t> parent_id = std::nullopt;
  std::vector<Cell *> cells;

  const bool explored() const;
  Robot::Point get_closest_point(const Robot::Point &pos) const;
  std::optional<Robot::Point>
  get_closest_unexplored(const Robot::Point &pos) const;
};

#endif
