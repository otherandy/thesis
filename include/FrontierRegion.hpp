#ifndef FRONTIER_REGION_HPP
#define FRONTIER_REGION_HPP

#include "Cell.hpp"
#include "Grid.hpp"
#include "cgal_types.hpp"
#include <memory>
#include <raylib-cpp.hpp>

const std::array<Color, 9> FrontierColors = {
    BLUE, LIME, VIOLET, DARKBLUE, DARKGREEN, DARKPURPLE, SKYBLUE, GREEN, PURPLE,
};

struct FrontierRegion {
  std::vector<Index2D> cells;
  Index2D min;
  Index2D max;

  std::optional<Robot::Point>
  get_closest_unexplored(const Grid2D<std::unique_ptr<Cell>> &grid,
                         const Robot::Point &pos) const;

  bool is_done(const Grid2D<std::unique_ptr<Cell>> &grid);
};

#endif
