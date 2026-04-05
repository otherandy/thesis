#ifndef FRONTIER_REGION_HPP
#define FRONTIER_REGION_HPP

#include "Cell.hpp"
#include "Graph.hpp"
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
    std::size_t id;
    std::vector<Cell *> cells;
    bool explored = false;

    Point get_closest_point(const Point &pos) const;
    std::optional<Point> get_closest_unexplored(const Point &pos) const;
};

const FrontierRegion *get_frontier_region_by_id(
    const std::vector<FrontierRegion> &regions,
    std::size_t id);

void compute_frontier_regions(
    std::vector<FrontierRegion> *frontier_regions,
    Grid2D<Cell> &grid,
    std::shared_ptr<StepTraversal> traversal_graph,
    std::size_t current_parent_region_id);

std::size_t get_nearest_frontier_region_id(
    const std::vector<FrontierRegion> &regions,
    const Point &position);

#endif