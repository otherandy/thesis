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
    std::size_t id;
    std::vector<Cell> cells;
    bool explored = false;

    std::vector<std::size_t> inner_region_ids;

    std::vector<Point> get_points() const;
    Polygon to_polygon() const;
    Point get_closest_from(const Point &pos) const;
    std::vector<Point> calculate_path_from(const Point &start) const;
};

void compute_frontier_regions(
    std::vector<FrontierRegion> *frontier_regions,
    Grid2D<Cell> &grid);

std::size_t get_nearest_frontier_region_id(
    const Point &position,
    const std::vector<FrontierRegion> &regions);

#endif