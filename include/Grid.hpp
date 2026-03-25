#ifndef GRID_HPP
#define GRID_HPP

#include "Environment.hpp"

constexpr double CELL_SIZE = 0.1;
const double INV_CELL_SIZE = 1.0 / CELL_SIZE;
constexpr std::size_t MAP_WIDTH = (ENV_WIDTH * 2.0 / CELL_SIZE);
constexpr std::size_t MAP_HEIGHT = (ENV_HEIGHT * 2.0 / CELL_SIZE);

template <typename T>
using Grid2D = std::array<std::array<T, MAP_WIDTH>, MAP_HEIGHT>;
using Index2D = std::pair<std::size_t, std::size_t>;

#endif