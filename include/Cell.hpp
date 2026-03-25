#ifndef CELL_HPP
#define CELL_HPP

#include "Grid.hpp"
#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

enum class CellState
{
  Unknown,
  Free,
  Occupied,
  Visited,
  Frontier
};

const std::map<CellState, Color> CellColors = {
    {CellState::Unknown, GRAY},
    {CellState::Free, YELLOW},
    {CellState::Occupied, BLACK},
    {CellState::Visited, RED},
    {CellState::Frontier, BLUE}};

struct Cell
{
  Point center;
  CellState state;
  std::optional<std::size_t> frontier_id;

  std::pair<double, double> get_position() const;
  std::array<Index2D, 8> get_neighbors();
};

Index2D get_cell_index_from(const double x, const double y);

#endif