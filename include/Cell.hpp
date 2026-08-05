#ifndef CELL_HPP
#define CELL_HPP

#include "Grid.hpp"
#include "cgal_types.hpp"
#include <optional>
#include <raylib-cpp.hpp>

enum class CellState { Unknown, Free, Occupied, Visited, Frontier };

struct Cell {
  Index2D index;
  Robot::Point center;
  CellState state = CellState::Unknown;
  std::optional<std::size_t> frontier_id = std::nullopt;
  std::optional<raylib::Color> debug_color = std::nullopt;

  std::array<Cell *, 8> get_neighbors(Grid2D<std::unique_ptr<Cell>> *grid);
};

Index2D get_cell_index_from(const double x, const double y);

#endif
