#include "Cell.hpp"

std::array<Cell *, 8> Cell::get_neighbors(Grid2D<std::unique_ptr<Cell>> *grid) {
  std::array<Cell *, 8> neighbors;

  int idx = 0;
  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (dx == 0 && dy == 0) {
        continue;
      }

      const auto [cell_y, cell_x] = get_cell_index_from(center.x(), center.y());
      const int neighbor_x = cell_x + dx;
      const int neighbor_y = cell_y + dy;

      neighbors[idx++] = (*grid)[neighbor_y][neighbor_x].get();
    }
  }

  return neighbors;
}

Index2D get_cell_index_from(const double x, const double y) {
  return std::make_pair(std::floor((y + ENV_HEIGHT) * INV_CELL_SIZE),
                        std::floor((x + ENV_WIDTH) * INV_CELL_SIZE));
}
