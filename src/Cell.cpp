#include "Cell.hpp"

std::array<std::unique_ptr<Cell>, 8> Cell::get_neighbors(Grid2D<Cell> grid)
{
  std::array<std::unique_ptr<Cell>, 8> neighbors;

  int idx = 0;
  for (int dy = -1; dy <= 1; ++dy)
  {
    for (int dx = -1; dx <= 1; ++dx)
    {
      if (dx == 0 && dy == 0)
      {
        continue;
      }

      const int neighbor_x = static_cast<int>((center.x() + dx * CELL_SIZE) * INV_CELL_SIZE + ENV_WIDTH * INV_CELL_SIZE);
      const int neighbor_y = static_cast<int>((center.y() + dy * CELL_SIZE) * INV_CELL_SIZE + ENV_HEIGHT * INV_CELL_SIZE);

      if (neighbor_x >= 0 && neighbor_x < MAP_WIDTH &&
          neighbor_y >= 0 && neighbor_y < MAP_HEIGHT)
      {
        neighbors[idx] = std::make_unique<Cell>(grid[neighbor_y][neighbor_x]);
      }
      else
      {
        neighbors[idx] = nullptr;
      }
    }
  }

  return neighbors;
}

inline bool is_valid_index(Index2D index)
{
  const int cell_x = index.first;
  const int cell_y = index.second;

  return cell_x >= 0 && cell_x < MAP_WIDTH &&
         cell_y >= 0 && cell_y < MAP_HEIGHT;
}

inline Index2D get_cell_index_from(const double x, const double y)
{
  Index2D index = std::make_pair(
      std::floor((y + ENV_HEIGHT) * INV_CELL_SIZE),
      std::floor((x + ENV_WIDTH) * INV_CELL_SIZE));

  return index;
}