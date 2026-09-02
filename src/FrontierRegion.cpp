#include "FrontierRegion.hpp"
#include "Grid.hpp"
#include <queue>
#include <unordered_set>

constexpr std::array<std::pair<int, int>, 4> directions{{
    {-1, 0}, // N
    {1, 0},  // S
    {0, -1}, // W
    {0, 1}   // E
}};

std::optional<Robot::Point> FrontierRegion::get_closest_unexplored(
    const Grid2D<std::unique_ptr<Cell>> &grid, const Robot::Point &pos) const {

  std::optional<Robot::Point> closest_point = std::nullopt;
  double closest_distance = std::numeric_limits<double>::max();

  for (const Index2D idx : cells) {
    const Cell *cell = grid[idx.first][idx.second].get();

    if (cell->state != CellState::Frontier) {
      continue;
    }

    const double distance = CGAL::squared_distance(pos, cell->center);
    if (distance < closest_distance) {
      closest_distance = distance;
      closest_point = cell->center;
    }
  }

  return closest_point;
}

double FrontierRegion::get_area() const {
  return (max.first - min.first) * (max.second - min.second);
}

double
FrontierRegion::get_area_slow(const Grid2D<std::unique_ptr<Cell>> &grid) const {
  if (physical) {
    return get_area();
  }

  if (cells.empty()) {
    return 0;
  }

  Cell *reference_cell = nullptr;

  for (const Index2D &idx : cells) {
    Cell *cell = grid[idx.first][idx.second].get();

    for (const auto &[dy, dx] : directions) {
      int ny = idx.first + dy;
      int nx = idx.second + dx;

      Cell *neighbor = grid[ny][nx].get();
      if (neighbor->state == CellState::Unknown) {
        reference_cell = cell;
        break;
      }
    }
  }

  if (reference_cell == nullptr) {
    return 0;
  }

  double area = 0;

  auto key = [](int y, int x) {
    return (static_cast<uint64_t>(y) << 32) | static_cast<uint32_t>(x);
  };

  std::queue<Index2D> q;
  std::unordered_set<uint64_t> visited;

  q.push(cells[0]);
  visited.insert(key(cells[0].first, cells[0].second));

  while (!q.empty()) {
    const Index2D idx = q.front();
    q.pop();

    Cell *cell = grid[idx.first][idx.second].get();
    area += CELL_SIZE;

    for (auto [dy, dx] : directions) {
      int ny = idx.first + dy;
      int nx = idx.second + dx;

      if (visited.insert(key(ny, nx)).second &&
          grid[ny][nx].get()->state == CellState::Unknown) {
        q.push({ny, nx});
      }
    }
  }

  return area;
}

bool FrontierRegion::is_done(const Grid2D<std::unique_ptr<Cell>> &grid) const {
  for (const Index2D idx : cells) {
    const Cell *cell = grid[idx.first][idx.second].get();

    if (cell->state == CellState::Frontier) {
      return false;
    }
  }

  return true;
}
