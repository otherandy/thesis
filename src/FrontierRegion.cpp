#include "FrontierRegion.hpp"
#include <algorithm>
#include <queue>

std::vector<Point> FrontierRegion::get_points() const
{
  std::vector<Point> points;
  for (const Cell &cell : cells)
  {
    points.push_back(cell.center);
  }
  return points;
}

Polygon FrontierRegion::to_polygon() const
{
  std::vector<Point> cell_centers;
  for (const Cell &cell : cells)
  {
    cell_centers.push_back(cell.center);
  }
  return Polygon(cell_centers.begin(), cell_centers.end());
}

Point FrontierRegion::get_closest_from(const Point &pos) const
{
  Point closest_point = cells.front().center;
  double closest_distance = CGAL::squared_distance(pos, closest_point);

  for (const Cell &cell : cells)
  {
    double distance = CGAL::squared_distance(pos, cell.center);
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_point = cell.center;
    }
  }

  return closest_point;
}

std::vector<Point> FrontierRegion::calculate_path_from(const Point &start) const
{
  std::vector<Point> path;

  if (cells.empty())
  {
    path.push_back(start);
    return path;
  }

  std::vector<Point> remaining_points;
  remaining_points.reserve(cells.size());
  for (const Cell &cell : cells)
  {
    remaining_points.push_back(cell.center);
  }

  path.reserve(remaining_points.size() + 2);
  path.push_back(start);

  Point current_point = start;
  while (!remaining_points.empty())
  {
    auto next_it = remaining_points.begin();
    double best_distance = CGAL::squared_distance(current_point, *next_it);

    for (auto it = std::next(remaining_points.begin()); it != remaining_points.end(); ++it)
    {
      const double distance = CGAL::squared_distance(current_point, *it);
      if (distance < best_distance)
      {
        best_distance = distance;
        next_it = it;
      }
    }

    current_point = *next_it;
    path.push_back(current_point);
    remaining_points.erase(next_it);
  }

  path.push_back(start);

  return path;
}

void compute_frontier_regions(
    std::vector<FrontierRegion> *frontier_regions,
    Grid2D<Cell> &grid)
{
  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      grid[y][x].frontier_id = -1;
    }
  }

  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      Cell &cell = grid[y][x];

      if (cell.state == CellState::Frontier && cell.frontier_id == -1)
      {
        FrontierRegion new_region;
        new_region.id = frontier_regions->size();

        std::queue<Cell> to_visit;
        to_visit.push(cell);
        cell.frontier_id = new_region.id;

        while (!to_visit.empty())
        {
          Cell current_cell = to_visit.front();
          to_visit.pop();

          new_region.cells.push_back(current_cell);

          for (Index2D neighbor_cell : current_cell.get_neighbors())
          {
            Cell &neighbor = grid[neighbor_cell.first][neighbor_cell.second];

            if (neighbor.state == CellState::Frontier &&
                neighbor.frontier_id == -1)
            {
              neighbor.frontier_id = new_region.id;
              to_visit.push(neighbor);
            }
          }
        }

        frontier_regions->push_back(new_region);
      }
    }
  }
}

std::size_t get_nearest_frontier_region_id(
    const Point &position,
    const std::vector<FrontierRegion> &regions)
{
  std::size_t nearest_region_id = 0;
  double nearest_distance = std::numeric_limits<double>::max();

  for (const FrontierRegion &region : regions)
  {
    if (region.explored)
    {
      continue;
    }

    const double distance = CGAL::squared_distance(
        position, region.get_closest_from(position));

    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_region_id = region.id;
    }
  }

  return nearest_region_id;
}
