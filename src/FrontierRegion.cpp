#include "FrontierRegion.hpp"
#include "Cell.hpp"
#include <algorithm>

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