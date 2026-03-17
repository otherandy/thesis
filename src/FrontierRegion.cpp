#include "FrontierRegion.hpp"
#include "Cell.hpp"

std::vector<Point> FrontierRegion::get_points() const
{
  std::vector<Point> points;
  for (const auto &cell : cells)
  {
    points.push_back(cell.center);
  }
  return points;
}

Polygon FrontierRegion::to_polygon() const
{
  std::vector<Point> cell_centers;
  for (const auto &cell : cells)
  {
    cell_centers.push_back(cell.center);
  }
  return Polygon(cell_centers.begin(), cell_centers.end());
}

Point FrontierRegion::get_closest_from(const Point &pos) const
{
  Point closest_point = cells.front().center;
  double closest_distance = CGAL::squared_distance(pos, closest_point);

  for (const auto &cell : cells)
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

  for (const auto &cell : cells)
  {
    path.push_back(cell.center);
  }

  std::sort(path.begin(), path.end(),
            [&start](const Point &a, const Point &b)
            {
              return CGAL::squared_distance(start, a) < CGAL::squared_distance(start, b);
            });

  return path;
}