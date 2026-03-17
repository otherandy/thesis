#include "FrontierRegion.hpp"

std::vector<Point> FrontierRegion::get_points() const
{
  return cell_centers;
}

Polygon FrontierRegion::to_polygon() const
{
  return Polygon(cell_centers.begin(), cell_centers.end());
}

Point FrontierRegion::get_closest_from(const Point &pos) const
{
  Point closest_point = cell_centers[0];
  double closest_distance = CGAL::squared_distance(pos, closest_point);

  for (const auto &center : cell_centers)
  {
    double distance = CGAL::squared_distance(pos, center);
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_point = center;
    }
  }

  return closest_point;
}
