#ifndef UTILS_H
#define UTILS_H

#include <raylib-cpp.hpp>
#include "Environment.hpp"

using Kernel = CGAL::Simple_cartesian<double>;
using Vector = Kernel::Vector_2;

struct Reading
{
  double angle;
  double distance;
};

const float WINDOW_PADDING = 10.0f;

const Vector NORTH(0, -1);
const Vector SOUTH(0, 1);
const Vector WEST(-1, 0);
const Vector EAST(1, 0);

inline double compute_angle_to_point(const Point &from, const Point &to)
{
  return atan2(to.y() - from.y(), to.x() - from.x());
}

inline Point point_at_reading(const Point &origin, const Reading &r)
{
  return Point(origin.x() + r.distance * cos(r.angle),
               origin.y() + r.distance * sin(r.angle));
}

inline Vector normalize_vector(const Vector &v)
{
  double length = std::sqrt(v.squared_length());
  return Vector(v.x() / length, v.y() / length);
}

inline float calculate_scale_factor(const raylib::Window &window)
{
  const float padded_width = window.GetWidth() - 2.0f * WINDOW_PADDING;
  const float padded_height = window.GetHeight() - 2.0f * WINDOW_PADDING;

  return std::min(padded_width / ENV_WIDTH,
                  padded_height / ENV_HEIGHT);
}

inline raylib::Vector2 calculate_offset(const raylib::Window &window, float scale_factor)
{
  const float draw_width = ENV_WIDTH * scale_factor;
  const float draw_height = ENV_HEIGHT * scale_factor;

  return raylib::Vector2{
      (window.GetWidth() - draw_width) * 0.5f,
      (window.GetHeight() - draw_height) * 0.5f};
}

#endif