#ifndef UTILS_HPP
#define UTILS_HPP

#include "Bot.hpp"
#include "cgal_types.hpp"
#include <filesystem>

inline std::size_t relative_index(std::size_t index, int offset)
{
  return (index + offset + MAX_LIDAR_SAMPLES) % MAX_LIDAR_SAMPLES;
}

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

inline void ensure_parent_dir_exists(const std::string &filename)
{
  const std::filesystem::path file_path(filename);
  const std::filesystem::path parent_dir = file_path.parent_path();
  if (parent_dir.empty())
  {
    return;
  }

  std::error_code ec;
  const bool created = std::filesystem::create_directories(parent_dir, ec);
  if (ec)
  {
    std::cerr << "BOT: Failed to create directory " << parent_dir.string()
              << " (" << ec.message() << ")" << std::endl;
    return;
  }

  if (created)
  {
    std::cout << "BOT: Created directory " << parent_dir.string() << std::endl;
  }
}

#endif