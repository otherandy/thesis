#ifndef UTILS_HPP
#define UTILS_HPP

#include "Bot.hpp"
#include "cgal_types.hpp"
#include <filesystem>

inline std::size_t relative_index(std::size_t index, int offset) {
  return (index + offset + LIDAR_SAMPLES) % LIDAR_SAMPLES;
}

inline double compute_angle_to_point(const Robot::Point &from,
                                     const Robot::Point &to) {
  return atan2(to.y() - from.y(), to.x() - from.x());
}

inline Robot::Point point_at_reading(const Robot::Point &origin,
                                     const Reading &r) {
  return Robot::Point(origin.x() + r.distance * cos(r.angle),
                      origin.y() + r.distance * sin(r.angle));
}

inline Robot::Vector normalize_vector(const Robot::Vector &v) {
  const double len2 = v.squared_length();
  const double inv_len = 1.0 / std::sqrt(len2);
  return Robot::Vector(v.x() * inv_len, v.y() * inv_len);
}

inline void ensure_parent_dir_exists(const std::string &filename) {
  const std::filesystem::path file_path(filename);
  const std::filesystem::path parent_dir = file_path.parent_path();

  if (parent_dir.empty()) {
    return;
  }

  std::error_code ec;
  const bool created = std::filesystem::create_directories(parent_dir, ec);

  if (ec) {
    std::cerr << "BOT: Failed to create directory " << parent_dir.string()
              << " (" << ec.message() << ")" << std::endl;
    return;
  }

  if (created) {
    std::cout << "BOT: Created directory " << parent_dir.string() << std::endl;
  }
}

inline Robot::Vector get_random_heading() {
  double heading = (static_cast<double>(rand()) / RAND_MAX) * 2.0 * M_PI;
  return Robot::Vector(cos(heading), sin(heading));
}

#endif
