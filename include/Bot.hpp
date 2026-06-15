#ifndef BOT_HPP
#define BOT_HPP

#include "DrawUtils.hpp"
#include "cgal_types.hpp"
#include <array>

constexpr std::size_t MAX_LIDAR_SAMPLES = 360;

const Robot::Point START_POSITION(9.0, 9.0);
constexpr double LIDAR_RADIUS = 1.5;
constexpr double LIDAR_RESOLUTION = LIDAR_RADIUS / 1000.0;

const float DRAWN_BODY_RADIUS = 5.0;
const float DRAWN_POINT_RADIUS = 3.0;

const double SPEED = 0.1;

struct Reading {
  double angle;
  double distance;
};

class Bot {
private:
  Robot::Point real_position;

protected:
  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::optional<std::size_t> closest_wall_reading_index = std::nullopt;

  void reset();

  Robot::Point get_real_position() const { return real_position; }
  void update_visited_positions();

  void draw_body(const DrawData &draw_data) const;
  void draw_lidar(const DrawData &draw_data) const;
  void draw_readings(const DrawData &draw_data) const;
  void draw_position_text() const;

public:
  Bot(const Robot::Point &start_pos);
  void take_lidar_readings();
  Robot::Vector move(const Robot::Vector &dir);
};

#endif
