#ifndef BOT_HPP
#define BOT_HPP

#include "DrawUtils.hpp"
#include <array>

constexpr std::size_t LIDAR_SAMPLES = 360;

constexpr double LIDAR_RADIUS = 1.5;
constexpr double LIDAR_RESOLUTION = LIDAR_RADIUS / 1000.0;
const double ANGLE_STEP = 2.0 * M_PI / LIDAR_SAMPLES;

const float DRAWN_BODY_RADIUS = 5.0;
const float DRAWN_POINT_RADIUS = 3.0;

const double SPEED = 0.1;

struct Reading {
  double angle;
  double distance;
};

class Bot {
public:
  Bot(const Robot::Point &start_pos);
  Robot::Vector move(const Robot::Vector &dir);
  void take_lidar_readings();

protected:
  std::array<Reading, LIDAR_SAMPLES> readings;
  std::optional<std::size_t> closest_wall_reading_index;

  void reset(const Robot::Point &start_pos);

  Robot::Point get_real_position() const { return real_position; }

  void draw_body(const DrawData &draw_data) const;
  void draw_range(const DrawData &draw_data) const;
  void draw_readings(const DrawData &draw_data) const;
  void draw_position_text() const;

private:
  Robot::Point real_position;
};

#endif
