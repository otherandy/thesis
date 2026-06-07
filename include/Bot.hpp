#ifndef BOT_HPP
#define BOT_HPP

#include "DrawUtils.hpp"
#include "cgal_types.hpp"
#include <array>

constexpr std::size_t MAX_LIDAR_SAMPLES = 360;

const Point START_POSITION(9.0, 9.0);
const double LIDAR_RADIUS = 1.5;
const double LIDAR_RESOLUTION = LIDAR_RADIUS / 1000.0;

const float DRAWN_BODY_RADIUS = 5.0;
const float DRAWN_POINT_RADIUS = 3.0;

const double SPEED = 0.1;

struct Reading {
  double angle;
  double distance;
};

class Bot {
private:
  Point real_position;

protected:
  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::optional<std::size_t> closest_wall_reading_index = std::nullopt;

  void reset();
  Point reading_index_to_point(std::size_t index) const;

  Point get_real_position() const { return real_position; }
  void update_visited_positions();

  void draw_body(const DrawData &draw_data) const;
  void draw_lidar(const DrawData &draw_data) const;
  void draw_readings(const DrawData &draw_data) const;
  void draw_position_text() const;

public:
  Bot(const Point &start_pos);
  void take_lidar_readings();
  Vector move(const Vector &dir);
};

#endif
