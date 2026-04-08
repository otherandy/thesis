#ifndef BOT_HPP
#define BOT_HPP

#include "cgal_types.hpp"
#include "DrawUtils.hpp"
#include <array>
#include <string>
#include <vector>

constexpr std::size_t MAX_LIDAR_SAMPLES = 360;

const Point START_POSITION(9.0, 9.0);
const double LIDAR_RADIUS = 1.5;
const double LIDAR_RESOLUTION = LIDAR_RADIUS / 1000.0;

const float DRAWN_BODY_RADIUS = 5.0;
const float DRAWN_POINT_RADIUS = 3.0;

struct Reading
{
  double angle;
  double distance;
};

class Bot
{
private:
  Point real_position;
  std::vector<Point> real_visited_positions;

protected:
  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::optional<std::size_t> closest_wall_reading_index = std::nullopt;

  bool draw_as_hud = true;
  double speed = 0.1;

  void reset();
  Point reading_index_to_point(std::size_t index) const;
  Vector move(const Vector &dir);

  Point get_real_position() const { return real_position; }
  void update_visited_positions();

  void take_lidar_readings();
  void draw_body(DrawData draw_data) const;
  void draw_lidar(DrawData draw_data) const;
  void draw_readings(DrawData draw_data) const;
  void draw_path(DrawData draw_data) const;
  void draw_position_text() const;

public:
  Bot(const Point &start_pos);
  void visited_to_file(const std::string &filename) const;
};

#endif