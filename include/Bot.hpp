#ifndef BOT_HPP
#define BOT_HPP

#include "cgal_types.hpp"
#include <array>
#include <string>
#include <vector>

#define INVALID_INDEX -1
#define MAX_LIDAR_SAMPLES 360

const Point START_POSITION(4.0, 4.0);
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
  int closest_wall_reading_index = INVALID_INDEX;

  bool draw_as_hud = true;
  double speed = 0.1;

  void reset();
  Point reading_index_to_point(int index) const;
  Vector move(const Vector &dir);
  Point get_real_position() const;
  void update_visited_positions();
  void take_lidar_readings();
  void draw_body(float scale_factor, float offset_x, float offset_y) const;
  void draw_lidar(float scale_factor, float offset_x, float offset_y) const;
  void draw_readings(float scale_factor, float offset_x, float offset_y) const;
  void draw_path(float scale_factor, float offset_x, float offset_y) const;

public:
  Bot(const Point &start_pos);
  void visited_to_file(const std::string &filename) const;
};

#endif