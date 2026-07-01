#include "Bot.hpp"
#include "Utils.hpp"
#include <raylib-cpp.hpp>

Bot::Bot(const Robot::Point &start_pos) : real_position(start_pos) {}

void Bot::reset(const Robot::Point &start_pos) { real_position = start_pos; }

// Returns delta applied to position
Robot::Vector Bot::move(const Robot::Vector &dir) {
  const Robot::Vector delta = normalize_vector(dir) * SPEED;
  const Robot::Point new_position = real_position + delta;

  if (!point_in_environment(new_position)) {
    return Robot::Vector(0, 0);
  }

  real_position = new_position;
  return delta;
}

void Bot::take_lidar_readings() {
  double closest_distance = std::numeric_limits<double>::max();
  closest_wall_reading_index = std::nullopt;

  for (int i = 0; i < LIDAR_SAMPLES; ++i) {
    double angle = ANGLE_STEP * i - M_PI;
    double distance = LIDAR_RADIUS;

    // Binary search for wall intersection
    double min_dist = LIDAR_RESOLUTION;
    double max_dist = LIDAR_RADIUS;

    while (max_dist - min_dist > LIDAR_RESOLUTION) {
      const double mid_dist = (min_dist + max_dist) / 2.0;
      const double sample_x = real_position.x() + mid_dist * cos(angle);
      const double sample_y = real_position.y() + mid_dist * sin(angle);

      if (point_in_environment(Robot::Point(sample_x, sample_y))) {
        min_dist = mid_dist;
      } else {
        max_dist = mid_dist;
      }
    }

    distance = max_dist;

    if (distance < LIDAR_RADIUS && distance < closest_distance) {
      closest_distance = distance;
      closest_wall_reading_index = i;
    }

    readings[i] = Reading{angle, distance};
  }
}

void Bot::draw_body(const DrawData &draw_data, raylib::Color color) const {
  DrawCircle(real_position.x() * draw_data.scale_factor + draw_data.offset_x,
             real_position.y() * draw_data.scale_factor + draw_data.offset_y,
             DRAWN_BODY_RADIUS, color);
}

void Bot::draw_range(const DrawData &draw_data) const {
  DrawCircleLines(
      real_position.x() * draw_data.scale_factor + draw_data.offset_x,
      real_position.y() * draw_data.scale_factor + draw_data.offset_y,
      LIDAR_RADIUS * draw_data.scale_factor, raylib::BLUE);
}

void Bot::draw_readings(const DrawData &draw_data) const {
  float pos_x = real_position.x() * draw_data.scale_factor + draw_data.offset_x;
  float pos_y = real_position.y() * draw_data.scale_factor + draw_data.offset_y;

  for (int i = 0; i < LIDAR_SAMPLES; ++i) {
    const Reading &r = readings[i];
    const float end_x =
        pos_x + r.distance * draw_data.scale_factor * cos(r.angle);
    const float end_y =
        pos_y + r.distance * draw_data.scale_factor * sin(r.angle);

    if (i == closest_wall_reading_index) {
      DrawLineEx({pos_x, pos_y}, {end_x, end_y}, 5.0, RED);
    } else {
      DrawLine(pos_x, pos_y, end_x, end_y, GRAY);
    }
  }
}

void Bot::draw_position_text() const {
  std::string pos_text = "Pos: (" + std::to_string(real_position.x()) + ", " +
                         std::to_string(real_position.y()) + ")";
  DrawText(pos_text.c_str(), 10, GetScreenHeight() - 30, 20, BLACK);
}
