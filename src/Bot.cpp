#include "Bot.hpp"
#include "Utils.hpp"
#include <raylib-cpp.hpp>

void Bot::reset() {
  real_position = START_POSITION;
  current_readings.fill({LIDAR_RADIUS, LIDAR_RADIUS});
  closest_wall_reading_index = std::nullopt;
}

Point Bot::reading_index_to_point(std::size_t index) const {
  const Reading &r = current_readings[index];
  return point_at_reading(real_position, r);
}

// Returns delta applied to position
Vector Bot::move(const Vector &dir) {
  Vector delta = normalize_vector(dir) * SPEED;
  Point new_position = real_position + delta;

  if (!point_in_environment(new_position)) {
    return Vector(0, 0);
  }

  real_position = new_position;
  return delta;
}

void Bot::take_lidar_readings() {
  const double angle_step = 2.0 * M_PI / MAX_LIDAR_SAMPLES;

  double closest_distance = std::numeric_limits<double>::max();
  closest_wall_reading_index = std::nullopt;

  for (int i = 0; i < MAX_LIDAR_SAMPLES; ++i) {
    double angle = angle_step * i;
    double distance = LIDAR_RADIUS;

    // Binary search for wall intersection
    double min_dist = LIDAR_RESOLUTION;
    double max_dist = LIDAR_RADIUS;

    while (max_dist - min_dist > LIDAR_RESOLUTION) {
      double mid_dist = (min_dist + max_dist) / 2.0;
      double sample_x = real_position.x() + mid_dist * cos(angle);
      double sample_y = real_position.y() + mid_dist * sin(angle);

      if (point_in_environment(Point(sample_x, sample_y))) {
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

    current_readings[i] = Reading{angle, distance};
  }
}

void Bot::draw_body(DrawData draw_data) const {
  DrawCircle(real_position.x() * draw_data.scale_factor + draw_data.offset_x,
             real_position.y() * draw_data.scale_factor + draw_data.offset_y,
             DRAWN_BODY_RADIUS, RED);
}

void Bot::draw_lidar(DrawData draw_data) const {
  DrawCircleLines(
      real_position.x() * draw_data.scale_factor + draw_data.offset_x,
      real_position.y() * draw_data.scale_factor + draw_data.offset_y,
      LIDAR_RADIUS * draw_data.scale_factor, BLUE);
}

void Bot::draw_readings(DrawData draw_data) const {
  float pos_x;
  float pos_y;

  pos_x = real_position.x() * draw_data.scale_factor + draw_data.offset_x;
  pos_y = real_position.y() * draw_data.scale_factor + draw_data.offset_y;

  for (int i = 0; i < MAX_LIDAR_SAMPLES; ++i) {
    const Reading &r = current_readings[i];
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

Bot::Bot(const Point &start_pos) : real_position(start_pos) {
  current_readings.fill({LIDAR_RADIUS, LIDAR_RADIUS});
}
