#include "Bot.hpp"
#include "Utils.hpp"
#include <memory>
#include <raylib-cpp.hpp>

Bot::Bot(std::size_t id, double radius, const Robot::Point &start_pos,
         std::shared_ptr<Environment> env)
    : id(id), radius(radius), real_position(start_pos), environment(env) {}

void Bot::reset(const Robot::Point &start_pos) { real_position = start_pos; }

// Returns delta applied to position
Robot::Vector Bot::move(const Robot::Vector &dir) {
  const Robot::Vector delta = normalize_vector(dir) * SPEED;
  const Robot::Point new_position = real_position + delta;

  if (!environment->contains(new_position)) {
    return Robot::Vector(0, 0);
  }

  real_position = new_position;
  return delta;
}

void Bot::take_lidar_readings() {
  double closest_distance = std::numeric_limits<double>::max();
  closest_wall_reading_index = std::nullopt;

  for (int i = 0; i < LIDAR_SAMPLES; ++i) {
    const double angle = ANGLE_STEP * i - M_PI;
    double distance = radius;

    Robot::Point origin(real_position.x(), real_position.y());
    Robot::Point end(real_position.x() + radius * cos(angle),
                     real_position.y() + radius * sin(angle));

    Robot::Segment ray(origin, end);
    std::vector<std::optional<
        Robot::AABB_tree::Intersection_and_primitive_id<Robot::Segment>::Type>>
        intersections;

    environment->get_tree().all_intersections(
        ray, std::back_inserter(intersections));

    for (auto &result : intersections) {
      if (const auto *pt = std::get_if<Robot::Point>(&(result->first))) {
        double d = std::sqrt(CGAL::squared_distance(origin, *pt));
        if (d < distance) {
          distance = d;
        }
      }
    }

    if (distance < radius && distance < closest_distance) {
      closest_distance = distance;
      closest_wall_reading_index = i;
    }

    readings[i] = Reading{angle, distance};
  }
}

void Bot::draw_body(const DrawData &draw_data, raylib::Color color) const {
  int x = real_position.x() * draw_data.scale_factor + draw_data.offset_x;
  int y = real_position.y() * draw_data.scale_factor + draw_data.offset_y;

  DrawCircle(x, y, DRAWN_BODY_RADIUS, color);

  if (id % 2 == 1) {
    x = x - 10;
  } else {
    x = x + 10;
  }

  if (id % 4 == 1 || id % 4 == 2) {
    y = y - 7;
  } else {
    y = y + 7;
  }

  DrawText(TextFormat("%zu", id), x, y, 10, BLACK);
}

void Bot::draw_range(const DrawData &draw_data) const {
  DrawCircleLines(
      real_position.x() * draw_data.scale_factor + draw_data.offset_x,
      real_position.y() * draw_data.scale_factor + draw_data.offset_y,
      radius * draw_data.scale_factor, raylib::BLUE);
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
      DrawLineEx({pos_x, pos_y}, {end_x, end_y}, 5.0, raylib::RED);
    } else {
      DrawLine(pos_x, pos_y, end_x, end_y, raylib::GRAY);
    }
  }
}
