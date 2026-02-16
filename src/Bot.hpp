#ifndef BOT_H
#define BOT_H

#include "Utils.hpp"

const Point START_POSITION(4.0, 4.0);
const int MAX_LIDAR_SAMPLES = 360;
constexpr double LIDAR_RADIUS = 1.5;
const double LIDAR_RESOLUTION = LIDAR_RADIUS / 100.0;

inline int relative_index(int index, int offset, int max_size = MAX_LIDAR_SAMPLES)
{
  return (index + offset + max_size) % max_size;
}

class Bot
{
private:
  Point real_position; // for drawing and LIDAR calculations

protected:
  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  int closest_wall_reading_index = -1;

  bool draw_as_hud = true;
  double speed = 0.1;

  inline Point reading_index_to_point(int index)
  {
    const Reading &r = current_readings[index];
    return point_at_reading(real_position, r);
  }

  // Returns the movement delta vector
  Vector move(const Vector &dir)
  {
    Vector delta = normalize_vector(dir) * speed;
    Point new_position = real_position + delta;

    if (!point_in_environment(new_position))
      return Vector(0, 0);

    real_position = new_position;
    return delta;
  }

public:
  Bot(const Point &start_pos)
      : real_position(start_pos)
  {
    current_readings.fill({LIDAR_RADIUS, LIDAR_RADIUS});
  }

  Point get_real_position() const
  {
    return real_position;
  }

  void take_lidar_readings(int num_samples = MAX_LIDAR_SAMPLES)
  {
    const double angle_step = 2.0 * M_PI / num_samples;

    double closest_distance = LIDAR_RADIUS;
    closest_wall_reading_index = -1;

    for (int i = 0; i < num_samples; ++i)
    {
      double angle = angle_step * i;
      double distance = LIDAR_RADIUS;

      // Binary search for wall intersection
      double min_dist = LIDAR_RESOLUTION;
      double max_dist = LIDAR_RADIUS;

      while (max_dist - min_dist > LIDAR_RESOLUTION)
      {
        double mid_dist = (min_dist + max_dist) / 2.0;
        double sample_x = real_position.x() + mid_dist * cos(angle);
        double sample_y = real_position.y() + mid_dist * sin(angle);

        if (point_in_environment(Point(sample_x, sample_y)))
          min_dist = mid_dist;
        else
          max_dist = mid_dist;
      }

      distance = max_dist;

      if (distance < closest_distance)
      {
        closest_distance = distance;
        closest_wall_reading_index = i;
      }

      current_readings[i] = Reading{angle, distance};
    }
  }

  void draw(float scale_factor, const raylib::Vector2 &offset) const
  {
    // Bot body
    DrawCircle(real_position.x() * scale_factor + offset.x,
               real_position.y() * scale_factor + offset.y,
               5,
               RED);

    // LIDAR radius
    DrawCircleLines(real_position.x() * scale_factor + offset.x,
                    real_position.y() * scale_factor + offset.y,
                    LIDAR_RADIUS * scale_factor,
                    BLUE);
  }

  void draw_readings(float scale_factor, const raylib::Vector2 &offset) const
  {
    float pos_x;
    float pos_y;

    if (draw_as_hud)
    {
      pos_x = LIDAR_RADIUS * scale_factor + WINDOW_PADDING;
      pos_y = LIDAR_RADIUS * scale_factor + WINDOW_PADDING;
    }
    else
    {
      pos_x = real_position.x() * scale_factor + offset.x;
      pos_y = real_position.y() * scale_factor + offset.y;
    }

    for (int i = 0; i < MAX_LIDAR_SAMPLES; ++i)
    {
      const Reading &r = current_readings[i];
      const float end_x = pos_x + r.distance * scale_factor * cos(r.angle);
      const float end_y = pos_y + r.distance * scale_factor * sin(r.angle);

      if (i == closest_wall_reading_index)
      {
        DrawLine(pos_x, pos_y,
                 end_x, end_y,
                 RED);
        DrawCircle(end_x, end_y, 3, RED);
      }
      else
      {
        DrawLine(pos_x, pos_y,
                 end_x, end_y,
                 GRAY);
      }
    }
  }
};

#endif