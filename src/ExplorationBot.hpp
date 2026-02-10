#ifndef EXPLORATION_BOT_H
#define EXPLORATION_BOT_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <iostream>
#include "Utils.hpp"

using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Polygon = CGAL::Polygon_2<Kernel>;
using Direction = Kernel::Direction_2;
using Vector = Kernel::Vector_2;

enum class ExplorationPhase
{
  Idle,
  WallDiscovery,
  WallAlignment,
  WallFollowing,
  Completed
};

// --- Constants ---
const Point START_POSITION(4.0, 4.0);
const int MAX_LIDAR_SAMPLES = 360 / 8;
const double LIDAR_RADIUS = 1.5;
const double LIDAR_RESOLUTION = LIDAR_RADIUS / 50.0;
const int LIDAR_EPSILON = 2;

const double DESIRED_WALL_DISTANCE = 0.15;
const double WALL_FOLLOWING_WEIGHT = 0.6;

// --- Classes ---
class ExplorationBot
{
private:
  Point real_position; // for drawing and LIDAR calculations
  Point real_first_contact_point = Point(-1, -1);

  Point relative_position = Point(0.0, 0.0); // exploration and path tracking
  Point relative_first_contact_point = Point(-1, -1);

  Point exploration_start_point = Point(0.0, 0.0);

  Vector current_follow_vector;

  bool draw_as_hud = true;
  double speed = 0.1;

  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::vector<Point> real_visited_positions;

  ExplorationPhase exploration_phase = ExplorationPhase::Idle;
  Direction random_direction;
  bool left_first_contact = false;

  int closest_wall_reading_index = -1;
  int cw_disconnect_reading_index = 0;
  int ccw_disconnect_reading_index = 0;

protected:
  void find_cw_disconnect_reading()
  {
    for (int i = closest_wall_reading_index; i >= 0; --i)
    {
      if (current_readings[i].distance >= LIDAR_RADIUS)
      {
        cw_disconnect_reading_index = i;
        return;
      }
    }

    for (int i = MAX_LIDAR_SAMPLES - 1; i > closest_wall_reading_index; --i)
    {
      if (current_readings[i].distance >= LIDAR_RADIUS)
      {
        cw_disconnect_reading_index = i;
        return;
      }
    }
  }

  void find_ccw_disconnect_reading()
  {
    for (int i = closest_wall_reading_index; i < MAX_LIDAR_SAMPLES; ++i)
    {
      if (current_readings[i].distance >= LIDAR_RADIUS)
      {
        ccw_disconnect_reading_index = i;
        return;
      }
    }

    for (int i = 0; i < closest_wall_reading_index; ++i)
    {
      if (current_readings[i].distance >= LIDAR_RADIUS)
      {
        ccw_disconnect_reading_index = i;
        return;
      }
    }
  }

  void create_follow_vector()
  {
    if (closest_wall_reading_index == -1)
      return;

    const int cw_epsilon_index = closest_wall_reading_index - LIDAR_EPSILON < 0 ? MAX_LIDAR_SAMPLES - 1 : closest_wall_reading_index - LIDAR_EPSILON;
    const Point clockwise_point = reading_index_to_point(cw_epsilon_index);

    const int ccw_epsilon_index = closest_wall_reading_index + LIDAR_EPSILON >= MAX_LIDAR_SAMPLES ? 0 : closest_wall_reading_index + LIDAR_EPSILON;
    const Point counterclockwise_point = reading_index_to_point(ccw_epsilon_index);

    current_follow_vector = clockwise_point - counterclockwise_point;
  }

  Vector calculate_wall_correction_vector()
  {
    if (closest_wall_reading_index == -1)
      return Vector(0, 0);

    const Reading r = current_readings[closest_wall_reading_index];

    double distance_error = r.distance - DESIRED_WALL_DISTANCE;
    double correction_angle = r.angle + M_PI / 2; // perpendicular to the wall

    return Vector(distance_error * cos(correction_angle),
                  distance_error * sin(correction_angle));
  }

  inline Point reading_index_to_point(int index)
  {
    const Reading r = current_readings[index];
    return point_at_reading(real_position, r);
  }

  void update_position_data()
  {
    if (exploration_phase != ExplorationPhase::Idle)
    {
      real_visited_positions.push_back(real_position);
    }
  }

  void move(const Direction &dir)
  {
    Vector delta = normalize_vector(dir.to_vector()) * speed;
    Point new_position = real_position + delta;

    if (!point_in_environment(new_position))
      return;

    update_position_data();

    real_position = new_position;
    relative_position = relative_position + delta;
  }

  void draw_specific_reading(int index, raylib::Vector2 &pos, float scale_factor, const raylib::Vector2 &offset, Color color)
  {
    const Reading r = current_readings[index];

    if (r.distance == LIDAR_RADIUS)
      return;

    float end_x = pos.x + r.distance * scale_factor * cos(r.angle);
    float end_y = pos.y + r.distance * scale_factor * sin(r.angle);

    DrawCircle(end_x,
               end_y,
               5,
               color);
  }

public:
  ExplorationBot(const Point &start_pos)
      : real_position(start_pos)
  {
    current_readings.fill({LIDAR_RADIUS, LIDAR_RADIUS});

    const double heading = (static_cast<double>(rand()) / RAND_MAX) * 2.0 * M_PI;
    random_direction = Direction(cos(heading), sin(heading));
  }

  void take_lidar_readings(int num_samples = MAX_LIDAR_SAMPLES)
  {
    closest_wall_reading_index = -1;

    for (int i = 0; i < num_samples; ++i)
    {
      double angle = 2 * M_PI * i / num_samples;
      double distance;

      for (distance = LIDAR_RESOLUTION;
           distance <= LIDAR_RADIUS;
           distance += LIDAR_RESOLUTION)
      {
        double sample_x = real_position.x() + distance * cos(angle);
        double sample_y = real_position.y() + distance * sin(angle);

        if (!point_in_environment(Point(sample_x, sample_y)))
          break;
      }

      if (distance < LIDAR_RADIUS)
      {
        if (closest_wall_reading_index == -1 || distance < current_readings[closest_wall_reading_index].distance)
        {
          closest_wall_reading_index = i;
        }
      }

      current_readings[i] = Reading{angle, distance};
    }

    find_cw_disconnect_reading();
    find_ccw_disconnect_reading();

    return;
  }

  void get_input_and_move()
  {
    if (IsKeyPressed(KEY_R))
    {
      draw_as_hud = !draw_as_hud;
    }

    if (exploration_phase != ExplorationPhase::Idle)
      return;

    if (IsKeyPressed(KEY_SPACE))
    {
      exploration_start_point = relative_position;
      exploration_phase = ExplorationPhase::WallDiscovery;
    }

    if (IsKeyDown(KEY_W))
    {
      move(NORTH);
    }
    if (IsKeyDown(KEY_S))
    {
      move(SOUTH);
    }
    if (IsKeyDown(KEY_A))
    {
      move(WEST);
    }
    if (IsKeyDown(KEY_D))
    {
      move(EAST);
    }
  }

  void visited_to_file(const std::string &filename)
  {
    std::ofstream f(filename);
    for (const auto &v : real_visited_positions)
    {
      f << v.x() << "," << v.y() << "\n";
    }
    f.close();
    std::cout << "BOT: Saved visited positions to " << filename << std::endl;
  }

  // --- Exploration Methods ---
  void phase1_wall_discovery()
  {
    if (std::sqrt(CGAL::squared_distance(relative_position, exploration_start_point)) >= EXPLORATION_RADIUS)
      return;

    for (const auto &r : current_readings)
    {
      if (r.distance < LIDAR_RADIUS)
      {
        std::cout << "EXPLORATION: Wall detected at position ("
                  << relative_position.x() << ", " << relative_position.y()
                  << ")\n";
        exploration_phase = ExplorationPhase::WallAlignment;
        return;
      }
    }

    move(random_direction);
  }

  void phase2_wall_alignment()
  {
    const Reading closest_reading = current_readings[closest_wall_reading_index];

    if (closest_reading.distance <= DESIRED_WALL_DISTANCE)
    {
      std::cout << "EXPLORATION: Aligned with wall at position ("
                << relative_position.x() << ", " << relative_position.y()
                << ")\n";
      real_first_contact_point = real_position;
      relative_first_contact_point = relative_position;
      exploration_phase = ExplorationPhase::WallFollowing;
      return;
    }

    const Vector to_wall = Vector(
        cos(closest_reading.angle),
        sin(closest_reading.angle));

    move(to_wall.direction());
  }

  void phase3_wall_following()
  {
    create_follow_vector();
    Vector wall_vector = calculate_wall_correction_vector();
    Vector desired_vector = current_follow_vector * WALL_FOLLOWING_WEIGHT + wall_vector * (1 - WALL_FOLLOWING_WEIGHT);

    move(desired_vector.direction());

    if (std::sqrt(CGAL::squared_distance(relative_position, relative_first_contact_point)) <= LIDAR_RESOLUTION)
    {
      if (!left_first_contact)
        return;

      std::cout << "EXPLORATION: Completed wall following loop.\n";
      exploration_phase = ExplorationPhase::Completed;
      return;
    }

    left_first_contact = true;
  }

  void run_exploration()
  {
    if (exploration_phase == ExplorationPhase::Idle)
      return;

    if (exploration_phase == ExplorationPhase::WallDiscovery)
    {
      phase1_wall_discovery();
      return;
    }

    if (exploration_phase == ExplorationPhase::WallAlignment)
    {
      phase2_wall_alignment();
      return;
    }

    if (exploration_phase == ExplorationPhase::WallFollowing)
    {
      phase3_wall_following();
      return;
    }
  }

  // --- Drawing Methods ---
  void draw_readings(float scale_factor, const raylib::Vector2 &offset)
  {
    raylib::Vector2 pos;

    if (draw_as_hud)
    {
      pos.x = LIDAR_RADIUS * scale_factor + WINDOW_PADDING;
      pos.y = LIDAR_RADIUS * scale_factor + WINDOW_PADDING;
    }
    else
    {
      pos.x = real_position.x() * scale_factor + offset.x;
      pos.y = real_position.y() * scale_factor + offset.y;
    }

    for (const auto &r : current_readings)
    {
      float end_x = pos.x + r.distance * scale_factor * cos(r.angle);
      float end_y = pos.y + r.distance * scale_factor * sin(r.angle);

      DrawLine(pos.x, pos.y,
               end_x, end_y,
               GRAY);
    }

    if (closest_wall_reading_index == -1)
      return;

    draw_specific_reading(closest_wall_reading_index, pos, scale_factor, offset, RED);
    draw_specific_reading(cw_disconnect_reading_index, pos, scale_factor, offset, ORANGE);
    draw_specific_reading(ccw_disconnect_reading_index, pos, scale_factor, offset, ORANGE);
  }

  void draw(float scale_factor, const raylib::Vector2 &offset)
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

  void draw_follow_vector(float scale_factor, const raylib::Vector2 &offset)
  {
    if (current_follow_vector.squared_length() == 0)
      return;

    Point end_point = real_position + normalize_vector(current_follow_vector);

    DrawLine(real_position.x() * scale_factor + offset.x,
             real_position.y() * scale_factor + offset.y,
             end_point.x() * scale_factor + offset.x,
             end_point.y() * scale_factor + offset.y,
             GREEN);
  }

  void draw_first_contact_point(float scale_factor, const raylib::Vector2 &offset)
  {
    if (exploration_phase < ExplorationPhase::WallFollowing)
      return;

    DrawCircle(real_first_contact_point.x() * scale_factor + offset.x,
               real_first_contact_point.y() * scale_factor + offset.y,
               5,
               PURPLE);
  }

  void draw_path(float scale_factor, const raylib::Vector2 &offset)
  {
    if (real_visited_positions.size() < 2)
      return;

    for (size_t i = 1; i < real_visited_positions.size(); ++i)
    {
      Point p1 = real_visited_positions[i - 1];
      Point p2 = real_visited_positions[i];
      DrawLine(p1.x() * scale_factor + offset.x,
               p1.y() * scale_factor + offset.y,
               p2.x() * scale_factor + offset.x,
               p2.y() * scale_factor + offset.y,
               RED);
    }
  }
};

#endif