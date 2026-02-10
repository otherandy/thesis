#ifndef EXPLORATION_BOT_H
#define EXPLORATION_BOT_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <iostream>
#include "Bot.hpp"

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

// --- Setup ---
const double DESIRED_WALL_DISTANCE = 0.2;
const double WALL_FOLLOWING_WEIGHT = 0.5;

// --- Classes ---
class ExplorationBot : public Bot
{
private:
  Point relative_position = Point(0.0, 0.0); // exploration and path tracking
  Point relative_first_contact_point = Point(-1, -1);
  Point real_first_contact_point = Point(-1, -1);

  Point exploration_start_point = Point(0.0, 0.0);

  Vector current_follow_vector;

  std::vector<Point> real_visited_positions;

  ExplorationPhase exploration_phase = ExplorationPhase::Idle;
  Direction random_direction;
  bool left_first_contact = false;

  int cw_disconnect_reading_index = 0;
  int ccw_disconnect_reading_index = 0;

protected:
  void move(const Direction &dir)
  {
    relative_position = relative_position + Bot::move(dir);

    if (exploration_phase != ExplorationPhase::Idle)
    {
      real_visited_positions.push_back(get_real_position()); // track path
    }
  }

public:
  ExplorationBot(const Point &start_pos)
      : Bot(start_pos)
  {
    const double heading = (static_cast<double>(rand()) / RAND_MAX) * 2.0 * M_PI;
    random_direction = Direction(cos(heading), sin(heading));
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

  void take_lidar_readings()
  {
    Bot::take_lidar_readings();

    find_cw_disconnect_reading();
    find_ccw_disconnect_reading();

    return;
  }

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

  // --- Exploration Methods ---
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
      real_first_contact_point = get_real_position();
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

  // --- Drawing Methods ---
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

  // --- Utility Methods ---
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
};

#endif