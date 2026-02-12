#ifndef EXPLORATION_BOT_H
#define EXPLORATION_BOT_H

#include <CGAL/linear_least_squares_fitting_2.h>
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
const double DESIRED_WALL_DISTANCE = 0.15;
const double READING_ANGLE_SPAN = 0.02;
const int READING_OFFSET = (MAX_LIDAR_SAMPLES * READING_ANGLE_SPAN) / 2 - 1;

// --- Classes ---
class ExplorationBot : public Bot
{
private:
  Point relative_position = Point(0.0, 0.0); // exploration and path tracking

  std::vector<Point> real_visited_positions;

  ExplorationPhase exploration_phase = ExplorationPhase::Idle;

  Vector random_direction;
  Point exploration_start_point = Point(0.0, 0.0);
  Point relative_first_contact_point = Point(-1, -1);
  Point real_first_contact_point = Point(-1, -1);
  bool left_first_contact = false;
  Vector current_follow_vector;

protected:
  void move(const Vector &dir)
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
    const double heading = (rand() / RAND_MAX) * 2.0 * M_PI;
    random_direction = Vector(cos(heading), sin(heading));
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

    if (IsKeyDown(KEY_UP))
    {
      move(NORTH);
    }
    if (IsKeyDown(KEY_DOWN))
    {
      move(SOUTH);
    }
    if (IsKeyDown(KEY_LEFT))
    {
      move(WEST);
    }
    if (IsKeyDown(KEY_RIGHT))
    {
      move(EAST);
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
    const Reading &closest_reading = current_readings[closest_wall_reading_index];

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

    move(to_wall);
  }

  void phase3_wall_following()
  {
    Vector wall_vector = calculate_wall_correction_vector();
    Vector desired_vector = current_follow_vector +
                            wall_vector;

    move(desired_vector);

    if (std::sqrt(CGAL::squared_distance(relative_position, relative_first_contact_point)) <= speed)
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

    std::array<Point, 2 * READING_OFFSET + 1> wall_points;

    for (int offset = -READING_OFFSET; offset <= READING_OFFSET; ++offset)
    {
      int idx = relative_index(closest_wall_reading_index, offset);
      wall_points[offset + READING_OFFSET] = reading_index_to_point(idx);
    }

    CGAL::Line_2<Kernel> fitted_line;
    CGAL::linear_least_squares_fitting_2(
        wall_points.begin(),
        wall_points.end(),
        fitted_line,
        CGAL::Dimension_tag<0>());

    int before_idx = relative_index(closest_wall_reading_index, -1);
    int after_idx = relative_index(closest_wall_reading_index, 1);
    Point before_point = reading_index_to_point(before_idx);
    Point after_point = reading_index_to_point(after_idx);
    Vector direction_hint = after_point - before_point;

    Vector fitted_vector = fitted_line.to_vector();
    if (fitted_vector * direction_hint < 0)
      fitted_vector = -fitted_vector;

    current_follow_vector = fitted_vector;
  }

  Vector calculate_wall_correction_vector()
  {
    if (closest_wall_reading_index == -1)
      return Vector(0, 0);

    const Reading &r = current_readings[closest_wall_reading_index];
    double distance_error = r.distance - DESIRED_WALL_DISTANCE;

    Vector to_wall = Vector(cos(r.angle), sin(r.angle));

    return to_wall * distance_error;
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

  void draw_follow_vector(float scale_factor, const raylib::Vector2 &offset)
  {
    if (closest_wall_reading_index == -1)
      return;

    Point pos = get_real_position();
    DrawLine(pos.x() * scale_factor + offset.x,
             pos.y() * scale_factor + offset.y,
             (pos.x() + current_follow_vector.x()) * scale_factor + offset.x,
             (pos.y() + current_follow_vector.y()) * scale_factor + offset.y,
             GREEN);
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