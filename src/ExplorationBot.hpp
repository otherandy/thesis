#ifndef EXPLORATION_BOT_H
#define EXPLORATION_BOT_H

#include <CGAL/linear_least_squares_fitting_2.h>
#include <iostream>
#include "OccupationGrid.hpp"

#define NEXT_INDEX 1
#define PREV_INDEX -1

using Polygon = CGAL::Polygon_2<Kernel>;
using Direction = Kernel::Direction_2;

const double DESIRED_WALL_DISTANCE = 0.2;
const double WALL_DISTANCE_STRENGTH = 0.9;
constexpr double READING_ANGLE_SPAN = 0.02;
constexpr int READING_OFFSET = (MAX_LIDAR_SAMPLES * READING_ANGLE_SPAN) / 2 - 1;
constexpr std::size_t WALL_POINT_COUNT = 2 * READING_OFFSET + 1;

bool is_paused = false;

const Vector NORTH(0, -1);
const Vector SOUTH(0, 1);
const Vector EAST(1, 0);
const Vector WEST(-1, 0);

enum class ExplorationPhase
{
  Idle,
  WallDiscovery,
  WallAlignment,
  WallFollowing,
  Completed
};

class ExplorationBot : public Bot
{
private:
  Point relative_position = Point(0.0, 0.0);
  ExplorationPhase exploration_phase = ExplorationPhase::Idle;
  OccupationGrid exploration_grid;
  std::vector<std::pair<double, double>> current_discontinuities;

  Vector random_direction;
  Point exploration_start_point;
  Vector current_follow_vector;
  std::array<Point, WALL_POINT_COUNT> wall_points;

  bool is_paused = false;

  void get_input_and_move()
  {
    if (IsKeyPressed(KEY_R))
    {
      draw_as_hud = !draw_as_hud;
    }

    if (IsKeyPressed(KEY_T))
    {
      reset();
      return;
    }

    if (IsKeyPressed(KEY_P))
    {
      is_paused = !is_paused;
      return;
    }

    if (exploration_phase != ExplorationPhase::Idle)
    {
      return;
    }

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

  void reset()
  {
    relative_position = Point(0.0, 0.0);
    exploration_phase = ExplorationPhase::Idle;
    exploration_grid = OccupationGrid(START_POSITION);
    is_paused = false;

    Bot::reset();
  }

  void move(const Vector &dir)
  {
    relative_position = relative_position + Bot::move(dir);

    if (exploration_phase != ExplorationPhase::Idle)
    {
      Bot::update_visited_positions();
      exploration_grid.mark_cells(relative_position,
                                  current_readings);
    }
  }

  void run_exploration()
  {
    if (exploration_phase == ExplorationPhase::Idle ||
        exploration_phase == ExplorationPhase::Completed ||
        is_paused)
    {
      return;
    }

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
    {
      return;
    }

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
    const Vector wall_vector = calculate_wall_correction_vector();
    const Vector desired_vector = current_follow_vector *
                                      (1.0 - WALL_DISTANCE_STRENGTH) +
                                  wall_vector * WALL_DISTANCE_STRENGTH;

    move(desired_vector);

    if (!exploration_grid.was_frontier_added())
    {
      std::cout << "EXPLORATION: Completed wall following loop.\n";
      exploration_phase = ExplorationPhase::Completed;
      return;
    }
  }

  void create_follow_vector()
  {
    if (closest_wall_reading_index == INVALID_INDEX)
    {
      return;
    }

    for (int offset = -READING_OFFSET; offset <= READING_OFFSET; ++offset)
    {
      const int idx = relative_index(closest_wall_reading_index, offset);
      wall_points[offset + READING_OFFSET] = reading_index_to_point(idx);
    }

    CGAL::Line_2<Kernel> fitted_line;
    CGAL::linear_least_squares_fitting_2(
        wall_points.begin(),
        wall_points.end(),
        fitted_line,
        CGAL::Dimension_tag<0>());

    const int before_idx = relative_index(closest_wall_reading_index, PREV_INDEX);
    const int after_idx = relative_index(closest_wall_reading_index, NEXT_INDEX);
    const Point before_point = reading_index_to_point(before_idx);
    const Point after_point = reading_index_to_point(after_idx);
    const Vector direction_hint = after_point - before_point;

    Vector fitted_vector = fitted_line.to_vector();
    if (fitted_vector * direction_hint < 0)
    {
      fitted_vector = -fitted_vector;
    }

    current_follow_vector = fitted_vector;
  }

  inline Vector calculate_wall_correction_vector() const
  {
    if (closest_wall_reading_index == INVALID_INDEX)
    {
      return Vector(0, 0);
    }

    const Reading &r = current_readings[closest_wall_reading_index];
    const double distance_error = r.distance - DESIRED_WALL_DISTANCE;
    const Vector to_wall = Vector(cos(r.angle), sin(r.angle));

    return to_wall * distance_error;
  }

  void draw_follow_vector(float scale_factor, float offset_x, float offset_y) const
  {
    if (closest_wall_reading_index == INVALID_INDEX)
    {
      return;
    }

    const Point &pos = get_real_position();

    DrawLine(pos.x() * scale_factor + offset_x,
             pos.y() * scale_factor + offset_y,
             (pos.x() + current_follow_vector.x()) * scale_factor + offset_x,
             (pos.y() + current_follow_vector.y()) * scale_factor + offset_y,
             GREEN);
  }

public:
  ExplorationBot(const Point &start_pos)
      : Bot(start_pos), exploration_grid(start_pos)
  {
    const double heading = (rand() / RAND_MAX) * 2.0 * M_PI;
    random_direction = Vector(cos(heading), sin(heading));
  }

  void update()
  {
    get_input_and_move();
    take_lidar_readings();
    current_discontinuities = get_reading_discontinuities(current_readings);
    create_follow_vector();
    run_exploration();
  }

  void draw(float scale_factor, float offset_x, float offset_y) const
  {
    exploration_grid.draw(scale_factor, offset_x, offset_y);
    draw_path(scale_factor, offset_x, offset_y);
    draw_readings(scale_factor, offset_x, offset_y);
    draw_discontinuities(current_discontinuities, scale_factor, offset_x, offset_y);
    draw_body(scale_factor, offset_x, offset_y);
    draw_lidar(scale_factor, offset_x, offset_y);
    draw_follow_vector(scale_factor, offset_x, offset_y);
  }

  void grid_to_file(const std::string &filename) const
  {
    exploration_grid.save_to_file(filename);
  }
};

#endif