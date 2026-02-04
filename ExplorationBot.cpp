#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <vector>
#include <set>
#include <map>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <raylib-cpp.hpp>

using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Polygon = CGAL::Polygon_2<Kernel>;
using Direction = Kernel::Direction_2;
using Vector = Kernel::Vector_2;

struct Reading
{
  double angle;
  double distance;
};

// --- Setup ---
Point ENV_POINTS[] = {Point(0, 0),
                      Point(8, 0),
                      Point(8, 6),
                      Point(12, 6),
                      Point(12, 12),
                      Point(4, 12),
                      Point(4, 6),
                      Point(0, 6)};
Polygon ENVIRONMENT(ENV_POINTS,
                    ENV_POINTS + sizeof(ENV_POINTS) / sizeof(ENV_POINTS[0]));
const Point START_POSITION(1.0, 1.0);
const int MAX_LIDAR_SAMPLES = 360 / 8;
const double INIT_EXPLORATION_RADIUS = 10.0;
const float WINDOW_PADDING = 10.0f;

const Direction NORTH(0, -1);
const Direction SOUTH(0, 1);
const Direction WEST(-1, 0);
const Direction EAST(1, 0);

// --- Utility Functions ---
double compute_angle_to_point(const Point &from, const Point &to)
{
  return atan2(to.y() - from.y(), to.x() - from.x());
}

bool point_in_environment(const Point &p)
{
  return ENVIRONMENT.bounded_side(p) == CGAL::ON_BOUNDED_SIDE;
}

// --- Classes ---
class ExplorationBot
{
public:
  Point position;
  bool player_controlling = true;
  bool draw_as_hud = true;

  const double lidar_radius = 1.5;
  const double lidar_resolution = lidar_radius / 100.0;
  const double lidar_reading_threshold = 0.1;
  const double speed = 0.05;

  Point first_contact_pos;
  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::vector<Point> visited_positions;

  int closest_wall_reading_index = -1;
  int clockwise_disconnect_reading_index = -1;
  Vector current_follow_vector;

  ExplorationBot(const Point &start_pos)
      : position(start_pos)
  {
    visited_positions.push_back(start_pos);
  }

  void take_lidar_readings(int num_samples = MAX_LIDAR_SAMPLES)
  {
    current_readings.fill({0, 0});
    closest_wall_reading_index = -1;

    for (int i = 0; i < num_samples; ++i)
    {
      double angle = 2 * M_PI * i / num_samples;
      double distance;

      for (distance = lidar_resolution; distance <= lidar_radius; distance += lidar_resolution)
      {
        double sample_x = position.x() + distance * cos(angle);
        double sample_y = position.y() + distance * sin(angle);

        if (!point_in_environment(Point(sample_x, sample_y)))
          break;
      }

      if (closest_wall_reading_index == -1 ||
          distance < current_readings[closest_wall_reading_index].distance)
      {
        closest_wall_reading_index = i;
      }

      current_readings[i] = Reading{angle, distance};
    }

    return;
  }

  void find_clockwise_disconnect_reading()
  {
    for (int i = closest_wall_reading_index; i >= 0; --i)
    {
      if (current_readings[i].distance >= lidar_radius)
      {
        clockwise_disconnect_reading_index = i;
        return;
      }
    }

    for (int i = MAX_LIDAR_SAMPLES - 1; i > closest_wall_reading_index; --i)
    {
      if (current_readings[i].distance >= lidar_radius)
      {
        clockwise_disconnect_reading_index = i;
        return;
      }
    }
  }

  void create_follow_vector()
  {
    // Closest wall reading to clockwise disconnect reading

    if (closest_wall_reading_index == -1 || clockwise_disconnect_reading_index == -1)
      return;

    const Reading closest_reading = current_readings[closest_wall_reading_index];
    const Reading clockwise_reading = current_readings[clockwise_disconnect_reading_index];

    const Point closest_point(position.x() + closest_reading.distance * cos(closest_reading.angle),
                              position.y() + closest_reading.distance * sin(closest_reading.angle));

    const Point clockwise_point(position.x() + clockwise_reading.distance * cos(clockwise_reading.angle),
                                position.y() + clockwise_reading.distance * sin(clockwise_reading.angle));

    current_follow_vector = clockwise_point - closest_point;
  }

  void move(const Direction &dir)
  {
    double delta_x = dir.dx() * speed;
    double delta_y = dir.dy() * speed;

    double new_x = position.x() + delta_x;
    double new_y = position.y() + delta_y;

    if (!point_in_environment(Point(new_x, new_y)))
      return;

    position = Point(new_x, new_y);
  }

  void get_input_and_move()
  {
    if (IsKeyDown(KEY_R))
    {
      draw_as_hud = !draw_as_hud;
    }

    if (!player_controlling)
      return;

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

  bool phase1_initial_wall_discovery()
  {
    std::cout << "=== Phase 1: Initial Wall Discovery ===\n";
    const double heading = ((double)rand() / RAND_MAX) * 2 * M_PI;

    while (std::sqrt(CGAL::squared_distance(START_POSITION, position)) < INIT_EXPLORATION_RADIUS)
    {
      take_lidar_readings();
      for (const auto &r : current_readings)
      {
        if (r.distance < lidar_radius - lidar_reading_threshold)
        {
          std::cout << "Wall detected at position (" << position.x() << ", " << position.y() << ")\n";
          first_contact_pos = position;
          return true;
        }
      }
      move(Direction(cos(heading), sin(heading)));
    }

    std::cout << "No wall found in exploration radius\n";
    return false;
  }

  bool run_exploration()
  {
    std::cout << "Starting exploration algorithm...\n";

    if (!phase1_initial_wall_discovery())
    {
      std::cout << "Failed at phase 1\n";
      return false;
    }

    std::cout << "\n=== Exploration Complete ===\n";
    return true;
  }

  void draw_readings(float scale_factor, const Vector2 &offset)
  {
    float pos_x = position.x() * scale_factor + offset.x;
    float pos_y = position.y() * scale_factor + offset.y;

    if (draw_as_hud)
    {
      pos_x = lidar_radius * scale_factor + WINDOW_PADDING;
      pos_y = lidar_radius * scale_factor + WINDOW_PADDING;
    }

    for (const auto &r : current_readings)
    {
      float end_x = pos_x + r.distance * scale_factor * cos(r.angle);
      float end_y = pos_y + r.distance * scale_factor * sin(r.angle);

      DrawLine(pos_x,
               pos_y,
               end_x, end_y,
               GRAY);
    }

    if (closest_wall_reading_index == -1)
      return;

    const Reading closest_reading = current_readings[closest_wall_reading_index];

    if (closest_reading.distance == lidar_radius)
      return;

    float closest_end_x = pos_x + closest_reading.distance * scale_factor * cos(closest_reading.angle);
    float closest_end_y = pos_y + closest_reading.distance * scale_factor * sin(closest_reading.angle);

    DrawCircle(closest_end_x,
               closest_end_y,
               5,
               RED);

    if (clockwise_disconnect_reading_index == -1)
      return;

    const Reading clockwise_reading = current_readings[clockwise_disconnect_reading_index];

    if (clockwise_reading.distance == lidar_radius)
      return;

    float clockwise_end_x = pos_x + clockwise_reading.distance * scale_factor * cos(clockwise_reading.angle);
    float clockwise_end_y = pos_y + clockwise_reading.distance * scale_factor * sin(clockwise_reading.angle);

    DrawCircle(clockwise_end_x,
               clockwise_end_y,
               5,
               ORANGE);
  }

  void draw(float scale_factor, const Vector2 &offset)
  {
    // Bot body
    DrawCircle(position.x() * scale_factor + offset.x,
               position.y() * scale_factor + offset.y,
               5,
               RED);

    // LIDAR radius
    DrawCircleLines(position.x() * scale_factor + offset.x,
                    position.y() * scale_factor + offset.y,
                    lidar_radius * scale_factor,
                    BLUE);
  }

  void draw_follow_vector(float scale_factor, const Vector2 &offset)
  {
    Point end_point = position + current_follow_vector;

    DrawLine(position.x() * scale_factor + offset.x,
             position.y() * scale_factor + offset.y,
             end_point.x() * scale_factor + offset.x,
             end_point.y() * scale_factor + offset.y,
             GREEN);
  }
};

// --- Drawing Functions ---
void draw_environment(float scale_factor, const Vector2 &offset)
{
  for (size_t i = 0; i < ENVIRONMENT.size(); ++i)
  {
    Point p1 = ENVIRONMENT[i];
    Point p2 = ENVIRONMENT[(i + 1) % ENVIRONMENT.size()];
    DrawLine(p1.x() * scale_factor + offset.x,
             p1.y() * scale_factor + offset.y,
             p2.x() * scale_factor + offset.x,
             p2.y() * scale_factor + offset.y,
             BLACK);
  }
}

// --- File Output ---
void visited_to_file(const std::string &filename, const std::vector<Point> &visited)
{
  std::ofstream f(filename);
  for (const auto &v : visited)
  {
    f << v.x() << "," << v.y() << "\n";
  }
}

int main()
{
  raylib::Window window(800, 600, "Exploration Bot Simulation");
  SetTargetFPS(60);

  const float env_width = ENVIRONMENT.bbox().xmax() - ENVIRONMENT.bbox().xmin();
  const float env_height = ENVIRONMENT.bbox().ymax() - ENVIRONMENT.bbox().ymin();

  ExplorationBot bot(START_POSITION);

  while (!window.ShouldClose())
  {
    // -- Update --
    bot.get_input_and_move();
    bot.take_lidar_readings();
    bot.find_clockwise_disconnect_reading();
    bot.create_follow_vector();

    // -- Draw --
    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float padded_width = window.GetWidth() - 2.0f * WINDOW_PADDING;
    const float padded_height = window.GetHeight() - 2.0f * WINDOW_PADDING;

    const float scale_factor = std::min(padded_width / env_width,
                                        padded_height / env_height);

    const float draw_width = env_width * scale_factor;
    const float draw_height = env_height * scale_factor;

    const Vector2 offset = {
        (window.GetWidth() - draw_width) * 0.5f,
        (window.GetHeight() - draw_height) * 0.5f};

    draw_environment(scale_factor, offset);

    // Draw bot path
    /*
    for (size_t i = 1; i < bot.visited_positions.size(); ++i)
    {
      Point p1 = bot.visited_positions[i - 1];
      Point p2 = bot.visited_positions[i];
      window.DrawLine(p1.x() * 50, p1.y() * 50, p2.x() * 50, p2.y() * 50, RED);
    }
    */

    bot.draw_readings(scale_factor, offset);
    bot.draw(scale_factor, offset);
    bot.draw_follow_vector(scale_factor, offset);

    window.EndDrawing();
  }

  // Save results
  // visited_to_file("out/visited_positions.csv", bot.visited_positions);
  // std::cout << "Saved: visited_positions.csv\n";

  CloseWindow();
  return 0;
}
