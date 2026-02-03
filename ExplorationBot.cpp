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
const double EXPLORATION_RADIUS = 10.0;
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
  bool playerControlling = true;

  const double lidar_radius = 1.5;
  const double lidar_resolution = lidar_radius / 100.0;
  const double lidar_reading_threshold = 0.1;
  const double speed = 0.05;

  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::vector<Point> visited_positions;

  Point first_contact_pos;

  ExplorationBot(const Point &start_pos)
      : position(start_pos)
  {
    visited_positions.push_back(start_pos);
  }

  void take_lidar_readings(int num_samples = MAX_LIDAR_SAMPLES)
  {
    current_readings.fill({0, 0});

    for (int i = 0; i < num_samples; ++i)
    {
      double angle = 2 * M_PI * i / num_samples;
      double distance;

      for (distance = lidar_resolution; distance < lidar_radius; distance += lidar_resolution)
      {
        double sample_x = position.x() + distance * cos(angle);
        double sample_y = position.y() + distance * sin(angle);

        if (!point_in_environment(Point(sample_x, sample_y)))
          break;
      }

      current_readings[i] = Reading{angle, distance};
    }

    return;
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
    if (!playerControlling)
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

  bool
  phase1_initial_wall_discovery()
  {
    std::cout << "=== Phase 1: Initial Wall Discovery ===\n";
    const double heading = ((double)rand() / RAND_MAX) * 2 * M_PI;

    while (std::sqrt(CGAL::squared_distance(START_POSITION, position)) < EXPLORATION_RADIUS)
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

  void draw_readings(float scale_factor)
  {
    float pos = lidar_radius * scale_factor + WINDOW_PADDING;

    for (const auto &r : current_readings)
    {
      float end_x = pos + r.distance * scale_factor * cos(r.angle);
      float end_y = pos + r.distance * scale_factor * sin(r.angle);

      DrawLine(pos,
               pos,
               end_x, end_y,
               GRAY);
    }
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

    bot.draw_readings(scale_factor);
    bot.draw(scale_factor, offset);

    window.EndDrawing();
  }

  // Save results
  // visited_to_file("out/visited_positions.csv", bot.visited_positions);
  // std::cout << "Saved: visited_positions.csv\n";

  CloseWindow();
  return 0;
}
