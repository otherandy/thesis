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
const double EXPLORATION_RADIUS = 10.0;
const float WINDOW_PADDING = 10.0f;

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
  double heading;
  bool playerControlling = true;

  const double lidar_radius = 1.5;
  const double lidar_resolution = lidar_radius / 100.0;
  const double speed = 0.05;

  std::vector<Point> visited_positions;

  Point first_contact_pos;
  double first_contact_heading;

  ExplorationBot(const Point &start_pos)
      : position(start_pos)
  {
    visited_positions.push_back(start_pos);
  }

  std::vector<Point> take_lidar_reading(int num_samples = 360)
  {
    std::vector<Point> points;
    for (int i = 0; i < num_samples; ++i)
    {
      double angle = 2 * M_PI * i / num_samples;
      Point sample_point = position;

      for (double step = 0; step <= lidar_radius; step += lidar_resolution)
      {
        sample_point = Point(position.x() + step * cos(angle), position.y() + step * sin(angle));
        if (!point_in_environment(sample_point))
          break;
      }

      points.push_back(sample_point);
    }

    return points;
  }

  void move_forward(double distance)
  {
    // visited_positions.push_back(position);
    double new_x = position.x() + distance * cos(heading);
    double new_y = position.y() + distance * sin(heading);

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
      move_forward(speed);
    }
    if (IsKeyDown(KEY_S))
    {
      move_forward(-speed);
    }
    if (IsKeyDown(KEY_A))
    {
      heading -= 0.05;
    }
    if (IsKeyDown(KEY_D))
    {
      heading += 0.05;
    }
  }

  bool phase1_initial_wall_discovery()
  {
    std::cout << "=== Phase 1: Initial Wall Discovery ===\n";
    heading = ((double)rand() / RAND_MAX) * 2 * M_PI;

    while (std::sqrt(CGAL::squared_distance(START_POSITION, position)) < EXPLORATION_RADIUS)
    {
      std::vector<Point> lidar_points = take_lidar_reading();
      for (const auto &p : lidar_points)
      {
        if (std::sqrt(CGAL::squared_distance(position, p)) < lidar_radius - 0.1)
        {
          std::cout << "Wall detected at position (" << position.x() << ", " << position.y() << ")\n";
          first_contact_pos = position;
          first_contact_heading = heading;
          return true;
        }
      }
      move_forward(speed);
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

    // Heading line
    DrawLine(position.x() * scale_factor + offset.x,
             position.y() * scale_factor + offset.y,
             (position.x() + cos(heading) * 0.5) * scale_factor + offset.x,
             (position.y() + sin(heading) * 0.5) * scale_factor + offset.y,
             BLACK);
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

    bot.draw(scale_factor, offset);

    window.EndDrawing();
  }

  // Save results
  // visited_to_file("out/visited_positions.csv", bot.visited_positions);
  // std::cout << "Saved: visited_positions.csv\n";

  CloseWindow();
  return 0;
}
