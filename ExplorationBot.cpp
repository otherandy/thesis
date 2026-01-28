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

// --- Environment Setup ---
Polygon ENVIRONMENT;
const double LIDAR_RADIUS = 1.5;
const double LIDAR_RESOLUTION = LIDAR_RADIUS / 100.0;
const double SPEED = 0.05;
const double EXPLORATION_RADIUS = 10.0;
const Point START_POSITION(1.0, 1.0);
const double WINDOW_SCALE = 50.0;
const double WINDOW_OFFSET_X = 50.0;
const double WINDOW_OFFSET_Y = 50.0;

// --- Utility Functions ---
double compute_angle_to_point(const Point &from, const Point &to)
{
  return atan2(to.y() - from.y(), to.x() - from.x());
}

bool point_in_environment(const Point &p)
{
  return ENVIRONMENT.bounded_side(p) == CGAL::ON_BOUNDED_SIDE;
}

// --- Wall Point Extraction ---
std::vector<Point> extract_wall_points(const std::vector<Point> &lidar_points, double distance_threshold = 0.15)
{
  std::vector<Point> wall_points;
  size_t i = 0;

  while (i < lidar_points.size())
  {
    std::vector<Point> cluster;
    cluster.push_back(lidar_points[i]);
    size_t j = i + 1;

    while (j < lidar_points.size() && std::sqrt(CGAL::squared_distance(lidar_points[i], lidar_points[j])) < distance_threshold)
    {
      cluster.push_back(lidar_points[j]);
      ++j;
    }

    double cx = 0, cy = 0;
    for (const auto &p : cluster)
    {
      cx += p.x();
      cy += p.y();
    }

    cx /= cluster.size();
    cy /= cluster.size();
    wall_points.push_back(Point(cx, cy));
    i = j;
  }
  return wall_points;
}

class ExplorationBot
{
public:
  Point position;
  double heading;

  std::set<Point> wall_points;
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

      for (double step = 0; step <= LIDAR_RADIUS; step += LIDAR_RESOLUTION)
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
    double new_x = position.x() + distance * cos(heading);
    double new_y = position.y() + distance * sin(heading);
    visited_positions.push_back(position);
    position = Point(new_x, new_y);
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
        if (std::sqrt(CGAL::squared_distance(position, p)) < LIDAR_RADIUS - 0.1)
        {
          std::cout << "Wall detected at position (" << position.x() << ", " << position.y() << ")\n";
          std::vector<Point> wall_pts = extract_wall_points(lidar_points);
          for (const auto &wp : wall_pts)
            wall_points.insert(wp);
          first_contact_pos = position;
          first_contact_heading = heading;
          return true;
        }
      }
      move_forward(SPEED);
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
};

class PlayerBot
{
public:
  Point position;

  PlayerBot(const Point &start_pos)
      : position(start_pos)
  {
  }

  void get_input_and_move()
  {
    if (IsKeyDown(KEY_W))
      position = Point(position.x(), position.y() - SPEED);
    if (IsKeyDown(KEY_S))
      position = Point(position.x(), position.y() + SPEED);
    if (IsKeyDown(KEY_A))
      position = Point(position.x() - SPEED, position.y());
    if (IsKeyDown(KEY_D))
      position = Point(position.x() + SPEED, position.y());
  }

  void draw()
  {
    // Bot body
    DrawCircle(position.x() * WINDOW_SCALE + WINDOW_OFFSET_X, position.y() * WINDOW_SCALE + WINDOW_OFFSET_Y, 5, RED);

    // LIDAR radius
    DrawCircleLines(position.x() * WINDOW_SCALE + WINDOW_OFFSET_X, position.y() * WINDOW_SCALE + WINDOW_OFFSET_Y, LIDAR_RADIUS * WINDOW_SCALE, BLUE);
  }
};

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
  // Define environment
  ENVIRONMENT.push_back(Point(0, 0));
  ENVIRONMENT.push_back(Point(8, 0));
  ENVIRONMENT.push_back(Point(8, 6));
  ENVIRONMENT.push_back(Point(12, 6));
  ENVIRONMENT.push_back(Point(12, 12));
  ENVIRONMENT.push_back(Point(4, 12));
  ENVIRONMENT.push_back(Point(4, 6));
  ENVIRONMENT.push_back(Point(0, 6));

  int screenWidth = 800;
  int screenHeight = 800;

  raylib::Window window(screenWidth, screenHeight, "Exploration Bot Simulation");
  SetTargetFPS(60);

  // Start bot
  // ExplorationBot bot(START_POSITION);
  // bot.run_exploration();

  PlayerBot player(START_POSITION);

  while (!window.ShouldClose())
  {
    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    // Draw environment
    for (size_t i = 0; i < ENVIRONMENT.size(); ++i)
    {
      Point p1 = ENVIRONMENT[i];
      Point p2 = ENVIRONMENT[(i + 1) % ENVIRONMENT.size()];
      DrawLine(p1.x() * WINDOW_SCALE + WINDOW_OFFSET_X, p1.y() * WINDOW_SCALE + WINDOW_OFFSET_Y, p2.x() * WINDOW_SCALE + WINDOW_OFFSET_X, p2.y() * WINDOW_SCALE + WINDOW_OFFSET_Y, BLACK);
    }

    // Draw bot path
    /*
    for (size_t i = 1; i < bot.visited_positions.size(); ++i)
    {
      Point p1 = bot.visited_positions[i - 1];
      Point p2 = bot.visited_positions[i];
      window.DrawLine(p1.x() * 50, p1.y() * 50, p2.x() * 50, p2.y() * 50, RED);
    }
    */

    player.get_input_and_move();
    player.draw();

    window.EndDrawing();
  }

  // Save results
  // visited_to_file("out/visited_positions.csv", bot.visited_positions);
  // std::cout << "Saved: visited_positions.csv\n";
  return 0;
}
