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
const Point ENV_POINTS[] = {Point(0, 0),
                            Point(8, 0),
                            Point(8, 6),
                            Point(12, 6),
                            Point(12, 12),
                            Point(4, 12),
                            Point(4, 6),
                            Point(0, 6)};

const Point SQUARE_ENV_POINTS[] = {Point(0, 0),
                                   Point(10, 0),
                                   Point(10, 10),
                                   Point(0, 10)};

const Point TRIANGLE_ENV_POINTS[] = {Point(0, 0),
                                     Point(10, 0),
                                     Point(5, 8)};

const Polygon ENVIRONMENT(ENV_POINTS,
                          ENV_POINTS + sizeof(ENV_POINTS) / sizeof(ENV_POINTS[0]));

const float ENV_WIDTH = ENVIRONMENT.bbox().xmax() -
                        ENVIRONMENT.bbox().xmin();
const float ENV_HEIGHT = ENVIRONMENT.bbox().ymax() -
                         ENVIRONMENT.bbox().ymin();

const double EXPLORATION_RADIUS = std::sqrt(
                                      std::pow(
                                          ENVIRONMENT.bbox().xmax() - ENVIRONMENT.bbox().xmin(), 2) +
                                      std::pow(
                                          ENVIRONMENT.bbox().ymax() - ENVIRONMENT.bbox().ymin(), 2)) /
                                  2.0;

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const float WINDOW_PADDING = 10.0f;
const int FRAME_RATE = 60;

const Point START_POSITION(1.0, 1.0);
const int MAX_LIDAR_SAMPLES = 360 / 8;
const double LIDAR_RADIUS = 1.5;
const double LIDAR_RESOLUTION = LIDAR_RADIUS / 100.0;
const double LIDAR_READING_THRESHOLD = 0.1;
const int LIDAR_EPSILON = 2;

const Direction NORTH(0, -1);
const Direction SOUTH(0, 1);
const Direction WEST(-1, 0);
const Direction EAST(1, 0);

// --- Utility Functions ---
inline double compute_angle_to_point(const Point &from, const Point &to)
{
  return atan2(to.y() - from.y(), to.x() - from.x());
}

inline bool point_in_environment(const Point &p)
{
  return ENVIRONMENT.bounded_side(p) == CGAL::ON_BOUNDED_SIDE;
}

inline Point point_at_reading(const Point &origin, const Reading &r)
{
  return Point(origin.x() + r.distance * cos(r.angle),
               origin.y() + r.distance * sin(r.angle));
}

inline Vector normalize_vector(const Vector &v)
{
  double length = std::sqrt(v.squared_length());
  return Vector(v.x() / length, v.y() / length);
}

// --- Classes ---
class ExplorationBot
{
private:
  Point real_position;
  Point relative_position = Point(0.0, 0.0);
  bool draw_as_hud = true;

  double speed = 0.2;

  int exploration_phase = 0;
  bool left_first_contact = false;

  std::array<Reading, MAX_LIDAR_SAMPLES> current_readings;
  std::vector<Point> visited_positions;

  Direction random_direction;
  Point real_first_contact_point = Point(-1, -1);
  Point relative_first_contact_point = Point(-1, -1);

  int closest_wall_reading_index = -1;
  int cw_disconnect_reading_index = -1;
  int ccw_disconnect_reading_index = -1;
  Vector current_follow_vector;

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

  inline Point reading_index_to_point(int index)
  {
    const Reading r = current_readings[index];
    return point_at_reading(real_position, r);
  }

  void move(const Direction &dir)
  {
    Vector delta = normalize_vector(dir.to_vector()) * speed;
    Point new_position = real_position + delta;

    if (!point_in_environment(new_position))
      return;

    if (exploration_phase != 0)
      visited_positions.push_back(real_position);

    real_position = new_position;
    relative_position = relative_position + delta;
  }

  void draw_specific_reading(int index, Vector2 &pos, float scale_factor, const Vector2 &offset, Color color)
  {
    if (index == -1)
      return;

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

    const double heading = ((double)rand() / RAND_MAX) * 2 * M_PI;
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

      if (closest_wall_reading_index == -1 ||
          distance < current_readings[closest_wall_reading_index].distance)
      {
        closest_wall_reading_index = i;
      }

      current_readings[i] = Reading{angle, distance};
    }

    return;
  }

  void find_disconnect_readings()
  {
    find_cw_disconnect_reading();
    find_ccw_disconnect_reading();
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

  void get_input_and_move()
  {
    if (IsKeyPressed(KEY_R))
    {
      draw_as_hud = !draw_as_hud;
    }

    if (exploration_phase != 0)
      return;

    if (IsKeyPressed(KEY_SPACE))
    {
      exploration_phase = 1;
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
    for (const auto &v : visited_positions)
    {
      f << v.x() << "," << v.y() << "\n";
    }
    f.close();
    std::cout << "BOT: Saved visited positions to " << filename << std::endl;
  }

  // --- Exploration Methods ---
  void phase1_wall_discovery()
  {
    if (std::sqrt(CGAL::squared_distance(relative_position, Point(0, 0))) >= EXPLORATION_RADIUS)
      return;

    for (const auto &r : current_readings)
    {
      if (r.distance < LIDAR_RADIUS - LIDAR_READING_THRESHOLD)
      {
        std::cout << "EXPLORATION: Wall detected at position ("
                  << relative_position.x() << ", " << relative_position.y() << ")\n";
        real_first_contact_point = real_position;
        relative_first_contact_point = relative_position;
        exploration_phase = 2;
        return;
      }
    }
    move(random_direction);
  }

  void phase2_wall_following()
  {
    create_follow_vector();
    move(current_follow_vector.direction());

    if (std::sqrt(CGAL::squared_distance(relative_first_contact_point, relative_position)) <=
        LIDAR_READING_THRESHOLD + speed * 2)
    {
      if (!left_first_contact)
        return;

      std::cout << "EXPLORATION: Completed wall following loop.\n";
      exploration_phase = 3;
      return;
    }

    left_first_contact = true;
  }

  void run_exploration()
  {
    if (exploration_phase == 0)
      return;

    if (exploration_phase == 1)
    {
      phase1_wall_discovery();
      return;
    }

    if (exploration_phase == 2)
    {
      phase2_wall_following();
      return;
    }
  }

  // --- Drawing Methods ---
  void draw_readings(float scale_factor, const Vector2 &offset)
  {
    Vector2 pos;

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

      DrawLine(pos.x,
               pos.y,
               end_x, end_y,
               GRAY);
    }

    draw_specific_reading(closest_wall_reading_index, pos, scale_factor, offset, RED);
    draw_specific_reading(cw_disconnect_reading_index, pos, scale_factor, offset, ORANGE);
    draw_specific_reading(ccw_disconnect_reading_index, pos, scale_factor, offset, ORANGE);
  }

  void draw(float scale_factor, const Vector2 &offset)
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

    // LIDAR threshold
    DrawCircleLines(real_position.x() * scale_factor + offset.x,
                    real_position.y() * scale_factor + offset.y,
                    (LIDAR_RADIUS - LIDAR_READING_THRESHOLD) * scale_factor,
                    BLUE);
  }

  void draw_follow_vector(float scale_factor, const Vector2 &offset)
  {
    Point end_point = real_position + normalize_vector(current_follow_vector);

    DrawLine(real_position.x() * scale_factor + offset.x,
             real_position.y() * scale_factor + offset.y,
             end_point.x() * scale_factor + offset.x,
             end_point.y() * scale_factor + offset.y,
             GREEN);
  }

  void draw_first_contact_point(float scale_factor, const Vector2 &offset)
  {
    if (exploration_phase < 2)
      return;

    DrawCircle(real_first_contact_point.x() * scale_factor + offset.x,
               real_first_contact_point.y() * scale_factor + offset.y,
               5,
               PURPLE);
  }

  void draw_path(float scale_factor, const Vector2 &offset)
  {
    if (visited_positions.size() < 2)
      return;

    for (size_t i = 1; i < visited_positions.size(); ++i)
    {
      Point p1 = visited_positions[i - 1];
      Point p2 = visited_positions[i];
      DrawLine(p1.x() * scale_factor + offset.x,
               p1.y() * scale_factor + offset.y,
               p2.x() * scale_factor + offset.x,
               p2.y() * scale_factor + offset.y,
               RED);
    }
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

inline float calculate_scale_factor(const raylib::Window &window)
{
  const float padded_width = window.GetWidth() - 2.0f * WINDOW_PADDING;
  const float padded_height = window.GetHeight() - 2.0f * WINDOW_PADDING;

  return std::min(padded_width / ENV_WIDTH,
                  padded_height / ENV_HEIGHT);
}

inline Vector2 calculate_offset(const raylib::Window &window, float scale_factor)
{
  const float draw_width = ENV_WIDTH * scale_factor;
  const float draw_height = ENV_HEIGHT * scale_factor;

  return Vector2{
      (window.GetWidth() - draw_width) * 0.5f,
      (window.GetHeight() - draw_height) * 0.5f};
}

int main()
{
  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT,
                        "Exploration Bot Simulation");
  SetTargetFPS(FRAME_RATE);

  ExplorationBot bot(START_POSITION);

  while (!window.ShouldClose())
  {
    // -- Update --
    bot.get_input_and_move();

    bot.take_lidar_readings();
    bot.find_disconnect_readings();

    bot.run_exploration();

    // -- Draw --
    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float scale_factor = calculate_scale_factor(window);
    const Vector2 offset = calculate_offset(window, scale_factor);

    draw_environment(scale_factor, offset);

    bot.draw_first_contact_point(scale_factor, offset);
    bot.draw_path(scale_factor, offset);
    bot.draw_readings(scale_factor, offset);
    bot.draw(scale_factor, offset);
    bot.draw_follow_vector(scale_factor, offset);

    window.EndDrawing();
  }

  // bot.visited_to_file("Testing/visited_positions.csv");

  CloseWindow();
  return 0;
}
