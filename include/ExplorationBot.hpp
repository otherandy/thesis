#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "OccupationGrid.hpp"
#include "cgal_types.hpp"

#define NEXT_INDEX 1
#define PREV_INDEX -1

const double DESIRED_WALL_DISTANCE = 0.2;
const double WALL_DISTANCE_STRENGTH = 0.9;
constexpr double READING_ANGLE_SPAN = 0.02;
constexpr int READING_OFFSET = (MAX_LIDAR_SAMPLES * READING_ANGLE_SPAN) / 2 - 1;
constexpr std::size_t WALL_POINT_COUNT = 2 * READING_OFFSET + 1;

enum class ExplorationPhase
{
  Idle,
  WallDiscovery,
  WallAlignment,
  WallFollowing,
  RegionDiscovery,
  RegionAlignment,
  RegionExploration,
  ReturnToStart,
  Completed
};

class ExplorationBot : public Bot
{
private:
  Point relative_position = Point(0.0, 0.0);
  ExplorationPhase exploration_phase = ExplorationPhase::Idle;
  OccupationGrid exploration_grid;

  Point exploration_start_point;
  Vector random_direction;
  Vector current_follow_vector;
  Point exploration_end_point;

  // bool graph_initialized = false;
  // Graph frontier_graph;
  // incremental_dfs frontier_dfs{frontier_graph, 1};
  // Vertex current_region_vertex;

  Point target_point = Point(0, 0);
  Point start_point = Point(0, 0);
  int current_frontier_idx = -1;

  std::vector<Point> current_region_path;
  int current_region_path_index;

  bool is_paused = false;

  void get_input_and_move();
  void reset();
  void move(const Vector &dir);
  void run_exploration();
  void phase1_wall_discovery();
  void phase2_wall_alignment();
  void phase3_wall_following();
  void create_follow_vector();
  inline Vector calculate_wall_correction_vector() const;
  void phase4_region_discovery();
  void phase5_region_alignment();
  void phase6_region_exploration();
  void phase7_return_to_start();
  void draw_follow_vector(float scale_factor,
                          float offset_x, float offset_y) const;
  void draw_target_point(float scale_factor,
                         float offset_x, float offset_y) const;

public:
  ExplorationBot(const Point &start_pos);
  void update();
  void draw(float scale_factor, float offset_x, float offset_y) const;
  void grid_to_file(const std::string &filename) const;
};
#endif