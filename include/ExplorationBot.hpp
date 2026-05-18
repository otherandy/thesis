#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "FrontierRegion.hpp"
#include "OccupationGrid.hpp"
#include "cgal_types.hpp"

#define NEXT_INDEX 1
#define PREV_INDEX -1

const double DESIRED_WALL_DISTANCE = 0.2;
const double WALL_DISTANCE_STRENGTH = 0.9;
constexpr double READING_ANGLE_SPAN = 0.02;
constexpr int READING_OFFSET = (MAX_LIDAR_SAMPLES * READING_ANGLE_SPAN) / 2 - 1;
constexpr std::size_t WALL_POINT_COUNT = 2 * READING_OFFSET + 1;

enum class ExplorationPhase {
  Idle,
  WallDiscovery,
  WallAlignment,
  WallFollowing,
  RegionDiscovery,
  RegionAlignment,
  RegionExploration,
  Completed
};

struct ExplorationData {
  Vector random_direction;
  Point start_point;
  Point first_wall_point;
  Point target_point;
  std::size_t last_closest_reading;
  bool left_wall_point;
};

class ExplorationBot : public Bot {
private:
  Point relative_position = Point(0.0, 0.0);
  ExplorationPhase exploration_phase = ExplorationPhase::Idle;
  std::shared_ptr<OccupationGrid> exploration_grid;

  ExplorationData exploration_data;

  Graph frontier_region_graph;
  std::shared_ptr<StepDFS> traversal_dfs;

  std::vector<FrontierRegion> frontier_regions;
  std::size_t current_frontier_region_id = 0;

  bool is_paused = false;

  void get_input_and_move();
  void reset();
  void move(const Vector &dir);
  void run_exploration();
  void phase1_wall_discovery();
  void phase2_wall_alignment();
  Vector compute_wall_following_vector();
  void phase3_wall_following();
  void phase4_region_discovery();
  void phase5_region_alignment();
  void phase6_region_exploration();
  void draw_target_point(DrawData draw_data) const;

public:
  ExplorationBot(const Point &start_pos);
  void update();
  void draw(DrawData draw_data) const;
  void grid_to_file(const std::string &filename) const;
};

#endif
