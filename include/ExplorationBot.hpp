#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "OccupationGrid.hpp"

const double DESIRED_WALL_DISTANCE = SPEED * 2;
const double LIDAR_DISTANCE_THRESHOLD = SPEED;

enum class ExplorationPhase {
  WallDiscovery,
  WallAlignment,
  WallFollowing,
  Idle,
  RegionDiscovery,
  RegionAlignment,
  RegionExploration,
  Complete
};

class ExplorationBot : public Bot {
public:
  const bool clockwise_following;

  ExplorationBot(const Robot::Point &start_pos, const Robot::Vector &start_dir,
                 bool clockwise);

  ExplorationPhase phase = ExplorationPhase::WallDiscovery;

  bool left_contact_point;

  vertex_t target_vertex;
  Robot::Point target_point;

  bool started_surround;
  double goal_distance;

  Robot::Point get_relative_position(const OccupationGrid *grid) const;

  Robot::Vector move(const Robot::Vector &dir);
  void reset();
  void update_grid(OccupationGrid *grid);

  void explore(const OccupationGrid *grid);

  void phase1_wall_discovery();
  void phase2_wall_alignment(const OccupationGrid *grid);
  void phase3_wall_following(const OccupationGrid *grid);
  void phase5_region_alignment(const OccupationGrid *grid);
  void phase6_region_exploration(const OccupationGrid *grid);

  void draw(const DrawData &draw_data) const;

private:
  const Robot::Point start_point;
  const Robot::Vector start_direction;

  Robot::Vector direction;
  Robot::Point contact_point;

  Robot::Vector compute_wall_following_vector(const OccupationGrid *grid);
  bool path_blocked_to(const Robot::Vector &target) const;

  void draw_direction(const DrawData &draw_data) const;
};

#endif
