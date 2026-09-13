#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "OccupationGrid.hpp"
#include "Timer.hpp"

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
  EmergencyFind,
  Complete
};

class ExplorationBot : public Bot {
public:
  bool clockwise_following;

  ExplorationBot(std::size_t id, const Robot::Point &start_pos,
                 const Robot::Vector &start_dir, double radius, bool clockwise,
                 std::shared_ptr<Environment> env);

  ExplorationPhase phase = ExplorationPhase::WallDiscovery;

  const Robot::Point start_point;

  bool left_contact_point;
  vertex_t target_vertex = 0;
  Robot::Point target_point;
  bool started_surround;
  double goal_distance;

  double distance_traveled = 0;
  Timer physical_time, virtual_time, alignment_time, exploration_time;

  Robot::Point get_relative_position(const OccupationGrid *grid) const;

  Robot::Vector move(const Robot::Vector &dir);
  void reset();
  void update_grid(OccupationGrid *grid);

  void pause_timers();

  void explore(const OccupationGrid *grid);

  void phase1_wall_discovery(const OccupationGrid *grid);
  void phase2_wall_alignment(const OccupationGrid *grid);
  void phase3_wall_following(const OccupationGrid *grid);
  void phase5_region_alignment(const OccupationGrid *grid);
  void phase6_region_exploration(const OccupationGrid *grid);
  void phase7_emergency_find(const OccupationGrid *grid);


  void draw(const DrawData &draw_data) const;

private:
  const Robot::Vector start_direction;

  Robot::Vector direction;
  Robot::Point contact_point;

  Robot::Vector compute_wall_following_vector(const OccupationGrid *grid);
  bool path_blocked_to(const Robot::Vector &target) const;

  void draw_direction(const DrawData &draw_data) const;
};

#endif
