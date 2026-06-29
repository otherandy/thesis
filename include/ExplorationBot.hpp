#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "OccupationGrid.hpp"

#define NEXT_INDEX 1
#define PREV_INDEX -1

const double DESIRED_WALL_DISTANCE = SPEED * 2;
const double WALL_DISTANCE_STRENGTH = 0.9;

enum class ExplorationPhase {
  WallDiscovery,
  WallAlignment,
  WallFollowing,
  RegionDiscovery,
  RegionAlignment,
  RegionExploration,
  Complete
};

class ExplorationBot : public Bot {
public:
  bool clockwise_following;

  ExplorationBot(const Robot::Point &start_pos, const Robot::Vector &start_dir,
                 bool clockwise);

  ExplorationPhase phase = ExplorationPhase::WallDiscovery;

  bool left_contact_point;
  vertex_t target_vertex;
  Robot::Point target_point;

  Robot::Point get_relative_position(const OccupationGrid *grid) const;

  void reset();
  void update_grid(OccupationGrid *grid);

  void explore(const OccupationGrid *grid);

  void phase1_wall_discovery();
  void phase2_wall_alignment(const OccupationGrid *grid);

  Robot::Vector compute_wall_following_vector(
      const OccupationGrid *grid,
      const Robot::Vector &preferred_direction = Robot::Vector(0, 0));

  void phase3_wall_following(const OccupationGrid *grid);
  void phase5_region_alignment(const OccupationGrid *grid);
  void phase6_region_exploration(const OccupationGrid *grid);

  void draw(const DrawData &draw_data) const;

private:
  const Robot::Point start_point;
  const Robot::Vector start_direction;

  Robot::Vector direction;
  Robot::Point contact_point;

  bool path_blocked_to(const Robot::Vector &target) const;

  Robot::Point reading_index_to_point(std::size_t index,
                                      const OccupationGrid *grid) const;
};

#endif
