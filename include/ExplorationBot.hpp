#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "FrontierRegion.hpp"
#include "OccupationGrid.hpp"
#include "cgal_types.hpp"

#define NEXT_INDEX 1
#define PREV_INDEX -1

const double DESIRED_WALL_DISTANCE = SPEED * 2;
const double WALL_DISTANCE_STRENGTH = 0.9;

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

class ExplorationBot : public Bot {
private:
  Vector direction;
  Point start_point;

  Point contact_point;
  bool left_contact_point;

  Point target_point;
  std::size_t last_closest_reading;

  Point get_relative_position() const;

  void draw_target_point(DrawData draw_data) const;

public:
  ExplorationBot(const Point &start_pos);
  void reset();
  void update();

  void phase1_wall_discovery(std::shared_ptr<ExplorationPhase> phase,
                             std::shared_ptr<OccupationGrid> grid);
  void phase2_wall_alignment(std::shared_ptr<ExplorationPhase> phase,
                             std::shared_ptr<OccupationGrid> grid);
  Vector compute_wall_following_vector();
  void phase3_wall_following(std::shared_ptr<ExplorationPhase> phase,
                             std::shared_ptr<OccupationGrid> grid);
  void phase4_region_discovery(std::shared_ptr<ExplorationPhase> phase,
                               std::shared_ptr<OccupationGrid> grid,
                               std::shared_ptr<StepDFS> traversal,
                               std::vector<FrontierRegion> &frontier_regions,
                               std::size_t &current_frontier_region_id);
  void phase5_region_alignment(std::shared_ptr<ExplorationPhase> phase,
                               std::shared_ptr<OccupationGrid> grid);
  void phase6_region_exploration(std::shared_ptr<ExplorationPhase> phase,
                                 std::shared_ptr<OccupationGrid> grid,
                                 std::vector<FrontierRegion> &frontier_regions,
                                 std::size_t &current_frontier_region_id);

  void draw(DrawData draw_data) const;
};

#endif
