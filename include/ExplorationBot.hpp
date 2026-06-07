#ifndef EXPLORATION_BOT_HPP
#define EXPLORATION_BOT_HPP

#include "Bot.hpp"
#include "OccupationGrid.hpp"
#include "cgal_types.hpp"

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
private:
  Vector direction;
  Point start_point;

  Point contact_point;

  bool clockwise_following;

  std::size_t last_closest_reading;

  void draw_target_point(const DrawData &draw_data) const;

public:
  ExplorationBot(const Point &start_pos, const Vector &start_dir,
                 bool clockwise);

  Point get_relative_position() const;
  Point target_point;
  bool left_contact_point = false;

  void reset();
  void update_grid(std::shared_ptr<OccupationGrid> grid);

  ExplorationPhase explore(ExplorationPhase phase,
                           std::shared_ptr<const OccupationGrid> grid,
                           const Cell *anchor_cell);

  ExplorationPhase phase1_wall_discovery();
  ExplorationPhase phase2_wall_alignment();
  Vector compute_wall_following_vector();
  ExplorationPhase
  phase3_wall_following(std::shared_ptr<const OccupationGrid> grid);
  ExplorationPhase
  phase5_region_alignment(std::shared_ptr<const OccupationGrid> grid);
  ExplorationPhase
  phase6_region_exploration(std::shared_ptr<const OccupationGrid> grid,
                            const Cell *anchor_cell);

  void draw(const DrawData &draw_data) const;
};

#endif
