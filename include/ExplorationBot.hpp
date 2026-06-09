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
  bool clockwise_following;

  Vector direction;
  Point start_point;
  Point contact_point;

  bool path_blocked_to(const Point &target) const;

public:
  ExplorationBot(const Point &start_pos, const Vector &start_dir,
                 bool clockwise);

  ExplorationPhase phase = ExplorationPhase::WallDiscovery;
  const FrontierRegion *target_region = nullptr;
  bool left_contact_point;

  Point get_relative_position() const;
  void reset();
  void update_grid(std::shared_ptr<OccupationGrid> grid);

  void explore(std::shared_ptr<const OccupationGrid> grid);

  void phase1_wall_discovery();
  void phase2_wall_alignment();
  Vector compute_wall_following_vector(
      const Vector &preferred_direction = Vector(0, 0));

  void phase3_wall_following(std::shared_ptr<const OccupationGrid> grid);
  void phase5_region_alignment(std::shared_ptr<const OccupationGrid> grid);
  void phase6_region_exploration(std::shared_ptr<const OccupationGrid> grid);

  void draw(const DrawData &draw_data) const;
};

#endif
