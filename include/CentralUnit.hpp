#ifndef CENTRALUNIT_HPP
#define CENTRALUNIT_HPP

#include "ExplorationBot.hpp"
#include "FrontierRegion.hpp"
#include "OccupationGrid.hpp"
#include <memory>
#include <vector>

class CentralUnit {
private:
  bool is_paused = false;

  std::vector<ExplorationBot *> bots;

  std::shared_ptr<ExplorationPhase> exploration_phase;
  std::shared_ptr<OccupationGrid> occupation_grid;

  std::shared_ptr<Graph> frontier_region_graph;
  std::shared_ptr<StepDFS> traversal_algorithm;

  std::vector<FrontierRegion> frontier_regions;
  std::size_t current_frontier_region_id = 0;

  void run_exploration();

public:
  CentralUnit();
  void register_bot(ExplorationBot *bot);
  void get_input_and_move();
  void update();
  void draw(DrawData draw_data);
  void reset();
};

#endif
