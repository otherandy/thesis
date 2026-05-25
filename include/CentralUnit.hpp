#ifndef CENTRALUNIT_HPP
#define CENTRALUNIT_HPP

#include "ExplorationBot.hpp"
#include "FrontierRegion.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include "Timer.hpp"
#include <memory>
#include <vector>

class CentralUnit {
private:
  bool is_paused = false;

  std::vector<ExplorationBot *> bots;

  std::shared_ptr<ExplorationPhase> exploration_phase;
  std::shared_ptr<OccupationGrid> occupation_grid;

  std::shared_ptr<Graph> frontier_region_graph;
  std::shared_ptr<StepTraversal> traversal_algorithm;

  std::vector<FrontierRegion> frontier_regions;
  std::size_t current_frontier_region_id = 0;

  Timer p1, p2, p3, p4, p5, p6;

  void run_exploration();

public:
  CentralUnit();
  void register_bot(ExplorationBot *bot);
  void get_input_and_move();
  void update();
  void draw(DrawData draw_data);
  void reset();
  void report_time();
};

#endif
