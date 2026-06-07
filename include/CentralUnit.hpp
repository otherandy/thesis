#ifndef CENTRALUNIT_HPP
#define CENTRALUNIT_HPP

#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include "Timer.hpp"
#include <memory>
#include <vector>

enum class CentralPhase {
  Idle,
  Explore,
  Complete,
};

class CentralUnit {
private:
  CentralPhase phase = CentralPhase::Idle;
  std::vector<ExplorationBot *> bots;
  std::vector<ExplorationPhase> bot_phases;

  bool is_paused = false;
  std::shared_ptr<OccupationGrid> occupation_grid;

  std::shared_ptr<Graph> frontier_region_graph;
  std::shared_ptr<StepTraversal> traversal_algorithm;

  std::size_t current_frontier_region_id = 1;

  Timer p1, p2, p3, p4, p5, p6;

  void assign_frontier_regions();
  void run_exploration();
  void check_collisions();

public:
  CentralUnit();
  void register_bot(ExplorationBot *bot);
  void get_input_and_move();
  void update();
  void draw(const DrawData &draw_data);
  void reset();
  void report_time();
};

#endif
