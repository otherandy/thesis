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

  bool is_paused = false;
  std::unique_ptr<OccupationGrid> occupation_grid;
  std::unique_ptr<DynamicScheduler> frontier_sched;

  vertex_t root;

  Timer physical_time, virtual_time, total_time;

  void assign_frontier_regions();
  void run_exploration();
  void check_collisions_during_wall();

public:
  CentralUnit();
  void register_bot(ExplorationBot *bot);
  void get_input_and_move();
  void update();
  void draw(const DrawData &draw_data);
  void reset();
  void report_time();
  void save_data();
};

#endif
