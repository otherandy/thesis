#ifndef CENTRALUNIT_HPP
#define CENTRALUNIT_HPP

#include "Environment.hpp"
#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include "Timer.hpp"
#include "cgal_types.hpp"
#include <memory>
#include <vector>

enum class CentralPhase {
  Idle,
  Explore,
  Complete,
};

class CentralUnit {
public:
  CentralUnit(EnvironmentPreset selected_env,
              const Robot::Point &start_position);

  void start_test() {
    test_mode = true;
    phase = CentralPhase::Explore;
  }

  bool test_finished() { return test_mode && phase == CentralPhase::Complete; }
  void enable_debug() { occupation_grid->enable_debug(); }

  void reset(EnvironmentPreset selected_env,
             const Robot::Point &start_position);
  void register_bot(const Robot::Vector &start_dir);
  void get_manual_input();
  void sense();
  void update();

  void draw(const DrawData &draw_data);
  void draw_graph(int screenW, int screenH);
  void draw_environment(const DrawData &draw_data);

  void report_time();
  void save_grid();
  void save_data();

private:
  CentralPhase phase = CentralPhase::Idle;
  std::vector<std::shared_ptr<ExplorationBot>> bots;

  bool has_started = false;
  bool is_paused = false;
  bool test_mode = false;

  std::shared_ptr<Environment> environment;
  std::unique_ptr<OccupationGrid> occupation_grid;
  std::unique_ptr<DynamicScheduler> frontier_scheduler;
  std::unique_ptr<DynamicScheduler> physical_scheduler;

  Timer total_time;

  void check_collisions_during_wall();
  void mark_done_frontiers();
  void assign_frontier_regions();
  void check_exterior();

  void run_exploration();
};

#endif
