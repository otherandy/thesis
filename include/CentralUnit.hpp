#ifndef CENTRALUNIT_HPP
#define CENTRALUNIT_HPP

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
  CentralUnit();
  void register_bot(std::size_t id, const Robot::Point &start_pos,
                    const Robot::Vector &start_dir, bool clockwise);
  void get_input_and_move();
  void update();
  void draw(const DrawData &draw_data);
  void reset();
  void report_time();
  void save_data();

private:
  CentralPhase phase = CentralPhase::Idle;
  std::vector<std::shared_ptr<ExplorationBot>> bots;

  bool is_paused = false;

  std::unique_ptr<OccupationGrid> occupation_grid;
  std::unique_ptr<DynamicScheduler> frontier_scheduler;

  vertex_t root;

  Timer physical_time, virtual_time, alignment_time, exploration_time,
      total_time;

  void check_collisions_during_wall();
  void assign_frontier_regions();
  void run_exploration();
};

#endif
