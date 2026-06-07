#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include <algorithm>
#include <future>
#include <memory>

CentralUnit::CentralUnit() { reset(); }

void CentralUnit::register_bot(ExplorationBot *bot) { bots.push_back(bot); }

void CentralUnit::get_input_and_move() {
  if (IsKeyPressed(KEY_R)) {
    reset();
    return;
  }

  if (IsKeyPressed(KEY_P)) {
    is_paused = !is_paused;
    return;
  }

  if (phase != CentralPhase::Idle) {
    return;
  }

  if (IsKeyPressed(KEY_SPACE)) {
    // start_point = relative_position;
    phase = CentralPhase::Start;
  }

  if (IsKeyDown(KEY_UP)) {
    bots.front()->move(Vector(0, -1));
  }
  if (IsKeyDown(KEY_DOWN)) {
    bots.front()->move(Vector(0, 1));
  }
  if (IsKeyDown(KEY_LEFT)) {
    bots.front()->move(Vector(-1, 0));
  }
  if (IsKeyDown(KEY_RIGHT)) {
    bots.front()->move(Vector(1, 0));
  }
}

void CentralUnit::run_exploration() {
  if (phase == CentralPhase::Idle || is_paused) {
    return;
  }

  if (phase == CentralPhase::Start) {
    for (ExplorationBot *bot : bots) {
      bot->change_phase(ExplorationPhase::WallDiscovery);
    }
    phase = CentralPhase::Explore;
  }

  if (phase == CentralPhase::Explore) {
    for (ExplorationBot *bot : bots) {
      bot->explore(occupation_grid, traversal_algorithm,
                   current_frontier_region_id);
    }
  }
}

void CentralUnit::update() {
  get_input_and_move();

  if (is_paused) {
    return;
  }

  std::vector<std::future<void>> update_jobs;

  for (ExplorationBot *bot : bots) {
    update_jobs.emplace_back(
        std::async(std::launch::async, [bot]() { bot->update(); }));
  }

  for (auto &job : update_jobs) {
    job.get();
  }

  run_exploration();
}

void CentralUnit::draw(const DrawData &draw_data) {
  occupation_grid->draw(draw_data);

  auto draw_bot = [&](ExplorationBot *bot) { bot->draw(draw_data); };
  std::for_each(bots.begin(), bots.end(), draw_bot);

  p1.pause();
  p2.pause();
  p3.pause();
  p4.pause();
  p5.pause();
  p6.pause();
}

void CentralUnit::reset() {
  is_paused = false;
  phase = CentralPhase::Idle;

  occupation_grid = std::make_shared<OccupationGrid>();

  frontier_region_graph = std::make_shared<Graph>();
  const vertex_t root = boost::add_vertex(*frontier_region_graph);
  traversal_algorithm = std::make_shared<StepBFS>(*frontier_region_graph, root);

  current_frontier_region_id = 0;

  for (ExplorationBot *bot : bots) {
    bot->reset();
  }

  p1.reset();
  p2.reset();
  p3.reset();
  p4.reset();
  p5.reset();
  p6.reset();
}

void CentralUnit::report_time() {
  const double p1t = p1.get_time();
  const double p2t = p2.get_time();
  const double p3t = p3.get_time();
  const double p4t = p4.get_time();
  const double p5t = p5.get_time();
  const double p6t = p6.get_time();
  const double total_time = p1t + p2t + p3t + p4t + p5t + p6t;

  std::cout << "Phase 1 (Wall Discovery): " << p1t << "s\n";
  std::cout << "Phase 2 (Wall Alignment): " << p2t << "s\n";
  std::cout << "Phase 3 (Wall Following): " << p3t << "s\n";

  std::cout << "Physical Time: " << p1t + p2t + p3t << "s\n";

  std::cout << "Phase 4 (Region Discovery): " << p4t << "s\n";
  std::cout << "Phase 5 (Region Alignment): " << p5t << "s\n";
  std::cout << "Phase 6 (Region Exploration): " << p6t << "s\n";

  std::cout << "Virtual Time: " << p4t + p5t + p6t << "s\n";

  std::cout << "Total Exploration Time: " << total_time << "s\n";
}
