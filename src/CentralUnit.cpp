#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include <algorithm>
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

  if (*exploration_phase != ExplorationPhase::Idle) {
    return;
  }

  if (IsKeyPressed(KEY_SPACE)) {
    // start_point = relative_position;
    *exploration_phase = ExplorationPhase::WallDiscovery;
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
  if (*exploration_phase == ExplorationPhase::Idle ||
      *exploration_phase == ExplorationPhase::Completed || is_paused) {
    return;
  }

  if (*exploration_phase == ExplorationPhase::WallDiscovery) {
    p1.start();
    for (ExplorationBot *bot : bots) {
      bot->phase1_wall_discovery(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::WallAlignment) {
    p2.start();
    for (ExplorationBot *bot : bots) {
      bot->phase2_wall_alignment(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::WallFollowing) {
    p3.start();
    for (ExplorationBot *bot : bots) {
      bot->phase3_wall_following(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::RegionDiscovery) {
    p4.start();
    for (ExplorationBot *bot : bots) {
      bot->phase4_region_discovery(exploration_phase, occupation_grid,
                                   traversal_algorithm,
                                   current_frontier_region_id);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::RegionAlignment) {
    p5.start();
    for (ExplorationBot *bot : bots) {
      bot->phase5_region_alignment(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::RegionExploration) {
    p6.start();
    for (ExplorationBot *bot : bots) {
      bot->phase6_region_exploration(exploration_phase, occupation_grid,
                                     current_frontier_region_id);
    }
    return;
  }
}

void CentralUnit::update() {
  get_input_and_move();

  if (is_paused) {
    return;
  }

  for (ExplorationBot *bot : bots) {
    bot->update();
  }

  run_exploration();
}

void CentralUnit::draw(DrawData draw_data) {
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

  exploration_phase =
      std::make_shared<ExplorationPhase>(ExplorationPhase::Idle);
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
