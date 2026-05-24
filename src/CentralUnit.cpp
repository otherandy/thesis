#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include <algorithm>
#include <memory>

CentralUnit::CentralUnit() { reset(); }

void CentralUnit::run_exploration() {
  if (*exploration_phase == ExplorationPhase::Idle ||
      *exploration_phase == ExplorationPhase::Completed || is_paused) {
    return;
  }

  if (*exploration_phase == ExplorationPhase::WallDiscovery) {
    for (ExplorationBot *bot : bots) {
      bot->phase1_wall_discovery(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::WallAlignment) {
    for (ExplorationBot *bot : bots) {
      bot->phase2_wall_alignment(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::WallFollowing) {
    for (ExplorationBot *bot : bots) {
      bot->phase3_wall_following(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::RegionDiscovery) {
    for (ExplorationBot *bot : bots) {
      bot->phase4_region_discovery(exploration_phase, occupation_grid,
                                   traversal_algorithm, frontier_regions,
                                   current_frontier_region_id);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::RegionAlignment) {
    for (ExplorationBot *bot : bots) {
      bot->phase5_region_alignment(exploration_phase, occupation_grid);
    }
    return;
  }

  if (*exploration_phase == ExplorationPhase::RegionExploration) {
    for (ExplorationBot *bot : bots) {
      bot->phase6_region_exploration(exploration_phase, occupation_grid,
                                     frontier_regions,
                                     current_frontier_region_id);
    }
    return;
  }
}

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

void CentralUnit::reset() {
  is_paused = false;

  exploration_phase =
      std::make_shared<ExplorationPhase>(ExplorationPhase::Idle);
  occupation_grid = std::make_shared<OccupationGrid>();

  frontier_region_graph = std::make_shared<Graph>();
  const vertex_t root = boost::add_vertex(*frontier_region_graph);
  traversal_algorithm = std::make_shared<StepDFS>(*frontier_region_graph, root);

  frontier_regions.clear();
  current_frontier_region_id = 0;

  for (ExplorationBot *bot : bots) {
    bot->reset();
  }
}

void CentralUnit::draw(DrawData draw_data) {
  occupation_grid->draw(draw_data);

  auto draw_bot = [&](ExplorationBot *bot) { bot->draw(draw_data); };
  std::for_each(bots.begin(), bots.end(), draw_bot);
}
