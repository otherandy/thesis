#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
#include <CGAL/number_utils.h>
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
    bot_phases.resize(bots.size(), ExplorationPhase::WallDiscovery);
    phase = CentralPhase::Explore;
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

void CentralUnit::assign_frontier_regions() {
  std::vector<std::size_t> phase4_bots;

  for (std::size_t i = 0; i < bots.size(); ++i) {
    ExplorationPhase bp = bot_phases[i];
    ExplorationBot *bot = bots[i];

    if (bp == ExplorationPhase::RegionDiscovery) {
      phase4_bots.push_back(i);
    }
  }

  if (phase4_bots.size() > 0) {
    occupation_grid->compute_frontier_regions(traversal_algorithm,
                                              current_frontier_region_id);

    while (true) {
      const std::optional<vertex_t> next_region = traversal_algorithm->next();
      if (!next_region) {
        phase = CentralPhase::Complete;
        return;
      }

      const auto target_region =
          occupation_grid->get_frontier_region_by_id(*next_region);

      if (!target_region) {
        continue;
      }

      if (target_region->explored()) {
        continue;
      }

      current_frontier_region_id = *next_region;

      for (std::size_t i : phase4_bots) {
        ExplorationBot *bot = bots[i];
        Point rp = bot->get_relative_position();
        bot->target_point = target_region->get_closest_point(rp);
        bot_phases[i] = ExplorationPhase::RegionAlignment;
      }
      break;
    }
  }
}

void CentralUnit::run_exploration() {
  if (phase == CentralPhase::Complete) {
    return;
  }

  if (phase == CentralPhase::Idle || is_paused) {
    return;
  }

  if (phase == CentralPhase::Explore) {
    std::vector<std::future<void>> jobs;

    assign_frontier_regions();

    for (std::size_t i = 0; i < bots.size(); ++i) {
      ExplorationPhase bp = bot_phases[i];
      ExplorationBot *bot = bots[i];

      auto f = [&bot_phases = bot_phases, i, bot, phase = bp,
                grid = occupation_grid,
                &region = current_frontier_region_id]() {
        bot_phases[i] = bot->explore(phase, grid, region);
      };

      jobs.emplace_back(std::async(std::launch::async, f));
    }

    for (auto &job : jobs) {
      job.get();
    }
  }
}

void CentralUnit::check_collisions() {
  for (std::size_t i = 0; i < bots.size(); ++i) {
    ExplorationBot *bot1 = bots[i];
    Point pos1 = bot1->get_relative_position();

    for (std::size_t j = i + 1; j < bots.size(); ++j) {
      ExplorationBot *bot2 = bots[j];
      Point pos2 = bot2->get_relative_position();

      const double distance = CGAL::sqrt(CGAL::squared_distance(pos1, pos2));

      if (distance < SPEED * 2) {
        if (bot1->left_contact_point && bot2->left_contact_point) {
          bot_phases[i] = ExplorationPhase::RegionDiscovery;
          bot_phases[j] = ExplorationPhase::RegionDiscovery;
        }
      }
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
    update_jobs.emplace_back(std::async(
        std::launch::async, [bot]() { bot->take_lidar_readings(); }));
  }

  for (auto &job : update_jobs) {
    job.get();
  }

  for (ExplorationBot *bot : bots) {
    bot->update_grid(occupation_grid);
  }

  check_collisions();

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
