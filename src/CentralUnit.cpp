#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include "FrontierRegion.hpp"
#include "Graph.hpp"
#include "Grid.hpp"
#include "OccupationGrid.hpp"
#include <CGAL/number_utils.h>
#include <algorithm>
#include <boost/graph/depth_first_search.hpp>
#include <future>
#include <memory>
#include <utility>

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
    phase = CentralPhase::Explore;
    total_time.start();
  }

  if (IsKeyDown(KEY_UP)) {
    bots.front()->move(Robot::Vector(0, -1));
  }
  if (IsKeyDown(KEY_DOWN)) {
    bots.front()->move(Robot::Vector(0, 1));
  }
  if (IsKeyDown(KEY_LEFT)) {
    bots.front()->move(Robot::Vector(-1, 0));
  }
  if (IsKeyDown(KEY_RIGHT)) {
    bots.front()->move(Robot::Vector(1, 0));
  }
}

void CentralUnit::check_collisions_during_wall() {
  for (ExplorationBot *bot1 : bots) {
    if (bot1->phase != ExplorationPhase::WallFollowing) {
      continue;
    }

    Robot::Point pos1 = bot1->get_relative_position(occupation_grid.get());

    for (ExplorationBot *bot2 : bots) {
      if (bot1 == bot2) {
        continue;
      }

      if (bot2->phase != ExplorationPhase::WallFollowing) {
        continue;
      }

      Robot::Point pos2 = bot2->get_relative_position(occupation_grid.get());

      const double distance = CGAL::sqrt(CGAL::squared_distance(pos1, pos2));

      if (distance < SPEED * 3 && bot1->left_contact_point &&
          bot2->left_contact_point &&
          bot1->clockwise_following != bot2->clockwise_following) {
        bot1->phase = ExplorationPhase::RegionDiscovery;
        bot2->phase = ExplorationPhase::RegionDiscovery;
      }
    }
  }
}

void CentralUnit::assign_frontier_regions() {
  bool ran_compute = false;
  vertex_t target_v;

  auto select_target = [&](ExplorationBot *b) -> std::optional<Robot::Point> {
    auto vd = frontier_sched->get_vertex_data(b->target_vertex);
    auto *r = vd.region.get();
    const auto grid = occupation_grid->get_data();

    const Robot::Point rp = b->get_relative_position(occupation_grid.get());
    const auto tp = r->get_closest_unexplored(*grid, rp);

    return tp;
  };

  for (ExplorationBot *bot : bots) {
    if (bot->phase == ExplorationPhase::RegionDiscovery) {
      if (!ran_compute) {
        occupation_grid->compute_frontier_regions(frontier_sched.get());
        auto vopt = frontier_sched->next();

        if (!vopt.has_value()) {
          vopt = frontier_sched->help();

          if (!vopt.has_value()) {
            bot->clockwise_following = !bot->clockwise_following;
            bot->phase = ExplorationPhase::WallDiscovery;
            return;
          }
        }

        target_v = *vopt;

        root = target_v;
        ran_compute = true;
      }

      bot->target_vertex = target_v;
      bot->target_point = select_target(bot).value();
      bot->phase = ExplorationPhase::RegionAlignment;
    }

    if (bot->phase == ExplorationPhase::RegionExploration) {
      const auto tp = select_target(bot);

      if (!tp.has_value()) {
        frontier_sched->done(bot->target_vertex);
        bot->phase = ExplorationPhase::RegionDiscovery;
        continue;
      }

      bot->target_point = tp.value();
    }
  }
}

void CentralUnit::run_exploration() {
  if (phase == CentralPhase::Complete) {
    total_time.pause();
    return;
  }

  if (phase == CentralPhase::Idle || is_paused) {
    return;
  }

  if (phase == CentralPhase::Explore) {
    check_collisions_during_wall();
    assign_frontier_regions();

    std::vector<std::future<void>> jobs;

    for (ExplorationBot *bot : bots) {
      auto f = [bot, grid = occupation_grid.get()]() { bot->explore(grid); };

      jobs.emplace_back(std::async(std::launch::async, f));
    }

    for (auto &job : jobs) {
      job.get();
    }
  }
}

void CentralUnit::update() {
  get_input_and_move();

  if (is_paused) {
    return;
  }

  physical_time.pause();
  virtual_time.pause();
  alignment_time.pause();
  exploration_time.pause();

  if (phase != CentralPhase::Complete) {
    for (ExplorationBot *bot : bots) {
      if (bot->phase == ExplorationPhase::WallDiscovery ||
          bot->phase == ExplorationPhase::WallAlignment ||
          bot->phase == ExplorationPhase::WallFollowing) {
        physical_time.start();
      }

      if (bot->phase == ExplorationPhase::RegionDiscovery ||
          bot->phase == ExplorationPhase::RegionAlignment ||
          bot->phase == ExplorationPhase::RegionExploration) {
        virtual_time.start();
      }

      if (bot->phase == ExplorationPhase::WallDiscovery ||
          bot->phase == ExplorationPhase::WallAlignment ||
          bot->phase == ExplorationPhase::RegionDiscovery ||
          bot->phase == ExplorationPhase::RegionAlignment) {
        alignment_time.start();
      }

      if (bot->phase == ExplorationPhase::WallFollowing ||
          bot->phase == ExplorationPhase::RegionExploration) {
        exploration_time.start();
      }
    }
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
    bot->update_grid(occupation_grid.get());
  }

  run_exploration();

  if (occupation_grid->frontier_cell_count == 0) {
    phase = CentralPhase::Complete;
  }
}

void CentralUnit::draw(const DrawData &draw_data) {
  occupation_grid->draw(draw_data);

  auto draw_bot = [&](ExplorationBot *bot) { bot->draw(draw_data); };
  std::for_each(bots.begin(), bots.end(), draw_bot);
}

void CentralUnit::reset() {
  is_paused = false;
  phase = CentralPhase::Idle;

  occupation_grid = std::make_unique<OccupationGrid>();
  frontier_sched = std::make_unique<DynamicScheduler>();

  auto outer_wall = std::make_shared<FrontierRegion>();
  outer_wall->min = std::make_pair(0, 0);
  outer_wall->max = std::make_pair(MAP_HEIGHT, MAP_WIDTH);
  root = frontier_sched->add_vertex(outer_wall, true);

  for (ExplorationBot *bot : bots) {
    bot->reset();
  }

  physical_time.reset();
  virtual_time.reset();
  alignment_time.reset();
  exploration_time.reset();
  total_time.reset();
}

void CentralUnit::report_time() {
  std::cout << "Physical Time: " << physical_time.get_time() << "s\n";
  std::cout << "Virtual Time: " << virtual_time.get_time() << "s\n";
  std::cout << "Alignment Time: " << alignment_time.get_time() << "s\n";
  std::cout << "Exploration Time: " << exploration_time.get_time() << "s\n";

  std::cout << "Total Exploration Time: " << total_time.get_time() << "s\n";
}

void CentralUnit::save_data() {
  occupation_grid->save_to_file("Testing/grid.csv");
}
