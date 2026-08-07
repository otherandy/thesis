#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include "FrontierRegion.hpp"
#include "Graph.hpp"
#include "Grid.hpp"
#include "OccupationGrid.hpp"
#include "Utils.hpp"
#include "cgal_types.hpp"
#include <CGAL/number_utils.h>
#include <algorithm>
#include <boost/graph/depth_first_search.hpp>
#include <future>
#include <memory>
#include <utility>

CentralUnit::CentralUnit(EnvironmentPreset selected_env,
                         const Robot::Point &start_position) {
  reset(selected_env, start_position);
}

void CentralUnit::reset(EnvironmentPreset selected_env,
                        const Robot::Point &start_position) {
  is_paused = false;
  phase = CentralPhase::Idle;

  environment = std::make_unique<Environment>(selected_env);
  occupation_grid = std::make_unique<OccupationGrid>(start_position);
  frontier_scheduler = std::make_unique<DynamicScheduler>();
  physical_scheduler = std::make_unique<DynamicScheduler>();

  auto outer_wall = std::make_shared<FrontierRegion>();
  outer_wall->min = std::make_pair(0, 0);
  outer_wall->max = std::make_pair(MAP_HEIGHT, MAP_WIDTH);
  frontier_scheduler->add_vertex(outer_wall, true);

  for (auto bot : bots) {
    bot->reset();
  }

  total_time.reset();
}

void CentralUnit::register_bot(const Robot::Vector &start_dir) {
  const auto start_pos = occupation_grid->get_origin();

  bool clockwise = true;

  if (!bots.empty()) {
    clockwise = !bots.back()->clockwise_following;
  }

  bots.emplace_back(std::make_shared<ExplorationBot>(
      bots.size() + 1, start_pos, start_dir, clockwise, environment));
}

void CentralUnit::get_manual_input() {
  if (IsKeyPressed(KEY_R)) {
    reset(environment->preset, occupation_grid->get_origin());
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
  for (auto bot1 : bots) {
    if (bot1->phase != ExplorationPhase::WallFollowing) {
      continue;
    }

    Robot::Point pos1 = bot1->get_relative_position(occupation_grid.get());

    for (auto bot2 : bots) {
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
        break;
      }
    }
  }
}

void CentralUnit::mark_done_frontiers() {
  auto vertices = frontier_scheduler->get_all_vertices();
  const auto grid = occupation_grid->get_data();

  for (auto v : vertices) {
    if (frontier_scheduler->is_done(v)) {
      continue;
    }

    auto vd = frontier_scheduler->get_vertex_data(v);

    if (vd.region->is_done(*grid)) {
      frontier_scheduler->mark_done(v);
    }
  }
}

void CentralUnit::assign_frontier_regions() {
  bool ran_compute = false;

  auto select_target = [&](ExplorationBot *b) -> std::optional<Robot::Point> {
    auto vd = frontier_scheduler->get_vertex_data(b->target_vertex);
    auto *r = vd.region.get();

    const Robot::Point rp = b->get_relative_position(occupation_grid.get());
    const auto grid = occupation_grid->get_data();
    const auto tp = r->get_closest_unexplored(*grid, rp);

    return tp;
  };

  for (auto bot : bots) {
    bool setup_alignment = false;

    if (bot->phase == ExplorationPhase::RegionDiscovery) {
      if (!ran_compute) {
        mark_done_frontiers();
        occupation_grid->compute_physical_obstacles(physical_scheduler.get());
        occupation_grid->compute_frontier_regions(frontier_scheduler.get());
        ran_compute = true;
      }

      bot->phase = ExplorationPhase::Idle;
    }

    if (bot->phase == ExplorationPhase::RegionAlignment) {
      if (frontier_scheduler->is_done(bot->target_vertex)) {
        bot->phase = ExplorationPhase::Idle;
      }
    }

    if (bot->phase == ExplorationPhase::Idle) {
      // const auto grid = occupation_grid->get_data();
      // const Robot::Point rp =
      // bot->get_relative_position(occupation_grid.get()); auto vopt =
      // frontier_scheduler->closest(*grid, rp);
      auto vopt = frontier_scheduler->next_or_help();

      if (!vopt.has_value()) {
        continue;
      }

      bot->target_vertex = *vopt;
      setup_alignment = true;
    }

    if (setup_alignment) {
      const auto tp = select_target(bot.get());

      if (!tp.has_value()) {
        frontier_scheduler->mark_done(bot->target_vertex);
        bot->phase = ExplorationPhase::RegionDiscovery;
        continue;
      }

      bot->target_point = tp.value();
      bot->started_surround = false;
      bot->goal_distance = 0;
      bot->phase = ExplorationPhase::RegionAlignment;
    }

    if (bot->phase == ExplorationPhase::RegionExploration) {
      const auto tp = select_target(bot.get());

      if (!tp.has_value()) {
        frontier_scheduler->mark_done(bot->target_vertex);
        bot->phase = ExplorationPhase::RegionDiscovery;
        continue;
      }

      bot->target_point = tp.value();
    }
  }
}

void CentralUnit::check_exterior() {
  if (!occupation_grid->found_exterior) {
    for (auto bot : bots) {
      if (bot->phase == ExplorationPhase::WallFollowing) {
        return;
      }
    }

    for (auto bot : bots) {
      if (bot->phase != ExplorationPhase::Idle) {
        continue;
      }

      bot->phase = ExplorationPhase::WallDiscovery;
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
    check_collisions_during_wall();
    assign_frontier_regions();
    check_exterior();

    std::vector<std::future<void>> jobs;

    for (auto bot : bots) {
      auto f = [bot = bot, grid = occupation_grid.get()]() {
        bot->pause_timers();
        bot->explore(grid);
      };

      jobs.emplace_back(std::async(std::launch::async, f));
    }

    for (auto &job : jobs) {
      job.get();
    }
  }
}

void CentralUnit::sense() {
  std::vector<std::future<void>> update_jobs;

  for (auto bot : bots) {
    update_jobs.emplace_back(std::async(
        std::launch::async, [bot = bot]() { bot->take_lidar_readings(); }));
  }

  for (auto &job : update_jobs) {
    job.get();
  }

  for (auto bot : bots) {
    bot->update_grid(occupation_grid.get());
  }
}

void CentralUnit::update() {
  get_manual_input();

  total_time.pause();

  if (is_paused) {
    return;
  }

  if (phase != CentralPhase::Complete) {
    total_time.start();
  }

  sense();

  run_exploration();

  if (phase != CentralPhase::Complete &&
      occupation_grid->frontier_cell_count == 0 &&
      frontier_scheduler->finished()) {
    phase = CentralPhase::Complete;
    report_time();
  }
}

void CentralUnit::draw(const DrawData &draw_data) {
  occupation_grid->draw(draw_data);

  auto draw_bot = [&](std::shared_ptr<ExplorationBot> bot) {
    bot->draw(draw_data);
  };

  std::for_each(bots.begin(), bots.end(), draw_bot);
}

void CentralUnit::draw_graph(int screenW, int screenH) {
  frontier_scheduler->draw(screenW, screenH);
}

void CentralUnit::draw_environment(const DrawData &draw_data) {
  environment->draw(draw_data);
}

void CentralUnit::report_time() {
  std::cout << "Total Time: " << total_time.get_time() << "s\n";
}

void CentralUnit::save_data() {
  std::string filename = "data/" + append_timestamp("", ".csv");

  ensure_parent_dir_exists(filename);

  std::ofstream f(filename, std::ios::app);

  if (!f.is_open()) {
    std::cerr << "ERROR: Failed to open " << filename << " for writing"
              << std::endl;
    return;
  }

  f << "environment,robots,total_time\n";
  f << static_cast<int>(environment->preset) << ",";
  f << bots.size() << ",";
  f << total_time.get_time() << "\n";

  f << "id,"
    << "physical_time,virtual_time,"
    << "alignment_time,exploration_time,"
    << "distance_traveled\n";

  for (auto bot : bots) {
    f << bot->id << ",";
    f << bot->physical_time.get_time() << ",";
    f << bot->virtual_time.get_time() << ",";
    f << bot->alignment_time.get_time() << ",";
    f << bot->exploration_time.get_time() << ",";
    f << bot->distance_traveled << "\n";
  }

  f.close();
  std::cout << "INFO: Simulation data saved to " << filename << std::endl;
}
