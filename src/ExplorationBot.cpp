#include "ExplorationBot.hpp"
#include "Utils.hpp"
#include "cgal_types.hpp"
#include <CGAL/linear_least_squares_fitting_2.h>

Point ExplorationBot::get_relative_position() const {
  const Point rp = get_real_position();
  const Point c = environment_center();
  return Point(rp.x() - c.x(), rp.y() - c.y());
}

void ExplorationBot::reset() {
  direction = get_random_heading();

  Bot::reset();
}

void ExplorationBot::phase1_wall_discovery(
    std::shared_ptr<ExplorationPhase> phase,
    std::shared_ptr<OccupationGrid> grid) {

  for (const auto &r : current_readings) {
    if (r.distance < LIDAR_RADIUS) {
      *phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  const Point rp = get_relative_position();
  grid->mark_cells(rp, current_readings);
  move(direction);
}

void ExplorationBot::phase2_wall_alignment(
    std::shared_ptr<ExplorationPhase> phase,
    std::shared_ptr<OccupationGrid> grid) {

  const Reading &closest_reading =
      current_readings[closest_wall_reading_index.value()];

  const Point rp = get_relative_position();

  if (closest_reading.distance <= DESIRED_WALL_DISTANCE) {
    left_contact_point = false;
    last_closest_reading = closest_wall_reading_index.value();
    contact_point = rp;

    const Vector to_wall =
        Vector(cos(closest_reading.angle), sin(closest_reading.angle));
    direction = Vector(-to_wall.y(), to_wall.x());

    *phase = ExplorationPhase::WallFollowing;
    return;
  }

  const Vector to_wall =
      Vector(cos(closest_reading.angle), sin(closest_reading.angle));

  grid->mark_cells(rp, current_readings);
  move(to_wall);
}

Vector ExplorationBot::compute_wall_following_vector() {
  constexpr bool FOLLOW_RIGHT_WALL = true;

  auto cross_z = [&](const Vector &a, const Vector &b) -> double {
    return a.x() * b.y() - a.y() * b.x();
  };

  auto safe_unit = [&](const Vector &v, const Vector &fallback) -> Vector {
    const double len2 = v.squared_length();
    if (len2 <= 1e-12) {
      return fallback;
    }
    const double inv_len = 1.0 / std::sqrt(len2);
    return Vector(v.x() * inv_len, v.y() * inv_len);
  };

  auto is_wall_hit = [&](std::size_t idx) {
    return current_readings[idx].distance < LIDAR_RADIUS;
  };

  const Vector heading_unit = safe_unit(direction, Vector(1, 0));

  std::optional<std::size_t> side_ref = std::nullopt;
  double best_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < MAX_LIDAR_SAMPLES; ++i) {
    const Reading &r = current_readings[i];
    if (r.distance >= LIDAR_RADIUS) {
      continue;
    }

    const Vector to_hit(cos(r.angle), sin(r.angle));
    const double side = cross_z(heading_unit, to_hit);

    const bool on_right = (side < 0.0);
    const bool on_left = (side > 0.0);
    const bool keep = FOLLOW_RIGHT_WALL ? on_right : on_left;

    if (!keep) {
      continue;
    }

    if (r.distance < best_dist) {
      best_dist = r.distance;
      side_ref = i;
    }
  }

  std::size_t ref;
  if (side_ref) {
    ref = *side_ref;
  } else if (closest_wall_reading_index) {
    ref = closest_wall_reading_index.value();
  } else {
    return direction;
  }

  last_closest_reading = ref;

  std::vector<Point> prev_points;
  std::vector<Point> wall_points;
  wall_points.reserve(64);

  {
    std::size_t idx = ref;
    for (std::size_t steps = 0; steps < MAX_LIDAR_SAMPLES; ++steps) {
      idx = relative_index(idx, PREV_INDEX);
      if (idx == ref || !is_wall_hit(idx)) {
        break;
      }
      prev_points.push_back(reading_index_to_point(idx));
    }
  }

  for (std::size_t i = prev_points.size(); i-- > 0;) {
    wall_points.push_back(prev_points[i]);
  }
  wall_points.push_back(reading_index_to_point(ref));

  {
    std::size_t idx = ref;
    for (std::size_t steps = 0; steps < MAX_LIDAR_SAMPLES; ++steps) {
      idx = relative_index(idx, NEXT_INDEX);
      if (idx == ref || !is_wall_hit(idx)) {
        break;
      }
      wall_points.push_back(reading_index_to_point(idx));
    }
  }

  const Reading &ref_r = current_readings[ref];
  const Vector to_wall(cos(ref_r.angle), sin(ref_r.angle));

  Vector forward;
  if (wall_points.size() < 2) {
    forward = Vector(-to_wall.y(), to_wall.x());
    if ((forward * heading_unit) < 0) {
      forward = -forward;
    }
  } else {
    CGAL::Line_2<Kernel> fitted_line;
    CGAL::linear_least_squares_fitting_2(wall_points.begin(), wall_points.end(),
                                         fitted_line, CGAL::Dimension_tag<0>());

    forward = fitted_line.to_vector();

    if ((forward * heading_unit) < 0) {
      forward = -forward;
    }
  }

  double distance_error = ref_r.distance - DESIRED_WALL_DISTANCE;
  const double max_error = DESIRED_WALL_DISTANCE;
  if (distance_error > max_error)
    distance_error = max_error;
  if (distance_error < -max_error)
    distance_error = -max_error;

  const Vector desired_vector = forward + to_wall * distance_error;

  direction = forward;

  return desired_vector;
}

void ExplorationBot::phase3_wall_following(
    std::shared_ptr<ExplorationPhase> phase,
    std::shared_ptr<OccupationGrid> grid) {

  const Point rp = get_relative_position();
  const double distance = std::sqrt(CGAL::squared_distance(rp, contact_point));

  if (!grid->was_frontier_cell_added() && distance < SPEED * 2 &&
      left_contact_point) {
    std::cout << "EXPLORATION: Completed wall following loop.\n";

    *phase = ExplorationPhase::RegionDiscovery;
  } else if (distance >= SPEED * 2) {
    left_contact_point = true;
  }

  Vector desired_vector = compute_wall_following_vector();

  grid->mark_cells(rp, current_readings);

  const Vector delta = move(desired_vector);
  if (delta.squared_length() <= 1e-12) {
    direction = -direction;
    return;
  }
}

void ExplorationBot::phase4_region_discovery(
    std::shared_ptr<ExplorationPhase> phase,
    std::shared_ptr<OccupationGrid> grid,
    std::shared_ptr<StepTraversal> traversal,
    std::vector<FrontierRegion> &frontier_regions,
    std::size_t &current_frontier_region_id) {

  compute_frontier_regions(grid->get_grid(), traversal, frontier_regions,
                           current_frontier_region_id);

  if (grid->get_frontier_cell_count() <= 2) {
    std::cout
        << "EXPLORATION: No frontier cells found. Exploration completed.\n";
    *phase = ExplorationPhase::Completed;
    return;
  }

  const Point rp = get_relative_position();

  while (true) {
    const std::optional<vertex_t> next_region = traversal->next();
    if (!next_region) {
      std::cout
          << "EXPLORATION: No frontier regions found. Exploration completed.\n";
      *phase = ExplorationPhase::Completed;
      return;
    }

    if (*next_region == 0) {
      target_point = rp;
      break;
    }

    if (*next_region > frontier_regions.size()) {
      continue;
    }

    auto target_region =
        get_frontier_region_by_id(frontier_regions, *next_region);

    if (target_region->explored) {
      continue;
    }

    current_frontier_region_id = *next_region;
    target_point = target_region->get_closest_point(rp);
    break;
  }

  std::cout << "EXPLORATION: Targeting frontier region "
            << current_frontier_region_id << ".\n";
  *phase = ExplorationPhase::RegionAlignment;
}

void ExplorationBot::phase5_region_alignment(
    std::shared_ptr<ExplorationPhase> phase,
    std::shared_ptr<OccupationGrid> grid) {

  const Point rp = get_relative_position();
  const double distance = std::sqrt(CGAL::squared_distance(rp, target_point));

  if (distance < SPEED * 2) {
    *phase = ExplorationPhase::RegionExploration;
    return;
  }

  Vector desired_vector;

  if (closest_wall_reading_index &&
      grid->there_is_obstacle_between(rp, target_point)) {
    desired_vector = compute_wall_following_vector();
  } else {
    desired_vector = target_point - rp;
  }

  grid->mark_cells(rp, current_readings);
  move(desired_vector);
}

void ExplorationBot::phase6_region_exploration(
    std::shared_ptr<ExplorationPhase> phase,
    std::shared_ptr<OccupationGrid> grid,
    std::vector<FrontierRegion> &frontier_regions,
    std::size_t &current_frontier_region_id) {

  if (current_frontier_region_id == 0) {
    *phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  auto current_region =
      get_frontier_region_by_id(frontier_regions, current_frontier_region_id);

  if (current_region->explored) {
    *phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  const Point rp = get_relative_position();

  auto closest_unexplored = current_region->get_closest_unexplored(rp);

  if (!closest_unexplored) {
    current_region->explored = true;
    *phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  Vector desired_vector;

  if (closest_wall_reading_index) {
    Point closest_wall_point =
        reading_index_to_point(closest_wall_reading_index.value());
    auto obstacle_cell = grid->get_cell_from_position(closest_wall_point);

    if (obstacle_cell.state == CellState::Unknown) {
      *phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  desired_vector = closest_unexplored.value() - rp;
  grid->mark_cells(rp, current_readings);
  move(desired_vector);
}

void ExplorationBot::draw_target_point(DrawData draw_data) const {
  if (target_point == Point(0, 0)) {
    return;
  }

  const Point rp = get_relative_position();

  const Point &pos = get_real_position();
  const Point target_screen_pos = Point(pos.x() + (target_point.x() - rp.x()),
                                        pos.y() + (target_point.y() - rp.y()));

  DrawCircle(
      target_screen_pos.x() * draw_data.scale_factor + draw_data.offset_x,
      target_screen_pos.y() * draw_data.scale_factor + draw_data.offset_y,
      DRAWN_POINT_RADIUS, ORANGE);
}

ExplorationBot::ExplorationBot(const Point &start_pos) : Bot(start_pos) {
  direction = Vector(1, 0);
}

void ExplorationBot::update() { take_lidar_readings(); }

void ExplorationBot::draw(DrawData draw_data) const {
  // draw_readings(draw_data);
  draw_body(draw_data);
  draw_lidar(draw_data);
  draw_position_text();
}
