#include "ExplorationBot.hpp"
#include "Graph.hpp"
#include "OccupationGrid.hpp"
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

void ExplorationBot::phase1_wall_discovery() {
  for (const auto &r : current_readings) {
    if (r.distance < LIDAR_RADIUS) {
      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  move(direction);
}

void ExplorationBot::phase2_wall_alignment() {
  const Reading &closest_reading =
      current_readings[closest_wall_reading_index.value()];

  if (closest_reading.distance <= DESIRED_WALL_DISTANCE) {
    left_contact_point = false;
    last_closest_reading = closest_wall_reading_index.value();
    contact_point = get_relative_position();

    const Vector to_wall =
        Vector(cos(closest_reading.angle), sin(closest_reading.angle));

    const Vector tangent_right(-to_wall.y(), to_wall.x());
    const Vector tangent_left(to_wall.y(), -to_wall.x());
    direction = clockwise_following ? tangent_right : tangent_left;

    phase = ExplorationPhase::WallFollowing;
    return;
  }

  const Vector to_wall =
      Vector(cos(closest_reading.angle), sin(closest_reading.angle));

  move(to_wall);
}

Vector ExplorationBot::compute_wall_following_vector() {
  auto cross_z = [&](const Vector &a, const Vector &b) -> double {
    return a.x() * b.y() - a.y() * b.x();
  };

  Vector heading_unit = normalize_vector(direction);
  if (heading_unit.squared_length() <= 1e-12) {
    heading_unit = Vector(1, 0);
  }

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
    const bool keep = clockwise_following ? on_right : on_left;

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

  std::vector<Point> wall_points;
  wall_points.reserve(64);

  // prev
  {
    std::size_t idx = ref;
    for (std::size_t steps = 0; steps < MAX_LIDAR_SAMPLES; ++steps) {
      idx = relative_index(idx, PREV_INDEX);
      if (idx == ref || current_readings[idx].distance >= LIDAR_RADIUS) {
        break;
      }
      wall_points.push_back(reading_index_to_point(idx));
    }
    std::reverse(wall_points.begin(), wall_points.end());
  }

  wall_points.push_back(reading_index_to_point(ref));

  // next
  {
    std::size_t idx = ref;
    for (std::size_t steps = 0; steps < MAX_LIDAR_SAMPLES; ++steps) {
      idx = relative_index(idx, NEXT_INDEX);
      if (idx == ref || current_readings[idx].distance >= LIDAR_RADIUS) {
        break;
      }
      wall_points.push_back(reading_index_to_point(idx));
    }
  }

  const auto &ref_r = current_readings[ref];
  const Vector to_wall(cos(ref_r.angle), sin(ref_r.angle));

  Vector forward;
  if (wall_points.size() < 2) {
    forward = Vector(-to_wall.y(), to_wall.x());
  } else {
    CGAL::Line_2<Kernel> fitted_line;
    CGAL::linear_least_squares_fitting_2(wall_points.begin(), wall_points.end(),
                                         fitted_line, CGAL::Dimension_tag<0>());
    forward = fitted_line.to_vector();
  }

  if ((forward * heading_unit) < 0) {
    forward = -forward;
  }

  direction = forward;

  double distance_error = ref_r.distance - DESIRED_WALL_DISTANCE;
  distance_error =
      std::clamp(distance_error, -DESIRED_WALL_DISTANCE, DESIRED_WALL_DISTANCE);

  return forward + to_wall * distance_error;
}

void ExplorationBot::phase3_wall_following(
    std::shared_ptr<OccupationGrid> grid) {

  const Point rp = get_relative_position();
  const double distance = std::sqrt(CGAL::squared_distance(rp, contact_point));

  if (!grid->was_frontier_cell_added() && distance < SPEED * 2 &&
      left_contact_point) {
    phase = ExplorationPhase::RegionDiscovery;
    return;
  } else if (distance >= SPEED * 2) {
    left_contact_point = true;
  }

  Vector desired_vector = compute_wall_following_vector();

  const Vector delta = move(desired_vector);
  if (delta.squared_length() <= 1e-12) {
    direction = -direction;
  }
}

void ExplorationBot::phase4_region_discovery(
    std::shared_ptr<OccupationGrid> grid,
    std::shared_ptr<StepTraversal> traversal,
    std::size_t &current_frontier_region_id) {

  grid->compute_frontier_regions(traversal, current_frontier_region_id);

  if (grid->get_frontier_cell_count() <= 2) {
    phase = ExplorationPhase::Completed;
    return;
  }

  const Point rp = get_relative_position();

  while (true) {
    const std::optional<vertex_t> next_region = traversal->next();
    if (!next_region) {
      phase = ExplorationPhase::Completed;
      return;
    }

    if (*next_region == 0) {
      target_point = rp;
      break;
    }

    auto target_region = grid->get_frontier_region_by_id(*next_region);

    if (target_region->explored) {
      continue;
    }

    current_frontier_region_id = *next_region;
    target_point = target_region->get_closest_point(rp);
    break;
  }

  phase = ExplorationPhase::RegionAlignment;
}

void ExplorationBot::phase5_region_alignment(
    std::shared_ptr<OccupationGrid> grid) {

  const Point rp = get_relative_position();
  const double distance = std::sqrt(CGAL::squared_distance(rp, target_point));

  if (distance < SPEED * 2) {
    phase = ExplorationPhase::RegionExploration;
    return;
  }

  Vector desired_vector;

  if (closest_wall_reading_index &&
      grid->there_is_obstacle_between(rp, target_point)) {
    desired_vector = compute_wall_following_vector();
  } else {
    desired_vector = target_point - rp;
  }

  move(desired_vector);
}

void ExplorationBot::phase6_region_exploration(
    std::shared_ptr<OccupationGrid> grid,
    std::size_t &current_frontier_region_id) {

  if (current_frontier_region_id == 0) {
    phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  auto current_region =
      grid->get_frontier_region_by_id(current_frontier_region_id);

  if (current_region->explored) {
    phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  const Point rp = get_relative_position();

  auto closest_unexplored = current_region->get_closest_unexplored(rp);

  if (!closest_unexplored) {
    current_region->explored = true;
    phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  Vector desired_vector;

  if (closest_wall_reading_index) {
    Point closest_wall_point =
        reading_index_to_point(closest_wall_reading_index.value());
    auto obstacle_cell = grid->get_cell_from_position(closest_wall_point);

    if (obstacle_cell.state == CellState::Unknown) {
      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  desired_vector = closest_unexplored.value() - rp;
  move(desired_vector);
}

void ExplorationBot::draw_target_point(const DrawData &draw_data) const {
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

ExplorationBot::ExplorationBot(const size_t id, const Point &start_pos,
                               const Vector &start_dir, bool clockwise)
    : Bot(start_pos), id(id), clockwise_following(clockwise) {

  direction = start_dir;
}

void ExplorationBot::update_grid(std::shared_ptr<OccupationGrid> grid) {
  const Point rp = get_relative_position();
  grid->mark_cells(rp, current_readings);
}

void ExplorationBot::change_phase(ExplorationPhase new_phase) {
  phase = new_phase;
}

ExplorationPhase
ExplorationBot::explore(std::shared_ptr<OccupationGrid> grid,
                        std::shared_ptr<StepTraversal> traversal,
                        std::size_t &current_frontier_region_id) {
  switch (phase) {
  case ExplorationPhase::WallDiscovery:
    phase1_wall_discovery();
    break;
  case ExplorationPhase::WallAlignment:
    phase2_wall_alignment();
    break;
  case ExplorationPhase::WallFollowing:
    phase3_wall_following(grid);
    break;
  case ExplorationPhase::RegionDiscovery:
    phase4_region_discovery(grid, traversal, current_frontier_region_id);
    break;
  case ExplorationPhase::RegionAlignment:
    phase5_region_alignment(grid);
    break;
  case ExplorationPhase::RegionExploration:
    phase6_region_exploration(grid, current_frontier_region_id);
    break;
  default:
    break;
  }

  return phase;
}

void ExplorationBot::draw(const DrawData &draw_data) const {
  // draw_readings(draw_data);
  draw_body(draw_data);
  draw_lidar(draw_data);
}
