#include "ExplorationBot.hpp"
#include "Bot.hpp"
#include "Cell.hpp"
#include "OccupationGrid.hpp"
#include "Utils.hpp"
#include "cgal_types.hpp"
#include <CGAL/linear_least_squares_fitting_2.h>
#include <cmath>

Robot::Point
ExplorationBot::get_relative_position(const OccupationGrid *grid) const {
  const Robot::Point rp = get_real_position();
  const Robot::Point c = grid->get_origin();
  return Robot::Point(rp.x() - c.x(), rp.y() - c.y());
}

void ExplorationBot::reset() {
  // direction = get_random_heading();
  phase = ExplorationPhase::WallDiscovery;

  Bot::reset();
}

void ExplorationBot::phase1_wall_discovery() {
  for (const Reading &r : current_readings) {
    if (r.distance < LIDAR_RADIUS) {
      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  move(direction);
}

void ExplorationBot::phase2_wall_alignment(const OccupationGrid *grid) {
  const Reading &closest_reading =
      current_readings[closest_wall_reading_index.value()];

  if (closest_reading.distance <= DESIRED_WALL_DISTANCE) {
    const Robot::Vector to_wall =
        Robot::Vector(cos(closest_reading.angle), sin(closest_reading.angle));

    const Robot::Vector tangent_right(-to_wall.y(), to_wall.x());
    const Robot::Vector tangent_left(to_wall.y(), -to_wall.x());
    direction = clockwise_following ? tangent_right : tangent_left;

    left_contact_point = false;
    contact_point = get_relative_position(grid);

    phase = ExplorationPhase::WallFollowing;
    return;
  }

  move(Robot::Vector(cos(closest_reading.angle), sin(closest_reading.angle)));
}

Robot::Vector ExplorationBot::compute_wall_following_vector(
    const OccupationGrid *grid, const Robot::Vector &preferred_direction) {

  auto cross_z = [&](const Robot::Vector &a, const Robot::Vector &b) -> double {
    return a.x() * b.y() - a.y() * b.x();
  };

  Robot::Vector heading_unit = normalize_vector(direction);
  if (heading_unit.squared_length() <= 1e-12) {
    heading_unit = Robot::Vector(1, 0);
  }

  std::optional<std::size_t> side_ref = std::nullopt;
  double best_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < MAX_LIDAR_SAMPLES; ++i) {
    const Reading &r = current_readings[i];
    if (r.distance >= LIDAR_RADIUS) {
      continue;
    }

    const Robot::Vector to_hit(cos(r.angle), sin(r.angle));
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

  std::vector<Robot::Point> wall_points;
  wall_points.reserve(64);

  // prev
  {
    std::size_t idx = ref;
    for (std::size_t steps = 0; steps < MAX_LIDAR_SAMPLES; ++steps) {
      idx = relative_index(idx, PREV_INDEX);
      if (idx == ref || current_readings[idx].distance >= LIDAR_RADIUS) {
        break;
      }
      wall_points.push_back(reading_index_to_point(idx, grid));
    }
    std::reverse(wall_points.begin(), wall_points.end());
  }

  wall_points.push_back(reading_index_to_point(ref, grid));

  // next
  {
    std::size_t idx = ref;
    for (std::size_t steps = 0; steps < MAX_LIDAR_SAMPLES; ++steps) {
      idx = relative_index(idx, NEXT_INDEX);
      if (idx == ref || current_readings[idx].distance >= LIDAR_RADIUS) {
        break;
      }
      wall_points.push_back(reading_index_to_point(idx, grid));
    }
  }

  const Reading &ref_r = current_readings[ref];
  const Robot::Vector to_wall(cos(ref_r.angle), sin(ref_r.angle));

  Robot::Vector forward;
  if (wall_points.size() < 2) {
    forward = Robot::Vector(-to_wall.y(), to_wall.x());
  } else {
    CGAL::Line_2<Robot::Kernel> fitted_line;
    CGAL::linear_least_squares_fitting_2(wall_points.begin(), wall_points.end(),
                                         fitted_line, CGAL::Dimension_tag<0>());
    forward = fitted_line.to_vector();
  }

  Robot::Vector desired_unit = normalize_vector(preferred_direction);
  if (desired_unit.squared_length() <= 1e-12) {
    desired_unit = heading_unit;
  }

  if ((forward * desired_unit) < 0) {
    forward = -forward;
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

void ExplorationBot::phase3_wall_following(const OccupationGrid *grid) {

  const Robot::Point rp = get_relative_position(grid);
  const double distance = std::sqrt(CGAL::squared_distance(rp, contact_point));

  if (left_contact_point && distance < SPEED * 2) {
    phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  if (!left_contact_point && distance > SPEED * 4) {
    left_contact_point = true;
  }

  Robot::Vector desired_vector = compute_wall_following_vector(grid);

  const Robot::Vector delta = move(desired_vector);
  if (delta.squared_length() <= 1e-12) {
    direction = -direction;
  }
}

bool ExplorationBot::path_blocked_to(const Robot::Vector &target) const {
  double target_angle = std::atan2(target.y(), target.x());
  double target_dist = std::sqrt(target.squared_length());

  double step = 2.0 * M_PI / MAX_LIDAR_SAMPLES;

  for (const auto &reading : current_readings) {
    double a = std::remainder(reading.angle, 2.0 * M_PI);
    double diff = std::remainder(a - target_angle, 2.0 * M_PI);

    if (std::abs(diff) > step / 2.0) {
      continue;
    }

    if (reading.distance < target_dist) {
      return true;
    }
  }

  return false;
}

void ExplorationBot::phase5_region_alignment(const OccupationGrid *grid) {
  const Robot::Point rp = get_relative_position(grid);
  const double distance = std::sqrt(CGAL::squared_distance(rp, target_point));

  if (distance < SPEED * 2) {
    phase = ExplorationPhase::RegionExploration;
    return;
  }

  Robot::Vector desired_vector = target_point - rp;

  if (path_blocked_to(desired_vector)) {
    desired_vector = compute_wall_following_vector(grid, desired_vector);
  }

  move(desired_vector);
}

Robot::Point
ExplorationBot::reading_index_to_point(std::size_t index,
                                       const OccupationGrid *grid) const {
  const Reading &r = current_readings[index];
  const Robot::Point rp = get_relative_position(grid);
  return point_at_reading(rp, r);
}

void ExplorationBot::phase6_region_exploration(const OccupationGrid *grid) {
  if (closest_wall_reading_index) {
    const Robot::Point closest_point =
        reading_index_to_point(*closest_wall_reading_index, grid);
    const Index2D index =
        get_cell_index_from(closest_point.x(), closest_point.y());

    const auto g = grid->get_data();
    const Cell *obstacle_cell = (*g)[index.first][index.second].get();

    if (obstacle_cell->state == CellState::Unknown) {
      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  const Robot::Point rp = get_relative_position(grid);
  Robot::Vector desired_vector = target_point - rp;

  move(desired_vector);
}

ExplorationBot::ExplorationBot(const Robot::Point &start_pos,
                               const Robot::Vector &start_dir, bool clockwise)
    : Bot(start_pos), clockwise_following(clockwise) {

  direction = start_dir;
}

void ExplorationBot::update_grid(OccupationGrid *grid) {
  const Robot::Point rp = get_relative_position(grid);
  grid->mark_cells(rp, current_readings);
}

void ExplorationBot::explore(const OccupationGrid *grid) {

  if (phase == ExplorationPhase::Complete) {
    return;
  }

  if (phase == ExplorationPhase::WallDiscovery) {
    phase1_wall_discovery();
    return;
  }

  if (phase == ExplorationPhase::WallAlignment) {
    phase2_wall_alignment(grid);
    return;
  }

  if (phase == ExplorationPhase::WallFollowing) {
    phase3_wall_following(grid);
    return;
  }

  if (phase == ExplorationPhase::RegionAlignment) {
    phase5_region_alignment(grid);
    return;
  }

  if (phase == ExplorationPhase::RegionExploration) {
    phase6_region_exploration(grid);
    return;
  }
}

void ExplorationBot::draw(const DrawData &draw_data) const {
  // draw_readings(draw_data);
  draw_body(draw_data);
  // draw_lidar(draw_data);
}
