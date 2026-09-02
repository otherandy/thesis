#include "ExplorationBot.hpp"
#include "Bot.hpp"
#include "Cell.hpp"
#include "OccupationGrid.hpp"
#include "Utils.hpp"
#include "cgal_types.hpp"
#include <CGAL/linear_least_squares_fitting_2.h>
#include <algorithm>
#include <cmath>

ExplorationBot::ExplorationBot(std::size_t id, const Robot::Point &start_pos,
                               const Robot::Vector &start_dir, bool clockwise,
                               std::shared_ptr<Environment> env)
    : Bot(id, start_pos, env), start_point(start_pos),
      start_direction(start_dir), direction(start_dir),
      clockwise_following(clockwise) {}

Robot::Point
ExplorationBot::get_relative_position(const OccupationGrid *grid) const {
  const Robot::Point rp = get_real_position();
  const Robot::Point c = grid->get_origin();
  return Robot::Point(rp.x() - c.x(), rp.y() - c.y());
}

void ExplorationBot::reset() {
  phase = ExplorationPhase::WallDiscovery;
  direction = start_direction;

  physical_time.reset();
  virtual_time.reset();
  alignment_time.reset();
  exploration_time.reset();

  Bot::reset(start_point);
}

Robot::Vector ExplorationBot::move(const Robot::Vector &dir) {
  direction = dir;
  Robot::Vector delta = Bot::move(dir);
  distance_traveled += std::sqrt(delta.squared_length());
  return delta;
}

void ExplorationBot::update_grid(OccupationGrid *grid) {
  const Robot::Point rp = get_relative_position(grid);
  grid->mark_cells(rp, readings);
}

void ExplorationBot::pause_timers() {
  physical_time.pause();
  virtual_time.pause();
  alignment_time.pause();
  exploration_time.pause();
}

void ExplorationBot::explore(const OccupationGrid *grid) {

  if (phase == ExplorationPhase::Complete) {
    return;
  }

  if (phase == ExplorationPhase::WallDiscovery) {
    phase1_wall_discovery(grid);
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

void ExplorationBot::phase1_wall_discovery(const OccupationGrid *grid) {
  physical_time.start();
  alignment_time.start();

  const Robot::Point rp = get_relative_position(grid);
  const auto g = grid->get_data();

  for (const Reading &r : readings) {
    if (r.distance < LIDAR_RADIUS) {
      const Robot::Point p = point_at_reading(rp, r);
      const Index2D index = get_cell_index_from(p.x(), p.y());
      const Cell *obstacle_cell = (*g)[index.first][index.second].get();

      if (obstacle_cell->state == CellState::Occupied &&
          obstacle_cell->frontier_id.has_value()) {
        direction = rp - p;
        continue;
      }

      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  move(direction);
}

void ExplorationBot::phase2_wall_alignment(const OccupationGrid *grid) {
  physical_time.start();
  alignment_time.start();

  const Reading &closest_reading = readings[closest_wall_reading_index.value()];

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

Robot::Vector
ExplorationBot::compute_wall_following_vector(const OccupationGrid *grid) {
  if (!closest_wall_reading_index.has_value()) {
    return -direction;
  }

  std::size_t ref = closest_wall_reading_index.value();

  const Robot::Point rp = get_relative_position(grid);

  std::optional<std::size_t> side_ref = std::nullopt;
  double best_dist = std::numeric_limits<double>::infinity();

  for (std::size_t i = 0; i < LIDAR_SAMPLES; ++i) {
    const Reading &r = readings[i];
    if (r.distance >= LIDAR_RADIUS) {
      continue;
    }

    const Robot::Vector to_hit(cos(r.angle), sin(r.angle));

    const double cross_z =
        direction.x() * to_hit.y() - direction.y() * to_hit.x();
    const bool on_right = (cross_z < 0.0);
    const bool on_left = (cross_z > 0.0);
    const bool keep = clockwise_following ? on_right : on_left;

    if (!keep) {
      continue;
    }

    if (r.distance < best_dist) {
      best_dist = r.distance;
      side_ref = i;
    }
  }

  if (side_ref) {
    ref = *side_ref;
  }

  std::vector<Robot::Point> wall_points;
  wall_points.reserve(LIDAR_SAMPLES / 2);

  const int prev_index = clockwise_following ? 1 : -1;
  const int next_index = clockwise_following ? -1 : 1;

  // prev
  {
    std::size_t idx = ref;
    double last_d = readings[ref].distance;
    for (std::size_t steps = 0; steps < LIDAR_SAMPLES / 4; ++steps) {
      idx = relative_index(idx, prev_index);
      const double d = readings[idx].distance;

      if (idx == ref || d >= LIDAR_RADIUS ||
          std::abs(d - last_d) > LIDAR_DISTANCE_THRESHOLD) {
        break;
      }

      last_d = d;
      wall_points.emplace_back(point_at_reading(rp, readings[idx]));
    }
    std::reverse(wall_points.begin(), wall_points.end());
  }

  wall_points.emplace_back(point_at_reading(rp, readings[ref]));

  // next
  {
    std::size_t idx = ref;
    double last_d = readings[ref].distance;
    for (std::size_t steps = 0; steps < LIDAR_SAMPLES / 4; ++steps) {
      idx = relative_index(idx, next_index);
      const double d = readings[idx].distance;

      if (idx == ref || d >= LIDAR_RADIUS ||
          std::abs(d - last_d) > LIDAR_DISTANCE_THRESHOLD) {
        break;
      }

      last_d = d;
      wall_points.emplace_back(point_at_reading(rp, readings[idx]));
    }
  }

  CGAL::Line_2<Robot::Kernel> fitted_line;
  CGAL::linear_least_squares_fitting_2(wall_points.begin(), wall_points.end(),
                                       fitted_line, CGAL::Dimension_tag<0>());
  Robot::Vector forward = fitted_line.to_vector();
  forward = normalize_vector(forward);

  const Reading &ref_r = readings[ref];
  const Robot::Vector to_wall(cos(ref_r.angle), sin(ref_r.angle));

  const double cross_z = to_wall.x() * forward.y() - to_wall.y() * forward.x();

  if (clockwise_following != (cross_z > 0.0)) {
    forward = -forward;
  }

  double distance_error = ref_r.distance - DESIRED_WALL_DISTANCE;
  distance_error =
      std::clamp(distance_error, -DESIRED_WALL_DISTANCE, DESIRED_WALL_DISTANCE);

  return forward + to_wall * distance_error;
}

void ExplorationBot::phase3_wall_following(const OccupationGrid *grid) {
  physical_time.start();
  exploration_time.start();

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
}

bool ExplorationBot::path_blocked_to(const Robot::Vector &target) const {
  if (!closest_wall_reading_index.has_value()) {
    return false;
  }

  const double target_angle = std::atan2(target.y(), target.x());
  const double target_dist = std::sqrt(target.squared_length());

  for (const auto &r : readings) {
    const double diff = std::fmod(r.angle - target_angle, 2.0 * M_PI);

    if (std::abs(diff) > ANGLE_STEP * 2) {
      continue;
    }

    if (r.distance < LIDAR_RADIUS && r.distance < target_dist &&
        r.distance <= readings[*closest_wall_reading_index].distance +
                          DESIRED_WALL_DISTANCE / 2) {
      return true;
    }
  }

  return false;
}

void ExplorationBot::phase5_region_alignment(const OccupationGrid *grid) {
  virtual_time.start();
  alignment_time.start();

  const Robot::Point rp = get_relative_position(grid);

  if (closest_wall_reading_index) {
    const Robot::Point closest_point =
        point_at_reading(rp, readings[*closest_wall_reading_index]);
    const Index2D index =
        get_cell_index_from(closest_point.x(), closest_point.y());

    const auto g = grid->get_data();
    const Cell *obstacle_cell = (*g)[index.first][index.second].get();

    if (obstacle_cell->state == CellState::Occupied &&
        !obstacle_cell->frontier_id.has_value()) {
      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  const double distance = std::sqrt(CGAL::squared_distance(rp, target_point));

  if (distance < SPEED * 2) {
    started_surround = false;
    goal_distance = 0;
    phase = ExplorationPhase::RegionExploration;
    return;
  }

  Robot::Vector desired_vector = target_point - rp;

  const bool is_blocked = path_blocked_to(desired_vector);

  if (is_blocked || (started_surround && distance > goal_distance)) {
    if (!started_surround) {
      goal_distance = distance;
      started_surround = true;
    }

    desired_vector = compute_wall_following_vector(grid);
  }

  if (distance < goal_distance) {
    goal_distance = distance;
  }

  move(desired_vector);
}

void ExplorationBot::phase6_region_exploration(const OccupationGrid *grid) {
  virtual_time.start();
  exploration_time.start();

  const Robot::Point rp = get_relative_position(grid);

  if (closest_wall_reading_index) {
    const Robot::Point closest_point =
        point_at_reading(rp, readings[*closest_wall_reading_index]);
    const Index2D index =
        get_cell_index_from(closest_point.x(), closest_point.y());

    const auto g = grid->get_data();
    const Cell *obstacle_cell = (*g)[index.first][index.second].get();

    if (obstacle_cell->state == CellState::Occupied &&
        !obstacle_cell->frontier_id.has_value()) {
      phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  const double distance = std::sqrt(CGAL::squared_distance(rp, target_point));
  Robot::Vector desired_vector = target_point - rp;

  const bool is_blocked = path_blocked_to(desired_vector);

  if (is_blocked) {
    phase = ExplorationPhase::RegionAlignment;
  }

  move(desired_vector);
}

void ExplorationBot::draw_direction(const DrawData &draw_data) const {
  const auto real = get_real_position();
  float pos_x = real.x() * draw_data.scale_factor + draw_data.offset_x;
  float pos_y = real.y() * draw_data.scale_factor + draw_data.offset_y;

  float end_x = pos_x + direction.x() * draw_data.scale_factor;
  float end_y = pos_y + direction.y() * draw_data.scale_factor;

  DrawLine(pos_x, pos_y, end_x, end_y, raylib::GREEN);
}

void ExplorationBot::draw(const DrawData &draw_data) const {
  // draw_readings(draw_data);

  raylib::Color color;

  if (phase == ExplorationPhase::WallDiscovery ||
      phase == ExplorationPhase::WallAlignment ||
      phase == ExplorationPhase::WallFollowing) {
    color = raylib::RED;
  } else if (phase == ExplorationPhase::Idle) {
    color = raylib::GRAY;
  } else if (phase == ExplorationPhase::RegionDiscovery) {
    color = raylib::GREEN;
  } else if (phase == ExplorationPhase::RegionAlignment) {
    color = raylib::BLUE;
  } else if (phase == ExplorationPhase::RegionExploration) {
    color = raylib::VIOLET;
  } else {
    color = raylib::ORANGE;
  }

  draw_body(draw_data, color);
  draw_direction(draw_data);
  draw_range(draw_data);
}
