#include "ExplorationBot.hpp"
#include "Bot.hpp"
#include "Utils.hpp"
#include <CGAL/linear_least_squares_fitting_2.h>

const Vector NORTH(0, -1);
const Vector SOUTH(0, 1);
const Vector EAST(1, 0);
const Vector WEST(-1, 0);

void ExplorationBot::get_input_and_move() {
  if (IsKeyPressed(KEY_L)) {
    draw_as_hud = !draw_as_hud;
  }

  if (IsKeyPressed(KEY_R)) {
    reset();
    return;
  }

  if (IsKeyPressed(KEY_P)) {
    is_paused = !is_paused;
    return;
  }

  if (exploration_phase != ExplorationPhase::Idle) {
    return;
  }

  if (IsKeyPressed(KEY_SPACE)) {
    exploration_data.start_point = relative_position;
    exploration_phase = ExplorationPhase::WallDiscovery;
  }

  if (IsKeyDown(KEY_UP)) {
    move(NORTH);
  }
  if (IsKeyDown(KEY_DOWN)) {
    move(SOUTH);
  }
  if (IsKeyDown(KEY_LEFT)) {
    move(WEST);
  }
  if (IsKeyDown(KEY_RIGHT)) {
    move(EAST);
  }
}

void ExplorationBot::reset() {
  relative_position = Point(0.0, 0.0);
  exploration_phase = ExplorationPhase::Idle;
  exploration_grid = std::make_shared<OccupationGrid>(START_POSITION);

  exploration_data = ExplorationData{};
  exploration_data.random_direction = get_random_heading();

  current_frontier_region_id = 0;
  is_paused = false;

  frontier_region_graph.clear();
  const vertex_t root = boost::add_vertex(frontier_region_graph);
  traversal_dfs = std::make_shared<StepDFS>(frontier_region_graph, root);
  frontier_regions.clear();

  Bot::reset();
}

void ExplorationBot::move(const Vector &dir) {
  relative_position = relative_position + Bot::move(dir);

  if (exploration_phase != ExplorationPhase::Idle) {
    Bot::update_visited_positions();
  }
}

void ExplorationBot::run_exploration() {
  if (exploration_phase == ExplorationPhase::Idle ||
      exploration_phase == ExplorationPhase::Completed || is_paused) {
    return;
  }

  if (exploration_phase == ExplorationPhase::WallDiscovery) {
    phase1_wall_discovery();
    return;
  }

  if (exploration_phase == ExplorationPhase::WallAlignment) {
    phase2_wall_alignment();
    return;
  }

  if (exploration_phase == ExplorationPhase::WallFollowing) {
    phase3_wall_following();
    return;
  }

  if (exploration_phase == ExplorationPhase::RegionDiscovery) {
    phase4_region_discovery();
    return;
  }

  if (exploration_phase == ExplorationPhase::RegionAlignment) {
    phase5_region_alignment();
    return;
  }

  if (exploration_phase == ExplorationPhase::RegionExploration) {
    phase6_region_exploration();
    return;
  }
}

void ExplorationBot::phase1_wall_discovery() {
  for (const auto &r : current_readings) {
    if (r.distance < LIDAR_RADIUS) {
      exploration_phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  exploration_grid->mark_cells(relative_position, current_readings);
  move(exploration_data.random_direction);
}

void ExplorationBot::phase2_wall_alignment() {
  const Reading &closest_reading =
      current_readings[closest_wall_reading_index.value()];

  if (closest_reading.distance <= DESIRED_WALL_DISTANCE) {
    exploration_data.last_closest_reading = closest_wall_reading_index.value();
    exploration_data.first_wall_point = relative_position;
    exploration_data.left_wall_point = false;
    exploration_phase = ExplorationPhase::WallFollowing;
    return;
  }

  const Vector to_wall =
      Vector(cos(closest_reading.angle), sin(closest_reading.angle));

  exploration_grid->mark_cells(relative_position, current_readings);
  move(to_wall);
}

Vector ExplorationBot::compute_wall_following_vector() {
  std::size_t reference_reading_index;

  const int difference = closest_wall_reading_index.value() -
                         exploration_data.last_closest_reading;

  if (difference % MAX_LIDAR_SAMPLES > MAX_LIDAR_SAMPLES / 2) {
    reference_reading_index = exploration_data.last_closest_reading;
  } else {
    reference_reading_index = closest_wall_reading_index.value();
    exploration_data.last_closest_reading = closest_wall_reading_index.value();
  }

  std::array<Point, WALL_POINT_COUNT> wall_points;

  for (int offset = -READING_OFFSET; offset <= READING_OFFSET; ++offset) {
    const std::size_t idx = relative_index(reference_reading_index, offset);

    if (current_readings[idx].distance >= LIDAR_RADIUS) {
      wall_points[offset + READING_OFFSET] =
          reading_index_to_point(reference_reading_index);
    } else {
      wall_points[offset + READING_OFFSET] = reading_index_to_point(idx);
    }
  }

  CGAL::Line_2<Kernel> fitted_line;
  CGAL::linear_least_squares_fitting_2(wall_points.begin(), wall_points.end(),
                                       fitted_line, CGAL::Dimension_tag<0>());

  const int before_idx = relative_index(reference_reading_index, PREV_INDEX);
  const int after_idx = relative_index(reference_reading_index, NEXT_INDEX);
  const Point before_point = reading_index_to_point(before_idx);
  const Point after_point = reading_index_to_point(after_idx);
  const Vector direction_hint = after_point - before_point;

  Vector fitted_vector = fitted_line.to_vector();
  if (fitted_vector * direction_hint < 0) {
    fitted_vector = -fitted_vector;
  }

  const Reading &r = current_readings[reference_reading_index];
  const double distance_error = r.distance - DESIRED_WALL_DISTANCE;
  const Vector to_wall = Vector(cos(r.angle), sin(r.angle));

  const Vector desired_vector =
      fitted_vector * (1.0 - WALL_DISTANCE_STRENGTH) +
      to_wall * distance_error * WALL_DISTANCE_STRENGTH;

  return desired_vector;
}

void ExplorationBot::phase3_wall_following() {
  Vector desired_vector = compute_wall_following_vector();

  exploration_grid->mark_cells(relative_position, current_readings);
  move(desired_vector);

  const double distance = std::sqrt(CGAL::squared_distance(
      relative_position, exploration_data.first_wall_point));

  if (!exploration_grid->was_frontier_cell_added() && distance < speed * 2 &&
      exploration_data.left_wall_point) {
    std::cout << "EXPLORATION: Completed wall following loop.\n";

    exploration_phase = ExplorationPhase::RegionDiscovery;
  } else if (distance >= speed * 2) {
    exploration_data.left_wall_point = true;
  }
}

void ExplorationBot::phase4_region_discovery() {
  compute_frontier_regions(frontier_regions, exploration_grid->get_grid(),
                           traversal_dfs, current_frontier_region_id);

  if (exploration_grid->get_frontier_cell_count() <= 2) {
    std::cout
        << "EXPLORATION: No frontier cells found. Exploration completed.\n";
    exploration_phase = ExplorationPhase::Completed;
    return;
  }

  while (true) {
    const std::optional<vertex_t> next_region = traversal_dfs->next();
    if (!next_region) {
      std::cout
          << "EXPLORATION: No frontier regions found. Exploration completed.\n";
      exploration_phase = ExplorationPhase::Completed;
      return;
    }

    if (*next_region == 0) {
      exploration_data.target_point = relative_position;
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
    exploration_data.target_point =
        target_region->get_closest_point(relative_position);
    break;
  }

  std::cout << "EXPLORATION: Targeting frontier region "
            << current_frontier_region_id << ".\n";
  exploration_phase = ExplorationPhase::RegionAlignment;
}

void ExplorationBot::phase5_region_alignment() {
  Vector desired_vector;

  if (exploration_grid->there_is_obstacle_between(
          relative_position, exploration_data.target_point) &&
      closest_wall_reading_index) {
    desired_vector = compute_wall_following_vector();
  } else {
    desired_vector = exploration_data.target_point - relative_position;
  }

  exploration_grid->mark_cells(relative_position, current_readings);
  move(desired_vector);

  const double distance = std::sqrt(
      CGAL::squared_distance(relative_position, exploration_data.target_point));

  if (distance < speed * 2) {
    exploration_phase = ExplorationPhase::RegionExploration;
    return;
  }
}

void ExplorationBot::phase6_region_exploration() {
  if (current_frontier_region_id == 0) {
    exploration_phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  auto current_region =
      get_frontier_region_by_id(frontier_regions, current_frontier_region_id);

  if (current_region->explored) {
    exploration_phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  auto closest_unexplored =
      current_region->get_closest_unexplored(relative_position);

  if (!closest_unexplored) {
    std::cout << "EXPLORATION: No unexplored cells found in region "
              << current_frontier_region_id << ". Moving to next region.\n";
    current_region->explored = true;
    exploration_phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  Vector desired_vector;

  if (closest_wall_reading_index) {
    Point closest_wall_point =
        reading_index_to_point(closest_wall_reading_index.value());
    auto obstacle_cell =
        exploration_grid->get_cell_from_position(closest_wall_point);

    if (obstacle_cell.state == CellState::Unknown) {
      exploration_phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  desired_vector = closest_unexplored.value() - relative_position;
  exploration_grid->mark_cells(relative_position, current_readings);
  move(desired_vector);
}

void ExplorationBot::draw_target_point(DrawData draw_data) const {
  if (exploration_data.target_point == Point(0, 0)) {
    return;
  }

  const Point &pos = get_real_position();
  const Point target_screen_pos = Point(
      pos.x() + (exploration_data.target_point.x() - relative_position.x()),
      pos.y() + (exploration_data.target_point.y() - relative_position.y()));

  DrawCircle(
      target_screen_pos.x() * draw_data.scale_factor + draw_data.offset_x,
      target_screen_pos.y() * draw_data.scale_factor + draw_data.offset_y,
      DRAWN_POINT_RADIUS, ORANGE);
}

ExplorationBot::ExplorationBot(const Point &start_pos)
    : Bot(start_pos),
      exploration_grid(std::make_shared<OccupationGrid>(start_pos)) {
  exploration_data.random_direction = EAST;

  vertex_t root = boost::add_vertex(frontier_region_graph);
  traversal_dfs = std::make_shared<StepDFS>(frontier_region_graph, root);
}

void ExplorationBot::update() {
  get_input_and_move();
  take_lidar_readings();
  run_exploration();
}

void ExplorationBot::draw(DrawData draw_data) const {
  exploration_grid->draw(draw_data);
  draw_readings(draw_data);
  draw_body(draw_data);
  draw_lidar(draw_data);
  draw_position_text();
}

void ExplorationBot::grid_to_file(const std::string &filename) const {
  exploration_grid->save_to_file(filename);
}
