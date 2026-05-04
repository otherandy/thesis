#include "ExplorationBot.hpp"
#include "Utils.hpp"
#include <CGAL/linear_least_squares_fitting_2.h>

const Vector NORTH(0, -1);
const Vector SOUTH(0, 1);
const Vector EAST(1, 0);
const Vector WEST(-1, 0);

void ExplorationBot::get_input_and_move()
{
  if (IsKeyPressed(KEY_L))
  {
    draw_as_hud = !draw_as_hud;
  }

  if (IsKeyPressed(KEY_R))
  {
    reset();
    return;
  }

  if (IsKeyPressed(KEY_P))
  {
    is_paused = !is_paused;
    return;
  }

  if (exploration_phase != ExplorationPhase::Idle)
  {
    return;
  }

  if (IsKeyPressed(KEY_SPACE))
  {
    exploration_data.start_point = relative_position;
    exploration_phase = ExplorationPhase::WallDiscovery;
  }

  if (IsKeyDown(KEY_UP))
  {
    move(NORTH);
  }
  if (IsKeyDown(KEY_DOWN))
  {
    move(SOUTH);
  }
  if (IsKeyDown(KEY_LEFT))
  {
    move(WEST);
  }
  if (IsKeyDown(KEY_RIGHT))
  {
    move(EAST);
  }
}

void ExplorationBot::reset()
{
  relative_position = Point(0.0, 0.0);
  exploration_phase = ExplorationPhase::Idle;
  exploration_grid = std::make_shared<OccupationGrid>(START_POSITION);

  exploration_data = ExplorationData{};
  exploration_data.random_direction = get_random_heading();

  movement_data = MovementData{};
  current_frontier_region_id = 0;
  is_paused = false;

  frontier_region_graph.clear();
  const vertex_t root = boost::add_vertex(frontier_region_graph);
  traversal_dfs = std::make_shared<StepDFS>(frontier_region_graph, root);
  frontier_regions.clear();

  Bot::reset();
}

void ExplorationBot::move(const Vector &dir)
{
  relative_position = relative_position + Bot::move(dir);

  if (exploration_phase != ExplorationPhase::Idle)
  {
    Bot::update_visited_positions();
  }
}

void ExplorationBot::run_exploration()
{
  if (exploration_phase == ExplorationPhase::Idle ||
      exploration_phase == ExplorationPhase::Completed ||
      is_paused)
  {
    return;
  }

  if (exploration_phase == ExplorationPhase::WallDiscovery)
  {
    phase1_wall_discovery();
    return;
  }

  if (exploration_phase == ExplorationPhase::WallAlignment)
  {
    phase2_wall_alignment();
    return;
  }

  if (exploration_phase == ExplorationPhase::WallFollowing)
  {
    phase3_wall_following();
    return;
  }

  if (exploration_phase == ExplorationPhase::RegionDiscovery)
  {
    phase4_region_discovery();
    return;
  }

  if (exploration_phase == ExplorationPhase::RegionAlignment)
  {
    phase5_region_alignment();
    return;
  }

  if (exploration_phase == ExplorationPhase::RegionExploration)
  {
    phase6_region_exploration();
    return;
  }
}

void ExplorationBot::phase1_wall_discovery()
{
  for (const auto &r : current_readings)
  {
    if (r.distance < LIDAR_RADIUS)
    {
      std::cout << "EXPLORATION: Wall detected at position ("
                << relative_position.x() << ", " << relative_position.y()
                << ")\n";
      exploration_phase = ExplorationPhase::WallAlignment;
      return;
    }
  }

  move(exploration_data.random_direction);
  exploration_grid->mark_cells(relative_position, current_readings);
}

void ExplorationBot::phase2_wall_alignment()
{
  const Reading &closest_reading = current_readings[closest_wall_reading_index.value()];

  if (closest_reading.distance <= DESIRED_WALL_DISTANCE)
  {
    exploration_data.first_wall_point = relative_position;
    std::cout << "EXPLORATION: Aligned with wall at position ("
              << relative_position.x() << ", " << relative_position.y()
              << ")\n";
    exploration_phase = ExplorationPhase::WallFollowing;
    return;
  }

  const Vector to_wall = Vector(
      cos(closest_reading.angle),
      sin(closest_reading.angle));

  move(to_wall);
  exploration_grid->mark_cells(relative_position, current_readings);
}

void ExplorationBot::phase3_wall_following()
{
  const Vector wall_vector = calculate_wall_correction_vector();
  const Vector desired_vector = movement_data.current_follow_vector *
                                    (1.0 - WALL_DISTANCE_STRENGTH) +
                                wall_vector * WALL_DISTANCE_STRENGTH;

  move(desired_vector);
  exploration_grid->mark_cells(relative_position, current_readings);

  const double distance = std::sqrt(
      CGAL::squared_distance(relative_position,
                             exploration_data.first_wall_point));

  if (!exploration_grid->was_frontier_cell_added() &&
      distance < speed * 2)
  {
    std::cout << "EXPLORATION: Completed wall following loop.\n";

    exploration_phase = ExplorationPhase::RegionDiscovery;
  }
}

void ExplorationBot::create_follow_vector()
{
  if (!closest_wall_reading_index)
  {
    return;
  }

  std::array<Point, WALL_POINT_COUNT> wall_points;

  for (int offset = -READING_OFFSET; offset <= READING_OFFSET; ++offset)
  {
    const std::size_t idx = relative_index(
        closest_wall_reading_index.value(), offset);
    wall_points[offset + READING_OFFSET] = reading_index_to_point(idx);
  }

  CGAL::Line_2<Kernel> fitted_line;
  CGAL::linear_least_squares_fitting_2(
      wall_points.begin(),
      wall_points.end(),
      fitted_line,
      CGAL::Dimension_tag<0>());

  const int before_idx = relative_index(
      closest_wall_reading_index.value(), PREV_INDEX);
  const int after_idx = relative_index(
      closest_wall_reading_index.value(), NEXT_INDEX);
  const Point before_point = reading_index_to_point(before_idx);
  const Point after_point = reading_index_to_point(after_idx);
  const Vector direction_hint = after_point - before_point;

  Vector fitted_vector = fitted_line.to_vector();
  if (fitted_vector * direction_hint < 0)
  {
    fitted_vector = -fitted_vector;
  }

  movement_data.current_follow_vector = fitted_vector;
}

inline Vector ExplorationBot::calculate_wall_correction_vector() const
{
  if (!closest_wall_reading_index)
  {
    return Vector(0, 0);
  }

  const Reading &r = current_readings[closest_wall_reading_index.value()];
  const double distance_error = r.distance - DESIRED_WALL_DISTANCE;
  const Vector to_wall = Vector(cos(r.angle), sin(r.angle));

  return to_wall * distance_error;
}

void ExplorationBot::phase4_region_discovery()
{
  compute_frontier_regions(&frontier_regions,
                           exploration_grid->get_grid(),
                           traversal_dfs,
                           current_frontier_region_id);

  if (exploration_grid->get_frontier_cell_count() <= 2)
  {
    std::cout << "EXPLORATION: No frontier cells found. Exploration completed.\n";
    exploration_phase = ExplorationPhase::Completed;
    return;
  }

  while (true)
  {
    const std::optional<vertex_t> next_region = traversal_dfs->next();
    if (!next_region)
    {
      std::cout << "EXPLORATION: No frontier regions found. Exploration completed.\n";
      exploration_phase = ExplorationPhase::Completed;
      return;
    }

    if (*next_region == 0)
    {
      movement_data.target_point = relative_position;
      break;
    }

    if (*next_region > frontier_regions.size())
    {
      continue;
    }

    const FrontierRegion *target_region = get_frontier_region_by_id(frontier_regions, *next_region);

    if (target_region->explored)
    {
      continue;
    }

    current_frontier_region_id = *next_region;
    movement_data.target_point = target_region->get_closest_point(relative_position);
    break;
  }

  std::cout << "EXPLORATION: Targeting frontier region "
            << current_frontier_region_id
            << " at position ("
            << movement_data.target_point.x() << ", "
            << movement_data.target_point.y() << ").\n";
  exploration_phase = ExplorationPhase::RegionAlignment;
}

void ExplorationBot::phase5_region_alignment()
{
  Vector desired_vector;

  if (exploration_grid->there_is_obstacle_between(
          relative_position, movement_data.target_point) &&
      closest_wall_reading_index)
  {
    const Vector wall_vector = calculate_wall_correction_vector();
    desired_vector = movement_data.current_follow_vector *
                         (1.0 - WALL_DISTANCE_STRENGTH) +
                     wall_vector * WALL_DISTANCE_STRENGTH;
  }
  else
  {
    desired_vector = movement_data.target_point - relative_position;
  }

  move(desired_vector);
  exploration_grid->mark_cells(relative_position, current_readings);

  const double distance = std::sqrt(
      CGAL::squared_distance(relative_position, movement_data.target_point));

  if (distance < LIDAR_RADIUS / 2)
  {
    const auto target_cell = exploration_grid->get_cell_from_position(movement_data.target_point);

    if (target_cell.state != CellState::Frontier)
    {
      std::cout << "EXPLORATION: Aligned with region "
                << current_frontier_region_id << "\n";

      exploration_phase = ExplorationPhase::RegionExploration;
      return;
    }
  }
}

void ExplorationBot::phase6_region_exploration()
{
  if (current_frontier_region_id == 0)
  {
    exploration_phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  FrontierRegion *current_region = const_cast<FrontierRegion *>(
      get_frontier_region_by_id(frontier_regions, current_frontier_region_id));

  if (current_region->explored)
  {
    std::cout << "EXPLORATION: Region "
              << current_frontier_region_id
              << " already explored. Moving to next region.\n";
    exploration_phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  std::optional<Point> closest_unexplored_opt = current_region->get_closest_unexplored(relative_position);

  if (!closest_unexplored_opt)
  {
    std::cout << "EXPLORATION: No unexplored cells found in region "
              << current_frontier_region_id << ". Moving to next region.\n";
    current_region->explored = true;
    exploration_phase = ExplorationPhase::RegionDiscovery;
    return;
  }

  Vector desired_vector;

  if (exploration_grid->there_is_obstacle_between(
          relative_position, closest_unexplored_opt.value()) &&
      closest_wall_reading_index)
  {
    const Vector wall_vector = calculate_wall_correction_vector();
    desired_vector = movement_data.current_follow_vector *
                         (1.0 - WALL_DISTANCE_STRENGTH) +
                     wall_vector * WALL_DISTANCE_STRENGTH;
  }
  else
  {
    desired_vector = closest_unexplored_opt.value() - relative_position;
  }

  move(desired_vector);
  exploration_grid->mark_cells(relative_position, current_readings);
}

void ExplorationBot::draw_follow_vector(DrawData draw_data) const
{
  if (!closest_wall_reading_index)
  {
    return;
  }

  const Point &pos = get_real_position();
  const int endPosX = (pos.x() + movement_data.current_follow_vector.x()) * draw_data.scale_factor + draw_data.offset_x;
  const int endPosY = (pos.y() + movement_data.current_follow_vector.y()) * draw_data.scale_factor + draw_data.offset_y;

  DrawLine(pos.x() * draw_data.scale_factor + draw_data.offset_x,
           pos.y() * draw_data.scale_factor + draw_data.offset_y,
           endPosX, endPosY,
           GREEN);
}

void ExplorationBot::draw_target_point(DrawData draw_data) const
{
  if (movement_data.target_point == Point(0, 0))
  {
    return;
  }

  const Point &pos = get_real_position();
  const Point target_screen_pos = Point(
      pos.x() + (movement_data.target_point.x() - relative_position.x()),
      pos.y() + (movement_data.target_point.y() - relative_position.y()));

  DrawCircle(
      target_screen_pos.x() * draw_data.scale_factor + draw_data.offset_x,
      target_screen_pos.y() * draw_data.scale_factor + draw_data.offset_y,
      DRAWN_POINT_RADIUS,
      ORANGE);
}

ExplorationBot::ExplorationBot(const Point &start_pos)
    : Bot(start_pos),
      exploration_grid(std::make_shared<OccupationGrid>(start_pos))
{
  exploration_data.random_direction = EAST;

  vertex_t root = boost::add_vertex(frontier_region_graph);
  traversal_dfs = std::make_shared<StepDFS>(frontier_region_graph, root);
}

void ExplorationBot::update()
{
  get_input_and_move();
  take_lidar_readings();
  create_follow_vector();
  run_exploration();
}

void ExplorationBot::draw(DrawData draw_data) const
{
  exploration_grid->draw(draw_data);
  // draw_path(draw_data);
  draw_readings(draw_data);
  draw_body(draw_data);
  draw_lidar(draw_data);
  draw_follow_vector(draw_data);
  draw_target_point(draw_data);
  draw_position_text();
}

void ExplorationBot::grid_to_file(const std::string &filename) const
{
  exploration_grid->save_to_file(filename);
}
