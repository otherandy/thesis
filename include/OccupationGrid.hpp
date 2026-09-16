#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"
#include "Cell.hpp"
#include "DrawUtils.hpp"
#include "Graph.hpp"
#include "Grid.hpp"

class OccupationGrid {
public:
  OccupationGrid(const Robot::Point &origin);

  bool found_exterior = false;
  std::size_t frontier_cell_count = 0;

  const auto get_data() const { return &grid; };
  const Robot::Point get_origin() const { return origin; };

  void enable_debug() { debug = true; }

  void mark_cells(const Robot::Point &relative_position,
                  const std::array<Reading, LIDAR_SAMPLES> &readings,
                  double radius);

  void clean_cells();
  void compute_frontier_regions(DynamicScheduler *sched);
  void compute_physical_obstacles(DynamicScheduler *sched);
  Cell *unmarked_obstacles_exist();

  void draw(const DrawData &draw_data) const;
  void draw_info(const DrawData &draw_data) const;
  void save_to_file() const;

private:
  const Robot::Point origin;
  Grid2D<std::unique_ptr<Cell>> grid;

  bool debug = false;

  Index2D grid_min = {MAP_HEIGHT, MAP_WIDTH};
  Index2D grid_max = {0, 0};

  void remove_dead_frontier_cells();
  void remove_dead_free_cells();

  bool has_neighbor_state(const Index2D &idx, CellState state,
                          bool include_corners = true);
  Cell *find_reference(const Index2D &idx, CellState state);

  void mark_free_along_ray(double start_x, double start_y, double end_x,
                           double end_y, const Index2D &end_cell_index);
  void mark_cell(Index2D index, CellState new_state, bool force_change = false);

  void draw_cell(Index2D index, const DrawData &draw_data) const;
};

#endif
