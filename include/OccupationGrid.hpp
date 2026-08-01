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

  std::size_t frontier_cell_count = 0;
  bool found_exterior = false;

  const auto get_data() const { return &grid; };
  const Robot::Point get_origin() const { return origin; };

  void mark_cells(const Robot::Point &relative_position,
                  const std::array<Reading, LIDAR_SAMPLES> &readings);

  void compute_frontier_regions(DynamicScheduler *sched);
  void compute_physical_obstacles(DynamicScheduler *sched);

  void draw(const DrawData &draw_data) const;
  void save_to_file(const std::string &filename) const;

private:
  const Robot::Point origin;
  Grid2D<std::unique_ptr<Cell>> grid;

  Index2D grid_min = {MAP_HEIGHT, MAP_WIDTH};
  Index2D grid_max = {0, 0};

  void remove_dead_frontier_cells();
  void remove_dead_free_cells();

  bool has_unknown_neighbor(const Index2D &idx);

  void mark_free_along_ray(double start_x, double start_y, double end_x,
                           double end_y, const Index2D &end_cell_index,
                           std::vector<Index2D> &demoted_frontier_cells);
  void mark_cell(Index2D index, CellState new_state, bool force_change = false);

  void draw_cell(Index2D index, const DrawData &draw_data) const;
};

#endif
