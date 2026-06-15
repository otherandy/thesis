#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"
#include "Cell.hpp"
#include "DrawUtils.hpp"
#include "Graph.hpp"
#include "Grid.hpp"

class OccupationGrid {
private:
  const Robot::Point origin = environment_center();
  Grid2D<std::unique_ptr<Cell>> grid;

  std::size_t grid_min_y = MAP_HEIGHT;
  std::size_t grid_max_y = 0;
  std::size_t grid_min_x = MAP_WIDTH;
  std::size_t grid_max_x = 0;

  bool mark_cell(Index2D index, CellState new_state);
  void draw_cell(Index2D index, const DrawData &draw_data) const;

public:
  OccupationGrid();

  const auto get_data() const { return &grid; };
  const Robot::Point get_origin() const { return origin; };

  void mark_cells(const Robot::Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);

  void compute_frontier_regions(DynamicScheduler *sched);

  void draw(const DrawData &draw_data) const;
  void save_to_file(const std::string &filename) const;
};

#endif
