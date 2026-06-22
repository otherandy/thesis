#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"
#include "Cell.hpp"
#include "DrawUtils.hpp"
#include "Graph.hpp"
#include "Grid.hpp"

class OccupationGrid {
public:
  OccupationGrid();

  std::size_t frontier_cell_count = 0;

  const auto get_data() const { return &grid; };
  const Robot::Point get_origin() const { return origin; };

  void mark_cells(const Robot::Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);

  void compute_frontier_regions(DynamicScheduler *sched);

  void draw(const DrawData &draw_data) const;
  void save_to_file(const std::string &filename) const;

private:
  const Robot::Point origin = environment_center();
  Grid2D<std::unique_ptr<Cell>> grid;

  Index2D grid_min = {MAP_HEIGHT, MAP_WIDTH};
  Index2D grid_max = {0, 0};

  bool mark_cell(Index2D index, CellState new_state);
  void draw_cell(Index2D index, const DrawData &draw_data) const;
};

#endif
