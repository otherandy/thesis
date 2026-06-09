#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"
#include "Cell.hpp"
#include "DrawUtils.hpp"
#include "FrontierRegion.hpp"
#include "Graph.hpp"

class OccupationGrid {
private:
  const Point origin = environment_center();
  Grid2D<Cell> grid;

  bool mark_cell(Index2D index, CellState new_state);
  void draw_cell(Index2D index, const DrawData &draw_data) const;

public:
  std::vector<FrontierRegion> frontier_regions;

  OccupationGrid();

  Grid2D<Cell> &get_grid() { return grid; }
  const Cell &get_cell_from_position(const Point &position) const;

  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);

  FrontierRegion *get_frontier_region_by_id(std::size_t id);
  const FrontierRegion *get_frontier_region_by_id(std::size_t id) const;

  void compute_frontier_regions(std::shared_ptr<StepTraversal> traversal_graph,
                                std::size_t &current_parent_region_id);

  void draw(const DrawData &draw_data) const;
  void save_to_file(const std::string &filename) const;
};

#endif
