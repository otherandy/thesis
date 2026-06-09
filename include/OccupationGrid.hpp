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

  bool frontier_cell_was_added = false;
  std::size_t number_of_frontier_cells = 0;

  bool mark_cell(Index2D index, CellState new_state);
  void draw_cell(Index2D index, const DrawData &draw_data) const;

public:
  std::vector<FrontierRegion> frontier_regions;

  OccupationGrid();

  Grid2D<Cell> &get_grid() { return grid; }

  bool was_frontier_cell_added() const { return frontier_cell_was_added; }
  int get_frontier_cell_count() const { return number_of_frontier_cells; }

  const Cell &get_cell_from_position(const Point &position) const;

  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);

  bool there_is_obstacle_between(const Point &from, const Point &to) const;

  FrontierRegion *get_frontier_region_by_id(std::size_t id);
  const FrontierRegion *get_frontier_region_by_id(std::size_t id) const;

  void compute_frontier_regions(std::shared_ptr<StepTraversal> traversal_graph,
                                std::size_t &current_parent_region_id);

  std::size_t get_nearest_frontier_region_id(const Point &position) const;

  void draw(const DrawData &draw_data) const;
  void save_to_file(const std::string &filename) const;
};

#endif
