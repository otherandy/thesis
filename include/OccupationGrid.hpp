#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "FrontierRegion.hpp"
#include "Bot.hpp"

class OccupationGrid
{
private:
  Point origin;
  Grid2D<Cell> grid;

  std::vector<FrontierRegion> frontier_regions;
  int current_frontier_id = 0;

  bool frontier_cell_was_added = false;
  int number_of_frontier_cells = 0;

  bool verify_and_mark_cell(Index2D index,
                            CellState new_state);

  void draw_cell(Index2D index,
                 float scale_factor,
                 float offset_x, float offset_y) const;

public:
  OccupationGrid(Point origin);
  bool was_frontier_cell_added() const;
  int get_frontier_cell_count() const;

  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);
  Cell &get_cell_from_position(const Point &position);

  void compute_frontier_regions();
  const FrontierRegion *get_nearest_frontier_region(const Point &position) const;

  void draw(float scale_factor, float offset_x, float offset_y) const;
  void draw_cell_centers(float scale_factor,
                         float offset_x, float offset_y) const;
  void draw_frontier_count() const;
  void save_to_file(const std::string &filename) const;
};

#endif