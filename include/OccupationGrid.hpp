#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Cell.hpp"
#include "Bot.hpp"

class OccupationGrid
{
private:
  Point origin;
  Grid2D<Cell> grid;

  bool frontier_cell_was_added = false;
  std::size_t number_of_frontier_cells = 0;

  bool mark_cell(Index2D index,
                 CellState new_state);

  void draw_cell(Index2D index, float scale_factor,
                 float offset_x, float offset_y) const;
  void draw_cell_center(Index2D index, float scale_factor,
                        float offset_x, float offset_y) const;

public:
  OccupationGrid(Point origin);

  Grid2D<Cell> &get_grid() { return grid; }

  bool was_frontier_cell_added() const { return frontier_cell_was_added; }
  int get_frontier_cell_count() const { return number_of_frontier_cells; }

  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);
  Cell &get_cell_from_position(const Point &position);

  bool there_is_obstacle_between(const Point &from, const Point &to) const;

  void draw(float scale_factor, float offset_x, float offset_y) const;
  void draw_frontier_count() const;
  void save_to_file(const std::string &filename) const;
};

#endif