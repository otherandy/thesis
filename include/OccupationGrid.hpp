#ifndef OCCUPATION_GRID_HPP
#define OCCUPATION_GRID_HPP

#include "Bot.hpp"
#include "Environment.hpp"
#include <raylib-cpp.hpp>

constexpr double CELL_SIZE = 0.1;
const double INV_CELL_SIZE = 1.0 / CELL_SIZE;
constexpr std::size_t MAP_WIDTH = (ENV_WIDTH * 2.0 / CELL_SIZE);
constexpr std::size_t MAP_HEIGHT = (ENV_HEIGHT * 2.0 / CELL_SIZE);
constexpr std::size_t MAP_SIZE = MAP_WIDTH * MAP_HEIGHT;

enum class CellState
{
  Unknown,
  Free,
  Occupied,
  Visited,
  Frontier
};

struct Cell
{
  CellState state;
  int frontier_id;
};

const std::map<CellState, Color> CellColors = {
    {CellState::Unknown, GRAY},
    {CellState::Free, YELLOW},
    {CellState::Occupied, BLACK},
    {CellState::Visited, RED},
    {CellState::Frontier, BLUE}};

const std::array<Color, 9> FrontierColors = {
    BLUE,
    LIME,
    VIOLET,
    DARKBLUE,
    DARKGREEN,
    DARKPURPLE,
    SKYBLUE,
    GREEN,
    PURPLE,
};

struct FrontierRegion
{
  int id;
  std::vector<Point> cell_centers;

  std::vector<Point> get_points() const;
  Polygon to_polygon() const;
  Point get_closest_from(const Point &pos) const;
};

class OccupationGrid
{
private:
  Point origin;
  std::array<Cell, MAP_SIZE> grid;

  int current_frontier_id = 0;
  bool frontier_cell_was_added = false;
  std::vector<FrontierRegion> frontier_regions;

  int number_of_frontier_cells = 0;

  // Track bounding box of frontier cells for optimization
  int min_frontier_idx = std::numeric_limits<int>::max();
  int max_frontier_idx = std::numeric_limits<int>::min();

  inline int coord_to_cell_x(double x) const;
  inline int coord_to_cell_y(double y) const;
  inline bool is_valid_cell(int cell_x, int cell_y) const;
  inline int get_cell_index(int cell_x, int cell_y) const;
  inline std::pair<int, int> get_cell_coordinates(int idx) const;
  inline Cell get_cell_from_position(const Point &pos) const;
  bool verify_and_mark_cell(int cell_x, int cell_y, CellState new_state);
  void draw_cell(int cell_x, int cell_y, float scale_factor,
                 float offset_x, float offset_y) const;

public:
  OccupationGrid(Point origin);
  bool was_frontier_cell_added() const;
  int get_frontier_cell_count() const;
  const std::vector<FrontierRegion> &get_frontier_regions() const;
  const Point get_cell_center(int idx) const;
  const int get_frontier_id_from(const Point &pos) const;
  void mark_cells(const Point &relative_position,
                  const std::array<Reading, MAX_LIDAR_SAMPLES> &readings);
  void compute_frontier_regions();
  std::vector<Point> calculate_path_from(const Point start,
                                         const int frontier_id) const;
  const Point target_frontier_from_readings(const Point &relative_position,
                                            const std::array<Reading, MAX_LIDAR_SAMPLES> &readings) const;
  void draw(float scale_factor, float offset_x, float offset_y) const;
  void draw_frontier_count() const;
  void save_to_file(const std::string &filename) const;
};

#endif