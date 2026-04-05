#include "FrontierRegion.hpp"
#include <algorithm>
#include <queue>

Point FrontierRegion::get_closest_point(const Point &pos) const
{
  Point closest_point;
  double closest_distance = std::numeric_limits<double>::max();

  for (const Cell *cell : cells)
  {
    const double distance = CGAL::squared_distance(pos, cell->center);
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_point = cell->center;
    }
  }

  return closest_point;
}

std::optional<Point> FrontierRegion::get_closest_unexplored(const Point &pos) const
{
  Point closest_point;
  double closest_distance = std::numeric_limits<double>::max();
  bool found_unexplored = false;

  for (const Cell *cell : cells)
  {
    if (cell->state != CellState::Frontier)
    {
      continue;
    }

    const double distance = CGAL::squared_distance(pos, cell->center);
    if (distance < closest_distance)
    {
      closest_distance = distance;
      closest_point = cell->center;
      found_unexplored = true;
    }
  }

  if (found_unexplored)
  {
    return closest_point;
  }
  else
  {
    return std::nullopt;
  }
}

void compute_frontier_regions(
    std::vector<FrontierRegion> *frontier_regions,
    Grid2D<Cell> &grid,
    std::shared_ptr<StepTraversal> traversal_graph,
    std::size_t current_parent_region_id)
{
  for (int y = 0; y < MAP_HEIGHT; ++y)
  {
    for (int x = 0; x < MAP_WIDTH; ++x)
    {
      Cell &cell = grid[y][x];

      if (cell.state != CellState::Frontier || cell.frontier_id)
      {
        continue;
      }

      FrontierRegion new_region;
      new_region.id = traversal_graph->add_vertex_and_edge(current_parent_region_id);

      std::queue<Cell> to_visit;
      to_visit.push(cell);
      cell.frontier_id = new_region.id;

      while (!to_visit.empty())
      {
        Cell current_cell = to_visit.front();
        to_visit.pop();

        new_region.cells.push_back(&current_cell);

        for (Index2D neighbor_cell : current_cell.get_neighbors())
        {
          Cell &neighbor = grid[neighbor_cell.first][neighbor_cell.second];

          if (neighbor.state == CellState::Frontier &&
              neighbor.frontier_id == std::nullopt)
          {
            neighbor.frontier_id = new_region.id;
            to_visit.push(neighbor);
          }
        }
      }

      frontier_regions->push_back(new_region);
    }
  }

  traversal_graph->post_update();
}

std::size_t get_nearest_frontier_region_id(
    const std::vector<FrontierRegion> &regions,
    const Point &position)
{
  std::size_t nearest_region_id = 0;
  double nearest_distance = std::numeric_limits<double>::max();

  for (const FrontierRegion &region : regions)
  {
    if (region.explored)
    {
      continue;
    }

    const double distance = CGAL::squared_distance(
        position, region.get_closest_point(position));

    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest_region_id = region.id;
    }
  }

  return nearest_region_id;
}
