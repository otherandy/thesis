#include "Environment.hpp"

Environment::Environment(EnvironmentPreset preset) : preset(preset) {
  const auto env_data = get_selected_environment_data(preset);

  Robot::Polygon outer;
  for (std::size_t i = 0; i < env_data.outer_size; ++i) {
    const auto point = env_data.outer_data[i];
    outer.push_back(Robot::Point(point.first, point.second));
  }

  if (outer.is_clockwise_oriented()) {
    outer.reverse_orientation();
  }

  std::vector<Robot::Polygon> holes;
  for (std::size_t hole_idx = 0; hole_idx < env_data.hole_count; ++hole_idx) {
    Robot::Polygon hole;
    for (std::size_t i = 0; i < env_data.hole_size_list[hole_idx]; ++i) {
      const auto point = env_data.hole_data_list[hole_idx][i];
      hole.push_back(Robot::Point(point.first, point.second));
    }

    if (hole.is_counterclockwise_oriented()) {
      hole.reverse_orientation();
    }

    holes.push_back(hole);
  }

  geometry_ = Robot::PolygonWithHoles(outer, holes.begin(), holes.end());

  auto add_polygon_edges = [&](const auto &poly) {
    for (auto e = poly.edges_begin(); e != poly.edges_end(); ++e) {
      segments_.emplace_back(*e);
    }
  };

  add_polygon_edges(geometry_.outer_boundary());
  for (auto h = geometry_.holes_begin(); h != geometry_.holes_end(); ++h) {
    add_polygon_edges(*h);
  }

  tree_ = Robot::AABB_tree(segments_.begin(), segments_.end());
}

bool Environment::contains(const Robot::Point &point) const {
  static const auto &ob = geometry_.outer_boundary();

  if (ob.bounded_side(point) != CGAL::ON_BOUNDED_SIDE) {
    return false;
  }

  for (auto h = geometry_.holes_begin(); h != geometry_.holes_end(); ++h) {
    if (h->bounded_side(point) != CGAL::ON_UNBOUNDED_SIDE) {
      return false;
    }
  }

  return true;
}

void Environment::draw(const DrawData &draw_data) {
  auto draw_polygon_edges = [&](const Robot::Polygon &poly) {
    for (std::size_t i = 0; i < poly.size(); ++i) {
      Robot::Point p1 = poly[i];
      Robot::Point p2 = poly[(i + 1) % poly.size()];
      DrawLine(p1.x() * draw_data.scale_factor + draw_data.offset_x,
               p1.y() * draw_data.scale_factor + draw_data.offset_y,
               p2.x() * draw_data.scale_factor + draw_data.offset_x,
               p2.y() * draw_data.scale_factor + draw_data.offset_y, BLACK);
    }
  };

  draw_polygon_edges(geometry_.outer_boundary());

  for (auto h = geometry_.holes_begin(); h != geometry_.holes_end(); ++h) {
    draw_polygon_edges(*h);
  }
}
