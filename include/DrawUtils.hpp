#ifndef DRAW_UTILS_HPP
#define DRAW_UTILS_HPP

#include "Environment.hpp"
#include <raylib-cpp.hpp>

const float WINDOW_PADDING = 10.0f;

struct DrawData {
  float scale_factor;
  float offset_x;
  float offset_y;
};

inline float calculate_scale_factor(const raylib::Window &window) {
  const float padded_width = window.GetWidth() - 2.0f * WINDOW_PADDING;
  const float padded_height = window.GetHeight() - 2.0f * WINDOW_PADDING;

  return std::min(padded_width / ENV_WIDTH, padded_height / ENV_HEIGHT);
}

inline std::pair<float, float> calculate_offset(const raylib::Window &window,
                                                float scale_factor) {
  const float draw_width = ENV_WIDTH * scale_factor;
  const float draw_height = ENV_HEIGHT * scale_factor;

  return std::make_pair((window.GetWidth() - draw_width) * 0.5f,
                        (window.GetHeight() - draw_height) * 0.5f);
}

inline void draw_environment(const DrawData &draw_data) {
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

  draw_polygon_edges(ENVIRONMENT.outer_boundary());
  for (auto h = ENVIRONMENT.holes_begin(); h != ENVIRONMENT.holes_end(); ++h) {
    draw_polygon_edges(*h);
  }
}

#endif
