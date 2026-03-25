#ifndef DRAW_UTILS_HPP
#define DRAW_UTILS_HPP

#include "Environment.hpp"
#include <raylib-cpp.hpp>

const float WINDOW_PADDING = 10.0f;

struct DrawData
{
  float scale_factor;
  float offset_x;
  float offset_y;
};

inline float calculate_scale_factor(const raylib::Window &window)
{
  const float padded_width = window.GetWidth() - 2.0f * WINDOW_PADDING;
  const float padded_height = window.GetHeight() - 2.0f * WINDOW_PADDING;

  return std::min(padded_width / ENV_WIDTH,
                  padded_height / ENV_HEIGHT);
}

inline std::pair<float, float> calculate_offset(
    const raylib::Window &window, float scale_factor)
{
  const float draw_width = ENV_WIDTH * scale_factor;
  const float draw_height = ENV_HEIGHT * scale_factor;

  return std::make_pair(
      (window.GetWidth() - draw_width) * 0.5f,
      (window.GetHeight() - draw_height) * 0.5f);
}

inline void draw_environment(DrawData draw_data)
{
  for (size_t i = 0; i < ENVIRONMENT.size(); ++i)
  {
    Point p1 = ENVIRONMENT[i];
    Point p2 = ENVIRONMENT[(i + 1) % ENVIRONMENT.size()];
    DrawLine(p1.x() * draw_data.scale_factor + draw_data.offset_x,
             p1.y() * draw_data.scale_factor + draw_data.offset_y,
             p2.x() * draw_data.scale_factor + draw_data.offset_x,
             p2.y() * draw_data.scale_factor + draw_data.offset_y,
             BLACK);
  }
}

#endif