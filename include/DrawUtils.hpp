#ifndef DRAW_UTILS_HPP
#define DRAW_UTILS_HPP

#include <raylib-cpp.hpp>

const float WINDOW_PADDING = 10.0f;

struct DrawData {
  float scale_factor;
  float offset_x;
  float offset_y;
  int screen_w;
  int screen_h;
};

float calculate_scale_factor(const int w, const int h);

std::pair<float, float> calculate_offset(const int w, const int h,
                                         float scale_factor);

#endif
