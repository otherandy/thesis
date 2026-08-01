#include "DrawUtils.hpp"
#include "Environment.hpp"

float calculate_scale_factor(const int w, const int h) {
  const float padded_width = w - 2.0f * WINDOW_PADDING;
  const float padded_height = h - 2.0f * WINDOW_PADDING;

  return std::min(padded_width / ENV_WIDTH, padded_height / ENV_HEIGHT);
}

std::pair<float, float> calculate_offset(const int w, const int h,
                                                float scale_factor) {
  const float draw_width = ENV_WIDTH * scale_factor;
  const float draw_height = ENV_HEIGHT * scale_factor;

  return std::make_pair((w - draw_width) * 0.5f, (h - draw_height) * 0.5f);
}
