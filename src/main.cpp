#include "CentralUnit.hpp"
#include "raylib.h"
#include <memory>

int main() {
  const int WINDOW_WIDTH = 800;
  const int WINDOW_HEIGHT = 600;
  const int FRAME_RATE = 60;
  const std::string WINDOW_TITLE = "Exploration Bot Simulation";

  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
  window.SetTargetFPS(FRAME_RATE);
  window.SetConfigFlags(FLAG_WINDOW_RESIZABLE);

  auto central_unit = std::make_unique<CentralUnit>();

  const Robot::Point START_POSITION(3.0, 3.0);
  central_unit->register_bot(START_POSITION, Robot::Vector(1, 0), true);
  central_unit->register_bot(START_POSITION, Robot::Vector(1, 0), false);
  central_unit->register_bot(START_POSITION, Robot::Vector(-1, 0), true);
  central_unit->register_bot(START_POSITION, Robot::Vector(-1, 0), false);

  while (!window.ShouldClose()) {
    central_unit->update();

    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float scale_factor = calculate_scale_factor(window);
    const auto [offset_x, offset_y] = calculate_offset(window, scale_factor);

    DrawData draw_data{scale_factor, offset_x, offset_y, window.GetWidth(),
                       window.GetHeight()};

    draw_environment(draw_data);
    central_unit->draw(draw_data);

    window.EndDrawing();
  }

  central_unit->report_time();

  return 0;
}
