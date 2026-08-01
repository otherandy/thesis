#include "CentralUnit.hpp"
#include "Environment.hpp"
#include "raylib.h"
#include <memory>

int main(int argc, char **argv) {
  const int WINDOW_WIDTH = 800 * 2;
  const int WINDOW_HEIGHT = 600;
  const int FRAME_RATE = 60;
  const std::string WINDOW_TITLE = "Exploration Bot Simulation";

  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
  window.SetTargetFPS(FRAME_RATE);
  window.SetConfigFlags(FLAG_WINDOW_RESIZABLE);

  EnvironmentPreset selected_env = EnvironmentPreset::Room;
  Robot::Point start_position(3.0, 3.0);

  if (argc >= 2) {
    selected_env = parse_environment(argv[1]);
  }

  auto central_unit =
      std::make_unique<CentralUnit>(selected_env, start_position);

  central_unit->register_bot(Robot::Vector(1, 0));
  central_unit->register_bot(Robot::Vector(1, 0));
  central_unit->register_bot(Robot::Vector(-1, 0));
  central_unit->register_bot(Robot::Vector(-1, 0));

  while (!window.ShouldClose()) {
    central_unit->update();

    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    int w = window.GetWidth() / 2;
    int h = window.GetHeight();
    const float scale_factor = calculate_scale_factor(w, h);
    const auto [offset_x, offset_y] = calculate_offset(w, h, scale_factor);

    DrawData draw_data{scale_factor, offset_x, offset_y, w, h};

    central_unit->draw_environment(draw_data);
    central_unit->draw(draw_data);
    central_unit->draw_graph(w, h);

    window.EndDrawing();
  }

  return 0;
}
