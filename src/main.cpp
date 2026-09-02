#include "CentralUnit.hpp"
#include "Environment.hpp"
#include "raylib.h"
#include <memory>

int main(int argc, char **argv) {
  const int WINDOW_WIDTH = 400 * 2;
  const int WINDOW_HEIGHT = 300;
  const int FRAME_RATE = 60;
  const std::string WINDOW_TITLE = "Exploration Bot Simulation";

  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
  window.SetTargetFPS(FRAME_RATE);
  window.SetConfigFlags(FLAG_WINDOW_RESIZABLE);

  EnvironmentPreset selected_env = EnvironmentPreset::Room;
  Robot::Point start_position(3.0, 3.0);

  unsigned long num_bots = 4;
  Robot::Vector EAST = Robot::Vector(1, 0);
  Robot::Vector WEST = Robot::Vector(-1, 0);
  Robot::Vector SOUTH = Robot::Vector(0, -1);
  Robot::Vector NORTH = Robot::Vector(0, 1);

  if (argc >= 2) {
    selected_env = parse_environment(argv[1]);
  }

  auto central_unit =
      std::make_unique<CentralUnit>(selected_env, start_position);

  if (argc >= 3) {
    num_bots = std::stoul(argv[2]);
  }

  for (std::size_t i = 0; i < num_bots; ++i) {
    if (i % 8 == 0) {
      central_unit->register_bot(EAST);
    } else if (i % 8 == 1) {
      central_unit->register_bot(WEST);
    } else if (i % 8 == 2) {
      central_unit->register_bot(EAST);
    } else if (i % 8 == 3) {
      central_unit->register_bot(WEST);
    } else if (i % 8 == 4) {
      central_unit->register_bot(SOUTH);
    } else if (i % 8 == 5) {
      central_unit->register_bot(NORTH);
    } else if (i % 8 == 6) {
      central_unit->register_bot(SOUTH);
    } else {
      central_unit->register_bot(NORTH);
    }
  }

  if (argc >= 4) {
    const std::string s = argv[3];
    if (s == "test") {
      central_unit->start_test();
    }
  }

  if (argc >= 5) {
    const std::string s = argv[4];
    if (s == "debug") {
      central_unit->enable_debug();
      std::cout << "INFO: DEBUG ENABLED" << std::endl;
    }
  }

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

    if (central_unit->test_finished()) {
      central_unit->save_data();
      break;
    }
  }

  // central_unit->save_grid();

  return 0;
}
