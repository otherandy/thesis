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

  unsigned long num_bots = 4;
  Robot::Vector bot_direction = Robot::Vector(1, 0);

  if (argc >= 3) {
    num_bots = std::stoul(argv[2]);
  }

  for (std::size_t i = 0; i < num_bots; ++i) {
    if (i + 1 % 4 > 2) {
      central_unit->register_bot(-bot_direction);
    } else {
      central_unit->register_bot(bot_direction);
    }
  }

  if (argc >= 4) {
    const std::string s = argv[3];
    if (s == "test") {
      central_unit->start_test();
    }
  }

  std::string filename = "data.csv";
  bool save_grid = false;

  if (argc >= 5) {
    filename = argv[4];
  }

  if (argc >= 6) {
    const std::string s = argv[5];
    save_grid = (s == "true");
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
      central_unit->save_data(filename, save_grid);
      break;
    }
  }

  return 0;
}
