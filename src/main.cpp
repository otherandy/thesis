#include "CentralUnit.hpp"
#include "ExplorationBot.hpp"
#include <memory>

int main() {
  const int WINDOW_WIDTH = 800;
  const int WINDOW_HEIGHT = 600;
  const int FRAME_RATE = 60;
  const std::string WINDOW_TITLE = "Exploration Bot Simulation";

  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
  window.SetTargetFPS(FRAME_RATE);

  auto central_unit = std::make_unique<CentralUnit>();
  auto bot1 =
      std::make_shared<ExplorationBot>(START_POSITION, Vector(1, 0), true);
  auto bot2 =
      std::make_shared<ExplorationBot>(START_POSITION, Vector(1, 0), false);
  auto bot3 =
      std::make_shared<ExplorationBot>(START_POSITION, Vector(-1, 0), true);
  auto bot4 =
      std::make_shared<ExplorationBot>(START_POSITION, Vector(-1, 0), false);

  central_unit->register_bot(bot1.get());
  central_unit->register_bot(bot2.get());
  central_unit->register_bot(bot3.get());
  central_unit->register_bot(bot4.get());

  while (!window.ShouldClose()) {
    central_unit->update();

    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float scale_factor = calculate_scale_factor(window);
    const auto [offset_x, offset_y] = calculate_offset(window, scale_factor);

    DrawData draw_data{scale_factor, offset_x, offset_y};

    draw_environment(draw_data);
    central_unit->draw(draw_data);

    window.EndDrawing();
  }

  central_unit->report_time();

  return 0;
}
