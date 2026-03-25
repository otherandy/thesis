#include "ExplorationBot.hpp"
#include "Utils.hpp"
#include <memory>
#include <raylib-cpp.hpp>

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int FRAME_RATE = 60;
const std::string WINDOW_TITLE = "Exploration Bot Simulation";

int main()
{
  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
  window.SetTargetFPS(FRAME_RATE);

  auto bot = std::make_unique<ExplorationBot>(START_POSITION);

  while (!window.ShouldClose())
  {
    bot->update();

    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float scale_factor = calculate_scale_factor(window);
    const auto [offset_x, offset_y] = calculate_offset(window, scale_factor);

    draw_environment(scale_factor, offset_x, offset_y);
    bot->draw(scale_factor, offset_x, offset_y);

    window.EndDrawing();
  }

  // bot->visited_to_file("Testing/real_visited_positions.csv");
  // bot->grid_to_file("Testing/exploration_grid.txt");

  return 0;
}
