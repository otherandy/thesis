#include "ExplorationBot.hpp"

const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 600;
const int FRAME_RATE = 60;

int main()
{
  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT,
                        "Exploration Bot Simulation");
  SetTargetFPS(FRAME_RATE);

  ExplorationBot bot(START_POSITION);

  while (!window.ShouldClose())
  {
    bot.update();

    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float scale_factor = calculate_scale_factor(window);
    const auto [offset_x, offset_y] = calculate_offset(window, scale_factor);

    draw_environment(scale_factor, offset_x, offset_y);
    bot.draw(scale_factor, offset_x, offset_y);

    window.EndDrawing();
  }

  // bot.visited_to_file("Testing/real_visited_positions.csv");
  // bot.grid_to_file("Testing/exploration_grid.txt");

  CloseWindow();
  return 0;
}
