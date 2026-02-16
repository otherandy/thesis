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
    // -- Update --
    bot.get_input_and_move();
    bot.take_lidar_readings();
    bot.create_follow_vector();
    bot.run_exploration();

    // -- Draw --
    window.BeginDrawing();
    window.ClearBackground(RAYWHITE);

    const float scale_factor = calculate_scale_factor(window);
    const raylib::Vector2 offset = calculate_offset(window, scale_factor);

    draw_environment(scale_factor, offset);

    bot.draw_grid(scale_factor, offset);
    bot.draw_first_contact_point(scale_factor, offset);
    bot.draw_path(scale_factor, offset);
    bot.draw_readings(scale_factor, offset);
    bot.draw(scale_factor, offset);
    bot.draw_follow_vector(scale_factor, offset);

    window.EndDrawing();
  }

  // bot.visited_to_file("Testing/real_visited_positions.csv");

  CloseWindow();
  return 0;
}
