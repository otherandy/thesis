#include "CentralUnit.hpp"
#include "Environment.hpp"
#include "raylib.h"
#include <boost/program_options.hpp>
#include <cstddef>
#include <memory>

namespace po = boost::program_options;

const Robot::Vector EAST = Robot::Vector(1, 0);
const Robot::Vector WEST = Robot::Vector(-1, 0);
const Robot::Vector SOUTH = Robot::Vector(0, -1);
const Robot::Vector NORTH = Robot::Vector(0, 1);

int main(int argc, char **argv) {
  po::options_description desc("Available arguments");

  desc.add_options()("help", "display options")(
      "width", po::value<int>(), "set window width")("height", po::value<int>(),
                                                     "set window height")(
      "env", po::value<std::string>(), "selected environment")(
      "numbots", po::value<std::size_t>(), "number of robots")(
      "startx", po::value<double>(), "robot starting x coordinate")(
      "starty", po::value<double>(), "robot starting x coordinate")(
      "test", "enable output of data files for tests")(
      "debug", "enable output of debug info");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  const int WINDOW_WIDTH =
      vm.count("width") ? vm["width"].as<int>() * 2 : 800 * 2;
  const int WINDOW_HEIGHT = vm.count("height") ? vm["height"].as<int>() : 600;
  const int FRAME_RATE = 60;
  const std::string WINDOW_TITLE = "Exploration Bot Simulation";

  raylib::Window window(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE);
  window.SetTargetFPS(FRAME_RATE);
  window.SetConfigFlags(FLAG_WINDOW_RESIZABLE);

  EnvironmentPreset selected_env = EnvironmentPreset::Room;
  Robot::Point start_position(3.0, 3.0);
  std::size_t num_bots =
      vm.count("numbots") ? vm["numbots"].as<std::size_t>() : 4;

  if (vm.count("env")) {
    selected_env = parse_environment(vm["env"].as<std::string>());
  }

  if (vm.count("startx")) {
    start_position =
        Robot::Point(vm["startx"].as<double>(), start_position.y());
  }

  if (vm.count("starty")) {
    start_position =
        Robot::Point(start_position.x(), vm["starty"].as<double>());
  }

  auto central_unit =
      std::make_unique<CentralUnit>(selected_env, start_position);

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

  if (vm.count("test")) {
    std::cout << "INFO: TESTS ENABLED" << std::endl;
    central_unit->start_test();
  }

  if (vm.count("debug")) {
    central_unit->enable_debug();
    std::cout << "INFO: DEBUG ENABLED" << std::endl;
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
