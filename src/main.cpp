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

  desc.add_options()("help", "display options and exit")(
      "width,w", po::value<int>(),
      "set window width")("height,h", po::value<int>(), "set window height")(
      "no-graph", "disable graph visualization")(
      "env,e", po::value<std::string>(), "set environment")(
      "numbots,n", po::value<std::size_t>(), "number of robots")(
      "radius,r", po::value<double>(), "set robot lidar radius")(
      "startx,x", po::value<double>(), "robot starting x coordinate")(
      "starty,y", po::value<double>(), "robot starting x coordinate")(
      "strategy,s", po::value<std::string>(),
      "set node selection strategy")("test", "close program on completion")(
      "debug", "enable output of debug info")("output-data",
                                              "enable output of data to file")(
      "output-grid", "enable output of the grid heatmap to file")(
      "output-graph", "enable output of the final graph to file")(
      "output-all", "enable output of everything to file");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  const bool no_graph = vm.count("no-graph");
  const int width_mult = no_graph ? 1 : 2;
  const int WINDOW_WIDTH =
      vm.count("width") ? vm["width"].as<int>() * width_mult : 800 * width_mult;
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
  double radius = vm.count("radius") ? vm["radius"].as<double>() : 1.0;

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

  std::string strategy = "";
  if (vm.count("strategy")) {
    strategy = vm["strategy"].as<std::string>();

    if (strategy != "largest" && strategy != "smallest" &&
        strategy != "closest") {
      throw std::invalid_argument("Invalid strategy: " + strategy);
    }
  }

  auto central_unit =
      std::make_unique<CentralUnit>(selected_env, start_position, strategy);

  for (std::size_t i = 0; i < num_bots; ++i) {
    if (i % 8 == 0) {
      central_unit->register_bot(EAST, radius);
    } else if (i % 8 == 1) {
      central_unit->register_bot(WEST, radius);
    } else if (i % 8 == 2) {
      central_unit->register_bot(EAST, radius);
    } else if (i % 8 == 3) {
      central_unit->register_bot(WEST, radius);
    } else if (i % 8 == 4) {
      central_unit->register_bot(SOUTH, radius);
    } else if (i % 8 == 5) {
      central_unit->register_bot(NORTH, radius);
    } else if (i % 8 == 6) {
      central_unit->register_bot(SOUTH, radius);
    } else {
      central_unit->register_bot(NORTH, radius);
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

    int w = no_graph ? window.GetWidth() : window.GetWidth() / 2;
    int h = window.GetHeight();
    const float scale_factor = calculate_scale_factor(w, h);
    const auto [offset_x, offset_y] = calculate_offset(w, h, scale_factor);

    DrawData draw_data{scale_factor, offset_x, offset_y, w, h};

    central_unit->draw_environment(draw_data);
    central_unit->draw(draw_data);

    if (!no_graph) {
      central_unit->draw_graph(w, h);
    }

    window.EndDrawing();

    if (central_unit->test_finished()) {
      break;
    }
  }

  if (vm.count("output-data") || vm.count("output-all")) {
    central_unit->save_data();
  }

  if (vm.count("output-grid") || vm.count("output-all")) {
    central_unit->save_grid();
  }

  if (vm.count("output-graph") || vm.count("output-all")) {
    central_unit->save_graph();
  }

  return 0;
}
