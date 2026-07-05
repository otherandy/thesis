#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

using EnvData = std::pair<double, double>;

constexpr EnvData POLYGON_ENV_DATA[] = {
    {0, 0}, {8, 0}, {8, 6}, {12, 6}, {12, 12}, {4, 12}, {4, 6}, {0, 6},
};

constexpr EnvData POLYGON2_ENV_DATA[] = {
    {0, 0}, {16, 0}, {16, 12}, {24, 12}, {24, 24}, {8, 24}, {8, 12}, {0, 12},
};

constexpr EnvData SQUARE_ENV_DATA[] = {
    {0, 0},
    {10, 0},
    {10, 10},
    {0, 10},
};

constexpr EnvData TRIANGLE_ENV_DATA[] = {
    {0, 0},
    {10, 0},
    {5, 8},
};

constexpr EnvData CUSTOM_ENV_DATA[] = {
    {0, 0}, {0, 10}, {10, 10}, {10, 9.5}, {15, 9.5}, {15, 9}, {10, 9}, {10, 0},
};

constexpr EnvData SQUARE2_ENV_DATA[] = {
    {0, 0},
    {0, 20},
    {20, 20},
    {20, 0},
};

constexpr EnvData SQUARE2_HOLE_DATA[] = {
    {6, 6},
    {6, 14},
    {14, 14},
    {14, 6},
};

constexpr EnvData POLYGON2_HOLE1_DATA[] = {
    {4, 2},
    {4, 6},
    {8, 6},
    {8, 2},
};

constexpr EnvData POLYGON2_HOLE2_DATA[] = {
    {16, 18},
    {16, 22},
    {20, 22},
    {20, 18},
};

constexpr EnvData CORRIDOR_DATA[] = {
    {0, 0},   {10, 0},  {10, 4}, {16, 4}, {16, 0},  {26, 0},
    {26, 10}, {16, 10}, {16, 5}, {10, 5}, {10, 10}, {0, 10},
};

constexpr EnvData LEGS_DATA[] = {
    {0, 0},  {20, 0}, {20, 4}, {14, 4}, {14, 10}, {18, 10}, {18, 12}, {12, 12},
    {12, 4}, {8, 4},  {8, 12}, {2, 12}, {2, 10},  {6, 10},  {6, 4},   {0, 4}};

constexpr EnvData STAR_DATA[] = {
    {2, 0}, {5, 2}, {8, 0}, {7, 4}, {10, 6},
    {6, 6}, {5, 9}, {4, 6}, {0, 6}, {3, 4},
};

constexpr EnvData LETTER_E_DATA[] = {
    {0, 0},   {20, 0}, {20, 22}, {0, 22}, {0, 16}, {14, 16},
    {14, 14}, {0, 14}, {0, 8},   {14, 8}, {14, 6}, {0, 6},
};

constexpr EnvData MONO_DATA[] = {
    {0, 6},   {1, 6},   {1, 5},   {2, 5},   {2, 4},   {1, 4},   {1, 3},
    {2, 3},   {2, 2},   {3, 2},   {3, 1},   {4, 1},   {4, 2},   {5, 2},
    {5, 1},   {6, 1},   {6, 0},   {7, 0},   {7, 1},   {8, 1},   {8, 2},
    {9, 2},   {9, 3},   {10, 3},  {10, 2},  {11, 2},  {11, 3},  {12, 3},
    {12, 4},  {11, 4},  {11, 5},  {12, 5},  {12, 6},  {11, 6},  {11, 7},
    {12, 7},  {12, 8},  {13, 8},  {13, 9},  {14, 9},  {14, 10}, {13, 10},
    {13, 11}, {12, 11}, {12, 12}, {13, 12}, {13, 13}, {12, 13}, {12, 14},
    {11, 14}, {11, 15}, {12, 15}, {12, 16}, {13, 16}, {13, 17}, {12, 17},
    {12, 18}, {11, 18}, {11, 19}, {10, 19}, {10, 18}, {9, 18},  {9, 17},
    {8, 17},  {8, 16},  {7, 16},  {7, 15},  {6, 15},  {6, 16},  {5, 16},
    {5, 17},  {4, 17},  {4, 16},  {3, 16},  {3, 17},  {2, 17},  {2, 16},
    {1, 16},  {1, 15},  {0, 15},  {0, 14},  {1, 14},  {1, 13},  {2, 13},
    {2, 12},  {3, 12},  {3, 13},  {4, 13},  {4, 12},  {5, 12},  {5, 13},
    {6, 13},  {6, 12},  {7, 12},  {7, 11},  {8, 11},  {8, 10},  {7, 10},
    {7, 9},   {8, 9},   {8, 8},   {7, 8},   {7, 7},   {8, 7},   {8, 6},
    {7, 6},   {7, 5},   {8, 5},   {8, 4},   {7, 4},   {7, 3},   {6, 3},
    {6, 4},   {5, 4},   {5, 5},   {6, 5},   {6, 6},   {5, 6},   {5, 7},
    {6, 7},   {6, 8},   {5, 8},   {5, 9},   {6, 9},   {6, 10},  {5, 10},
    {5, 11},  {4, 11},  {4, 10},  {3, 10},  {3, 11},  {2, 11},  {2, 10},
    {1, 10},  {1, 9},   {0, 9},   {0, 8},   {1, 8},   {1, 7},   {0, 7},
};

enum class EnvironmentPreset {
  Polygon,
  Polygon2,
  Polygon2WithHoles,
  Square,
  Triangle,
  Custom,
  Square2WithHole,
  Corridor,
  Legs,
  Star,
  LetterE,
  Mono,
};

// Change this single line to switch the environment before compiling.
constexpr EnvironmentPreset SELECTED_ENVIRONMENT = EnvironmentPreset::Polygon2WithHoles;

struct SelectedEnvironmentData {
  const EnvData *outer_data;
  std::size_t outer_size;
  const EnvData *const *hole_data_list;
  const std::size_t *hole_size_list;
  std::size_t hole_count;
};

constexpr const EnvData *NO_HOLE_DATA[] = {};
constexpr std::size_t NO_HOLE_SIZES[] = {};

constexpr const EnvData *SQUARE2_HOLE_DATA_LIST[] = {
    SQUARE2_HOLE_DATA,
};
constexpr std::size_t SQUARE2_HOLE_SIZE_LIST[] = {
    sizeof(SQUARE2_HOLE_DATA) / sizeof(SQUARE2_HOLE_DATA[0]),
};

constexpr const EnvData *POLYGON2_HOLE_DATA_LIST[] = {
    POLYGON2_HOLE1_DATA,
    POLYGON2_HOLE2_DATA,
};
constexpr std::size_t POLYGON2_HOLE_SIZE_LIST[] = {
    sizeof(POLYGON2_HOLE1_DATA) / sizeof(POLYGON2_HOLE1_DATA[0]),
    sizeof(POLYGON2_HOLE2_DATA) / sizeof(POLYGON2_HOLE2_DATA[0]),
};

constexpr SelectedEnvironmentData get_selected_environment_data() {
  switch (SELECTED_ENVIRONMENT) {
  case EnvironmentPreset::Polygon:
    return {POLYGON_ENV_DATA,
            sizeof(POLYGON_ENV_DATA) / sizeof(POLYGON_ENV_DATA[0]),
            NO_HOLE_DATA, NO_HOLE_SIZES, 0};
  case EnvironmentPreset::Polygon2:
    return {POLYGON2_ENV_DATA,
            sizeof(POLYGON2_ENV_DATA) / sizeof(POLYGON2_ENV_DATA[0]),
            NO_HOLE_DATA, NO_HOLE_SIZES, 0};
  case EnvironmentPreset::Polygon2WithHoles:
    return {POLYGON2_ENV_DATA,
            sizeof(POLYGON2_ENV_DATA) / sizeof(POLYGON2_ENV_DATA[0]),
            POLYGON2_HOLE_DATA_LIST, POLYGON2_HOLE_SIZE_LIST,
            sizeof(POLYGON2_HOLE_DATA_LIST) /
                sizeof(POLYGON2_HOLE_DATA_LIST[0])};
  case EnvironmentPreset::Square:
    return {SQUARE_ENV_DATA,
            sizeof(SQUARE_ENV_DATA) / sizeof(SQUARE_ENV_DATA[0]), NO_HOLE_DATA,
            NO_HOLE_SIZES, 0};
  case EnvironmentPreset::Triangle:
    return {TRIANGLE_ENV_DATA,
            sizeof(TRIANGLE_ENV_DATA) / sizeof(TRIANGLE_ENV_DATA[0]),
            NO_HOLE_DATA, NO_HOLE_SIZES, 0};
  case EnvironmentPreset::Custom:
    return {CUSTOM_ENV_DATA,
            sizeof(CUSTOM_ENV_DATA) / sizeof(CUSTOM_ENV_DATA[0]), NO_HOLE_DATA,
            NO_HOLE_SIZES, 0};
  case EnvironmentPreset::Square2WithHole:
    return {SQUARE2_ENV_DATA,
            sizeof(SQUARE2_ENV_DATA) / sizeof(SQUARE2_ENV_DATA[0]),
            SQUARE2_HOLE_DATA_LIST, SQUARE2_HOLE_SIZE_LIST,
            sizeof(SQUARE2_HOLE_DATA_LIST) / sizeof(SQUARE2_HOLE_DATA_LIST[0])};
  case EnvironmentPreset::Corridor:
    return {CORRIDOR_DATA, sizeof(CORRIDOR_DATA) / sizeof(CORRIDOR_DATA[0]),
            NO_HOLE_DATA, 0};
  case EnvironmentPreset::Legs:
    return {LEGS_DATA, sizeof(LEGS_DATA) / sizeof(LEGS_DATA[0]), NO_HOLE_DATA,
            0};
  case EnvironmentPreset::Star:
    return {STAR_DATA, sizeof(STAR_DATA) / sizeof(STAR_DATA[0]), NO_HOLE_DATA,
            0};
  case EnvironmentPreset::LetterE:
    return {LETTER_E_DATA, sizeof(LETTER_E_DATA) / sizeof(LETTER_E_DATA[0]),
            NO_HOLE_DATA, 0};
  case EnvironmentPreset::Mono:
    return {MONO_DATA, sizeof(MONO_DATA) / sizeof(MONO_DATA[0]), NO_HOLE_DATA,
            0};
  }

  return {SQUARE_ENV_DATA, sizeof(SQUARE_ENV_DATA) / sizeof(SQUARE_ENV_DATA[0]),
          NO_HOLE_DATA, NO_HOLE_SIZES, 0};
}

constexpr auto SELECTED_ENV_DATA = get_selected_environment_data();

constexpr auto get_bounds(const EnvData *data, std::size_t size) {
  double xmin = data[0].first, xmax = xmin;
  double ymin = data[0].second, ymax = ymin;
  for (std::size_t i = 0; i < size; ++i) {
    if (data[i].first < xmin) {
      xmin = data[i].first;
    }
    if (data[i].first > xmax) {
      xmax = data[i].first;
    }
    if (data[i].second < ymin) {
      ymin = data[i].second;
    }
    if (data[i].second > ymax) {
      ymax = data[i].second;
    }
  }
  return std::make_tuple(xmin, xmax, ymin, ymax);
}

constexpr auto POLYGON_BOUNDS =
    get_bounds(SELECTED_ENV_DATA.outer_data, SELECTED_ENV_DATA.outer_size);

constexpr double ENV_MIN_X = std::get<0>(POLYGON_BOUNDS);
constexpr double ENV_MAX_X = std::get<1>(POLYGON_BOUNDS);
constexpr double ENV_MIN_Y = std::get<2>(POLYGON_BOUNDS);
constexpr double ENV_MAX_Y = std::get<3>(POLYGON_BOUNDS);

constexpr double ENV_WIDTH = ENV_MAX_X - ENV_MIN_X;
constexpr double ENV_HEIGHT = ENV_MAX_Y - ENV_MIN_Y;

constexpr double ENV_CENTER_X = (ENV_MIN_X + ENV_MAX_X) * 0.5;
constexpr double ENV_CENTER_Y = (ENV_MIN_Y + ENV_MAX_Y) * 0.5;

inline Robot::Point environment_center() {
  return Robot::Point(ENV_CENTER_X, ENV_CENTER_Y);
}

inline const Robot::PolygonWithHoles &get_environment() {
  static Robot::PolygonWithHoles env;
  static bool initialized = false;

  if (!initialized) {
    Robot::Polygon outer;
    for (std::size_t i = 0; i < SELECTED_ENV_DATA.outer_size; ++i) {
      const auto point = SELECTED_ENV_DATA.outer_data[i];
      outer.push_back(Robot::Point(point.first, point.second));
    }

    if (outer.is_clockwise_oriented()) {
      outer.reverse_orientation();
    }

    std::vector<Robot::Polygon> holes;
    for (std::size_t hole_idx = 0; hole_idx < SELECTED_ENV_DATA.hole_count;
         ++hole_idx) {
      Robot::Polygon hole;
      for (std::size_t i = 0; i < SELECTED_ENV_DATA.hole_size_list[hole_idx];
           ++i) {
        const auto point = SELECTED_ENV_DATA.hole_data_list[hole_idx][i];
        hole.push_back(Robot::Point(point.first, point.second));
      }

      if (hole.is_counterclockwise_oriented()) {
        hole.reverse_orientation();
      }

      holes.push_back(hole);
    }

    env = Robot::PolygonWithHoles(outer, holes.begin(), holes.end());
    initialized = true;
  }

  return env;
}

inline const Robot::PolygonWithHoles &ENVIRONMENT = get_environment();

inline bool point_in_environment(const Robot::Point &p) {
  static const auto &ob = ENVIRONMENT.outer_boundary();

  if (ob.bounded_side(p) != CGAL::ON_BOUNDED_SIDE) {
    return false;
  }

  for (auto h = ENVIRONMENT.holes_begin(); h != ENVIRONMENT.holes_end(); ++h) {
    if (h->bounded_side(p) != CGAL::ON_UNBOUNDED_SIDE) {
      return false;
    }
  }

  return true;
}

#endif
