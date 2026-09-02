#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "DrawUtils.hpp"
#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

using EnvData = std::pair<double, double>;

constexpr double ENV_WIDTH = 40;
constexpr double ENV_HEIGHT = 40;

enum class EnvironmentPreset {
  Polygon,
  Polygon2,
  Polygon2WithHoles,
  Square,
  Triangle,
  Custom,
  Square2WithHole,
  Square4,
  Square4WithHoles,
  Corridor,
  Legs,
  Star,
  LetterE,
  Mono,
  Room,
  Cross,
};

EnvironmentPreset parse_environment(const std::string &name);
std::string get_environment_name(EnvironmentPreset preset);

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

constexpr const EnvData *SQUARE2_HOLE_DATA_LIST[] = {
    SQUARE2_HOLE_DATA,
};
constexpr std::size_t SQUARE2_HOLE_SIZE_LIST[] = {
    sizeof(SQUARE2_HOLE_DATA) / sizeof(SQUARE2_HOLE_DATA[0]),
};

constexpr const EnvData SQUARE4_ENV_DATA[] = {
    {0, 0},
    {0, 40},
    {40, 40},
    {40, 0},
};

constexpr EnvData SQUARE4_HOLE1_DATA[] = {{6, 6}, {6, 11}, {7, 11}, {7, 6}};
constexpr EnvData SQUARE4_HOLE2_DATA[] = {{6, 14}, {6, 19}, {7, 19}, {7, 14}};
constexpr EnvData SQUARE4_HOLE3_DATA[] = {{6, 21}, {6, 26}, {7, 26}, {7, 21}};
constexpr EnvData SQUARE4_HOLE4_DATA[] = {{6, 29}, {6, 34}, {7, 34}, {7, 29}};
constexpr EnvData SQUARE4_HOLE5_DATA[] = {{13, 8}, {13, 9}, {14, 9}, {14, 8}};
constexpr EnvData SQUARE4_HOLE6_DATA[] = {
    {13, 16}, {13, 17}, {14, 17}, {14, 16}};
constexpr EnvData SQUARE4_HOLE7_DATA[] = {
    {13, 23}, {13, 24}, {14, 24}, {14, 23}};
constexpr EnvData SQUARE4_HOLE8_DATA[] = {
    {13, 31}, {13, 32}, {14, 32}, {14, 31}};
constexpr EnvData SQUARE4_HOLE9_DATA[] = {{22, 10}, {23, 7}, {24, 7}, {25, 10}};
constexpr EnvData SQUARE4_HOLE10_DATA[] = {
    {22, 30}, {23, 33}, {24, 33}, {25, 30}};
constexpr EnvData SQUARE4_HOLE11_DATA[] = {
    {22, 18}, {23, 19}, {22, 20}, {23, 21}, {22, 22}, {23, 23}, {24, 23},
    {25, 22}, {24, 21}, {25, 20}, {24, 19}, {25, 18}, {24, 17}, {23, 17}};
constexpr EnvData SQUARE4_HOLE12_DATA[] = {
    {31, 24}, {32, 26}, {30, 27}, {32, 27}, {33, 29},
    {34, 27}, {36, 27}, {34, 26}, {35, 24}, {33, 25}};

constexpr const EnvData *SQUARE4_HOLE_DATA_LIST[] = {
    SQUARE4_HOLE1_DATA,  SQUARE4_HOLE2_DATA,  SQUARE4_HOLE3_DATA,
    SQUARE4_HOLE4_DATA,  SQUARE4_HOLE5_DATA,  SQUARE4_HOLE6_DATA,
    SQUARE4_HOLE7_DATA,  SQUARE4_HOLE8_DATA,  SQUARE4_HOLE9_DATA,
    SQUARE4_HOLE10_DATA, SQUARE4_HOLE11_DATA, SQUARE4_HOLE12_DATA,
};
constexpr std::size_t SQUARE4_HOLE_SIZE_LIST[] = {
    sizeof(SQUARE4_HOLE1_DATA) / sizeof(SQUARE4_HOLE1_DATA[0]),
    sizeof(SQUARE4_HOLE2_DATA) / sizeof(SQUARE4_HOLE2_DATA[0]),
    sizeof(SQUARE4_HOLE3_DATA) / sizeof(SQUARE4_HOLE3_DATA[0]),
    sizeof(SQUARE4_HOLE4_DATA) / sizeof(SQUARE4_HOLE4_DATA[0]),
    sizeof(SQUARE4_HOLE5_DATA) / sizeof(SQUARE4_HOLE5_DATA[0]),
    sizeof(SQUARE4_HOLE6_DATA) / sizeof(SQUARE4_HOLE6_DATA[0]),
    sizeof(SQUARE4_HOLE7_DATA) / sizeof(SQUARE4_HOLE7_DATA[0]),
    sizeof(SQUARE4_HOLE8_DATA) / sizeof(SQUARE4_HOLE8_DATA[0]),
    sizeof(SQUARE4_HOLE9_DATA) / sizeof(SQUARE4_HOLE9_DATA[0]),
    sizeof(SQUARE4_HOLE10_DATA) / sizeof(SQUARE4_HOLE10_DATA[0]),
    sizeof(SQUARE4_HOLE11_DATA) / sizeof(SQUARE4_HOLE11_DATA[0]),
    sizeof(SQUARE4_HOLE12_DATA) / sizeof(SQUARE4_HOLE12_DATA[0]),
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

constexpr const EnvData *POLYGON2_HOLE_DATA_LIST[] = {
    POLYGON2_HOLE1_DATA,
    POLYGON2_HOLE2_DATA,
};
constexpr std::size_t POLYGON2_HOLE_SIZE_LIST[] = {
    sizeof(POLYGON2_HOLE1_DATA) / sizeof(POLYGON2_HOLE1_DATA[0]),
    sizeof(POLYGON2_HOLE2_DATA) / sizeof(POLYGON2_HOLE2_DATA[0]),
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

constexpr EnvData ROOM_HOLE1_DATA[] = {{0, 5}, {4, 5}, {4, 5.1}, {0, 5.1}};
constexpr EnvData ROOM_HOLE2_DATA[] = {{8, 0}, {8, 5}, {8.1, 5}, {8.1, 0}};
constexpr EnvData ROOM_HOLE3_DATA[] = {
    {10, 3},
    {15, 3},
    {15, 4},
    {10, 4},
};
constexpr EnvData ROOM_HOLE4_DATA[] = {{9, 13}, {13, 13}, {11, 15}};

constexpr std::size_t ROOM_HOLE_SIZE_LIST[] = {
    sizeof(ROOM_HOLE1_DATA) / sizeof(ROOM_HOLE1_DATA[0]),
    sizeof(ROOM_HOLE2_DATA) / sizeof(ROOM_HOLE2_DATA[0]),
    sizeof(ROOM_HOLE3_DATA) / sizeof(ROOM_HOLE3_DATA[0]),
    sizeof(ROOM_HOLE4_DATA) / sizeof(ROOM_HOLE4_DATA[0]),
};
constexpr const EnvData *ROOM_HOLE_DATA_LIST[] = {
    ROOM_HOLE1_DATA,
    ROOM_HOLE2_DATA,
    ROOM_HOLE3_DATA,
    ROOM_HOLE4_DATA,
};

constexpr EnvData CROSS_ENV_DATA[] = {
    {0, 0},   {5, 0},   {5, 4},   {9, 4},   {9, 8},   {12, 8},
    {12, 4},  {16, 4},  {16, 0},  {21, 0},  {21, 5},  {17, 5},
    {17, 9},  {14, 9},  {14, 12}, {17, 12}, {17, 16}, {21, 16},
    {21, 21}, {16, 21}, {16, 17}, {12, 17}, {12, 13}, {9, 13},
    {9, 17},  {5, 17},  {5, 21},  {0, 21},  {0, 16},  {4, 16},
    {4, 12},  {7, 12},  {7, 9},   {4, 9},   {4, 5},   {0, 5}};

struct SelectedEnvironmentData {
  const EnvData *outer_data;
  std::size_t outer_size;
  const EnvData *const *hole_data_list;
  const std::size_t *hole_size_list;
  std::size_t hole_count;
};

constexpr const EnvData *NO_HOLE_DATA[] = {};
constexpr std::size_t NO_HOLE_SIZES[] = {};

constexpr SelectedEnvironmentData
get_selected_environment_data(EnvironmentPreset preset) {
  switch (preset) {
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
  case EnvironmentPreset::Square4:
    return {SQUARE4_ENV_DATA,
            sizeof(SQUARE4_ENV_DATA) / sizeof(SQUARE4_ENV_DATA[0]),
            NO_HOLE_DATA, 0};
  case EnvironmentPreset::Square4WithHoles:
    return {SQUARE4_ENV_DATA,
            sizeof(SQUARE4_ENV_DATA) / sizeof(SQUARE4_ENV_DATA[0]),
            SQUARE4_HOLE_DATA_LIST, SQUARE4_HOLE_SIZE_LIST,
            sizeof(SQUARE4_HOLE_DATA_LIST) / sizeof(SQUARE4_HOLE_DATA_LIST[0])};
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
  case EnvironmentPreset::Room:
    return {SQUARE2_ENV_DATA,
            sizeof(SQUARE2_ENV_DATA) / sizeof(SQUARE2_ENV_DATA[0]),
            ROOM_HOLE_DATA_LIST, ROOM_HOLE_SIZE_LIST,
            sizeof(ROOM_HOLE_DATA_LIST) / sizeof(ROOM_HOLE_DATA_LIST[0])};
  case EnvironmentPreset::Cross:
    return {CROSS_ENV_DATA, sizeof(CROSS_ENV_DATA) / sizeof(CROSS_ENV_DATA[0]),
            NO_HOLE_DATA, 0};
  }

  return {SQUARE_ENV_DATA, sizeof(SQUARE_ENV_DATA) / sizeof(SQUARE_ENV_DATA[0]),
          NO_HOLE_DATA, NO_HOLE_SIZES, 0};
}

class Environment {
public:
  explicit Environment(EnvironmentPreset preset);

  EnvironmentPreset preset;

  const Robot::PolygonWithHoles &get_geometry() const { return geometry_; }
  const Robot::AABB_tree &get_tree() const { return tree_; }

  bool contains(const Robot::Point &point) const;
  void draw(const DrawData &draw_data);

private:
  Robot::PolygonWithHoles geometry_;
  std::vector<Robot::Segment> segments_;
  Robot::AABB_tree tree_;
};

#endif
