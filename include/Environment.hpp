#ifndef ENVIRONMENT_HPP
#define ENVIRONMENT_HPP

#include "cgal_types.hpp"
#include <raylib-cpp.hpp>

using EnvData = std::pair<double, double>;

constexpr EnvData POLYGON_ENV_DATA[] = {
    {0, 0},
    {8, 0},
    {8, 6},
    {12, 6},
    {12, 12},
    {4, 12},
    {4, 6},
    {0, 6},
};

constexpr EnvData POLYGON2_ENV_DATA[] = {
    {0, 0},
    {16, 0},
    {16, 12},
    {24, 12},
    {24, 24},
    {8, 24},
    {8, 12},
    {0, 12},
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
    {0, 0},
    {0, 10},
    {10, 10},
    {10, 9.5},
    {15, 9.5},
    {15, 9},
    {10, 9},
    {10, 0},
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

enum class EnvironmentPreset
{
    Polygon,
    Polygon2,
    Square,
    Triangle,
    Custom,
    Square2WithHole,
};

// Change this single line to switch the environment before compiling.
constexpr EnvironmentPreset SELECTED_ENVIRONMENT = EnvironmentPreset::Square2WithHole;

struct SelectedEnvironmentData
{
    const EnvData *outer_data;
    std::size_t outer_size;
    const EnvData *hole_data;
    std::size_t hole_size;
};

constexpr SelectedEnvironmentData get_selected_environment_data()
{
    switch (SELECTED_ENVIRONMENT)
    {
    case EnvironmentPreset::Polygon:
        return {POLYGON_ENV_DATA,
                sizeof(POLYGON_ENV_DATA) / sizeof(POLYGON_ENV_DATA[0]),
                nullptr,
                0};
    case EnvironmentPreset::Polygon2:
        return {POLYGON2_ENV_DATA,
                sizeof(POLYGON2_ENV_DATA) / sizeof(POLYGON2_ENV_DATA[0]),
                nullptr,
                0};
    case EnvironmentPreset::Square:
        return {SQUARE_ENV_DATA,
                sizeof(SQUARE_ENV_DATA) / sizeof(SQUARE_ENV_DATA[0]),
                nullptr,
                0};
    case EnvironmentPreset::Triangle:
        return {TRIANGLE_ENV_DATA,
                sizeof(TRIANGLE_ENV_DATA) / sizeof(TRIANGLE_ENV_DATA[0]),
                nullptr,
                0};
    case EnvironmentPreset::Custom:
        return {CUSTOM_ENV_DATA,
                sizeof(CUSTOM_ENV_DATA) / sizeof(CUSTOM_ENV_DATA[0]),
                nullptr,
                0};
    case EnvironmentPreset::Square2WithHole:
        return {SQUARE2_ENV_DATA,
                sizeof(SQUARE2_ENV_DATA) / sizeof(SQUARE2_ENV_DATA[0]),
                SQUARE2_HOLE_DATA,
                sizeof(SQUARE2_HOLE_DATA) / sizeof(SQUARE2_HOLE_DATA[0])};
    }

    return {SQUARE_ENV_DATA,
            sizeof(SQUARE_ENV_DATA) / sizeof(SQUARE_ENV_DATA[0]),
            nullptr,
            0};
}

constexpr auto SELECTED_ENV_DATA = get_selected_environment_data();

constexpr auto get_bounds(const EnvData *data, std::size_t size)
{
    double xmin = data[0].first, xmax = xmin;
    double ymin = data[0].second, ymax = ymin;
    for (std::size_t i = 0; i < size; ++i)
    {
        if (data[i].first < xmin)
            xmin = data[i].first;
        if (data[i].first > xmax)
            xmax = data[i].first;
        if (data[i].second < ymin)
            ymin = data[i].second;
        if (data[i].second > ymax)
            ymax = data[i].second;
    }
    return std::make_tuple(xmin, xmax, ymin, ymax);
}

constexpr auto POLYGON_BOUNDS = get_bounds(SELECTED_ENV_DATA.outer_data,
                                           SELECTED_ENV_DATA.outer_size);

constexpr double ENV_WIDTH = std::get<1>(POLYGON_BOUNDS) -
                             std::get<0>(POLYGON_BOUNDS);
constexpr double ENV_HEIGHT = std::get<3>(POLYGON_BOUNDS) -
                              std::get<2>(POLYGON_BOUNDS);

const double EXPLORATION_RADIUS = std::sqrt(
                                      std::pow(ENV_WIDTH, 2) +
                                      std::pow(ENV_HEIGHT, 2)) /
                                  2.0;

inline const PolygonWithHoles &get_environment()
{
    static PolygonWithHoles env;
    static bool initialized = false;
    if (!initialized)
    {
        Polygon outer;
        for (std::size_t i = 0; i < SELECTED_ENV_DATA.outer_size; ++i)
        {
            outer.push_back(Point(SELECTED_ENV_DATA.outer_data[i].first,
                                  SELECTED_ENV_DATA.outer_data[i].second));
        }

        if (outer.is_clockwise_oriented())
        {
            outer.reverse_orientation();
        }

        std::vector<Polygon> holes;
        if (SELECTED_ENV_DATA.hole_data != nullptr)
        {
            Polygon hole;
            for (std::size_t i = 0; i < SELECTED_ENV_DATA.hole_size; ++i)
            {
                hole.push_back(Point(SELECTED_ENV_DATA.hole_data[i].first,
                                     SELECTED_ENV_DATA.hole_data[i].second));
            }

            if (hole.is_counterclockwise_oriented())
            {
                hole.reverse_orientation();
            }

            holes.push_back(hole);
        }

        env = PolygonWithHoles(outer, holes.begin(), holes.end());
        initialized = true;
    }

    return env;
}

inline const PolygonWithHoles &ENVIRONMENT = get_environment();

inline bool point_in_environment(const Point &p)
{
    if (ENVIRONMENT.outer_boundary().bounded_side(p) != CGAL::ON_BOUNDED_SIDE)
    {
        return false;
    }

    for (auto h = ENVIRONMENT.holes_begin(); h != ENVIRONMENT.holes_end(); ++h)
    {
        if (h->bounded_side(p) != CGAL::ON_UNBOUNDED_SIDE)
        {
            return false;
        }
    }

    return true;
}

#endif