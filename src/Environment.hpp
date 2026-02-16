#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <raylib-cpp.hpp>

using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Polygon = CGAL::Polygon_2<Kernel>;

constexpr std::pair<double, double> POLYGON_ENV_DATA[] = {
    {0, 0}, {8, 0}, {8, 6}, {12, 6}, {12, 12}, {4, 12}, {4, 6}, {0, 6}};

constexpr std::pair<double, double> SQUARE_ENV_DATA[] = {
    {0, 0}, {10, 0}, {10, 10}, {0, 10}};

constexpr std::pair<double, double> TRIANGLE_ENV_DATA[] = {
    {0, 0}, {10, 0}, {5, 8}};

constexpr std::pair<double, double> CUSTOM_ENV_DATA[] = {
    {0, 0}, {0, 10}, {10, 10}, {10, 9.5}, {15, 9.5}, {15, 9}, {10, 9}, {10, 0}};

constexpr auto get_bounds(const std::pair<double, double> *data, std::size_t size)
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

constexpr auto POLYGON_BOUNDS = get_bounds(POLYGON_ENV_DATA,
                                           sizeof(POLYGON_ENV_DATA) /
                                               sizeof(POLYGON_ENV_DATA[0]));

inline const Polygon &get_environment()
{
    static Polygon env;
    static bool initialized = false;
    if (!initialized)
    {
        for (const auto &p : POLYGON_ENV_DATA)
        {
            env.push_back(Point(p.first, p.second));
        }
        initialized = true;
    }
    return env;
}

const Polygon &ENVIRONMENT = get_environment();

constexpr double ENV_WIDTH = std::get<1>(POLYGON_BOUNDS) -
                             std::get<0>(POLYGON_BOUNDS);
constexpr double ENV_HEIGHT = std::get<3>(POLYGON_BOUNDS) -
                              std::get<2>(POLYGON_BOUNDS);

const double EXPLORATION_RADIUS = std::sqrt(
                                      std::pow(ENV_WIDTH, 2) +
                                      std::pow(ENV_HEIGHT, 2)) /
                                  2.0;

inline bool point_in_environment(const Point &p)
{
    return ENVIRONMENT.bounded_side(p) == CGAL::ON_BOUNDED_SIDE;
}

void draw_environment(float scale_factor, const raylib::Vector2 &offset)
{
    for (size_t i = 0; i < ENVIRONMENT.size(); ++i)
    {
        Point p1 = ENVIRONMENT[i];
        Point p2 = ENVIRONMENT[(i + 1) % ENVIRONMENT.size()];
        DrawLine(p1.x() * scale_factor + offset.x,
                 p1.y() * scale_factor + offset.y,
                 p2.x() * scale_factor + offset.x,
                 p2.y() * scale_factor + offset.y,
                 BLACK);
    }
}

#endif