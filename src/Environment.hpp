#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polygon_2.h>
#include <raylib-cpp.hpp>

using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Polygon = CGAL::Polygon_2<Kernel>;

const Point POLYGON_ENV_POINTS[] = {
    Point(0, 0),
    Point(8, 0),
    Point(8, 6),
    Point(12, 6),
    Point(12, 12),
    Point(4, 12),
    Point(4, 6),
    Point(0, 6)};

const Point SQUARE_ENV_POINTS[] = {
    Point(0, 0),
    Point(10, 0),
    Point(10, 10),
    Point(0, 10)};

const Point TRIANGLE_ENV_POINTS[] = {
    Point(0, 0),
    Point(10, 0),
    Point(5, 8)};

const Point ENV_POINTS[] = {
    Point(0, 0),
    Point(0, 10),
    Point(10, 10),
    Point(10, 9.5),
    Point(15, 9.5),
    Point(15, 9),
    Point(10, 9),
    Point(10, 0)};

const Polygon ENVIRONMENT(POLYGON_ENV_POINTS,
                          POLYGON_ENV_POINTS + sizeof(POLYGON_ENV_POINTS) / sizeof(POLYGON_ENV_POINTS[0]));

const float ENV_WIDTH = ENVIRONMENT.bbox().xmax() -
                        ENVIRONMENT.bbox().xmin();
const float ENV_HEIGHT = ENVIRONMENT.bbox().ymax() -
                         ENVIRONMENT.bbox().ymin();

const double EXPLORATION_RADIUS = std::sqrt(
                                      std::pow(
                                          ENVIRONMENT.bbox().xmax() - ENVIRONMENT.bbox().xmin(), 2) +
                                      std::pow(
                                          ENVIRONMENT.bbox().ymax() - ENVIRONMENT.bbox().ymin(), 2)) /
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