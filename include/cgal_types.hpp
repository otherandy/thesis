#ifndef CGAL_TYPES_HPP
#define CGAL_TYPES_HPP

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Simple_cartesian.h>

using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Vector = Kernel::Vector_2;
using Direction = Kernel::Direction_2;
using Polygon = CGAL::Polygon_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;

#endif
