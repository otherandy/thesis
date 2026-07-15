#ifndef CGAL_TYPES_HPP
#define CGAL_TYPES_HPP

#include <CGAL/AABB_segment_primitive_2.h>
#include <CGAL/AABB_traits_2.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Simple_cartesian.h>

namespace Robot {
using Kernel = CGAL::Simple_cartesian<double>;
using Point = Kernel::Point_2;
using Vector = Kernel::Vector_2;
using Polygon = CGAL::Polygon_2<Kernel>;
using PolygonWithHoles = CGAL::Polygon_with_holes_2<Kernel>;
using Rectangle = CGAL::Iso_rectangle_2<Kernel>;
using Segment = Kernel::Segment_2;
using SegmentIterator = std::vector<Segment>::iterator;
using AABB_traits = CGAL::AABB_traits_2<
    Kernel, CGAL::AABB_segment_primitive_2<Kernel, SegmentIterator>>;
using AABB_tree = CGAL::AABB_tree<AABB_traits>;
} // namespace Robot

#endif
