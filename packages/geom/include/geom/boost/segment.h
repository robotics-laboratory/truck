#pragma once

#include "geom/segment.h"

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/segment.hpp>

BOOST_GEOMETRY_REGISTER_SEGMENT(truck::geom::Segment, truck::geom::Vec2, begin, end)
