#pragma once

#include "geom/polyline.h"
#include "geom/boost/point.h"


#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>

BOOST_GEOMETRY_REGISTER_LINESTRING(truck::geom::Polyline)
