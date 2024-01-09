#pragma once

#include "geom/complex_polygon.h"

#include "fastgrid/interpolation.h"

namespace truck::map {

class Map {
  public:
    Map(geom::ComplexPolygons polygons);

    static Map fromGeoJson(const std::string& path);

    const geom::ComplexPolygons& polygons() const;

    geom::ComplexPolygons clip(const fastgrid::Domain& domain) const noexcept;

  private:
    geom::Polygon clip(const fastgrid::Domain& domain, const geom::Polygon& polygon) const noexcept;

    geom::ComplexPolygons polygons_;
};

}  // namespace truck::map