#include "fastgrid/poly_to_grid.h"

#include "common/exception.h"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace truck::fastgrid {

namespace impl {

__always_inline std::vector<cv::Point> PolyToInput(const geom::Polygon& poly, const U8Grid& grid) {
    std::vector<cv::Point> input;
    input.reserve(poly.size());
    for (const auto& vertex : poly) {
        const auto point = grid.transform(vertex);
        input.push_back(cv::Point(point.x, point.y));
    }

    return input;
}

__always_inline std::vector<std::vector<cv::Point>> ComplexPolyToInput(
    const geom::ComplexPolygon& complex_poly, const U8Grid& grid) {
    std::vector<std::vector<cv::Point>> input;
    input.reserve(complex_poly.inners.size() + 1);

    {
        std::vector<cv::Point> points;
        for (const auto& vertex : complex_poly.outer) {
            const auto point = grid.transform(vertex);
            points.push_back(cv::Point(point.x, point.y));
        }
        input.push_back(std::move(points));
    }

    for (const auto& inner : complex_poly.inners) {
        std::vector<cv::Point> points;
        for (const auto& vertex : inner) {
            const auto point = grid.transform(vertex);
            points.push_back(cv::Point(point.x, point.y));
        }
        input.push_back(std::move(points));
    }

    return input;
}

__always_inline void FillPoly(cv::InputArrayOfArrays input, U8Grid& grid) {
    cv::Mat mat = cv::Mat(grid.size.width, grid.size.height, CV_8U, grid.data);
    cv::fillPoly(mat, input, 0, cv::LINE_8);
}

__always_inline void PolyToGrid(const geom::Polygon& poly, U8Grid& grid) {
    FillPoly(PolyToInput(poly, grid), grid);
}

__always_inline void ComplexPolyToGrid(const geom::ComplexPolygon& complex_poly, U8Grid& grid) {
    FillPoly(ComplexPolyToInput(complex_poly, grid), grid);
}

}  // namespace impl

void PolyToGrid(const geom::Polygon& poly, U8Grid& grid) { impl::PolyToGrid(poly, grid); }

void ComplexPolyToGrid(const geom::ComplexPolygon& complex_poly, U8Grid& grid) {
    impl::ComplexPolyToGrid(complex_poly, grid);
}

}  // namespace truck::fastgrid