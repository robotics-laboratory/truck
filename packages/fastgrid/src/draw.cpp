#include "fastgrid/draw.h"

#include "common/exception.h"

#include <opencv2/imgproc.hpp>

#include <vector>

namespace truck::fastgrid {

namespace impl {

__always_inline std::vector<cv::Point> polyToInput(const geom::Polygon& poly, const U8Grid& grid) {
    std::vector<cv::Point> input;
    input.reserve(poly.size());
    for (const auto& vertex : poly) {
        const auto point = grid.transform(vertex);
        input.emplace_back(point.x, point.y);
    }

    return input;
}

__always_inline std::vector<std::vector<cv::Point>> complexPolyToInput(
    const geom::ComplexPolygon& complex_poly, const U8Grid& grid) {
    std::vector<std::vector<cv::Point>> input;
    input.reserve(complex_poly.inners.size() + 1);

    {
        std::vector<cv::Point> points;
        for (const auto& vertex : complex_poly.outer) {
            const auto point = grid.transform(vertex);
            points.emplace_back(point.x, point.y);
        }
        input.push_back(std::move(points));
    }

    for (const auto& inner : complex_poly.inners) {
        std::vector<cv::Point> points;
        for (const auto& vertex : inner) {
            const auto point = grid.transform(vertex);
            points.emplace_back(point.x, point.y);
        }
        input.push_back(std::move(points));
    }

    return input;
}

__always_inline void fillPoly(cv::InputArrayOfArrays input, U8Grid& grid) {
    cv::Mat mat = cv::Mat(grid.size.width, grid.size.height, CV_8U, grid.data);
    cv::fillPoly(mat, input, 0, cv::LINE_8);
}

__always_inline void draw(const geom::Polygon& poly, U8Grid& grid) {
    fillPoly(polyToInput(poly, grid), grid);
}

__always_inline void draw(const geom::ComplexPolygon& complex_poly, U8Grid& grid) {
    fillPoly(complexPolyToInput(complex_poly, grid), grid);
}

}  // namespace impl

void Draw(const geom::Polygon& poly, U8Grid& grid) { impl::draw(poly, grid); }

void Draw(const geom::ComplexPolygon& complex_poly, U8Grid& grid) {
    impl::draw(complex_poly, grid);
}

}  // namespace truck::fastgrid
