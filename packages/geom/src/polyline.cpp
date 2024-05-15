#include "geom/polyline.h"

#include "geom/distance.h"
#include "common/exception.h"

#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>

namespace truck::geom {

namespace {

Vec2 toVec2(const Eigen::Vector2d& eigen_point) { return Vec2(eigen_point.x(), eigen_point.y()); }

Eigen::Vector2d toEigenVector2d(const Vec2& point) { return Eigen::Vector2d(point.x, point.y); }

Eigen::MatrixXd toEigenMatrixXd(const std::vector<Vec2>& points) {
    VERIFY(points.size() > 0);

    size_t points_count = points.size();
    Eigen::MatrixXd eigen_mat(2, points_count);

    for (size_t i = 0; i < points_count; i++) {
        eigen_mat.col(i) = toEigenVector2d(points[i]);
    }

    return eigen_mat;
}

}  // namespace

UniformStepper<Polyline> Polyline::ubegin() const noexcept { return UniformStepper(this); }

UniformStepper<Polyline> Polyline::ubegin(double step_length) const noexcept {
    return UniformStepper(this, step_length);
}

UniformStepper<Polyline> Polyline::uend() const noexcept {
    return UniformStepper(this, this->end() - 1);
}

double Polyline::len() const noexcept {
    VERIFY(this->size() > 1);

    double length = 0.0;

    for (auto it = this->begin(); it != this->end() - 1; it++) {
        length += geom::distance(*it, *(it + 1));
    }

    return length;
}

Polyline toSpline(const Polyline& polyline, double step, size_t degree) noexcept {
    VERIFY(step > 0);
    VERIFY(degree > 0);
    VERIFY(polyline.size() > degree);

    Eigen::Spline<double, 2> spline = Eigen::SplineFitting<Eigen::Spline<double, 2>>::Interpolate(
        toEigenMatrixXd(polyline), degree);

    size_t spline_points = std::ceil<size_t>(polyline.len() / step);
    spline_points = (spline_points < 2) ? 2 : spline_points;

    Polyline polyline_smoothed;

    for (size_t i = 0; i < spline_points; i++) {
        double t = static_cast<double>(i) / (spline_points - 1);
        polyline_smoothed.emplace_back(toVec2(spline(t)));
    }

    return polyline_smoothed;
}

Polyline toQuadraticSpline(const Polyline& polyline, double step) noexcept {
    return toSpline(polyline, step, 2);
}

Polyline toLinearSpline(const Polyline& polyline, double step) noexcept {
    return toSpline(polyline, step, 1);
}

}  // namespace truck::geom
