#pragma once

#include "geom/pose.h"
#include "geom/distance.h"
#include "geom/interpolation.h"
#include "geom/motion_state.h"
#include "geom/interpolation.h"
#include "common/exception.h"
#include <vector>
#include <optional>

namespace truck::geom {

struct PolylineIdx {
    std::size_t seg_n = 0;
    double t = 0;  // [0, 1]
};

template<class P>
struct AdvanceResult {
    P point;
    double dist = 0.0;
    PolylineIdx poly_idx;
    bool reached_end = false;
};

template<typename P, typename Interpolator = LinearInterpolator<P>>
class PolylineIndex {
  public:
    PolylineIndex(std::vector<P>&& range) : points_(std::move(range)) {
        VERIFY(points_.size() >= 2);

        distances_.reserve(points_.size());
        distances_.push_back(0);

        // std::cerr << "distances:\n[ ";
        auto prev = points_.begin();
        for (auto curr = points_.begin() + 1; curr != points_.end(); prev = curr, ++curr) {
            distances_.push_back(geom::distance(*prev, *curr) + distances_.back());
            // std::cerr << fmt("%.2f", distances_.back()) << ' ';
        }
        // std::cerr << "]\n\n";
    }

    P StateAt(const PolylineIdx idx) const {
        const auto& a = points_[idx.seg_n];
        const auto& b = points_[idx.seg_n + 1];
        return Interpolator{}(a, b, idx.t);
    }

    double DistAt(const PolylineIdx idx) const {
        const auto& a = distances_[idx.seg_n];
        const auto& b = distances_[idx.seg_n + 1];
        return Interpolator{}(a, b, idx.t);
    }

    std::pair<P, double> At(const PolylineIdx idx) const {
        return std::make_pair(StateAt(idx), DistAt(idx));
    }

    // default parameter to use in the loop initialization
    AdvanceResult<P> AdvanceFromBegin(double dist = 0) const {
        double start = 0;
        double target = start + dist;

        if (target >= Length()) {
            return AdvanceResult<P>{
                .point = points_.back(),
                .dist = Length(),
                .poly_idx = {.seg_n = points_.size() - 1, .t = 1},
                .reached_end = true};
        }

        // index of the first point at which distance traveled exceeds `target`
        size_t index = std::distance(
            distances_.begin(), std::upper_bound(distances_.begin(), distances_.end(), target));

        VERIFY(index < distances_.size());

        --index;

        // std::cerr << fmt(
        //     "index: %d, dist[index]: %.2f, target: %.2f\n", index, distances_[index], target);

        const double rem_dist = target - distances_[index];
        const double seg_len = geom::distance(points_[index], points_[index + 1]);
        const double t = seg_len ? clamp(rem_dist / seg_len, 0., 1.) : 0;

        PolylineIdx polyidx = {.seg_n = index, .t = t};

        // std::cerr << fmt(
        //     "rem: %.3f, t: %.3f, reached_end: %s\n", rem_dist, t, (rem_dist < 0) ? "true" :
        //     "false")
        //           << '\n';

        return AdvanceResult<P>{
            .point = StateAt(polyidx), .dist = target, .poly_idx = polyidx, .reached_end = false};
    }

    AdvanceResult<P> AdvanceFrom(const PolylineIdx from_id, double dist) const {
        double start = DistAt(from_id);
        return AdvanceFromBegin(start + dist);
    }

    double Length() const { return distances_.back(); }

    const auto& Points() const { return points_; }
    const auto& Distances() const { return distances_; }

  private:
    std::vector<P> points_;
    std::vector<double> distances_;
};
}  // namespace truck::geom
