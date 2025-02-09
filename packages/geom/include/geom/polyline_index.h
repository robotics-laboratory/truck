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
    PolylineIdx poly_idx;
    bool reached_end = false;
};

template<typename P>
class PolylineIndex {
  public:
    PolylineIndex(std::vector<P>&& range) : points_(std::move(range)) {
        VERIFY(points_.size() >= 2);

        distances_.reserve(points_.size());
        distances_.push_back(0);

        std::cerr << "distances:\n[ ";
        auto prev = points_.begin();
        for (auto curr = points_.begin() + 1; curr != points_.end(); prev = curr, ++curr) {
            distances_.push_back(distance(*prev, *curr) + distances_.back());
            std::cerr << fmt("%.2f", distances_.back()) << ' ';
        }
        std::cerr << "]\n\n";
    }

    P At(const PolylineIdx idx) const {
        const auto& a = points_[idx.seg_n];
        const auto& b = points_[idx.seg_n + 1];
        return LinearInterpolator<P>{}(a, b, idx.t);
    }

    AdvanceResult<P> AdvanceFromBegin(double dist) const {
        return AdvanceFrom(PolylineIdx{.seg_n = 0, .t = 0}, dist);
    }

    AdvanceResult<P> AdvanceFrom(PolylineIdx from_id, double dist) const {
        double start = LinearInterpolator<double>{}(
            distances_[from_id.seg_n], distances_[from_id.seg_n + 1], from_id.t);
        double target = start + dist;

        if (target > ArcLength()) {
            return AdvanceResult<P>{
                .point = points_.back(),
                .poly_idx = {.seg_n = points_.size(), .t = 0},
                .reached_end = true};
        }

        std::cerr << fmt("seg_n: %d, start: %.2f, target: %.2f\n", from_id.seg_n, start, target);

        size_t l = from_id.seg_n;
        size_t r = distances_.size();
        size_t index = 0;

        // std::cerr << "\t---bs---\n";
        while (l < r) {
            index = l + (r - l) / 2;
            // std::cerr << fmt(
            //     "\tl: %d, r: %d, index: %d, dist: %.2f, target: %.2f\n",
            //     l,
            //     r,
            //     index,
            //     distances_[index],
            //     target);

            if (distances_[index] < target) {
                l = index + 1;
            } else {
                r = index;
            }
        }
        // std::cerr << "\t---\\bs---\n";

        if (distances_[index] > target) {
            // std::cerr << "Decrementing\n";
            index--;
        }

        std::cerr << fmt(
            "index: %d, dist[index]: %.2f, target: %.2f\n", index, distances_[index], target);

        const auto rem_dist = target - distances_[index];
        const auto t = clamp(rem_dist / distance(points_[index], points_[index + 1]), 0., 1.);

        std::cerr << fmt(
            "rem: %.3f, t: %.3f, reached_end: %s\n", rem_dist, t, (rem_dist < 0) ? "true" : "false")
                  << '\n';

        return AdvanceResult<P>{
            .point = LinearInterpolator<P>{}(points_[index], points_[index + 1], t),
            .poly_idx = {.seg_n = index, .t = t},
            .reached_end = false};
    }

    auto begin() const { return points_.begin(); }
    auto end() const { return points_.end(); }

    const auto& Distances() const { return distances_; }
    double ArcLength() const { return distances_.back(); }

  private:
    std::vector<P> points_;
    std::vector<double> distances_;
};
}  // namespace truck::geom
