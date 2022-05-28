#pragma once

#include <vector>
#include <array>
#include <optional>

namespace pure_pursuit::detail {

template<size_t powers, class F>
class TaylorSeriesIntgrator {
private:
    std::vector<std::array<double, powers - 1>> precalc;
    double from, step;
public:
    TaylorSeriesIntgrator(const F& func, double from, double to, size_t n): from(from), step((to - from) / n) {
        precalc.resize(n + 1);
        double x2, f1, f2;
        x2 = from;
        f2 = func(from);
        for (size_t i = 1; i <= n; ++i) {
            f1 = f2;
            x2 = from + (to - from) * i / n;
            f2 = func(x2);
            double v1 = f1, v2 = f2;
            for (size_t p = 1; p < powers; ++p) {
                precalc[i][p - 1] = precalc[i - 1][p - 1] + (v1 + v2) * step / 2;
                v1 *= f1;
                v2 *= f2;
            }
        }
    }

    double getAnaliticFuncIntegral(const std::array<double, powers>& coefs, double a, double b, double k) {
        double ans = coefs[0] * (b - a);
        size_t l = std::max<long long>(std::floor((a - from) / step), 0);
        size_t r = std::min<long long>(std::ceil((b - from) / step), precalc.size() - 1);
        double p = k;
        for (size_t i = 1; i < powers; ++i) {
            ans += coefs[i] * p * (precalc[r][i - 1] - precalc[l][i - 1]);
            p *= k;
        }
        return ans;
    }
};

template<size_t powers, class F>
inline auto makeTaylorSeriesIntgrator(const F& func, double from, double to, size_t n) {
    return TaylorSeriesIntgrator<powers, F>(func, from, to, n);
}

};
