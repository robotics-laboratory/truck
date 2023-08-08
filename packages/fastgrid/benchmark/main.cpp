#include <benchmark/benchmark.h>

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "fastgrid/distance_transform.h"

#include <exception>
#include <fstream>
#include <iostream>
#include <string_view>

namespace {

truck::fastgrid::U8GridHolder ReadGrid(std::string_view file_path) {
    std::ifstream file(file_path.data());
    truck::fastgrid::Size size;
    file >> size.height >> size.width;
    truck::fastgrid::U8GridHolder holder = MakeGrid<uint8_t>(size, 1);
    for (int i = 0; i < size.height; ++i) {
        for (int j = 0, v; j < size.width; ++j) {
            file >> v;
            holder.ptr[i * size.width + j] = v;
        }
    }
    file.close();
    return holder;
}

}  // namespace

class DistanceTransformSmallFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State&) override {
        holder = ReadGrid("/truck/packages/fastgrid/benchmark/test_small.txt");
    }

    truck::fastgrid::U8GridHolder holder;
};

class DistanceTransformMediumFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State&) override {
        holder = ReadGrid("/truck/packages/fastgrid/benchmark/test_medium.txt");
    }

    truck::fastgrid::U8GridHolder holder;
};

class DistanceTransformLargeFixture : public ::benchmark::Fixture {
  public:
    void SetUp(const ::benchmark::State&) override {
        holder = ReadGrid("/truck/packages/fastgrid/benchmark/test_large.txt");
    }

    truck::fastgrid::U8GridHolder holder;
};

BENCHMARK_DEFINE_F(DistanceTransformSmallFixture, DistanceTransform3Small)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::DistanceTransformApprox3(holder.grid);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformSmallFixture, DistanceTransform5Small)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::DistanceTransformApprox5(holder.grid);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformMediumFixture, DistanceTransform3Medium)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::DistanceTransformApprox3(holder.grid);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformMediumFixture, DistanceTransform5Medium)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::DistanceTransformApprox5(holder.grid);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformLargeFixture, DistanceTransform3Large)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::DistanceTransformApprox3(holder.grid);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformLargeFixture, DistanceTransform5Large)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::DistanceTransformApprox5(holder.grid);
    }
}

BENCHMARK_REGISTER_F(DistanceTransformSmallFixture, DistanceTransform3Small)
    ->Unit(benchmark::kMillisecond)
    ->Repetitions(100)
    ->Iterations(1)
    ->Threads(1)
    ->UseRealTime();
BENCHMARK_REGISTER_F(DistanceTransformSmallFixture, DistanceTransform5Small)
    ->Unit(benchmark::kMillisecond)
    ->Repetitions(100)
    ->Iterations(1)
    ->Threads(1)
    ->UseRealTime();
BENCHMARK_REGISTER_F(DistanceTransformMediumFixture, DistanceTransform3Medium)
    ->Unit(benchmark::kMillisecond)
    ->Repetitions(100)
    ->Iterations(1)
    ->Threads(1)
    ->UseRealTime();
BENCHMARK_REGISTER_F(DistanceTransformMediumFixture, DistanceTransform5Medium)
    ->Unit(benchmark::kMillisecond)
    ->Repetitions(100)
    ->Iterations(1)
    ->Threads(1)
    ->UseRealTime();
BENCHMARK_REGISTER_F(DistanceTransformLargeFixture, DistanceTransform3Large)
    ->Unit(benchmark::kMillisecond)
    ->Repetitions(100)
    ->Iterations(1)
    ->Threads(1)
    ->UseRealTime();
BENCHMARK_REGISTER_F(DistanceTransformLargeFixture, DistanceTransform5Large)
    ->Unit(benchmark::kMillisecond)
    ->Repetitions(100)
    ->Iterations(1)
    ->Threads(1)
    ->UseRealTime();