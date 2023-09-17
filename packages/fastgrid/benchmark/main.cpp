#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "fastgrid/distance_transform.h"

#include <benchmark/benchmark.h>
#include <opencv2/opencv.hpp>

#include <fstream>
#include <iostream>
#include <string_view>

namespace {

truck::fastgrid::U8GridHolder readGrid(std::string_view file_path) {
    std::ifstream file(file_path.data());
    truck::fastgrid::Size size;
    file >> size.height >> size.width;
    truck::fastgrid::U8GridHolder holder = makeGrid<uint8_t>(size, 1);
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

struct DistanceTransformFixture : public ::benchmark::Fixture {
  public:
    void SetUpInput(const std::string& path) {
        holder = readGrid(path);
        buf3 = makeGrid<int>(holder->size.extend(2), holder->resolution);
        buf5 = makeGrid<int>(holder->size.extend(4), holder->resolution);
        result = makeGrid<float>(holder->size, holder->resolution);
        mat = cv::Mat(holder->size.height, holder->size.width, CV_8UC1, holder->data);
    }

    truck::fastgrid::U8GridHolder holder;
    truck::fastgrid::S32GridHolder buf3, buf5;
    truck::fastgrid::F32GridHolder result;
    cv::Mat mat;
};

struct DistanceTransformSmallFixture : public DistanceTransformFixture {
    void SetUp(const ::benchmark::State&) override {
        SetUpInput("/truck/packages/fastgrid/benchmark/test_small.txt");
    }
};

struct DistanceTransformMediumFixture : public DistanceTransformFixture {
    void SetUp(const ::benchmark::State&) override {
        SetUpInput("/truck/packages/fastgrid/benchmark/test_medium.txt");
    }
};

struct DistanceTransformLargeFixture : public DistanceTransformFixture {
    void SetUp(const ::benchmark::State&) override {
        SetUpInput("/truck/packages/fastgrid/benchmark/test_large.txt");
    }
};

BENCHMARK_DEFINE_F(DistanceTransformSmallFixture, DistanceTransform3Small)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::distanceTransformApprox3(*holder, *buf3, *result);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformSmallFixture, DistanceTransform5Small)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::distanceTransformApprox5(*holder, *buf5, *result);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformMediumFixture, DistanceTransform3Medium)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::distanceTransformApprox3(*holder, *buf3, *result);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformMediumFixture, DistanceTransform3MediumOpenCV)
(benchmark::State& state) {
    for (auto _ : state) {
        cv::Mat dst;
        cv::distanceTransform(mat, dst, cv::DIST_L2, 3);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformMediumFixture, DistanceTransform5Medium)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::distanceTransformApprox5(*holder, *buf5, *result);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformMediumFixture, DistanceTransform5MediumOpenCV)
(benchmark::State& state) {
    for (auto _ : state) {
        cv::Mat dst;
        cv::distanceTransform(mat, dst, cv::DIST_L2, 5);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformLargeFixture, DistanceTransform3Large)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::distanceTransformApprox3(*holder, *buf3, *result);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformLargeFixture, DistanceTransform3LargeOpenCV)
(benchmark::State& state) {
    for (auto _ : state) {
        cv::Mat dst;
        cv::distanceTransform(mat, dst, cv::DIST_L2, 3);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformLargeFixture, DistanceTransform5Large)
(benchmark::State& state) {
    for (auto _ : state) {
        truck::fastgrid::distanceTransformApprox5(*holder, *buf5, *result);
    }
}

BENCHMARK_DEFINE_F(DistanceTransformLargeFixture, DistanceTransform5LargeOpenCV)
(benchmark::State& state) {
    for (auto _ : state) {
        cv::Mat dst;
        cv::distanceTransform(mat, dst, cv::DIST_L2, 5);
    }
}

BENCHMARK_REGISTER_F(DistanceTransformSmallFixture, DistanceTransform3Small)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(1000)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformSmallFixture, DistanceTransform5Small)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(1000)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformMediumFixture, DistanceTransform3Medium)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(200)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformMediumFixture, DistanceTransform3MediumOpenCV)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(200)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformMediumFixture, DistanceTransform5Medium)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(200)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformMediumFixture, DistanceTransform5MediumOpenCV)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(200)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformLargeFixture, DistanceTransform3Large)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(50)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformLargeFixture, DistanceTransform3LargeOpenCV)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(50)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformLargeFixture, DistanceTransform5Large)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(50)
    ->UseRealTime();

BENCHMARK_REGISTER_F(DistanceTransformLargeFixture, DistanceTransform5LargeOpenCV)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(50)
    ->UseRealTime();

BENCHMARK_MAIN();
