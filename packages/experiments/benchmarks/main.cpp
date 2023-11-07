#include <benchmark/benchmark.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <string_view>

constexpr size_t matrix_size = 1 << 13;
constexpr size_t matrix_data_size = matrix_size * matrix_size;

constexpr size_t cache_line_size = 64;
constexpr size_t block_size = cache_line_size / sizeof(int);

struct BasicMatrixTransposeFixture : public ::benchmark::Fixture {
    void SetUp(const ::benchmark::State&) override {
        std::iota(matrix_data, matrix_data + matrix_data_size, 0);
    }

    int matrix_data[matrix_data_size];
};

struct CacheAwareMatrixTransposeFixture : public ::benchmark::Fixture {
    void SetUp(const ::benchmark::State&) override {
        std::iota(matrix_data, matrix_data + matrix_data_size, 0);
    }

    alignas(cache_line_size) int matrix_data[matrix_data_size];
};

BENCHMARK_DEFINE_F(BasicMatrixTransposeFixture, BasicMatrixTranspose)
(benchmark::State& state) {
    for (auto _ : state) {
        for (size_t i = 0; i < matrix_size; ++i) {
            for (size_t j = 0; j < matrix_size; ++j) {
                std::swap(matrix_data[i * matrix_size + j], matrix_data[j * matrix_size + i]);
            }
        }
    }
}

BENCHMARK_DEFINE_F(CacheAwareMatrixTransposeFixture, CacheAwareMatrixTranspose)
(benchmark::State& state) {
    for (auto _ : state) {
        for (size_t i = 0; i < matrix_size; i += block_size) {
            for (size_t j = 0; j < i; j += block_size) {
                for (size_t p = i; p < i + block_size && p < matrix_size; ++p) {
                    for (size_t q = j; q < j + block_size && q < i; ++q) {
                        std::swap(
                            matrix_data[p * matrix_size + q], matrix_data[q * matrix_size + p]);
                    }
                }
            }
        }
    }
}

BENCHMARK_REGISTER_F(BasicMatrixTransposeFixture, BasicMatrixTranspose)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(20)
    ->UseRealTime();

BENCHMARK_REGISTER_F(CacheAwareMatrixTransposeFixture, CacheAwareMatrixTranspose)
    ->Unit(benchmark::kMillisecond)
    ->Iterations(20)
    ->UseRealTime();

BENCHMARK_MAIN();
