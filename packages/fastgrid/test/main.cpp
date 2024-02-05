#include <gtest/gtest.h>

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"
#include "fastgrid/manhattan_distance.h"
#include "fastgrid/distance_transform.h"
#include "fastgrid/interpolation.h"
#include "fastgrid/Draw.h"
#include "geom/common.h"
#include "geom/pose.h"

#include <cmath>
#include <limits>
#include <memory>
#include <vector>

using namespace truck::geom;
using namespace truck::fastgrid;

TEST(Size, construction) {
    const Size a = {3, 2};
    EXPECT_EQ(a.width, 3);
    EXPECT_EQ(a.height, 2);
    EXPECT_EQ(a(), 6);
}

TEST(Size, extend) {
    const Size a = {3, 2};
    const Size ext = a.extend(2);

    EXPECT_EQ(ext.width, 5);
    EXPECT_EQ(ext.height, 4);
    EXPECT_EQ(ext(), 20);
}

TEST(Grid, index) {
    const Size size = {10, 10};
    const Pose origin = {{1, 0}, AngleVec2::fromVector(0, 1)};
    const Grid<int> grid(size, 0.5, origin);

    const Vec2 point = {-1.01, 2.01};
    const auto ref_point = grid.transform(point);
    EXPECT_EQ(ref_point.x, 2.01);
    EXPECT_EQ(ref_point.y, 2.01);

    const auto ref_index = grid.toIndex(ref_point);
    EXPECT_EQ(ref_index.x, 4);
    EXPECT_EQ(ref_index.y, 4);

    const auto ref_plain_index = grid.toPlainIndex(ref_point);
    EXPECT_EQ(ref_plain_index, 44);

    const auto index = grid.getIndex(point);
    EXPECT_EQ(index.x, 4);
    EXPECT_EQ(index.y, 4);

    const auto plain_index = grid.getPlainIndex(point);
    EXPECT_EQ(plain_index, 44);

    EXPECT_TRUE(grid.tryGetIndex(point));
    EXPECT_TRUE(grid.tryGetPlainIndex(point));
}

TEST(Grid, access) {
    const Size size = {3, 2};
    const Pose pose({0, 0}, AngleVec2::fromVector(1, 0));

    std::array<int, 6> buffer = {1, 2, 3, 4, 5, 6};
    EXPECT_EQ(std::size(buffer), static_cast<size_t>(size()));

    Grid<int> grid(size, 1, pose);
    grid.reset(buffer.data());

    EXPECT_EQ(grid[0][0], 1);
    EXPECT_EQ(grid[0][1], 2);
    EXPECT_EQ(grid[0][2], 3);
    EXPECT_EQ(grid[1][0], 4);
    EXPECT_EQ(grid[1][1], 5);
    EXPECT_EQ(grid[1][2], 6);
}

TEST(Grid, allocation) {
    const Size size = {.width = 2, .height = 3};
    const Pose pose({0, 0}, AngleVec2::fromVector(1, 0));

    Grid<int> grid(size, 1, pose);
    auto data = Allocate<int>(size);
    grid.reset(data.get());
    grid.SetTo(0);

    auto holder = makeGrid<int>(size, 1, pose);
    holder->SetTo(0);
}

TEST(ManhattanDistance, case_1) {
    const float unreachable = std::numeric_limits<float>::max();

    const Size size = {.width = 2, .height = 3};
    const double resolution = 1.0;
    const Pose origin = {{0, 0}, AngleVec2::fromVector(1, 0)};

    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 1;
    grid[1][0] = 1;
    grid[1][1] = sqrt(2);
    grid[2][0] = 2;
    grid[2][1] = sqrt(5);

    const F32GridHolder result = manhattanDistance(grid, {1.5, 2.5}, 0.5);

    EXPECT_EQ(result.grid[0][0], unreachable);
    EXPECT_EQ(result.grid[0][1], 2);
    EXPECT_EQ(result.grid[1][0], 2);
    EXPECT_EQ(result.grid[1][1], 1);
    EXPECT_EQ(result.grid[2][0], 1);
    EXPECT_EQ(result.grid[2][1], 0);
}

TEST(ManhattanDistance, case_2) {
    const float unreachable = std::numeric_limits<float>::max();

    const Size size = {2, 3};
    const double resolution = 1.0;
    const Pose pose({0, 0}, AngleVec2::fromVector(2, 0));

    auto holder = makeGrid<float>(size, resolution, pose);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 1;
    grid[1][0] = 1;
    grid[1][1] = sqrt(2);
    grid[2][0] = 2;
    grid[2][1] = sqrt(5);

    const F32GridHolder result = manhattanDistance(grid, {1.5, 2.5}, 1.5);

    EXPECT_EQ(result.grid[0][0], unreachable);
    EXPECT_EQ(result.grid[0][1], unreachable);
    EXPECT_EQ(result.grid[1][0], unreachable);
    EXPECT_EQ(result.grid[1][1], unreachable);
    EXPECT_EQ(result.grid[2][0], 1);
    EXPECT_EQ(result.grid[2][1], 0);
}

TEST(ManhattanDistance, case_3) {
    const float unreachable = std::numeric_limits<float>::max();

    const Size size = {5, 5};
    const double resolution = 1.0;
    const Pose pose({0, 0}, AngleVec2::fromVector(1, 0));

    auto holder = makeGrid<float>(size, resolution, pose);
    auto& grid = *holder;

    grid[0][0] = sqrt(5);
    grid[0][1] = sqrt(2);
    grid[0][2] = 1;
    grid[0][3] = 1;
    grid[0][4] = sqrt(2);
    grid[1][0] = sqrt(2);
    grid[1][1] = 1;
    grid[1][2] = 0;
    grid[1][3] = 0;
    grid[1][4] = 1;
    grid[2][0] = 1;
    grid[2][1] = 0;
    grid[2][2] = 1;
    grid[2][3] = 1;
    grid[2][4] = sqrt(2);
    grid[3][0] = 1;
    grid[3][1] = 0;
    grid[3][2] = 1;
    grid[3][3] = 0;
    grid[3][4] = 1;
    grid[4][0] = sqrt(2);
    grid[4][1] = 1;
    grid[4][2] = 1;
    grid[4][3] = 0;
    grid[4][4] = 1;

    const F32GridHolder result = manhattanDistance(grid, {2.1, 2.1}, 0.5);

    EXPECT_EQ(result.grid[0][0], 8);
    EXPECT_EQ(result.grid[0][1], 7);
    EXPECT_EQ(result.grid[0][2], 6);
    EXPECT_EQ(result.grid[0][3], 5);
    EXPECT_EQ(result.grid[0][4], 4);
    EXPECT_EQ(result.grid[1][0], 7);
    EXPECT_EQ(result.grid[1][1], 8);
    EXPECT_EQ(result.grid[1][2], unreachable);
    EXPECT_EQ(result.grid[1][3], unreachable);
    EXPECT_EQ(result.grid[1][4], 3);
    EXPECT_EQ(result.grid[2][0], 6);
    EXPECT_EQ(result.grid[2][1], unreachable);
    EXPECT_EQ(result.grid[2][2], 0);
    EXPECT_EQ(result.grid[2][3], 1);
    EXPECT_EQ(result.grid[2][4], 2);
    EXPECT_EQ(result.grid[3][0], 5);
    EXPECT_EQ(result.grid[3][1], unreachable);
    EXPECT_EQ(result.grid[3][2], 1);
    EXPECT_EQ(result.grid[3][3], unreachable);
    EXPECT_EQ(result.grid[3][4], 3);
    EXPECT_EQ(result.grid[4][0], 4);
    EXPECT_EQ(result.grid[4][1], 3);
    EXPECT_EQ(result.grid[4][2], 2);
    EXPECT_EQ(result.grid[4][3], unreachable);
    EXPECT_EQ(result.grid[4][4], 4);
}

TEST(ManhattanDistance, case_4) {
    const float unreachable = std::numeric_limits<float>::max();

    const Size size = {5, 5};
    const double resolution = 1.0;
    const Pose pose({0, 0}, AngleVec2::fromVector(1, 0));

    auto holder = makeGrid<float>(size, resolution, pose);
    auto& grid = *holder;

    grid[0][0] = sqrt(5);
    grid[0][1] = sqrt(2);
    grid[0][2] = 1;
    grid[0][3] = 1;
    grid[0][4] = sqrt(2);
    grid[1][0] = sqrt(2);
    grid[1][1] = 1;
    grid[1][2] = 0;
    grid[1][3] = 0;
    grid[1][4] = 1;
    grid[2][0] = 1;
    grid[2][1] = 0;
    grid[2][2] = 1;
    grid[2][3] = 1;
    grid[2][4] = sqrt(2);
    grid[3][0] = 1;
    grid[3][1] = 0;
    grid[3][2] = 1;
    grid[3][3] = 0;
    grid[3][4] = 1;
    grid[4][0] = sqrt(2);
    grid[4][1] = 1;
    grid[4][2] = 1;
    grid[4][3] = 0;
    grid[4][4] = 1;

    F32GridHolder result = manhattanDistance(grid, {2, 1}, 0.5);

    EXPECT_EQ(result.grid[0][0], unreachable);
    EXPECT_EQ(result.grid[0][1], unreachable);
    EXPECT_EQ(result.grid[0][2], unreachable);
    EXPECT_EQ(result.grid[0][3], unreachable);
    EXPECT_EQ(result.grid[0][4], unreachable);
    EXPECT_EQ(result.grid[1][0], unreachable);
    EXPECT_EQ(result.grid[1][1], unreachable);
    EXPECT_EQ(result.grid[1][2], unreachable);
    EXPECT_EQ(result.grid[1][3], unreachable);
    EXPECT_EQ(result.grid[1][4], unreachable);
    EXPECT_EQ(result.grid[2][0], unreachable);
    EXPECT_EQ(result.grid[2][1], unreachable);
    EXPECT_EQ(result.grid[2][2], unreachable);
    EXPECT_EQ(result.grid[2][3], unreachable);
    EXPECT_EQ(result.grid[2][4], unreachable);
    EXPECT_EQ(result.grid[3][0], unreachable);
    EXPECT_EQ(result.grid[3][1], unreachable);
    EXPECT_EQ(result.grid[3][2], unreachable);
    EXPECT_EQ(result.grid[3][3], unreachable);
    EXPECT_EQ(result.grid[3][4], unreachable);
    EXPECT_EQ(result.grid[4][0], unreachable);
    EXPECT_EQ(result.grid[4][1], unreachable);
    EXPECT_EQ(result.grid[4][2], unreachable);
    EXPECT_EQ(result.grid[4][3], unreachable);
    EXPECT_EQ(result.grid[4][4], unreachable);
}

TEST(DistanceTranformApprox, case_1) {
    const Size size{.width = 10, .height = 10};
    const float resolution = 1.0;
    const Pose origin{{1., 2.}, AngleVec2::fromVector(3., 4.)};

    auto holder = makeGrid<uint8_t>(size, resolution, origin);
    auto& grid = *holder;

    grid.SetTo(1);

    grid[1][4] = 0;
    grid[2][7] = 0;
    grid[3][7] = 0;
    grid[4][1] = 0;
    grid[5][5] = 0;
    grid[6][7] = 0;

    std::vector<float> data3{
        4.2343,   3.2793,   2.3243,   1.36929,  0.955002, 1.36929,  2.3243,   1.91,    2.3243,
        2.73859,  3.2793,   2.86501,  1.91,     0.955002, 0,        0.955002, 1.36929, 0.955002,
        1.36929,  2.3243,   2.3243,   1.91,     2.3243,   1.36929,  0.955002, 1.36929, 0.955002,
        0,        0.955002, 1.91,     1.36929,  0.955002, 1.36929,  2.3243,   1.91,    1.91,
        0.955002, 0,        0.955002, 1.91,     0.955002, 0,        0.955002, 1.91,    1.36929,
        0.955002, 1.36929,  0.955002, 1.36929,  2.3243,   1.36929,  0.955002, 1.36929, 1.91,
        0.955002, 0,        0.955002, 0.955002, 1.36929,  2.3243,   2.3243,   1.91,    2.3243,
        2.3243,   1.36929,  0.955002, 0.955002, 0,        0.955002, 1.91,     3.2793,  2.86501,
        3.2793,   2.73859,  2.3243,   1.91,     1.36929,  0.955002, 1.36929,  2.3243,  4.2343,
        3.82001,  4.10788,  3.69359,  3.2793,   2.73859,  2.3243,   1.91,     2.3243,  2.73859,
        5.1893,   4.77501,  5.06288,  4.64859,  4.10788,  3.69359,  3.2793,   2.86501, 3.2793,
        3.69359};

    Grid<float> expected3(size, resolution, origin);
    expected3.reset(data3.data());

    const double eps3 = 0.41;
    const auto result3 = distanceTransformApprox3(grid);
    equal(*result3, expected3, eps3);

    std::vector<float> data5{
        4.1969,  3.1969, 2.1969,  1.39999, 1,       1.39999, 2.1969,  2, 2.1969,  2.79999,
        3.1969,  3,      2,       1,       0,       1,       1.39999, 1, 1.39999, 2.1969,
        2.1969,  2,      2.1969,  1.39999, 1,       1.39999, 1,       0, 1,       2,
        1.39999, 1,      1.39999, 2.1969,  2,       2,       1,       0, 1,       2,
        1,       0,      1,       2,       1.39999, 1,       1.39999, 1, 1.39999, 2.1969,
        1.39999, 1,      1.39999, 2,       1,       0,       1,       1, 1.39999, 2.1969,
        2.1969,  2,      2.1969,  2.1969,  1.39999, 1,       1,       0, 1,       2,
        3.1969,  3,      3.1969,  2.79999, 2.1969,  2,       1.39999, 1, 1.39999, 2.1969,
        4.1969,  4,      4.1969,  3.59689, 3.1969,  2.79999, 2.1969,  2, 2.1969,  2.79999,
        5.1969,  5,      4.99689, 4.3938,  4.1969,  3.59689, 3.1969,  3, 3.1969,  3.59689,
    };

    Grid<float> expected5(size, resolution, origin);
    expected5.reset(data5.data());

    const double eps5 = 0.2;
    const auto result5 = distanceTransformApprox5(grid);
    equal(*result5, expected5, eps5);
}

TEST(DistanceTranformApprox, case_2) {
    const Size size{.width = 10, .height = 10};
    const float resolution = 1.0;
    const Pose origin{{1., 2.}, AngleVec2::fromVector(3., 4.)};

    auto holder = makeGrid<uint8_t>(size, resolution, origin);
    auto& grid = *holder;

    grid.SetTo(1);
    grid[0][0] = 0;
    grid[9][9] = 0;

    std::vector<float> data3{
        0,       0.955002, 1.91,    2.86501, 3.82001, 4.77501, 5.73001, 6.68501, 7.64001,
        8.59502, 0.955002, 1.36929, 2.3243,  3.2793,  4.2343,  5.1893,  6.1443,  7.0993,
        8.05431, 7.64001,  1.91,    2.3243,  2.73859, 3.69359, 4.64859, 5.60359, 6.55859,
        7.5136,  7.0993,   6.68501, 2.86501, 3.2793,  3.69359, 4.10788, 5.06288, 6.01788,
        6.97289, 6.55859,  6.1443,  5.73001, 3.82001, 4.2343,  4.64859, 5.06288, 5.47717,
        6.43217, 6.01788,  5.60359, 5.1893,  4.77501, 4.77501, 5.1893,  5.60359, 6.01788,
        6.43217, 5.47717,  5.06288, 4.64859, 4.2343,  3.82001, 5.73001, 6.1443,  6.55859,
        6.97289, 6.01788,  5.06288, 4.10788, 3.69359, 3.2793,  2.86501, 6.68501, 7.0993,
        7.5136,  6.55859,  5.60359, 4.64859, 3.69359, 2.73859, 2.3243,  1.91,    7.64001,
        8.05431, 7.0993,   6.1443,  5.1893,  4.2343,  3.2793,  2.3243,  1.36929, 0.955002,
        8.59502, 7.64001,  6.68501, 5.73001, 4.77501, 3.82001, 2.86501, 1.91,    0.955002,
        0,
    };

    Grid<float> expected3(size, resolution, origin);
    expected3.reset(data3.data());

    const double eps3 = 0.41;
    const auto result3 = distanceTransformApprox3(grid);
    equal(*result3, expected3, eps3);

    std::vector<float> data5{
        0, 1,       2,       3,       4,       5,       6,       7,       8,       9,
        1, 1.39999, 2.1969,  3.1969,  4.1969,  5.1969,  6.1969,  7.1969,  8.1969,  8,
        2, 2.1969,  2.79999, 3.59689, 4.3938,  5.3938,  6.3938,  7.3938,  7.1969,  7,
        3, 3.1969,  3.59689, 4.19998, 4.99689, 5.79379, 6.5907,  6.3938,  6.1969,  6,
        4, 4.1969,  4.3938,  4.99689, 5.59998, 6.39688, 5.79379, 5.3938,  5.1969,  5,
        5, 5.1969,  5.3938,  5.79379, 6.39688, 5.59998, 4.99689, 4.3938,  4.1969,  4,
        6, 6.1969,  6.3938,  6.5907,  5.79379, 4.99689, 4.19998, 3.59689, 3.1969,  3,
        7, 7.1969,  7.3938,  6.3938,  5.3938,  4.3938,  3.59689, 2.79999, 2.1969,  2,
        8, 8.1969,  7.1969,  6.1969,  5.1969,  4.1969,  3.1969,  2.1969,  1.39999, 1,
        9, 8,       7,       6,       5,       4,       3,       2,       1,       0,
    };

    Grid<float> expected5(size, resolution, origin);
    expected5.reset(data5.data());

    const double eps5 = 0.2;
    const auto result5 = distanceTransformApprox5(grid);
    equal(*result5, expected5, eps5);
}

TEST(BilinearInterpolation, case_1) {
    const double eps = 1e-5;

    const Size size{.width = 2, .height = 2};
    const double resolution = 1;
    const Pose origin({0, 0}, AngleVec2::fromVector(1, 0));
    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = -1.0;
    grid[0][1] = +1.0;
    grid[1][0] = +1.0;
    grid[1][1] = -1.0;

    Bilinear<float> bilinear(grid);

    EXPECT_NEAR(bilinear({0.5, 0.5}), -1.0, eps);
    EXPECT_NEAR(bilinear({0.5, 1.0}), 0.0, eps);
    EXPECT_NEAR(bilinear({0.5, 1.5}), +1.0, eps);
    EXPECT_NEAR(bilinear({1.5, 0.5}), +1.0, eps);
    EXPECT_NEAR(bilinear({1.5, 1.0}), 0.0, eps);
    EXPECT_NEAR(bilinear({1.5, 1.5}), -1.0, eps);
    EXPECT_NEAR(bilinear({1.0, 1.0}), 0.0, eps);
}

TEST(BilinearInterpolation, case_2) {
    constexpr double eps = 1e-5;

    constexpr Size size{.width = 2, .height = 2};
    constexpr double resolution = 1;
    constexpr Pose origin({0, 0}, AngleVec2::fromVector(3, 4));
    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 0.5;
    grid[1][0] = 0.5;
    grid[1][1] = 1;

    Bilinear<float> bilinear(grid);

    EXPECT_NEAR(bilinear({-0.2, 1.4}), 0.5, eps);
    EXPECT_NEAR(bilinear({-0.15, 1.05}), 0.25, eps);
}

TEST(BilinearInterpolation, case_3) {
    constexpr double eps = 1e-5;

    constexpr Size size{.width = 2, .height = 2};
    constexpr double resolution = 1;
    constexpr Pose origin({0, 0}, AngleVec2::fromVector(3, 4));
    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 0.5;
    grid[1][0] = 0.8;
    grid[1][1] = 1;

    Bilinear<float> bilinear(grid);
    EXPECT_NEAR(bilinear({-0.2, 1.4}), 0.575, eps);
}

TEST(BilinearInterpolation, case_4) {
    constexpr double eps = 1e-5;

    constexpr Size size{.width = 2, .height = 2};
    constexpr double resolution = 1;
    constexpr Pose origin({0, 0}, AngleVec2::fromVector(3, 4));
    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 0.3;
    grid[1][0] = 0.8;
    grid[1][1] = 1;

    Bilinear<float> bilinear(grid);
    EXPECT_NEAR(bilinear({-0.2, 1.4}), 0.525, eps);
}

TEST(BilinearInterpolation, case_5) {
    constexpr double eps = 1e-5;

    constexpr Size size{.width = 2, .height = 2};
    constexpr double resolution = 1;
    constexpr Pose origin({0, 0}, AngleVec2::fromVector(3, 4));
    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 0.4;
    grid[1][0] = 0.8;
    grid[1][1] = 1;

    Bilinear<float> bilinear(grid);
    EXPECT_NEAR(bilinear({-0.46, 1.22}), 0.536, eps);
}

TEST(BilinearInterpolation, case_6) {
    constexpr double eps = 1e-5;

    constexpr Size size{.width = 3, .height = 3};
    constexpr double resolution = 0.5;
    constexpr Pose origin({0, 0}, AngleVec2::fromVector(1, 2));
    auto holder = makeGrid<float>(size, resolution, origin);
    auto& grid = *holder;

    grid[0][0] = 0;
    grid[0][1] = 0.5;
    grid[0][2] = 0.75;
    grid[1][0] = 0.5;
    grid[1][1] = 1;
    grid[1][2] = 1.5;
    grid[2][0] = 0;
    grid[2][1] = 1;
    grid[2][2] = 2;

    Bilinear<float> bilinear(grid);
    EXPECT_NEAR(bilinear({-0.268328, 0.581378}), 0.4, eps);
    EXPECT_NEAR(bilinear({-0.0447214, 1.02859}), 0.8625, eps);
    EXPECT_NEAR(bilinear({-0.402492, 1.20748}), 1.195, eps);
}

TEST(PolyToGrid, case_1) {
    constexpr double eps = 1e-7;

    Size size{.width = 3, .height = 3};
    double resolution = 1.0;
    Pose origin({0, 0}, AngleVec2::fromVector(1, 0));
    auto holder = makeGrid<uint8_t>(size, resolution, origin);
    auto& grid = *holder;

    {
        grid.SetTo(1);

        Polygon poly{Vec2(1, 0), Vec2(3, 0), Vec2(3, 2 - eps), Vec2(1, 2 - eps)};
        Draw(poly, grid);

        EXPECT_EQ(grid[0][0], 1);
        EXPECT_EQ(grid[0][1], 0);
        EXPECT_EQ(grid[0][2], 0);
        EXPECT_EQ(grid[1][0], 1);
        EXPECT_EQ(grid[1][1], 0);
        EXPECT_EQ(grid[1][2], 0);
        EXPECT_EQ(grid[2][0], 1);
        EXPECT_EQ(grid[2][1], 1);
        EXPECT_EQ(grid[2][2], 1);
    }

    {
        grid.SetTo(1);

        Polygon poly{Vec2(1.3, 0.3), Vec2(2.7, 1.7), Vec2(2.9, 2.7), Vec2(1.2, 2.3)};
        Draw(poly, grid);

        EXPECT_EQ(grid[0][0], 1);
        EXPECT_EQ(grid[0][1], 0);
        EXPECT_EQ(grid[0][2], 1);
        EXPECT_EQ(grid[1][0], 1);
        EXPECT_EQ(grid[1][1], 0);
        EXPECT_EQ(grid[1][2], 0);
        EXPECT_EQ(grid[2][0], 1);
        EXPECT_EQ(grid[2][1], 0);
        EXPECT_EQ(grid[2][2], 0);
    }

    {
        grid.SetTo(1);

        Polygon poly{Vec2(-0.1, -0.9), Vec2(3.9, 0.1), Vec2(3.1, 2.8), Vec2(0.2, 3.2)};
        Draw(poly, grid);

        EXPECT_EQ(grid[0][0], 0);
        EXPECT_EQ(grid[0][1], 0);
        EXPECT_EQ(grid[0][2], 0);
        EXPECT_EQ(grid[1][0], 0);
        EXPECT_EQ(grid[1][1], 0);
        EXPECT_EQ(grid[1][2], 0);
        EXPECT_EQ(grid[2][0], 0);
        EXPECT_EQ(grid[2][1], 0);
        EXPECT_EQ(grid[2][2], 0);
    }
}

TEST(PolyToGrid, case_2) {
    Size size{.width = 3, .height = 3};
    double resolution = 0.5;
    Pose origin({0, 0}, AngleVec2::fromVector(0, 1));
    auto holder = makeGrid<uint8_t>(size, resolution, origin);
    auto& grid = *holder;

    {
        grid.SetTo(1);

        Polygon poly{Vec2(-2.5, 2.5), Vec2(-0.5, 0.5), Vec2(-2.5, 0.5)};
        Draw(poly, grid);

        EXPECT_EQ(grid[0][0], 0);
        EXPECT_EQ(grid[0][1], 1);
        EXPECT_EQ(grid[0][2], 1);
        EXPECT_EQ(grid[1][0], 0);
        EXPECT_EQ(grid[1][1], 0);
        EXPECT_EQ(grid[1][2], 1);
        EXPECT_EQ(grid[2][0], 0);
        EXPECT_EQ(grid[2][1], 0);
        EXPECT_EQ(grid[2][2], 0);
    }
}

TEST(ComplexPolyToGrid, case_1) {
    constexpr double eps = 1e-7;

    Size size{.width = 3, .height = 3};
    double resolution = 1.0;
    Pose origin({0, 0}, AngleVec2::fromVector(1, 0));
    auto holder = makeGrid<uint8_t>(size, resolution, origin);
    auto& grid = *holder;

    {
        grid.SetTo(1);

        ComplexPolygon poly;
        poly.outer = {Vec2(0, 0), Vec2(0, 3), Vec2(3, 3), Vec2(3, 0)};
        poly.inners = {
            {Vec2(1 - eps, 1 - eps),
             Vec2(1 - eps, 2 + eps),
             Vec2(2 + eps, 2 + eps),
             Vec2(2 + eps, 1 - eps)}};

        Draw(poly, grid);

        EXPECT_EQ(grid[0][0], 0);
        EXPECT_EQ(grid[0][1], 0);
        EXPECT_EQ(grid[0][2], 0);
        EXPECT_EQ(grid[1][0], 0);
        EXPECT_EQ(grid[1][1], 1);
        EXPECT_EQ(grid[1][2], 0);
        EXPECT_EQ(grid[2][0], 0);
        EXPECT_EQ(grid[2][1], 0);
        EXPECT_EQ(grid[2][2], 0);
    }
}

TEST(ComplexPolyToGrid, case_2) {
    constexpr double eps = 1e-7;

    Size size{.width = 5, .height = 5};
    double resolution = 1.0;
    Pose origin({0, 0}, AngleVec2::fromVector(1, 0));
    auto holder = makeGrid<uint8_t>(size, resolution, origin);
    auto& grid = *holder;

    {
        grid.SetTo(1);

        ComplexPolygon poly;
        poly.outer = {Vec2(0, 0), Vec2(0, 5), Vec2(5, 5), Vec2(5, 0)};
        poly.inners = {
            {Vec2(1 - eps, 1 - eps),
             Vec2(1 - eps, 2 + eps),
             Vec2(2 + eps, 2 + eps),
             Vec2(2 + eps, 1 - eps)},
            {Vec2(3 - eps, 3 - eps),
             Vec2(3 - eps, 4 + eps),
             Vec2(4 + eps, 4 + eps),
             Vec2(4 + eps, 3 - eps)}};

        Draw(poly, grid);

        EXPECT_EQ(grid[0][0], 0);
        EXPECT_EQ(grid[0][1], 0);
        EXPECT_EQ(grid[0][2], 0);
        EXPECT_EQ(grid[0][3], 0);
        EXPECT_EQ(grid[0][4], 0);
        EXPECT_EQ(grid[1][0], 0);
        EXPECT_EQ(grid[1][1], 1);
        EXPECT_EQ(grid[1][2], 0);
        EXPECT_EQ(grid[1][3], 0);
        EXPECT_EQ(grid[1][4], 0);
        EXPECT_EQ(grid[2][0], 0);
        EXPECT_EQ(grid[2][1], 0);
        EXPECT_EQ(grid[2][2], 0);
        EXPECT_EQ(grid[2][3], 0);
        EXPECT_EQ(grid[2][4], 0);
        EXPECT_EQ(grid[3][0], 0);
        EXPECT_EQ(grid[3][1], 0);
        EXPECT_EQ(grid[3][2], 0);
        EXPECT_EQ(grid[3][3], 1);
        EXPECT_EQ(grid[3][4], 0);
        EXPECT_EQ(grid[4][0], 0);
        EXPECT_EQ(grid[4][1], 0);
        EXPECT_EQ(grid[4][2], 0);
        EXPECT_EQ(grid[4][3], 0);
        EXPECT_EQ(grid[4][4], 0);
    }
}