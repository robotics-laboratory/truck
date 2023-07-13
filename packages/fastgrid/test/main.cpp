#include <gtest/gtest.h>

#include "fastgrid/grid.h"
#include "fastgrid/holder.h"

#include "geom/pose.h"

#include <memory>

using namespace truck::geom;
using namespace truck::fastgrid;

TEST(Size, workability) {
    const Size a = {.width = 1, .height = 2};
    EXPECT_EQ(a.width, 1U);
    EXPECT_EQ(a.height, 2U);
    EXPECT_EQ(a(), 2U);

    const int64_t big_value = 1e9 + 7;
    const Size b = {.width = big_value, .height = big_value};
    EXPECT_EQ(b.width, static_cast<size_t>(big_value));
    EXPECT_EQ(b.height, static_cast<size_t>(big_value));
    EXPECT_EQ(b(), static_cast<size_t>(big_value) * static_cast<size_t>(big_value));
}

TEST(Grid, workability) {
    const Size sz_1 = {.width = 2, .height = 3};
    const int res_1 = 10;
    const Pose pose_1({1, 2}, {3, 4});
    std::unique_ptr<int> data_1(new int[6]);
    *(data_1.get() + 1) = 2;

    Grid<int> grid_1(sz_1, res_1, pose_1);
    grid_1.Reset(data_1.get());
    EXPECT_EQ(grid_1.size.width, 2U);
    EXPECT_EQ(grid_1.size.height, 3U);
    EXPECT_EQ(grid_1.resolution, 10);
    EXPECT_EQ(*(grid_1.data + 1), 2);
    EXPECT_EQ(grid_1.origin->pos.x, 1);
    EXPECT_EQ(grid_1.origin->pos.y, 2);
    EXPECT_EQ(grid_1.origin->dir.x, 3);
    EXPECT_EQ(grid_1.origin->dir.y, 4);

    const Size sz_2 = {.width = 2, .height = 3};
    const int res_2 = 10;

    U8Grid grid_2(sz_2, res_2);
    EXPECT_EQ(grid_2.size.width, 2U);
    EXPECT_EQ(grid_2.size.height, 3U);
    EXPECT_EQ(grid_2.resolution, 10);
    EXPECT_EQ(grid_2.origin, std::nullopt);
}

TEST(GridDataPtr, allocation_and_workabillity) {
    const Size sz = {.width = 2, .height = 3};

    GridDataPtr<int> ptr_1 = Allocate<int>(sz);
    *(ptr_1.get() + 5) = 2;
    EXPECT_EQ(*(ptr_1.get() + 5), 2);

    U8GridDataPtr ptr_2 = Allocate<uint8_t>(sz);
    *(ptr_2.get() + 5) = 2;
    EXPECT_EQ(*(ptr_2.get() + 5), 2);
}

TEST(GridHolder, make_grid) {
    const Size sz_1 = {.width = 2, .height = 3};
    const int res_1 = 10;
    const Pose pose_1({1, 2}, {3, 4});
    std::unique_ptr<int> data_1(new int[6]);
    *(data_1.get() + 1) = 2;

    Grid<int> grid_1(sz_1, res_1, pose_1);
    grid_1.Reset(data_1.get());

    GridHolder<int> grid_holder_1 = MakeGridLike<int>(grid_1);
    EXPECT_EQ(grid_holder_1.grid.size.width, 2U);
    EXPECT_EQ(grid_holder_1.grid.size.height, 3U);
    EXPECT_EQ(grid_holder_1.grid.resolution, 10);
    EXPECT_EQ(*(grid_holder_1.grid.data + 1), 2);
    EXPECT_EQ(grid_holder_1.grid.origin->pos.x, 1);
    EXPECT_EQ(grid_holder_1.grid.origin->pos.y, 2);
    EXPECT_EQ(grid_holder_1.grid.origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_1).origin->dir.y, 4);
    EXPECT_EQ((*grid_holder_1).size.width, 2U);
    EXPECT_EQ((*grid_holder_1).size.height, 3U);
    EXPECT_EQ((*grid_holder_1).resolution, 10);
    EXPECT_EQ(*((*grid_holder_1).data + 1), 2);
    EXPECT_EQ((*grid_holder_1).origin->pos.x, 1);
    EXPECT_EQ((*grid_holder_1).origin->pos.y, 2);
    EXPECT_EQ((*grid_holder_1).origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_1).origin->dir.y, 4);
    EXPECT_EQ(grid_holder_1->size.width, 2U);
    EXPECT_EQ(grid_holder_1->size.height, 3U);
    EXPECT_EQ(grid_holder_1->resolution, 10);
    EXPECT_EQ(*(grid_holder_1->data + 1), 2);
    EXPECT_EQ(grid_holder_1->origin->pos.x, 1);
    EXPECT_EQ(grid_holder_1->origin->pos.y, 2);
    EXPECT_EQ(grid_holder_1->origin->dir.x, 3);

    GridHolder<int> grid_holder_2(MakeGridLike<int>(grid_holder_1));
    EXPECT_EQ(grid_holder_2.grid.size.width, 2U);
    EXPECT_EQ(grid_holder_2.grid.size.height, 3U);
    EXPECT_EQ(grid_holder_2.grid.resolution, 10);
    EXPECT_EQ(*(grid_holder_2.grid.data + 1), 2);
    EXPECT_EQ(grid_holder_2.grid.origin->pos.x, 1);
    EXPECT_EQ(grid_holder_2.grid.origin->pos.y, 2);
    EXPECT_EQ(grid_holder_2.grid.origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_2).origin->dir.y, 4);
    EXPECT_EQ((*grid_holder_2).size.width, 2U);
    EXPECT_EQ((*grid_holder_2).size.height, 3U);
    EXPECT_EQ((*grid_holder_2).resolution, 10);
    EXPECT_EQ(*((*grid_holder_2).data + 1), 2);
    EXPECT_EQ((*grid_holder_2).origin->pos.x, 1);
    EXPECT_EQ((*grid_holder_2).origin->pos.y, 2);
    EXPECT_EQ((*grid_holder_2).origin->dir.x, 3);
    EXPECT_EQ((*grid_holder_2).origin->dir.y, 4);
    EXPECT_EQ(grid_holder_2->size.width, 2U);
    EXPECT_EQ(grid_holder_2->size.height, 3U);
    EXPECT_EQ(grid_holder_2->resolution, 10);
    EXPECT_EQ(*(grid_holder_2->data + 1), 2);
    EXPECT_EQ(grid_holder_2->origin->pos.x, 1);
    EXPECT_EQ(grid_holder_2->origin->pos.y, 2);
    EXPECT_EQ(grid_holder_2->origin->dir.x, 3);
}
