#include <gtest/gtest.h>

#include "model/params.h"

using namespace truck::model;

TEST(Model, yaml_params) {
    // check config
    Params params("config/model.yaml");
}

int main(int argc, char *argv[]) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
