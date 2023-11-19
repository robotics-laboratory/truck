find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_test main.cpp)
target_link_libraries(${PROJECT_NAME}_test simulator_engine)
