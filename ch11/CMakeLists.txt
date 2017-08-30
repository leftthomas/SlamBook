cmake_minimum_required(VERSION 2.8)

project(pose_graph)

# 添加cmake模块以使用g2o库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 找g2o库
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# 添加一个可执行程序
add_executable(pose_graph_g2o_se3 pose_graph_g2o_se3.cpp)
target_link_libraries(pose_graph_g2o_se3 g2o_core g2o_stuff g2o_types_sba)

add_executable(pose_graph_lie_algebra pose_graph_lie_algebra.cpp)
target_link_libraries(pose_graph_lie_algebra g2o_core g2o_stuff g2o_types_sba)



