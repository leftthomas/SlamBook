cmake_minimum_required(VERSION 2.8)

project(dense_reconstruction)

# Eigen
include_directories(/usr/local/Cellar/eigen/3.3.4/include/eigen3)
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})
# pcl
find_package(PCL REQUIRED COMPONENT common io filters)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})
# octomap
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

# 添加一个可执行程序
add_executable(dense_monocular dense_monocular.cpp)
target_link_libraries(dense_monocular ${OpenCV_LIBS} ${Sophus_LIBRARIES})

add_executable(dense_rgbd dense_rgbd.cpp)
target_link_libraries(dense_rgbd ${OpenCV_LIBS} ${PCL_LIBRARIES})

add_executable(octomap_mapping octomap_mapping.cpp)
target_link_libraries(octomap_mapping ${OpenCV_LIBS} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES})
