cmake_minimum_required(VERSION 2.8)

project(curve_fitting)

# 找Ceres库
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# 找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 添加一个可执行程序
add_executable(useCeres useCeres.cpp)
target_link_libraries(useCeres ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(useG2o useG2o.cpp)
target_link_libraries(useG2o ${CERES_LIBRARIES} ${OpenCV_LIBRARIES})


