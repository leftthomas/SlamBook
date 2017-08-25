cmake_minimum_required(VERSION 2.8)

project(vo2)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(lkFlow lkFlow.cpp)
target_link_libraries(lkFlow ${OpenCV_LIBRARIES})



