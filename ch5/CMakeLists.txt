cmake_minimum_required(VERSION 2.8)

project(useOpenCV)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(imageBasics imageBasics.cpp)
target_link_libraries(imageBasics ${OpenCV_LIBRARIES})


# eigen
#include_directories(/usr/include/eigen3)

# in osx and brew install
include_directories(/usr/local/Cellar/eigen/3.3.4/include/eigen3)

# pcl
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})



