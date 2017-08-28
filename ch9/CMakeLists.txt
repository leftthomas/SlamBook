cmake_minimum_required(VERSION 2.8)

project(myslam)
#这一句是防止Mac编译的时候报warning
set(CMAKE_MACOSX_RPATH 1)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Eigen
include_directories(/usr/local/Cellar/eigen/3.3.4/include/eigen3)
# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

set(THIRD_PARTY_LIBS ${OpenCV_LIBS} ${Sophus_LIBRARIES})

include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(src)
add_subdirectory(test)



