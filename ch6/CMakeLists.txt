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

# 添加cmake模块以使用g2o库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 找g2o库
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

# eigen
#include_directories(/usr/include/eigen3)

# in osx and brew install
include_directories(/usr/local/Cellar/eigen/3.3.4/include/eigen3)

add_executable(useG2o useG2o.cpp)
target_link_libraries(useG2o ${G2O_LIBRARIES} ${OpenCV_LIBRARIES})


