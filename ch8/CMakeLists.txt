cmake_minimum_required(VERSION 2.8)

project(vo2)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(optical_flow optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBRARIES})

# 添加cmake模块以使用g2o库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 找g2o库
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
###以下这两句Mac下必须有，ubuntu不需要#####
set(LINK_DIR /usr/local/lib)
link_directories(${LINK_DIR})
#########不然的话g2o库链接不上############

# eigen
include_directories(/usr/local/Cellar/eigen/3.3.4/include/eigen3)

add_executable(direct_sparse direct_sparse.cpp)
target_link_libraries(direct_sparse ${OpenCV_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)

add_executable(direct_semidense direct_semidense.cpp)
target_link_libraries(direct_semidense ${OpenCV_LIBRARIES} g2o_core g2o_stuff g2o_types_sba)





