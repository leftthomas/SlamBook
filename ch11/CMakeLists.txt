cmake_minimum_required(VERSION 2.8)

project(pose_graph)

# 添加cmake模块以使用g2o库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# 找g2o库
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})

###以下这两句Mac下必须有，ubuntu不需要#####
set(LINK_DIR /usr/local/lib)
link_directories(${LINK_DIR})
##########不然的话g2o库链接不上############

# eigen
#include_directories(/usr/local/Cellar/eigen/3.3.4/include/eigen3)

# sophus
#find_package(Sophus REQUIRED)
#include_directories(${Sophus_INCLUDE_DIRS})

#find_package( Cholmod REQUIRED )
#include_directories( ${CHOLMOD_INCLUDE_DIR} )

# 添加一个可执行程序
add_executable(pose_graph_g2o_se3 pose_graph_g2o_se3.cpp)
target_link_libraries(pose_graph_g2o_se3 g2o_core g2o_stuff g2o_types_slam3d)

#add_executable(pose_graph_lie_algebra pose_graph_lie_algebra.cpp)
#target_link_libraries(pose_graph_lie_algebra g2o_core g2o_stuff)



