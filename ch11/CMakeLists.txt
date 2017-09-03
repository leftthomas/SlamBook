cmake_minimum_required(VERSION 2.8)

project(pose_graph)

# 添加cmake模块以使用g2o、Cholmod库
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

# Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Cholmod REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

# gtsam
find_package(GTSAMCMakeTools)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})

# 找g2o库
find_package(G2O REQUIRED)
include_directories(${G2O_INCLUDE_DIRS})
###以下这两句Mac下必须有，ubuntu不需要#####
set(LINK_DIR /usr/local/lib)
link_directories(${LINK_DIR})
##########不然的话g2o库链接不上############

add_executable(pose_graph_lie_algebra pose_graph_lie_algebra.cpp)
target_link_libraries(pose_graph_lie_algebra g2o_core g2o_stuff ${Sophus_LIBRARIES} ${CHOLMOD_LIBRARIES})

add_executable(pose_graph_gtsam pose_graph_gtsam.cpp)
target_link_libraries(pose_graph_gtsam ${CHOLMOD_LIBRARIES} gtsam)


