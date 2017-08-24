cmake_minimum_required(VERSION 2.8)

project(vo1)

# 找OpenCV库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(feature_extraction feature_extraction.cpp)
target_link_libraries(feature_extraction ${OpenCV_LIBRARIES})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp common.h)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBRARIES})

# 添加cmake模块以使用g2o、CSparse库
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

# 找CSparse库
find_package(CSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBRARIES} ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)

add_executable(pose_estimation_3d3d pose_estimation_3d3d.cpp)
target_link_libraries(pose_estimation_3d3d ${OpenCV_LIBRARIES} ${CSPARSE_LIBRARY}
        g2o_core g2o_stuff g2o_types_sba g2o_csparse_extension)
