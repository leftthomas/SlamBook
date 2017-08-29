cmake_minimum_required(VERSION 2.8)

project(backend)

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/common ${PROJECT_SOURCE_DIR}/common/tools
        ${PROJECT_SOURCE_DIR}/common/flags)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

# 添加一个可执行程序
add_executable(g2o_customBundle g2o_customBundle.cpp)
target_link_libraries(g2o_customBundle BALProblem ParseCmd ${CERES_LIBRARIES})