cmake_minimum_required(VERSION 2.8)

project(backend)
#这一句是防止Mac编译的时候报warning
set(CMAKE_MACOSX_RPATH 1)

find_package(Ceres REQUIRED)

include_directories(${CERES_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/common ${PROJECT_SOURCE_DIR}/common/tools
        ${PROJECT_SOURCE_DIR}/common/flags)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

# 添加一个可执行程序
add_executable(g2o_bundle g2o_bundle.cpp)
target_link_libraries(g2o_bundle BALProblem ParseCmd)