cmake_minimum_required(VERSION 2.8)

project(backend)
#这一句是防止Mac编译的时候报warning
set(CMAKE_MACOSX_RPATH 1)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Ceres REQUIRED)
find_Package(G2O REQUIRED)

set(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear
        g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface
        g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense
        g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

include_directories(${CERES_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/common ${PROJECT_SOURCE_DIR}/common/tools
        ${PROJECT_SOURCE_DIR}/common/flags)

add_library(BALProblem SHARED ${PROJECT_SOURCE_DIR}/common/BALProblem.cpp)
add_library(ParseCmd SHARED ${PROJECT_SOURCE_DIR}/common/flags/command_args.cpp)

# 添加一个可执行程序
add_executable(g2o_bundle g2o_bal_class.h g2o_bundle.cpp)
target_link_libraries(g2o_bundle BALProblem ParseCmd)