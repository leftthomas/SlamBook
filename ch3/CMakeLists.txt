# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(useEigen)

# 添加 Eigen 头文件
#include_directories(/usr/include/eigen3)

# in osx and brew install
include_directories(/usr/local/Cellar/eigen/3.3.3/include/eigen3)

# 添加一个可执行程序
add_executable(eigenMatrix eigenMatrix.cpp)


