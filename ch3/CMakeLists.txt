# 声明要求的 cmake 最低版本
cmake_minimum_required(VERSION 2.8)

# 声明一个 cmake 工程
project(useEigen)

# 注意，这里必须设置成Release，否则的话，在测试QR分解和求逆解方程会看到不符合书本描述的结果
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")

# 添加 Eigen 头文件
#include_directories(/usr/include/eigen3)

# in osx and brew install
include_directories(/usr/local/Cellar/eigen/3.3.3/include/eigen3)

# 添加一个可执行程序
add_executable(eigenMatrix eigenMatrix.cpp)


