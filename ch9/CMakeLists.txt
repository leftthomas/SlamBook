cmake_minimum_required(VERSION 2.8)

project(myslam)

# 使用find_package找到Sophus
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# 添加一个可执行程序
add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus ${Sophus_LIBRARIES})



