add_library(myslam SHARED config.cpp camera.cpp frame.cpp visual_odometry.cpp)
target_link_libraries(myslam ${THIRD_PARTY_LIBS})