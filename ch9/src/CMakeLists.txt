add_library(myslam SHARED
        ../include/myslam/
        frame.cpp mappoint.cpp map.cpp camera.cpp config.cpp g2o_types.cpp visual_odometry.cpp)
target_link_libraries(myslam ${THIRD_PARTY_LIBS})