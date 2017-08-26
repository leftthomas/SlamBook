add_library(myslam SHARED ../include/myslam/config.h
        ../include/myslam/frame.h ../include/myslam/map.h ../include/myslam/mappoint.h
        frame.cpp mappoint.cpp map.cpp camera.cpp config.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})