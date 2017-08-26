//
// Created by Left Thomas on 2017/8/26.
//

#ifndef SLAMBOOK_MAP_H
#define SLAMBOOK_MAP_H

#include "myslam/common_include.h"
#include "myslam/mappoint.h"
#include "myslam/frame.h"

namespace myslam {
    class Map {
    public:
        typedef shared_ptr<Map> Ptr;
//        all landmarks
        unordered_map<unsigned long, MapPoint::Ptr> map_points_;
//        all key frames
        unordered_map<unsigned long, Frame::Ptr> key_frames_;

        Map();

        void insertKeyFrame(Frame::Ptr frame);

        void insertMapPoint(MapPoint::Ptr map_points);
    };
}



#endif //SLAMBOOK_MAP_H
