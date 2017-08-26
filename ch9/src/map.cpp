//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/map.h"

namespace myslam {

    Map::Map() = default;

    void Map::insertKeyFrame(Frame::Ptr frame) {
//        cout<<"key frame size = "<<key_frames_.size()<<endl;
        if (key_frames_.find(frame->id_) == key_frames_.end()) {
            key_frames_.insert(make_pair(frame->id_, frame));
        } else {
            key_frames_[frame->id_] = frame;
        }

    }

    void Map::insertMapPoint(MapPoint::Ptr map_point) {
        if (map_points_.find(map_point->id_) == map_points_.end()) {
            map_points_.insert(make_pair(map_point->id_, map_point));
        } else {
            map_points_[map_point->id_] = map_point;
        }
    }
}