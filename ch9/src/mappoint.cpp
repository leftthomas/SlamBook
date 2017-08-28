//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/mappoint.h"

namespace myslam {

    MapPoint::MapPoint() : id_(static_cast<unsigned long>(-1)), pos_(Vector3d(0, 0, 0)),
                           norm_(Vector3d(0, 0, 0)), observed_times_(0), correct_times_(0) {}

    MapPoint::MapPoint(unsigned long id, const Vector3d &position, const Vector3d &norm, Frame *frame,
                       const Mat &descriptor) {

    }

    MapPoint::Ptr MapPoint::createMapPoint() {
        return myslam::MapPoint::Ptr();
    }

    MapPoint::Ptr
    MapPoint::createMapPoint(const Vector3d &pos_world, const Vector3d &norm, const Mat &descriptor, Frame *frame) {
        return myslam::MapPoint::Ptr();
    }

}