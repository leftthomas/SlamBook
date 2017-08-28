//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/mappoint.h"

namespace myslam {

    MapPoint::MapPoint() : id_(static_cast<unsigned long>(-1)), pos_(Vector3d(0, 0, 0)), good_(true),
                           norm_(Vector3d(0, 0, 0)), matched_times_(0), visible_times_(0) {}

    MapPoint::MapPoint(unsigned long id, const Vector3d &position, const Vector3d &norm, Frame *frame,
                       const Mat &descriptor) : id_(id), pos_(position), norm_(norm), good_(true),
                                                matched_times_(1), visible_times_(1), descriptor_(descriptor) {
        observed_frames_.push_back(frame);
    }

    MapPoint::Ptr MapPoint::createMapPoint() {
        return MapPoint::Ptr(new MapPoint(factory_id_++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }

    MapPoint::Ptr
    MapPoint::createMapPoint(const Vector3d &pos_world, const Vector3d &norm, const Mat &descriptor, Frame *frame) {
        return MapPoint::Ptr(new MapPoint(factory_id_++, pos_world, norm, frame, descriptor));
    }

    unsigned long MapPoint::factory_id_ = 0;
}