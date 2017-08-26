//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/mappoint.h"

namespace myslam {

    MapPoint::MapPoint() : id_(static_cast<unsigned long>(-1)), pos_(Vector3d(0, 0, 0)),
                           norm_(Vector3d(0, 0, 0)), observed_times_(0), correct_times_(0) {}

    MapPoint::MapPoint(unsigned long id_, const Eigen::Matrix<double, 3, 1> &pos_, const Eigen::Matrix
            <double, 3, 1> &norm_) : id_(id_), pos_(pos_), norm_(norm_), observed_times_(0), correct_times_(0) {}

    MapPoint::Ptr MapPoint::createMapPoint() {
        static unsigned long factory_id = 0;
        return MapPoint::Ptr(new MapPoint(factory_id++, Vector3d(0, 0, 0), Vector3d(0, 0, 0)));
    }
}