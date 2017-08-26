//
// Created by Left Thomas on 2017/8/26.
//

#include <Eigen/src/Core/Matrix.h>
#include <opencv2/core/mat.hpp>
#include <myslam/mappoint.h>

namespace myslam {

    MapPoint::MapPoint() {}

    MapPoint::MapPoint(unsigned long id_, const Eigen::Matrix<double, 3, 1> &pos_,
                       const Eigen::Matrix<double, 3, 1> &norm_) : id_(id_), pos_(pos_), norm_(norm_) {}

    MapPoint::Ptr MapPoint::createMapPoint() {
        return myslam::MapPoint::Ptr();
    }
}