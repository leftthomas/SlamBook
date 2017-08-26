//
// Created by Left Thomas on 2017/8/26.
//

#ifndef SLAMBOOK_MAPPOINT_H
#define SLAMBOOK_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {
    class MapPoint {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long id_;
        Vector3d pos_;
//        Normal of viewing direction
        Vector3d norm_;
        Mat descriptor_;
        int observed_times_;
//        being an inliner in pose estimation
        int correct_times_;

        MapPoint();

        MapPoint(unsigned long id_, const Vector3d &pos_, const Vector3d &norm_);

//        factory function
        static MapPoint::Ptr createMapPoint();
    };
}



#endif //SLAMBOOK_MAPPOINT_H
