//
// Created by Left Thomas on 2017/8/26.
//

#ifndef SLAMBOOK_MAPPOINT_H
#define SLAMBOOK_MAPPOINT_H

#include "myslam/common_include.h"
#include "myslam/frame.h"

namespace myslam {
    class MapPoint {
    public:
        typedef shared_ptr<MapPoint> Ptr;
        unsigned long id_;
        static unsigned long factory_id_;
//        whether a good point
        bool good_;
        Vector3d pos_;
//        Normal of viewing direction
        Vector3d norm_;
        Mat descriptor_;
        int observed_times_;
//        being an inliner in pose estimation
        int correct_times_;
//        key frames that can observe this point
        list<Frame *> observed_frames_;
//        being an inliner in pose estimation
        int matched_times_;
//        being visible in current frame
        int visible_times_;

        MapPoint();

        MapPoint(unsigned long id, const Vector3d &position, const Vector3d &norm,
                 Frame *frame = nullptr, const Mat &descriptor = Mat());

        inline cv::Point3f getPositionCV() const {
            return cv::Point3f(pos_(0, 0), pos_(1, 0), pos_(2, 0));
        }
//        factory function
        static MapPoint::Ptr createMapPoint();

        static MapPoint::Ptr createMapPoint(const Vector3d &pos_world, const Vector3d &norm,
                                            const Mat &descriptor, Frame *frame);
    };
}



#endif //SLAMBOOK_MAPPOINT_H
