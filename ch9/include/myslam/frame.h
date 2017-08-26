//
// Created by Left Thomas on 2017/8/26.
//

#ifndef SLAMBOOK_FRAME_H
#define SLAMBOOK_FRAME_H

#include "myslam/common_include.h"
#include "camera.h"

namespace myslam {
class Frame {
public:
    typedef shared_ptr<Frame> Ptr;
//    id of this frame
    unsigned long id_;
//    when it's recorded
    double time_stamp_;
//    transform from world to camera
    SE3 T_c_w;
//    Pinhole RGB-D Camera model
    Camera::Ptr camera_;
//    color and depth image
    Mat color_, depth_;

    Frame();

    Frame(unsigned long id_, double time_stamp_, const SE3 &T_c_w, const Ptr &camera_,
          const Mat &color_, const Mat &depth_);

    virtual ~Frame();

//    factory function
    static Frame::Ptr createFrame();

//    find the depth in depth map
    double findDepth(const cv::KeyPoint &kp);

//    get camera center
    Vector3d getCameraCenter() const;

//    check if a point is in this frame
    bool isInFrame(const Vector3d &pt_world);
};
}



#endif //SLAMBOOK_FRAME_H
