//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/frame.h"

namespace myslam {


    Frame::Frame() : id_(static_cast<unsigned long>(-1)), time_stamp_(-1), camera_(nullptr) {

    }

    Frame::Frame(unsigned long id_, double time_stamp_, const SE3 &T_c_w, const Camera::Ptr &camera_, const Mat &color_,
                 const Mat &depth_) : id_(id_), time_stamp_(time_stamp_), T_c_w(T_c_w), camera_(camera_),
                                      color_(color_), depth_(depth_) {}

    Frame::~Frame() = default;

    Frame::Ptr Frame::createFrame() {
        static unsigned long factory_id = 0;
        return Frame::Ptr(new Frame(factory_id++));
    }

    double Frame::findDepth(const cv::KeyPoint &kp) {
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        ushort d = depth_.ptr<ushort>(y)[x];
        if (d != 0) {
            return double(d) / camera_->depth_scale_;
        }
//      check the nearby 4 points
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; ++i) {
            d = depth_.ptr<ushort>(y + dy[i])[x + dx[i]];
            if (d != 0) {
                return double(d) / camera_->depth_scale_;
            }
        }
//        other while
        return -1.0;
    }

    Vector3d Frame::getCameraCenter() const {
        return T_c_w.inverse().translation();
    }

    bool Frame::isInFrame(const Vector3d &pt_world) {
        Vector3d p_cam = camera_->world2camera(pt_world, T_c_w);
        if (p_cam(2, 0) < 0)
            return false;
        Vector2d p_pixel = camera_->camera2pixel(p_cam);
        return p_pixel(0, 0) > 0 && p_pixel(1, 0) > 0 && p_pixel(0, 0) < color_.cols && p_pixel(1, 0) < color_.rows;
    }

}
