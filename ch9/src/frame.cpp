//
// Created by Left Thomas on 2017/8/26.
//
#include <sophus/se3.h>
#include <opencv2/core/mat.hpp>
#include <myslam/frame.h>

namespace myslam {

    Frame::Frame() {}

    Frame::~Frame() {

    }

    Frame::Frame(unsigned long id_, double time_stamp_, const Sophus::SE3 &T_c_w,
                 const std::__1::shared_ptr<myslam::Camera> &camera_, const cv::Mat &color_,
                 const cv::Mat &depth_) : id_(id_), time_stamp_(time_stamp_), T_c_w(T_c_w), camera_(camera_),
                                          color_(color_), depth_(depth_) {}
}
