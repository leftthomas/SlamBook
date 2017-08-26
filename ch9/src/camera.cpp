//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/camera.h"

namespace myslam {
    Camera::Camera() {}

    Vector3d Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w) {
        return Eigen::Vector3d();
    }

    Vector3d Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w) {
        return Eigen::Vector3d();
    }

    Vector2d Camera::camera2pixel(const Vector3d &p_c) {
        return Eigen::Vector2d();
    }

    Vector3d Camera::pixel2camera(const Vector2d &p_p, double depth) {
        return Eigen::Vector3d();
    }

    Vector2d Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w) {
        return Eigen::Vector2d();
    }

    Vector3d Camera::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth) {
        return Eigen::Vector3d();
    }
}

