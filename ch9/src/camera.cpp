//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/camera.h"

namespace myslam {
    Camera::Camera() = default;

    Vector3d Camera::world2camera(const Vector3d &p_w, const SE3 &T_c_w) {
        return T_c_w * p_w;
    }

    Vector3d Camera::camera2world(const Vector3d &p_c, const SE3 &T_c_w) {
        return T_c_w.inverse() * p_c;
    }

    Vector2d Camera::camera2pixel(const Vector3d &p_c) {
        return (Vector2d(fx_ * p_c(0, 0) / p_c(2, 0) + cx_, fy_ * p_c(1, 0) / p_c(2, 0) + cy_));
    }

    Vector3d Camera::pixel2camera(const Vector2d &p_p, double depth) {
        return (Vector3d((p_p(0, 0) - cx_) * depth / fx_, (p_p(1, 0) - cy_) * depth / fy_, depth));
    }

    Vector2d Camera::world2pixel(const Vector3d &p_w, const SE3 &T_c_w) {
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    Vector3d Camera::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth) {
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }
}

