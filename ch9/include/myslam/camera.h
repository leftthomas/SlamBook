//
// Created by Left Thomas on 2017/8/26.
//

#ifndef SLAMBOOK_CAMERA_H
#define SLAMBOOK_CAMERA_H

#include "myslam/common_include.h"

namespace myslam {
//    pinhole RGB-D camera model
    class Camera {
    public:
        typedef shared_ptr<Camera> Ptr;
//    Camera intrinsics
        float fx_, fy_, cx_, cy_, depth_scale_;

        Camera();

        Camera::Camera(float fx_, float fy_, float cx_, float cy_, float depth_scale_) :
                fx_(fx_), fy_(fy_), cx_(cx_), cy_(cy_), depth_scale_(depth_scale_) {}

//    coordinate transform:world,camera,pixel
        Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w);

        Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w);

        Vector2d camera2pixel(const Vector3d &p_c);

        Vector3d pixel2camera(const Vector2d &p_p, double depth = 1);

        Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w);

        Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth = 1);

    };

}

#endif //SLAMBOOK_CAMERA_H
