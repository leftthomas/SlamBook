//
// Created by Left Thomas on 2017/8/27.
//

#include "myslam/g2o_types.h"

namespace myslam {

    void EdgeProjectXYZ2UVPoseOnly::computeError() {
        const g2o::VertexSE3Expmap *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
        _error = _measurement - camera_->camera2pixel(pose->estimate().map(point_));
    }

    void EdgeProjectXYZ2UVPoseOnly::linearizeOplus() {
        g2o::VertexSE3Expmap *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::SE3Quat T(pose->estimate());
        Vector3d xyz_trans = T.map(point_);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];
        double z_2 = z * z;

        _jacobianOplusXi(0, 0) = x * y / z_2 * camera_->fx_;
        _jacobianOplusXi(0, 1) = -(1 + (x * x / z_2)) * camera_->fx_;
        _jacobianOplusXi(0, 2) = y / z * camera_->fx_;
        _jacobianOplusXi(0, 3) = -1. / z * camera_->fx_;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = x / z_2 * camera_->fx_;

        _jacobianOplusXi(1, 0) = (1 + y * y / z_2) * camera_->fy_;
        _jacobianOplusXi(1, 1) = -x * y / z_2 * camera_->fy_;
        _jacobianOplusXi(1, 2) = -x / z * camera_->fy_;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1. / z * camera_->fy_;
        _jacobianOplusXi(1, 5) = y / z_2 * camera_->fy_;

    }

    bool EdgeProjectXYZ2UVPoseOnly::read(std::istream &is) {
        return false;
    }

    bool EdgeProjectXYZ2UVPoseOnly::write(std::ostream &os) const {
        return false;
    }
}
