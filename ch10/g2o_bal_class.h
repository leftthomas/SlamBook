//
// Created by Left Thomas on 2017/8/29.
//

#ifndef SLAMBOOK_G2O_BAL_CLASS_H
#define SLAMBOOK_G2O_BAL_CLASS_H

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <Eigen/Core>
#include <ceres/autodiff_cost_function.h>
#include "common/tools/rotation.h"
#include "common/projection.h"

class VertexCameraBAL : public g2o::BaseVertex<9, Eigen::VectorXd> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexCameraBAL() = default;

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void oplusImpl(const double *update) override {
        Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
        _estimate += v;
    }

    void setToOriginImpl() override {

    }

};

class VertexPointBAL : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPointBAL() = default;

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

    void oplusImpl(const double *update) override {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }

    void setToOriginImpl() override {

    }

};

class EdgeObservationBAL : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, VertexCameraBAL, VertexPointBAL> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeObservationBAL() = default;

    void computeError() override {
        const auto *cam = dynamic_cast<const VertexCameraBAL *>(vertex(0));
        const auto *point = dynamic_cast<const VertexPointBAL *>(vertex(1));
        (*this)(cam->estimate().data(), point->estimate().data(), _error.data());
    }

    bool read(std::istream &is) override {
        return false;
    }

    bool write(std::ostream &os) const override {
        return false;
    }

//    为了使用Ceres求导功能而定义的函数，让本类成为拟函数类
    template<typename T>
    bool operator()(const T *camera, const T *point, const T *residuals) const {
        T predictions[2];
        CamProjectionWithDistortion(camera, point, predictions);
        residuals[0] = predictions[0] - T(measurement()(0));
        residuals[1] = predictions[1] - T(measurement()(1));
        return true;
    }

    void linearizeOplus() override {
        const auto *cam = dynamic_cast<const VertexCameraBAL *>(vertex(0));
        const auto *point = dynamic_cast<const VertexPointBAL *>(vertex(1));
        typedef ceres::internal::AutoDiff<EdgeObservationBAL, double,
                VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;
        Eigen::Matrix<double, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Eigen::Matrix<double, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
        double *parameters[] = {const_cast<double *>(cam->estimate().data()),
                                const_cast<double *>(point->estimate().data())};
        double *jacobians[] = {dError_dCamera.data(), dError_dPoint.data()};
        double value[Dimension];

    }
};


#endif //SLAMBOOK_G2O_BAL_CLASS_H
