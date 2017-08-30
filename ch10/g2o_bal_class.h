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


#endif //SLAMBOOK_G2O_BAL_CLASS_H
