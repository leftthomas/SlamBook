//
// Created by Left Thomas on 2017/8/27.
//

#ifndef SLAMBOOK_G2O_TYPES_H
#define SLAMBOOK_G2O_TYPES_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

namespace myslam {
    class EdgeProjectXYZ2UVPoseOnly : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        void computeError() override;

        bool read(std::istream &is) override;

        bool write(std::ostream &os) const override;

        void linearizeOplus() override;

        Vector3d point_;
        Camera *camera_;
    };
}


#endif //SLAMBOOK_G2O_TYPES_H
