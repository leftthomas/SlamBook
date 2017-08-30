#include<iostream>
#include <Eigen/Core>
#include <fstream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <sophus/so3.h>
#include <sophus/se3.h>
using namespace std;
using Sophus::SE3;
using Sophus::SO3;

typedef Eigen::Matrix<double, 6, 6> Matrix6d;

// 给定误差求J_R^{-1}的近似
Matrix6d JRInv(SE3 e) {
    Matrix6d J;
    J.block(0, 0, 3, 3) = SO3::hat(e.so3().log());
    J.block(0, 3, 3, 3) = SO3::hat(e.translation());
    J.block(3, 0, 3, 3) = Eigen::Matrix3d::Zero(3, 3);
    J.block(3, 3, 3, 3) = SO3::hat(e.so3().log());
    J = 0.5 * J + Matrix6d::Identity();
    return J;
}

//李代数顶点
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class VertexSE3LieAlgebra : public g2o::BaseVertex<6, SE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool read(std::istream &is) override {
        double data[7];
        for (int i = 0; i < 7; ++i) {
            is >> data[i];
        }
        setEstimate(SE3(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                        Eigen::Vector3d(data[0], data[1], data[2])));
        return true;
    }

    bool write(std::ostream &os) const override {
        os << id() << " ";
        Eigen::Quaterniond q = _estimate.unit_quaternion();
        os << _estimate.translation().transpose() << " ";
        os << q.coeffs()[0] << " " << q.coeffs()[1] << " " << q.coeffs()[2] << " " << q.coeffs()[3] << endl;
        return true;
    }

protected:
//    左乘更新
    void oplusImpl(const double *v) override {
        Sophus::SE3 up(Sophus::SO3(v[3], v[4], v[5]), Eigen::Vector3d(v[0], v[1], v[2]));
        _estimate = up * _estimate;
    }

    void setToOriginImpl() override {
        _estimate = Sophus::SE3();
    }
};

/**
 * 本程序演示了g2o pose graph lie algebra优化
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    return 0;
}
