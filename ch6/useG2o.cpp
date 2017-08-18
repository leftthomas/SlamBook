#include<iostream>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>

using namespace std;

//曲线模型的顶点，参数：优化变量维度和数据类型
class CurveFittingVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//    重置
    virtual void setToOriginImpl() {
        _estimate << 0, 0, 0;
    }

//    更新
    virtual void oplusImpl(const double *update) {
        _estimate += Eigen::Vector3d(update);
    }

//    存盘和读盘：留空
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

/**
 * 本程序演示了g2o的使用
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {


    return 0;
}
