//
// Created by Left Thomas on 2017/8/25.
//
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;

#ifndef SLAMBOOK_COMMON_H
#define SLAMBOOK_COMMON_H


// 一次测量的值，包括一个世界坐标系下三维点与一个灰度值
struct Measurement {
    Measurement(Eigen::Vector3d p, float g) : pos_world(p), grayscale(g) {}

    Eigen::Vector3d pos_world;
    float grayscale;
};

inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale) {
    float zz = float(d) / scale;
    float xx = zz * (x - cx) / fx;
    float yy = zz * (y - cy) / fy;
    return Eigen::Vector3d(xx, yy, zz);
}

inline Eigen::Vector2d project3Dto2D(float x, float y, float z, float fx, float fy, float cx, float cy) {
    float u = fx * x / z + cx;
    float v = fy * y / z + cy;
    return Eigen::Vector2d(u, v);
}


// 自定义一元边,和相机姿态顶点连接
class EdgeSE3ProjectDirect : public BaseUnaryEdge<1, double, VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//    世界坐标系下的点
    Eigen::Vector3d x_world_;
//    相机内参
    float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0;
//    参考帧
    cv::Mat *image_ = nullptr;

    EdgeSE3ProjectDirect() = default;

    EdgeSE3ProjectDirect(Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat *image) :
            x_world_(std::move(point)), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image) {}

    void computeError() override {
        const VertexSE3Expmap *v = static_cast<const VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(x_world_);
        float x = x_local[0] * fx_ / x_local[2] + cx_;
        float y = x_local[1] * fy_ / x_local[2] + cy_;
//        检查x,y是否在图像中
        if ((x - 4) < 0 || (x + 4) > image_->cols || (y - 4) < 0 || (y + 4) > image_->rows) {
            _error(0, 0) = 0.0;
            this->setLevel(1);
        } else {
            _error(0, 0) = getPixelValue(x, y) - _measurement;
        }

    }

    void linearizeOplus() override {

        if (level() == 1) {
            _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
            return;
        }

        auto *vtx = static_cast<VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d xyz_trans = vtx->estimate().map(x_world_);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double invz = 1.0 / xyz_trans[2];
        double invz_2 = invz * invz;
        float u = x * fx_ * invz + cx_;
        float v = y * fy_ * invz + cy_;

        Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

        jacobian_uv_ksai(0, 0) = -x * y * invz_2 * fx_;
        jacobian_uv_ksai(0, 1) = (1 + (x * x * invz_2)) * fx_;
        jacobian_uv_ksai(0, 2) = -y * invz * fx_;
        jacobian_uv_ksai(0, 3) = invz * fx_;
        jacobian_uv_ksai(0, 4) = 0;
        jacobian_uv_ksai(0, 5) = -x * invz_2 * fx_;

        jacobian_uv_ksai(1, 0) = -(1 + y * y * invz_2) * fy_;
        jacobian_uv_ksai(1, 1) = x * y * invz_2 * fy_;
        jacobian_uv_ksai(1, 2) = x * invz * fy_;
        jacobian_uv_ksai(1, 3) = 0;
        jacobian_uv_ksai(1, 4) = invz * fy_;
        jacobian_uv_ksai(1, 5) = -y * invz_2 * fy_;

        Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

        jacobian_pixel_uv(0, 0) = (getPixelValue(u + 1, v) - getPixelValue(u - 1, v)) / 2;
        jacobian_pixel_uv(0, 1) = (getPixelValue(u, v + 1) - getPixelValue(u, v - 1)) / 2;

        _jacobianOplusXi = jacobian_pixel_uv * jacobian_uv_ksai;
    }

    bool read(istream &in) override { return false; }

    bool write(ostream &out) const override { return false; }

protected:
//    双线性插值获取像素灰度值
    inline float getPixelValue(float x, float y) {
        uchar *data = &image_->data[int(y) * image_->step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float((1 - xx) * (1 - yy) * data[0] + xx * (1 - yy) * data[1] +
                     (1 - xx) * yy * data[image_->step] + xx * yy * data[(int) (image_->step) + 1]
        );
    }
};

// 直接法估计位姿
// 输入：测量值（空间点的灰度），新的灰度图，相机内参； 输出：相机位姿

void poseEstimationDirect(const vector<Measurement> &measurements, cv::Mat *gray, Eigen::Matrix3f &K,
                          Eigen::Isometry3d &Tcw) {
    // 初始化g2o
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType *linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock *solver_ptr = new DirectBlock(linearSolver);
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    g2o::VertexSE3Expmap *pose = new g2o::VertexSE3Expmap();
    pose->setEstimate(g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    // 添加边
    int id = 1;
    for (Measurement m: measurements) {
        EdgeSE3ProjectDirect *edge = new EdgeSE3ProjectDirect(
                m.pos_world,
                K(0, 0), K(1, 1), K(0, 2), K(1, 2), gray
        );
        edge->setVertex(0, pose);
        edge->setMeasurement(m.grayscale);
        edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());
        edge->setId(id++);
        optimizer.addEdge(edge);
    }
    cout << "edges in graph: " << optimizer.edges().size() << endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw = pose->estimate();
}


#endif //SLAMBOOK_COMMON_H
