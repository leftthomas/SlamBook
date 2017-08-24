#include<iostream>
#include "common.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

// ICP姿态评估
void pose_estimation_3d3d(const vector<Point3f> &pts_1, const vector<Point3f> &pts_2, Mat &R, Mat &t);

// BA求解
void bundleAdjustment(const vector<Point3f> &pts_1, const vector<Point3f> &pts_2, Mat &R, Mat &t);

// 自定义3D-3D的边
class EdgeProjectXYZRGBDPoseOnly : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
protected:
    Eigen::Vector3d _point;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    explicit EdgeProjectXYZRGBDPoseOnly(const Eigen::Vector3d &point) : _point(point) {}

    void computeError() override {
        const auto *pose = static_cast<const g2o::VertexSE3Expmap *>(_vertices[0]);
//        measurement is p, point is p'
        _error = _measurement - pose->estimate().map(_point);
    }

    void linearizeOplus() override {
        auto *pose = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        const g2o::SE3Quat &T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(_point);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }

    bool read(istream &in) override { return false; }

    bool write(ostream &out) const override { return false; }
};

/**
 * 本程序演示了ICP求解相机位姿以及BA优化
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    if (argc != 5) {
        cout << "usage: pose_estimation_3d3d img1 img2 depth1 depth2" << endl;
        return 1;
    }
    //-- 读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> key_points_1, key_points_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, key_points_1, key_points_2, matches);
    cout << "一共找到" << matches.size() << "组匹配点" << endl;

//    建立3D点,深度图为16位无符号,单通道
    Mat d1 = imread(argv[3], CV_LOAD_IMAGE_UNCHANGED);
    Mat d2 = imread(argv[4], CV_LOAD_IMAGE_UNCHANGED);
    Mat_<double> K(3, 3);
    K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
    vector<Point3f> pts_1, pts_2;
    for (auto &match : matches) {
        ushort d_1 = d1.ptr<unsigned short>(int(key_points_1[match.queryIdx].pt.y))
        [int(key_points_1[match.queryIdx].pt.x)];
        ushort d_2 = d2.ptr<unsigned short>(int(key_points_2[match.trainIdx].pt.y))
        [int(key_points_2[match.trainIdx].pt.x)];
        if (d_1 == 0 || d_2 == 0)
            continue;
        double dd_1 = d_1 / 1000.0, dd_2 = d_2 / 1000.0;
        Point2d p1 = pixel2cam(key_points_1[match.queryIdx].pt, K);
        Point2d p2 = pixel2cam(key_points_2[match.trainIdx].pt, K);
        pts_1.push_back(Point3d(p1.x * dd_1, p1.y * dd_1, dd_1));
        pts_2.push_back(Point3d(p2.x * dd_2, p2.y * dd_2, dd_2));
    }
    cout << "3d-3d pairs: " << pts_1.size() << endl;

    Mat R, t;
//    SVD求解
    pose_estimation_3d3d(pts_1, pts_2, R, t);
    cout << "ICP via SVD results: " << endl;
//    第二帧到第一帧
    cout << "R=\n" << R << endl;
    cout << "t=\n" << t << endl;
//    第一帧到第二帧
    cout << "R_inv = \n" << R.t() << endl;
    cout << "t_inv = \n" << -R.t() * t << endl;

//    bundle adjustment
    cout << "calling bundle adjustment" << endl;
    bundleAdjustment(pts_1, pts_2, R, t);
//    verify p1 = R*p2 + t
    for (int i = 0; i < 5; i++) {
        cout << "p1 = " << pts_1[i] << endl;
        cout << "p2 = " << pts_2[i] << endl;
        cout << "(R*p2+t) = \n" << R * (Mat_<double>(3, 1) << pts_2[i].x, pts_2[i].y, pts_2[i].z) + t << endl;
        cout << endl;
    }
    return 0;
}


/**
 * ICP姿态评估
 * @param pts_1 
 * @param pts_2 
 * @param R 
 * @param t 
 */
void pose_estimation_3d3d(const vector<Point3f> &pts_1, const vector<Point3f> &pts_2, Mat &R, Mat &t) {
//    质心
    Point3f p_1, p_2;
    int N = pts_1.size();
    for (int i = 0; i < N; ++i) {
        p_1 += pts_1[i];
        p_2 += pts_2[i];
    }
    p_1 /= N;
    p_2 /= N;
//    去质心坐标
    vector<Point3f> q_1(N), q_2(N);
    for (int j = 0; j < N; ++j) {
        q_1[j] = pts_1[j] - p_1;
        q_2[j] = pts_2[j] - p_2;
    }
//    计算W=q_1*q_2^T
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (int i = 0; i < N; ++i) {
        W += Eigen::Vector3d(q_1[i].x, q_1[i].y, q_1[i].z) *
             Eigen::Vector3d(q_2[i].x, q_2[i].y, q_2[i].z).transpose();
    }
    cout << "W=\n" << W << endl;
//    对W进行SVD分解
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    cout << "U=\n" << U << endl;
    cout << "V=\n" << V << endl;
//    求R和t
    Eigen::Matrix3d R_ = U * (V.transpose());
    Eigen::Vector3d t_ = Eigen::Vector3d(p_1.x, p_1.y, p_1.z) - R_ * Eigen::Vector3d(p_2.x, p_2.y, p_2.z);
//    转成cv::Mat
    R = (Mat_<double>(3, 3) << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0), R_(1, 1), R_(1, 2), R_(2, 0), R_(2, 1), R_(2,
                                                                                                                  2));
    t = (Mat_<double>(3, 1) << t_(0, 0), t_(1, 0), t_(2, 0));
}


/**
 * BA求解
 * @param pts_1
 * @param pts_2
 * @param R
 * @param t
 */
void bundleAdjustment(const vector<Point3f> &pts_1, const vector<Point3f> &pts_2, Mat &R, Mat &t) {
//    初始化g2o,pose维度为6,landmark维度为3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverEigen<Block::PoseMatrixType>();
    auto *solver_ptr = new Block(linearSolver);
    auto *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

//    vertex
    auto *pose = new g2o::VertexSE3Expmap();
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0)));
    optimizer.addVertex(pose);

//    edges
    int index = 1;
    vector<EdgeProjectXYZRGBDPoseOnly *> edges;
    for (size_t i = 0; i < pts_1.size(); i++) {
        auto *edge = new EdgeProjectXYZRGBDPoseOnly(Eigen::Vector3d(pts_2[i].x, pts_2[i].y, pts_2[i].z));
        edge->setId(index);
        edge->setVertex(0, pose);
        edge->setMeasurement(Eigen::Vector3d(pts_1[i].x, pts_1[i].y, pts_1[i].z));
        edge->setInformation(Eigen::Matrix3d::Identity());
        optimizer.addEdge(edge);
        index++;
        edges.push_back(edge);
    }

    auto t1 = chrono::steady_clock::now();
    optimizer.setVerbose(true);
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    auto t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double >>(t2 - t1);
    cout << "optimization costs time: " << time_used.count() << " seconds." << endl;
    cout << "\nafter optimization:\n" << "T=\n" << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}