#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
using namespace std;
using namespace cv;

// 找到图像间的特征点匹配
void find_feature_matches(
        const Mat &img_1, const Mat &img_2,
        std::vector<KeyPoint> &key_points_1,
        std::vector<KeyPoint> &key_points_2,
        std::vector<DMatch> &matches);

// 像素坐标转相机归一化坐标
Point2d pixel2cam(const Point2d &p, const Mat &K);

// BA求解
void bundleAdjustment(vector<Point3f> points_3d, vector<Point2f> points_2d, const Mat &K, Mat &R, Mat &t);

/**
 * 本程序演示了PnP求解相机位姿,BA优化位姿与3D空间点坐标
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    if (argc != 5) {
        cout << "usage: pose_estimation_3d2d img1 img2 depth1 depth2" << endl;
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
    Mat_<double> K(3, 3);
    K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
    vector<Point3f> pts_3d;
    vector<Point2f> pts_2d;
    for (auto &match : matches) {
        ushort d = d1.ptr<unsigned short>(int(key_points_1[match.queryIdx].pt.y))
        [int(key_points_1[match.queryIdx].pt.x)];
        if (d == 0)
            continue;
        double dd = d / 1000.0;
        Point2d p1 = pixel2cam(key_points_1[match.queryIdx].pt, K);
        pts_3d.push_back(Point3d(p1.x * dd, p1.y * dd, dd));
        pts_2d.push_back(key_points_2[match.trainIdx].pt);
    }
    cout << "3d-2d pairs: " << pts_3d.size() << endl;

    Mat r, t;
//    调用OpenCV的PnP求解
    solvePnP(pts_3d, pts_2d, K, Mat(), r, t, false, cv::SOLVEPNP_EPNP);
    Mat R;
//    通过罗德里格斯公式将旋转向量转为旋转矩阵
    cv::Rodrigues(r, R);
    cout << "R=\n" << R << endl;
    cout << "t=\n" << t << endl;
//    BA
    bundleAdjustment(pts_3d, pts_2d, K, R, t);
    return 0;
}

void find_feature_matches(const Mat &img_1, const Mat &img_2,
                          std::vector<KeyPoint> &key_points_1,
                          std::vector<KeyPoint> &key_points_2,
                          std::vector<DMatch> &matches) {
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect(img_1, key_points_1);
    detector->detect(img_2, key_points_2);

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute(img_1, key_points_1, descriptors_1);
    descriptor->compute(img_2, key_points_2, descriptors_2);

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match, noArray());

    //-- 第四步:匹配点对筛选
    double min_dist = 10000, max_dist = 0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for (int i = 0; i < descriptors_1.rows; i++) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for (int i = 0; i < descriptors_1.rows; i++) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}

Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
            (
                    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
}

void bundleAdjustment(vector<Point3f> points_3d, vector<Point2f> points_2d,
                      const Mat &K, Mat &R, Mat &t) {
//    初始化g2o,pose维度为6,landmark维度为3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>> Block;
    Block::LinearSolverType *linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>();
    auto *solver_ptr = new Block(linearSolver);
    auto *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

//    vertex
    auto *pose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;
    R_mat << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(R_mat, Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))));
    optimizer.addVertex(pose);
//    landmarks
    int index = 1;
    for (const Point3f &p:points_3d) {
        auto *point = new g2o::VertexSBAPointXYZ();
        point->setId(index++);
        point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        point->setMarginalized(true);
        optimizer.addVertex(point);
    }

//    parameter: camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
            K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);

//    edges
    index = 1;
    for (const Point2f &p:points_2d) {
        auto *edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId(index);
        edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(index)));
        edge->setVertex(1, pose);
        edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
        edge->setParameterId(0, 0);
        edge->setInformation(Eigen::Matrix2d::Identity());
        optimizer.addEdge(edge);
        index++;
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