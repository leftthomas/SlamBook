#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>

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

// ICP姿态评估
void pose_estimation_3d3d(const vector<Point3f> &pts_1, const vector<Point3f> &pts_2, Mat &R, Mat &t);

/**
 * 本程序演示了ICP求解相机位姿
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

void pose_estimation_3d3d(const vector<Point3f> &pts_1, const vector<Point3f> &pts_2, Mat &R, Mat &t) {
//    质心
    Point3f p_1, p_2;
    auto N = pts_1.size();
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
    Eigen::Matrix3d R_ = U * V.transpose();
    Eigen::Vector3d t_ = Eigen::Vector3d(p_1.x, p_1.y, p_1.z) - R_ * Eigen::Vector3d(p_2.x, p_2.y, p_2.z);
//    转成cv::Mat
    R = Mat_<double>(3, 3);
    R << R_(0, 0), R_(0, 1), R_(0, 2), R_(1, 0), R_(1, 1), R_(1, 2), R_(2, 0), R_(2, 1), R_(2, 2);
    t = Mat_<double>(3, 1);
    t << t_(0, 0), t_(1, 0), t_(2, 0);
}
