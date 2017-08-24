#include "common.h"
#include <iostream>

// 求解相机运动
void pose_estimation_2d2d(vector<KeyPoint> key_points_1, vector<KeyPoint> key_points_2,
                          vector<DMatch> matches, Mat &R, Mat &t);

// 三角测量
void triangulation(const vector<KeyPoint> &key_points_1, const vector<KeyPoint> &key_points_2,
                   const vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points);


/**
 * 本程序演示了对极约束求解相机运动以及三角测量
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

//    判断命令行参数是否有给出两张图片路径
    if (argc != 3) {
        cout << "usage: pose_estimation_2d2d img1 img2" << endl;
        return 1;
    }

//    读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

    vector<KeyPoint> key_points_1, key_points_2;
    vector<DMatch> matches;
    find_feature_matches(img_1, img_2, key_points_1, key_points_2, matches);
    cout << "一共找到" << matches.size() << "组匹配点" << endl;

//    估计图像间的运动
    Mat R, t;
    pose_estimation_2d2d(key_points_1, key_points_2, matches, R, t);

//    验证E=t^R*scale
    Mat_<double> t_x(3, 3);
    t_x << 0, -t.at<double>(2, 0), t.at<double>(1, 0), t.at<double>(2, 0), 0, -t.at<double>(0, 0),
            -t.at<double>(1, 0), t.at<double>(0, 0), 0;
    cout << "t^R=\n" << t_x * R << endl;

//    三角化
    vector<Point3d> points;
    triangulation(key_points_1, key_points_2, matches, R, t, points);

    Mat_<double> K(3, 3);
    K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
//    验证对极约束
    for (DMatch m : matches) {
        Point2d pt1 = pixel2cam(key_points_1[m.queryIdx].pt, K);
        Mat_<double> y1(3, 1);
        y1 << pt1.x, pt1.y, 1;
        Point2d pt2 = pixel2cam(key_points_2[m.trainIdx].pt, K);
        Mat_<double> y2(3, 1);
        y2 << pt2.x, pt2.y, 1;
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }
//    验证三角化点与特征点的重投影关系
    for (int i = 0; i < matches.size(); ++i) {
//        第一幅图
        Point2d pt1_cam = pixel2cam(key_points_1[matches[i].queryIdx].pt, K);
        Point2d pt1_cam_3d(points[i].x / points[i].z, points[i].y / points[i].z);
        cout << "point in the first camera frame: " << pt1_cam << endl;
        cout << "point projected from 3D " << pt1_cam_3d << ", d=" << points[i].z << endl;
//        第二幅图
        Point2d pt2_cam = pixel2cam(key_points_2[matches[i].trainIdx].pt, K);
        Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        pt2_trans /= pt2_trans.at<double>(2, 0);
        cout << "point in the second camera frame: " << pt2_cam << endl;
        cout << "point reprojected from second frame " << pt2_trans.t() << endl;
        cout << endl;
    }
    return 0;
}


/**
 * 求解相机运动
 * @param key_points_1
 * @param key_points_2
 * @param matches
 * @param R
 * @param t
 */
void pose_estimation_2d2d(vector<KeyPoint> key_points_1, vector<KeyPoint> key_points_2, vector<DMatch> matches, Mat &R,
                          Mat &t) {

//    将匹配点转换成Point2f格式
    vector<Point2f> points_1;
    vector<Point2f> points_2;
    for (auto &match : matches) {
        points_1.push_back(key_points_1[match.queryIdx].pt);
        points_2.push_back(key_points_2[match.trainIdx].pt);
    }

//    计算基础矩阵,采用八点法
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat(points_1, points_2, CV_FM_8POINT);
    cout << "fundamental matrix is \n" << fundamental_matrix << endl;

//    计算本质矩阵
//    光心,TUM dataset标定值
    Point2d principal_point(325.1, 249.7);
//    焦距,TUM dataset标定值
    int focal_length = 521;
    Mat essential_matrix;
    essential_matrix = findEssentialMat(points_1, points_2, focal_length, principal_point);
    cout << "essential matrix is \n" << essential_matrix << endl;

//    计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography(points_1, points_2, RANSAC);
    cout << "homography matrix is \n" << homography_matrix << endl;

//    从本质矩阵中恢复旋转和平移
    recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point);
    cout << "R is \n" << R << endl;
    cout << "t is \n" << t << endl;
}


/**
 * 三角测量
 * @param key_points_1
 * @param key_points_2
 * @param matches
 * @param R
 * @param t
 * @param points
 */
void
triangulation(const vector<KeyPoint> &key_points_1, const vector<KeyPoint> &key_points_2, const vector<DMatch> &matches,
              const Mat &R, const Mat &t, vector<Point3d> &points) {
    Mat_<double> T1(3, 4);
    T1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
    Mat_<double> T2(3, 4);
    T2 << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2), t.at<double>(0, 0),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2), t.at<double>(1, 0),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2), t.at<double>(2, 0);

//        将像素坐标转换为相机坐标
    Mat_<double> K(3, 3);
    K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;
    vector<Point2d> pts_1, pts_2;
    for (DMatch m : matches) {
        pts_1.push_back(pixel2cam(key_points_1[m.queryIdx].pt, K));
        pts_2.push_back(pixel2cam(key_points_2[m.trainIdx].pt, K));
    }

//    三角测量
    Mat pts_4d;
    triangulatePoints(T1, T2, pts_1, pts_2, pts_4d);

//    转换成非齐次坐标
    for (int i = 0; i < pts_4d.cols; ++i) {
        Mat x = pts_4d.col(i);
//        归一化
        x /= x.at<double>(3, 0);
        Point3d p(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));
        points.push_back(p);
    }
};