#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;


/**
 * 找到图像间的特征点匹配
 * @param img_1
 * @param img_2
 * @param key_points_1
 * @param key_points_2
 * @param matches
 */
void find_feature_matches(const Mat &img_1, const Mat &img_2, vector<KeyPoint> &key_points_1,
                          vector<KeyPoint> &key_points_2, vector<DMatch> &matches) {

    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");

//    第一步：检测 oriented FAST角点位置
    detector->detect(img_1, key_points_1);
    detector->detect(img_2, key_points_2);

//    第二步：计算BRIEF描述子
    descriptor->compute(img_1, key_points_1, descriptors_1);
    descriptor->compute(img_2, key_points_2, descriptors_2);

//    第三步：匹配描述子，使用Hamming距离
    vector<DMatch> match;
    matcher->match(descriptors_1, descriptors_2, match, noArray());

//    第四步：匹配点对筛选,找出最相似的和最不相似的两组点之间的距离
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; ++i) {
        double dist = match[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

//    当描述子之间的距离大于两倍的最小距离时,即认为匹配有误,同时设置一个最小距离下限,这里取了经验值30
    for (int i = 0; i < descriptors_1.rows; ++i) {
        if (match[i].distance <= max(2 * min_dist, 30.0)) {
            matches.push_back(match[i]);
        }
    }
}


/**
 * 求解相机运动
 * @param key_points_1
 * @param key_points_2
 * @param matches
 * @param R
 * @param t
 */
void pose_estimation_2d2d(vector<KeyPoint> key_points_1, vector<KeyPoint> key_points_2,
                          vector<DMatch> matches, Mat &R, Mat &t) {

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
    essential_matrix = findEssentialMat(points_1, points_2, focal_length, principal_point, RANSAC);
    cout << "essential matrix is \n" << essential_matrix << endl;

//    计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography(points_1, points_2, RANSAC, 3, noArray(), 2000, 0.99);
    cout << "homography matrix is \n" << homography_matrix << endl;

//    从本质矩阵中恢复旋转和平移
    recoverPose(essential_matrix, points_1, points_2, R, t, focal_length, principal_point);
    cout << "R is \n" << R << endl;
    cout << "t is \n" << t << endl;
};


/**
 * 根据相机内参矩阵将像素坐标转换为相机坐标
 * @param p
 * @param K
 * @return
 */
Point2d pixel2cam(const Point2d &p, const Mat &K) {
    return Point2d
            (
                    (p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
                    (p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
            );
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
void triangulation(const vector<KeyPoint> &key_points_1, const vector<KeyPoint> &key_points_2,
                   const vector<DMatch> &matches, const Mat &R, const Mat &t, vector<Point3d> &points) {
    Mat_<float> T1(3, 4);
    T1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
    Mat_<float> T2(3, 4);
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
        x /= x.at<float>(3, 0);
        Point3d p(x.at<float>(0, 0), x.at<float>(1, 0), x.at<float>(2, 0));
        points.push_back(p);
    }
}


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
        Mat1d y1(3, 1);
        y1 << pt1.x, pt1.y, 1;
        Point2d pt2 = pixel2cam(key_points_2[m.trainIdx].pt, K);
        Mat1d y2(3, 1);
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
        Point2f pt2_cam = pixel2cam(key_points_2[matches[i].trainIdx].pt, K);
        Mat pt2_trans = R * (Mat_<double>(3, 1) << points[i].x, points[i].y, points[i].z) + t;
        pt2_trans /= pt2_trans.at<double>(2, 0);
        cout << "point in the second camera frame: " << pt2_cam << endl;
        cout << "point reprojected from second frame " << pt2_trans.t() << endl;
        cout << endl;
    }
    return 0;
}