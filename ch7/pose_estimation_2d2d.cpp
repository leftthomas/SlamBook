#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

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
void find_feature_matches(const Mat &img_1, Mat &img_2, vector<KeyPoint> &key_points_1,
                          vector<KeyPoint> &key_points_2, vector<DMatch> &matches) {

    Mat descriptors_1, descriptors_2;
    Ptr<ORB> orb = ORB::create();

//    第一步：检测 oriented FAST角点位置
    orb->detect(img_1, key_points_1);
    orb->detect(img_2, key_points_2);

//    第二步：计算BRIEF描述子
    orb->compute(img_1, key_points_1, descriptors_1);
    orb->compute(img_2, key_points_2, descriptors_2);

//    第三步：匹配描述子，使用Hamming距离
    vector<DMatch> match;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, match, noArray());

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
            matches.push_back(matches[i]);
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
//    相机内参，TUM Freiburg2
    Mat1d K(3, 3);
    K << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1;

//    将匹配点转换成Point2f格式
    vector<Point2f> points_1;
    vector<Point2f> points_2;
    for (int i = 0; i < matches.size(); ++i) {
        points_1.push_back(key_points_1[matches[i].queryIdx].pt);
        points_2.push_back(key_points_2[matches[i].trainIdx].pt);
    }
};


/**
 * 本程序演示了对极约束求解相机运动
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    return 0;
}
