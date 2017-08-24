//
// Created by Left Thomas on 2017/8/24.
//

#ifndef SLAMBOOK_COMMON_H
#define SLAMBOOK_COMMON_H

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


#endif //SLAMBOOK_COMMON_H
