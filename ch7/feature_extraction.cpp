#include<iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

/**
 * 本程序演示了ORB特征提取与匹配
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

//    判断命令行参数是否有给出两张图片路径
    if (argc != 3) {
        cout << "usage: feature_extraction img1 img2" << endl;
        return 1;
    }

//    读取图像
    Mat img_1 = imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat img_2 = imread(argv[2], CV_LOAD_IMAGE_COLOR);

//    初始化关键点集、描述子、ORB特征
    vector<KeyPoint> key_points_1, key_points_2;
    Mat descriptors_1, descriptors_2;
//    采用默认参数
    Ptr<ORB> orb = ORB::create();

//    第一步：检测 oriented FAST角点位置
    orb->detect(img_1, key_points_1);
    orb->detect(img_2, key_points_2);

//    第二步：计算BRIEF描述子
    orb->compute(img_1, key_points_1, descriptors_1);
    orb->compute(img_2, key_points_2, descriptors_2);

//    绘制ORB特征点
    Mat out_img_1;
    drawKeypoints(img_1, key_points_1, out_img_1);
    imshow("ORB特征点", out_img_1);
    waitKey(0);

//    第三步：匹配描述子，使用Hamming距离
    vector<DMatch> matches;
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, matches, noArray());

//    第四步：匹配点对筛选,找出最相似的和最不相似的两组点之间的距离
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < descriptors_1.rows; ++i) {
        double dist = matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("-- max dist : %f \n", max_dist);
    printf("-- min dist : %f \n", min_dist);


    return 0;
}
