#include<iostream>
#include <fstream>
#include <sophus/se3.h>
#include <opencv2/core/core.hpp>

using namespace std;
using Sophus::SE3;
using namespace Eigen;
using namespace cv;

//    parameters
//    边缘宽度
const int boarder = 20;
//    宽度
const int width = 640;
//    高度
const int height = 480;
//    相机内参
const double fx = 481.2f;
const double fy = -480.0f;
const double cx = 319.5f;
const double cy = 239.5f;
//    NCC取的窗口半宽度
const int ncc_window_size = 2;
//    NCC窗口面积
const int ncc_area = (2 * ncc_window_size + 1) * (2 * ncc_window_size + 1);
//    收敛判定：最小方差
const double min_cov = 0.1;
//    发散判定：最大方差
const double max_cov = 10;

//从REMODE数据集读取数据
bool readDatasetFiles(const string &path, vector<string> &color_image_files, vector<SE3> &poses) {
    ifstream fin(path + "/first_200_frames_traj_over_table_input_sequence.txt");
    if (!fin) {
        return false;
    }
    while (!fin.eof()) {
//        数据格式：图像文件名 tx,ty,tz,qx,qy,qz,qw，是TWC而不是TCW
        string image;
        fin >> image;
        double data[7];
        for (double &d:data) {
            fin >> d;
        }
        color_image_files.push_back(path + string("/images/") + image);
        poses.emplace_back(Quaterniond(data[6], data[3], data[4], data[5]), Vector3d(data[0], data[1], data[2]));
        if (!fin.good())
            break;
    }
    return true;
}

//根据新的图像更新深度估计
bool update(const Mat &ref, const Mat &curr, const SE3 &T_C_R, Mat &depth, Mat &depth_cov) {
#pragma omp parallel for
    for (int x = boarder; x < width - boarder; ++x) {
#pragma omp parallel for
        for (int y = boarder; y < height - boarder; ++y) {
//            遍历每个像素
//            深度已收敛或发散
            if (depth_cov.ptr<double>(y)[x] < min_cov || depth_cov.ptr<double>(y)[x] > max_cov)
                continue;
//            在极线上搜索(x,y)的匹配
            Vector2d pt_curr;
            bool ret = epipo
        }
    }
}

/**
 * 本程序演示了单目稠密重建
 * 使用极线搜索+NCC匹配
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    return 0;
}
