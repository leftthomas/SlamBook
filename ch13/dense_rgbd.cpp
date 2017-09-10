#include<iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

/**
 * 本程序演示了RGBD点云地图稠密重建
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    // 彩色图和深度图
    vector<cv::Mat> colorImgs, depthImgs;
    // 相机位姿
    vector<Eigen::Isometry3d> poses;

    ifstream fin("../../ch13/data/pose.txt");
    if (!fin) {
        cerr << "can't find pose file" << endl;
        return 1;
    }

    for (int i = 0; i < 5; i++) {
        //图像文件格式
        boost::format fmt("../../ch13/data/%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        // 使用-1读取原始图像
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));

        double data[7] = {0};
        for (int j = 0; j < 7; j++) {
            fin >> data[j];
        }
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }

    // 计算点云并拼接
    // 相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    cout << "正在将图像转换为点云..." << endl;
    return 0;
}
