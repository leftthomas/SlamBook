#include<iostream>
#include <fstream>
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

/**
 * 本程序演示了PCL的使用、PCL点云地图的拼接
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

//    彩色图和深度图
    vector<cv::Mat> colorImgs, depthImgs;
//    相机位姿
    vector<Eigen::Isometry3d> poses;

    ifstream fin("../../ch5/pose.txt");
    if (!fin) {
        cerr << "找不到pose.txt文件" << endl;
        return 1;
    }

    for (int i = 0; i < 5; ++i) {
//        图像文件名格式
        boost::format fmt("../../ch5/%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
//        使用-1读取原始图像
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));
    }

    return 0;
}
