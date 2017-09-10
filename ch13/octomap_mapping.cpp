#include<iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <octomap/octomap.h>
#include <Eigen/Geometry>
#include <boost/format.hpp>

using namespace std;

/**
 * 本程序演示了octomap八叉树地图
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
        for (double &j : data) {
            fin >> j;
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

    cout << "正在将图像转换为 OctoMap ..." << endl;

    // octomap tree
    // 参数为分辨率
    octomap::OcTree tree(0.05);

    for (int i = 0; i < 5; i++) {
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
        // the point cloud in octomap
        octomap::Pointcloud cloud;

        for (int v = 0; v < color.rows; v++)
            for (int u = 0; u < color.cols; u++) {
                // 深度值
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                // 为0表示没有测量到
                if (d == 0) continue;
                // 深度太大时不稳定，去掉
                if (d >= 7000) continue;
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
                Eigen::Vector3d pointWorld = T * point;
                // 将世界坐标系的点放入点云
                cloud.push_back(pointWorld[0], pointWorld[1], pointWorld[2]);
            }

        // 将点云存入八叉树地图，给定原点，这样可以计算投射线
        tree.insertPointCloud(cloud, octomap::point3d(T(0, 3), T(1, 3), T(2, 3)));
    }

    // 更新中间节点的占据信息并写入磁盘
    tree.updateInnerOccupancy();
    cout << "saving octomap ... " << endl;
    tree.writeBinary("octomap.bt");
    return 0;
}
