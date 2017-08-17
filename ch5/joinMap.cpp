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
//        读取对应位姿
        double data[7] = {0};
        for (auto &d:data) {
            fin >> d;
        }
//        构造变换矩阵
        Eigen::Quaterniond q(data[6], data[3], data[4], data[5]);
        Eigen::Isometry3d T(q);
        T.pretranslate(Eigen::Vector3d(data[0], data[1], data[2]));
        poses.push_back(T);
    }


//    相机内参
    double cx = 325.5;
    double cy = 253.5;
    double fx = 518.0;
    double fy = 519.0;
    double depthScale = 1000.0;

    cout << "正在将图像转换为点云..." << endl;
//    定义点云使用的格式为XYZRGB
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> PointCloud;

//    新建点云
    PointCloud::Ptr pointCloud(new PointCloud);
    for (int i = 0; i < 5; ++i) {
        cout << "转换图像中:" << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Eigen::Isometry3d T = poses[i];
//        v在前,u在后
        for (int v = 0; v < color.rows; ++v) {
            for (int u = 0; u < color.cols; ++u) {
//                深度值
                unsigned int d = depth.ptr<unsigned short>(v)[u];
//                0表示没有测量到
                if (d == 0) continue;
//                得到相机坐标
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;
                point[0] = (u - cx) * point[2] / fx;
                point[1] = (v - cy) * point[2] / fy;
//                得到世界坐标
                Eigen::Vector3d pointWorld = T * point;

//                构造点云的点
                PointT p;
                p.x = pointWorld[0];
                p.y = pointWorld[1];
                p.z = pointWorld[2];
//                OpenCV默认图像排列是BGR
                p.b = color.data[v * color.step + u * color.channels()];
                p.g = color.data[v * color.step + u * color.channels() + 1];
                p.r = color.data[v * color.step + u * color.channels() + 2];
                pointCloud->points.push_back(p);
            }
        }
    }

    pointCloud->is_dense = false;
    cout << "点云共有" << pointCloud->size() << "个点." << endl;
    pcl::io::savePCDFileBinary("../../ch5/map.pcd", *pointCloud);
    /**
     * 如果是通过brew install pcl安装的pcl,那么查看pcd文件在Mac上需要通过
     * /usr/local/Cellar/pcl/1.8.0_8/pcl_viewer.app/Contents/MacOS/pcl_viewer map.pcd 调用
     * 这个很奇葩，不像ubuntu那样可以直接通过pcl_viewer map.pcd 调用
     */
    return 0;
}
