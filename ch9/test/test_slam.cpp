//
// Created by Left Thomas on 2017/8/26.
// 在Mac上需要下载OpenCV3源码，并通过brew install vtk等安装好依赖项
// 手动编译安装OpenCV3，不要通过brew install opencv安装，否则是没有viz模块的
//
#include<iostream>
#include <fstream>
#include <myslam/config.h>
#include <myslam/visual_odometry.h>

using namespace std;

int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "usage: test_slam parameter_file" << endl;
        return 1;
    }
    myslam::Config::setParameterFile(argv[1]);
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

    string dataset_dir = myslam::Config::get<string>("dataset_dir");
    cout << "dataset: " << dataset_dir << endl;
    ifstream fin(dataset_dir + "/associate.txt");
    if (!fin) {
        cout << "please generate the associate file called associate.txt!" << endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;
    while (!fin.eof()) {
        string rgb_time, rgb_file, depth_time, depth_file;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));
        rgb_files.push_back(dataset_dir + "/" + rgb_file);
        depth_files.push_back(dataset_dir + "/" + depth_file);
        if (!fin.good())
            break;
    }

    myslam::Camera::Ptr camera(new myslam::Camera);
    return 0;
}
