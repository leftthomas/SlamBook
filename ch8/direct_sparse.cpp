#include<iostream>
#include <fstream>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;


/**
 * 本程序演示了稀疏直接法
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: direct_sparse data_set_path" << endl;
        return 1;
    }
    string data_set_path = argv[1];
    string associate_file = data_set_path + "/associate.txt";
    ifstream fin(associate_file);
    string rgb_file, depth_file, time_rgb, time_depth;

    list<cv::Point2f> key_points;
    cv::Mat color, depth, last_color;
//    573为associate.txt文件行数
    for (int index = 0; index < 573; ++index) {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(data_set_path + "/" + rgb_file);
        depth = cv::imread(data_set_path + "/" + depth_file, -1);
    }
    return 0;
}
