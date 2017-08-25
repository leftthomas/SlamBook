#include<iostream>
#include <fstream>
#include <list>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;


/**
 * 本程序演示了LK光流法
 * data文件去TUM网站下载，注意下载associate.py之后需要修改以下两条语句：
 * first_keys = first_list.keys() ————————> first_keys = list(first_list.keys())
 * second_keys = second_list.keys() ————————> second_keys = list(second_list.keys())
 * 然后运行 python associate.py rgb.txt depth.txt > associate.txt 生成 associate.txt 文件
 * 否则在Python3下运行会报错
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: lkFlow data_set_path" << endl;
        return 1;
    }
    string data_set_path = argv[1];
    string associate_file = data_set_path + "/associate.txt";
    ifstream fin(associate_file);
    string rgb_file, depth_file, time_rgb, time_depth;

    list<cv::Point2f> key_points;
    cv::Mat color, depth, last_color;
    for (int index = 0; index < 100; ++index) {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(string.append(data_set_path + "/" + rgb_file));
        depth = cv::imread(string.append(data_set_path + "/" + depth_file), -1);
        if (index == 0) {

        }
    }
    return 0;
}
