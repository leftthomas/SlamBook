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
//    573为associate.txt文件行数
    for (int index = 0; index < 573; ++index) {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(data_set_path + "/" + rgb_file);
        depth = cv::imread(data_set_path + "/" + depth_file, -1);
        if (index == 0) {
//            对第一帧提取FAST特征点
            vector<cv::KeyPoint> kps;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, kps);
            for (auto kp:kps) {
                key_points.push_back(kp.pt);
            }
            last_color = color;
            continue;
        }
        if (color.data == nullptr || depth.data == nullptr)
            continue;
//        对其他帧用LK跟踪特征点
        vector<cv::Point2f> next_key_points;
        vector<cv::Point2f> prev_key_points;
        for (const auto &kp:key_points) {
            prev_key_points.push_back(kp);
        }
        vector<unsigned char> status;
        vector<float> error;
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        cv::calcOpticalFlowPyrLK(last_color, color, prev_key_points, next_key_points, status, error);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        auto time_used = chrono::duration_cast<chrono::duration<double >>(t2 - t1);
        cout << index << "--LK Flow costs time: " << time_used.count() << " seconds." << endl;
//        删掉跟丢的点
        int i = 0;
        for (auto iter = key_points.begin(); iter != key_points.end(); i++) {
            if (status[i] == 0) {
                iter = key_points.erase(iter);
                continue;
            }
            *iter = next_key_points[i];
            iter++;
        }
        cout << "tracked key points: " << key_points.size() << endl;
        if (key_points.empty()) {
            cout << "all key points are lost." << endl;
            break;
        }
//        画出key points
        cv::Mat img_show = color.clone();
        for (const auto &kp:key_points) {
            cv::circle(img_show, kp, 10, cv::Scalar(0, 240, 0), 1);
        }
        cv::imshow("corners", img_show);
        cv::waitKey(0);
        last_color = color;
    }
    return 0;
}
