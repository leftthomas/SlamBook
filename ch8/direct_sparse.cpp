#include <fstream>
#include "common.h"

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
    srand((unsigned int) time(0));
    string data_set_path = argv[1];
    string associate_file = data_set_path + "/associate.txt";
    ifstream fin(associate_file);
    string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    vector<Measurement> measurements;

    // 相机内参
    float cx = 325.5;
    float cy = 253.5;
    float fx = 518.0;
    float fy = 519.0;
    float depth_scale = 1000.0;
    Eigen::Matrix3f K;
    K << fx, 0.f, cx, 0.f, fy, cy, 0.f, 0.f, 1.0f;

    Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();

    cv::Mat prev_color;
    // 以第一个图像为参考，对后续图像和参考图像做直接法
    for (int index = 0; index < 10; index++) {
        cout << "*********** loop " << index << " ************" << endl;
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(data_set_path + "/" + rgb_file);
        depth = cv::imread(data_set_path + "/" + depth_file, -1);
        if (color.data == nullptr || depth.data == nullptr)
            continue;
        cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
        if (index == 0) {
            // 对第一帧提取FAST特征点
            vector<cv::KeyPoint> keypoints;
            cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create();
            detector->detect(color, keypoints);
            for (auto kp:keypoints) {
                // 去掉邻近边缘处的点
                if (kp.pt.x < 20 || kp.pt.y < 20 || (kp.pt.x + 20) > color.cols || (kp.pt.y + 20) > color.rows)
                    continue;
                ushort d = depth.ptr<ushort>(cvRound(kp.pt.y))[cvRound(kp.pt.x)];
                if (d == 0)
                    continue;
                Eigen::Vector3d p3d = project2Dto3D(kp.pt.x, kp.pt.y, d, fx, fy, cx, cy, depth_scale);
                float grayscale = float(gray.ptr<uchar>(cvRound(kp.pt.y))[cvRound(kp.pt.x)]);
                measurements.push_back(Measurement(p3d, grayscale));
            }
            prev_color = color.clone();
            continue;
        }
        // 使用直接法计算相机运动
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        poseEstimationDirect(measurements, &gray, K, Tcw);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
        cout << "direct method costs time: " << time_used.count() << " seconds." << endl;
        cout << "Tcw=" << Tcw.matrix() << endl;

        // plot the feature points
        cv::Mat img_show(color.rows * 2, color.cols, CV_8UC3);
        prev_color.copyTo(img_show(cv::Rect(0, 0, color.cols, color.rows)));
        color.copyTo(img_show(cv::Rect(0, color.rows, color.cols, color.rows)));
        for (Measurement m:measurements) {
            if (rand() > RAND_MAX / 5)
                continue;
            Eigen::Vector3d p = m.pos_world;
            Eigen::Vector2d pixel_prev = project3Dto2D(p(0, 0), p(1, 0), p(2, 0), fx, fy, cx, cy);
            Eigen::Vector3d p2 = Tcw * m.pos_world;
            Eigen::Vector2d pixel_now = project3Dto2D(p2(0, 0), p2(1, 0), p2(2, 0), fx, fy, cx, cy);
            if (pixel_now(0, 0) < 0 || pixel_now(0, 0) >= color.cols || pixel_now(1, 0) < 0 ||
                pixel_now(1, 0) >= color.rows)
                continue;

            float b = 255 * float(rand()) / RAND_MAX;
            float g = 255 * float(rand()) / RAND_MAX;
            float r = 255 * float(rand()) / RAND_MAX;
            cv::circle(img_show, cv::Point2d(pixel_prev(0, 0), pixel_prev(1, 0)), 8, cv::Scalar(b, g, r), 2);
            cv::circle(img_show, cv::Point2d(pixel_now(0, 0), pixel_now(1, 0) + color.rows), 8, cv::Scalar(b, g, r), 2);
            cv::line(img_show, cv::Point2d(pixel_prev(0, 0), pixel_prev(1, 0)),
                     cv::Point2d(pixel_now(0, 0), pixel_now(1, 0) + color.rows), cv::Scalar(b, g, r), 1);
        }
        cv::imshow("result", img_show);
        cv::waitKey(0);
    }
    return 0;
}
