//
// Created by Left Thomas on 2017/9/5.
//
#include <DBoW3/DBoW3.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace std;
using namespace cv;

/**
 * 本程序演示了字典相似度计算
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

//    read the images and database
    cout << "reading database." << endl;
    DBoW3::Vocabulary vocab("./vocabulary.yml.gz");
    if (vocab.empty()) {
        cerr << "vocabulary does not exist." << endl;
        return 1;
    }
    cout << "reading images." << endl;
    vector<Mat> images;
    for (int i = 0; i < 10; ++i) {
        string path = "../../ch12/data/" + to_string(i + 1) + ".png";
        images.push_back(imread(path));
    }
//    detect ORB features
    cout << "detecting ORB features." << endl;
    Ptr<Feature2D> detector = ORB::create();
    vector<Mat> descriptors;
    for (Mat &image:images) {
        vector<KeyPoint> keypoints;
        Mat descriptor;
        detector->detectAndCompute(image, Mat(), keypoints, descriptor);
        descriptors.push_back(descriptor);
    }

    cout << "comparing image with images." << endl;

    return 0;
}

