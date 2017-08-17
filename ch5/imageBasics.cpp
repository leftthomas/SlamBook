#include<iostream>

using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * 本程序演示了OpenCV的使用
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

//    读取argv[1]指定路径下的图像(这里使用了"/Users/left/workspace/slambook/ch5/rgb.jpg")
    cv::Mat image;
    image = cv::imread(argv[1]);
    if (image.data == nullptr) {
        cerr << "文件" << argv[1] << "不存在。" << endl;
        return 0;
    }

//    显示读取的图像文件信息
    cout << "图像宽为" << image.cols << ",高为" << image.rows << ",通道数为" << image.channels() << endl;
    cv::imshow("image", image);
    cv::waitKey(0);

//    判断image的类型(这里的判断不知道有啥用，反正我试过rgbd、depth图都可以，所以也就不知道什么图不可以了)
    if (image.type() != CV_8UC1 && image.type() != CV_8UC3) {
        cout << "请输入一张彩色图或灰度图。" << endl;
        return 0;
    }

//    使用chrono计时
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    return 0;
}
