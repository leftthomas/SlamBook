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
//    遍历图像
    for (size_t y = 0; y < image.rows; ++y) {
        for (size_t x = 0; x < image.cols; ++x) {
//            获得图像行指针
            unsigned char *row_ptr = image.ptr<unsigned char>(y);
//            获得指定像素数据指针
            unsigned char *data_ptr = &row_ptr[x * image.channels()];
//            输出该像素每个通道数据
            for (int c = 0; c != image.channels(); ++c) {
//                data为I(x,y)第c个通道的值
                unsigned char data = data_ptr[c];
            }
        }
    }
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "遍历图像用时：" << time_used.count() << "秒。" << endl;

//    直接赋值图像不会复制数据
    cv::Mat image_another = image;
//    将图像左上角100*100的块置零
    image_another(cv::Rect(0, 0, 100, 100)).setTo(0);
//    修改image_another会导致image也发生变化
    cv::imshow("image", image);
    cv::waitKey(0);

//    使用clone函数复制图像，会直接复制数据
    cv::Mat image_clone = image.clone();
    image_clone(cv::Rect(0, 0, 100, 100)).setTo(255);
    cv::imshow("image", image);
    cv::imshow("image_clone", image_clone);
    cv::waitKey(0);

//    貌似这句话并没什么用
    cv::destroyAllWindows();
    return 0;
}
