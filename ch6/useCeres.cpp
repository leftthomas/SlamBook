#include<iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>

using namespace std;

//代价函数的计算模型
struct CURVE_FITTING_COST {

//    x,y数据
    const double _x, _y;

    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

//    残差的计算
    template<typename T>
    bool operator()(const T *const abc,//模型参数，有三维
                    T *residual) const//残差
    {
//        y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
};

/**
 * 本程序演示了Ceres的使用
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

//    真实参数值
    double a = 1.0, b = 2.0, c = 1.0;
//    数据点个数
    int N = 100;
//    噪声sigma值
    double w_sigma = 1.0;
//    OpenCV随机数产生器
    cv::RNG rng;
//    abc参数估计值
    double abc[3] = {0, 0, 0};
//    数据
    vector<double> x_data, y_data;

    cout << "生成数据：" << endl;
    for (int i = 0; i < N; ++i) {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(a * x * x + b * x + c) + rng.gaussian(w_sigma));
        cout << x_data[i] << " " << y_data[i] << endl;
    }
    return 0;
}
