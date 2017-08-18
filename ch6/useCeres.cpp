#include<iostream>
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

    return 0;
}
