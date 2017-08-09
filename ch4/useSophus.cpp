#include<iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"


/**
 * 本程序演示了sophus中so3与se3的使用
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    //声明一个沿Z轴旋转90度的旋转矩阵
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(0, 0, 1)).toRotationMatrix();
//    从旋转矩阵直接构造SO(3)
    Sophus::SO3 SO3_R(R);
//    从旋转向量构造
    Sophus::SO3 SO3_v(0, 0, M_PI / 2);
//    从四元数构造
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);

    cout << "SO(3) from matrix: " << SO3_R << endl;
    cout << "SO(3) from vector: " << SO3_v << endl;
    cout << "SO(3) from quaternion: " << SO3_q << endl;

//    使用对数映射得到李代数
    Eigen::Vector3d so3 = SO3_R.log();
    cout << "so(3) = " << so3.transpose() << endl;
//    将so(3)以反对称矩阵的形式输出
    cout << "so(3) hat = \n" << Sophus::SO3::hat(so3) << endl;
//    将反对称矩阵转回so(3)
    cout << "so(3) hat vee = " << Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose() << endl;

//    利用李代数扰动模型来更新位姿
    Eigen::Vector3d update_so3(1e-4, 0, 0);
//    左乘更新
    Sophus::SO3 SO3_updated = Sophus::SO3::exp(update_so3) * SO3_R;
    cout << "SO(3) updated = " << SO3_updated << endl;
    return 0;
}
