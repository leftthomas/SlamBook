#include<iostream>
#include <cmath>

using namespace std;

#include <Eigen/Core>
//Eigen几何模块
#include <Eigen/Geometry>

/*
 * 本程序演示了Eigen几何模块的使用
 */

int main(int argc, char **argv) {

//    3D旋转矩阵直接使用Matrix3d或Matrix3f
    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
//    旋转向量使用AngleAxis（也就是轴角），初始化为沿Z轴旋转45度
    Eigen::AngleAxisd rotation_vector(M_PI / 4, Eigen::Vector3d(0, 0, 1));
//    设置精度为3,即显示小数点后三位
    cout.precision(3);
//    用matrix()将旋转向量转换成旋转矩阵
    cout << "rotation matrix =\n" << rotation_vector.matrix() << endl;
//    cout<<rotation_matrix<<endl;
//    或者直接赋值给旋转矩阵
    rotation_matrix = rotation_vector.toRotationMatrix();
//    用AngleAxis进行坐标变换
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

    return 0;
}
