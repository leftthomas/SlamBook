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
//    或使用旋转矩阵
    v_rotated = rotation_matrix * v;
    cout << "(1,0,0) after rotation = " << v_rotated.transpose() << endl;

//    将旋转矩阵转换成欧拉角，按照ZYX的顺序，即RPY角
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;

//    欧式变换矩阵使用Isometry,注意，这是个4*4的矩阵
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//    cout<<T.matrix()<<endl;
//    按照rotation_vector进行旋转
    T.rotate(rotation_vector);
//    这里也可以用T.prerotate(rotation_vector);我建议使用这个
    T.pretranslate(Eigen::Vector3d(1, 3, 4));
//    注意，不能用T.translate(Eigen::Vector3d(1,3,4));
    cout << "Transform matrix =\n" << T.matrix() << endl;
    return 0;
}
