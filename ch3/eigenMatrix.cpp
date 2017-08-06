#include<iostream>

using namespace std;

//Eigen部分
#include <Eigen/Core>
//稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>

#define MATRIX_SIZE 50

/*
 * 本程序演示了Eigen基本类型的使用
 */

int main(int argc, char **argv) {

    //声明一个2x3的float类型矩阵
    Eigen::Matrix<float, 2, 3> matrix_23;
    //Vector3d本质是Matrix<double ,3,1>
    Eigen::Vector3d v_3d;
    //Matrix3d本质是Matrix<double ,3,3>
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); //初始化为零
    //动态矩阵
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
    //或者
    Eigen::MatrixXd matrix_x;

    //矩阵操作
    //输入数据
    matrix_23 << 1, 2, 3, 4, 5, 6;
    //输出
    cout << matrix_23 << endl;

    return 0;
}
