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

    //访问矩阵元素
    for (int i = 0; i < 1; i++) {
        for (int j = 0; j < 2; j++) {
            cout << matrix_23(i, j) << endl;
        }
    }

    v_3d << 3, 2, 1;
    //矩阵和向量相乘，注意类型转换以及运算结果的维数
    Eigen::Matrix<double, 2, 1> result = matrix_23.cast<double>() * v_3d;
    cout << result << endl;

    matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;

    //转置
    cout << matrix_33.transpose() << endl;
    //各元素和
    cout << matrix_33.sum() << endl;
    //迹
    cout << matrix_33.trace() << endl;
    //数乘
    cout << 10 * matrix_33 << endl;
    //逆
    cout << matrix_33.inverse() << endl;
    //行列式
    cout << matrix_33.determinant() << endl;

    //特征值与特征向量计算（实对称矩阵可以保证对角化成功）
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(matrix_33.transpose() * matrix_33);
    cout << "Eigen values = " << eigen_solver.eigenvalues() << endl;
    cout << "Eigen vectors = " << eigen_solver.eigenvectors() << endl;

    //解方程：matrix_NN * x = v_Nd
    Eigen::Matrix<double, MATRIX_SIZE, MATRIX_SIZE> matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix<double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE, 1);

    clock_t time_stt = clock();//计时
    //直接求逆（求逆运算量大，耗时）
    Eigen::Matrix<double, MATRIX_SIZE, 1> x = matrix_NN.inverse() * v_Nd;
    cout << "time used in normal inverse is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    //用矩阵分解来求，例如QR分解
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time used in QR composition is " << 1000 * (clock() - time_stt) / (double) CLOCKS_PER_SEC << "ms" << endl;
    return 0;
}
