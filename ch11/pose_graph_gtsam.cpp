//
// Created by Left Thomas on 2017/9/3.
//

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <sophus/se3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

using namespace std;
using Sophus::SE3;
using Sophus::SO3;

/**
 * 本程序演示如何使用gtsam进行位姿图优化
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: pose_graph_gtsam sphere.g2o" << endl;
        return 1;
    }
    ifstream fin(argv[1]);
    if (!fin) {
        cout << "file " << argv[1] << " does not exist." << endl;
        return 1;
    }

//    gtsam的因子图
    gtsam::NonlinearFactorGraph::shared_ptr graph(new gtsam::NonlinearFactorGraph);
//    初始值
    gtsam::Values::shared_ptr initial(new gtsam::Values);
//    从g2o文件中读取节点和边的信息
    int cntVertex = 0, cntEdge = 0;
    cout << "reading from g2o file" << endl;

    while (!fin.eof()) {
        string tag;
        fin >> tag;
        if (tag == "VERTEX_SE3:QUAT") {
//            顶点
            gtsam::Key id;
            fin >> id;
            double data[7];
            for (double &i : data) {
                fin >> i;
            }
//            转换至gtsam的Pose3
            gtsam::Rot3 R = gtsam::Rot3::quaternion(data[6], data[3], data[4], data[5]);
            gtsam::Point3 t(data[0], data[1], data[2]);
//            添加初始值
            initial->insert(id, gtsam::Pose3(R, t));
            cntVertex++;
        } else if (tag == "EDGE_SE3:QUAT") {
//            边，对应到因子图中的因子
            gtsam::Matrix m = gtsam::Matrix6::Identity();//信息矩阵
            gtsam::Key id1, id2;
            fin >> id1 >> id2;
            double data[7];
            for (double &i : data) {
                fin >> i;
            }
            gtsam::Rot3 R = gtsam::Rot3::quaternion(data[6], data[3], data[4], data[5]);
            gtsam::Point3 t(data[0], data[1], data[2]);
            for (int i = 0; i < 6; ++i) {
                for (int j = i; j < 6; ++j) {
                    double mij;
                    fin >> mij;
                    m(i, j) = mij;
                    m(j, i) = mij;
                }
            }

//            gtsam信息矩阵
            gtsam::Matrix mgtsam = gtsam::Matrix6::Identity();
//            cov rotation
            mgtsam.block(0, 0, 3, 3) = m.block(3, 3, 3, 3);
        }
    }

}