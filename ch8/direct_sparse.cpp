#include<iostream>
#include <fstream>
#include <list>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/video/tracking.hpp>
using namespace std;
using namespace g2o;

// 自定义一元边,和相机姿态顶点连接
class EdgeSE3ProjectDirect : public BaseUnaryEdge<1, double, VertexSE3Expmap> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
//    世界坐标系下的点
    Eigen::Vector3d x_world_;
//    相机内参
    float cx_ = 0, cy_ = 0, fx_ = 0, fy_ = 0;
//    参考帧
    cv::Mat *image_ = nullptr;

    EdgeSE3ProjectDirect() {}

    EdgeSE3ProjectDirect(Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat *image) :
            x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image) {}

    void computeError() override {
        const auto *v = static_cast<const VertexSE3Expmap *>(_vertices[0]);
        Eigen::Vector3d x_local = v->estimate().map(x_world_);
        float x = x_local[0] * fx_ / x_local[2] + cx_;
        float y = x_local[1] * fy_ / x_local[2] + cy_;
//        检查x,y是否在图像中
        if ((x - 4) < 0 || (x + 4) > image_->cols || (y - 4) < 0 || (y + 4) > image_->rows) {
            _error(0, 0) = 0.0;
            this->setLevel(1);
        } else {
            _error(0, 0) = getPoxelValue(x, y) - _measurement;
        }

    }

    void linearizeOplus() override {
        auto *pose = static_cast<VertexSE3Expmap *>(_vertices[0]);
        const SE3Quat &T(pose->estimate());
        Eigen::Vector3d xyz_trans = T.map(x_world_);
        double x = xyz_trans[0];
        double y = xyz_trans[1];
        double z = xyz_trans[2];

        _jacobianOplusXi(0, 0) = 0;
        _jacobianOplusXi(0, 1) = -z;
        _jacobianOplusXi(0, 2) = y;
        _jacobianOplusXi(0, 3) = -1;
        _jacobianOplusXi(0, 4) = 0;
        _jacobianOplusXi(0, 5) = 0;

        _jacobianOplusXi(1, 0) = z;
        _jacobianOplusXi(1, 1) = 0;
        _jacobianOplusXi(1, 2) = -x;
        _jacobianOplusXi(1, 3) = 0;
        _jacobianOplusXi(1, 4) = -1;
        _jacobianOplusXi(1, 5) = 0;

        _jacobianOplusXi(2, 0) = -y;
        _jacobianOplusXi(2, 1) = x;
        _jacobianOplusXi(2, 2) = 0;
        _jacobianOplusXi(2, 3) = 0;
        _jacobianOplusXi(2, 4) = 0;
        _jacobianOplusXi(2, 5) = -1;
    }

    bool read(istream &in) override { return false; }

    bool write(ostream &out) const override { return false; }
};
/**
 * 本程序演示了稀疏直接法
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {

    if (argc != 2) {
        cout << "usage: direct_sparse data_set_path" << endl;
        return 1;
    }
    string data_set_path = argv[1];
    string associate_file = data_set_path + "/associate.txt";
    ifstream fin(associate_file);
    string rgb_file, depth_file, time_rgb, time_depth;

    list<cv::Point2f> key_points;
    cv::Mat color, depth, last_color;
//    573为associate.txt文件行数
    for (int index = 0; index < 573; ++index) {
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        color = cv::imread(data_set_path + "/" + rgb_file);
        depth = cv::imread(data_set_path + "/" + depth_file, -1);
    }
    return 0;
}
