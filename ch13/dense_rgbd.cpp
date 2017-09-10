#include<iostream>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Geometry>
#include <boost/format.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

using namespace std;

/**
 * 本程序演示了RGBD点云地图稠密重建
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        cout << "Usage: dense_monocular path_to_test_dataset" << endl;
        return -1;
    }

    return 0;
}
