//
// Created by Left Thomas on 2017/8/27.
//

#ifndef SLAMBOOK_VISULA_ODOMETRY_H
#define SLAMBOOK_VISULA_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/map.h"
#include <opencv2/features2d/features2d.hpp>

namespace myslam {
    class VisualOdometry {
    public:
        typedef shared_ptr<VisualOdometry> Ptr;
        enum VOState {
            INITIALIZING = -1,
            OK = 0,
            LOST
        };
//        current VO status
        VOState state_;
        Map::Ptr map_;
        Frame::Ptr ref_;
        Frame::Ptr curr_;
//        orb detector and computer
        cv::Ptr<cv::ORB> orb_;

        vector<cv::KeyPoint> keypoints_curr_;
        Mat descriptors_curr_;

        // flann matcher
        cv::FlannBasedMatcher matcher_flann_;
        // matched 3d points
        vector<MapPoint::Ptr> match_3dpts_;
        // matched 2d pixels (index of kp_curr)
        vector<int> match_2dkp_index_;

        SE3 T_c_r_estimated_;
        int num_inliers_;
        int num_lost_;

//        paramters;
        int num_of_features_;
//        scale in image pyramid
        float scale_factor_;
        int level_pyramid_;
//        ratio for selecting good matches
        float match_ratio_;
//        max number of continuous lost frames
        int max_num_lost_;
        int min_inliers_;
//        minimal rotation of two key frames
        double key_frame_min_rot_;
//        minimal translation of two key frames
        double key_frame_min_trans_;
//        remove map point ratio
        double map_point_erase_ratio_;

//        functions
        VisualOdometry();

        virtual ~VisualOdometry();

//        add a new frame
        bool addFrame(Frame::Ptr frame);

    protected:
        void extractKeyPoints();

        void computeDescriptors();

        void featuresMatching();

        void poseEstimationPnP();

        void optimizeMap();

        double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);

        void addMapPoints();

        void addKeyFrame();

        bool checkEstimatedPose();

        bool checkKeyFrame();
    };
}

#endif //SLAMBOOK_VISULA_ODOMETRY_H
