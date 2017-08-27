//
// Created by Left Thomas on 2017/8/27.
//

#include "myslam/visual_odometry.h"
#include "myslam/config.h"

namespace myslam {

    VisualOdometry::VisualOdometry() : state_(INITIALIZING), map_(new Map), ref_(nullptr), curr_(nullptr),
                                       num_inliers_(0), num_lost_(0) {
        num_of_features_ = Config::get<int>("number_of_features");
        scale_factor_ = Config::get<float>("scale_factor");
        level_pyramid_ = Config::get<int>("level_pyramid");
        match_ratio_ = Config::get<float>("match_ratio");
        max_num_lost_ = Config::get<int>("max_num_lost");
        min_inliers_ = Config::get<int>("min_inliers");
        key_frame_min_rot_ = Config::get<double>("key_frame_rotation");
        key_frame_min_trans_ = Config::get<double>("key_frame_translation");
        orb_ = cv::ORB::create(num_of_features_, scale_factor_, level_pyramid_);
    }

    VisualOdometry::~VisualOdometry() = default;

    bool VisualOdometry::addFrame(Frame::Ptr frame) {
        switch (state_) {
            case INITIALIZING: {
                state_ = OK;
                curr_ = ref_ = frame;
                map_->insertKeyFrame(frame);
//                extract features from first frame
                extractKeyPoints();
                computeDescriptors();
//                computer the 3d position of features on ref frame
                setRef3DPoints();
                break;
            }
            case OK: {
                curr_ = frame;
                extractKeyPoints();
                computeDescriptors();
                featuresMatching();
                poseEstimationPnP();
                if (checkEstimatedPose()) {
                    curr_->T_c_w = T_c_r_estimated_ * ref_->T_c_w;
                    ref_ = curr_;
                    setRef3DPoints();
                    num_lost_ = 0;
                    if (checkKeyFrame())
                        addKeyFrame();
                } else {
                    num_lost_++;
                    if (num_lost_ > max_num_lost_)
                        state_ = LOST;
                    return false;
                }
                break;
            }
            case LOST: {
                cout << "vo has lost." << endl;
                break;
            }
        }
        return true;
    }

    void VisualOdometry::extractKeyPoints() {

    }

    void VisualOdometry::computeDescriptors() {

    }

    void VisualOdometry::featuresMatching() {

    }

    void VisualOdometry::poseEstimationPnP() {

    }

    void VisualOdometry::setRef3DPoints() {

    }

    void VisualOdometry::addKeyFrame() {

    }

    bool VisualOdometry::checkEstimatedPose() {
        return false;
    }

    bool VisualOdometry::checkKeyFrame() {
        return false;
    }

}