//
// Created by Left Thomas on 2017/8/27.
//

#include "myslam/visual_odometry.h"
#include "myslam/config.h"
#include <opencv2/calib3d/calib3d.hpp>

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
        orb_->detect(curr_->color_, keypoints_curr_);
    }

    void VisualOdometry::computeDescriptors() {
        orb_->compute(curr_->color_, keypoints_curr_, descriptors_curr_);
    }

    void VisualOdometry::setRef3DPoints() {
        pts_3d_ref_.clear();
        descriptors_ref_ = Mat();
        for (int i = 0; i < keypoints_curr_.size(); ++i) {
            double d = ref_->findDepth(keypoints_curr_[i]);
            if (d > 0) {
                Vector3d p_cam = ref_->camera_->pixel2camera(Vector2d(
                        keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y), d);
                pts_3d_ref_.push_back(cv::Point3f(p_cam(0, 0), p_cam(1, 0), p_cam(2, 0)));
                descriptors_ref_.push_back(descriptors_curr_.row(i));
            }
        }
    }

    void VisualOdometry::featuresMatching() {
        vector<cv::DMatch> matches;
        cv::BFMatcher matcher(cv::NORM_HAMMING);
        matcher.match(descriptors_ref_, descriptors_curr_, matches, cv::noArray());

        float min_dist = min_element(matches.begin(), matches.end(), [](
                const cv::DMatch &m1, const cv::DMatch &m2) {
            return m1.distance < m2.distance;
        })->distance;

        features_matches_.clear();
        for (cv::DMatch &m:matches) {
            if (m.distance < max<float>(match_ratio_ * min_dist, 30.0)) {
                features_matches_.push_back(m);
            }
        }
//        cout<<"good matches:"<<features_matches_.size()<<endl;
    }

    void VisualOdometry::poseEstimationPnP() {
        vector<cv::Point3f> pts_3d;
        vector<cv::Point2f> pts_2d;
        for (cv::DMatch &m:features_matches_) {
            pts_3d.push_back(pts_3d_ref_[m.queryIdx]);
            pts_2d.push_back(keypoints_curr_[m.trainIdx].pt);
        }
        cv::Mat_<double> K(3, 3);
        K << ref_->camera_->fx_, 0, ref_->camera_->cx_, 0, ref_->camera_->fy_, ref_->camera_->cy_, 0, 0, 1;
        Mat rvec, tvec, inliers;
        cv::solvePnPRansac(pts_3d, pts_2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);
        num_inliers_ = inliers.rows;
//        cout<<"PnP inliers: "<<num_inliers_<<endl;
        T_c_r_estimated_ = SE3(SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
                               Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));
    }

    bool VisualOdometry::checkEstimatedPose() {
        if (num_inliers_ < min_inliers_) {
            cout << "reject because the number of inliers is too small: " << num_inliers_ << endl;
            return false;
        }
        Sophus::Vector6d d = T_c_r_estimated_.log();
        if (d.norm() > 5.0) {
            cout << "reject because the motion is too large: " << d.norm() << endl;
            return false;
        }
        return true;
    }

    bool VisualOdometry::checkKeyFrame() {
        Sophus::Vector6d d = T_c_r_estimated_.log();
//        注意，平移在前，旋转在后
        Vector3d trans = d.head(3);
        Vector3d rot = d.tail(3);
        return trans.norm() > key_frame_min_trans_ || rot.norm() > key_frame_min_rot_;
    }

    void VisualOdometry::addKeyFrame() {
//        cout << "adding a key frame " << endl;
        map_->insertKeyFrame(curr_);
    }
}