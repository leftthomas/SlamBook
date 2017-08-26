//
// Created by Left Thomas on 2017/8/26.
//

#include "myslam/config.h"

namespace myslam {

    shared_ptr<Config> Config::config_ = nullptr;

    Config::~Config() {
        if (file_.isOpened())
            file_.release();
    }

    void Config::setParameterFile(const string &filename) {
        if (config_ == nullptr)
            config_ = shared_ptr<Config>(new Config);
        config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        if (!config_->file_.isOpened()) {
            cerr << "parameter file " << "does not exist." << endl;
            config_->file_.release();
        }
    }
}