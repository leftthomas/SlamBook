//
// Created by Left Thomas on 2017/8/26.
//

#ifndef SLAMBOOK_CONFIG_H
#define SLAMBOOK_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {
    class Config {
    private:
        static shared_ptr<Config> config_;
        cv::FileStorage file_;

//        private constructor makes it as a singleton
        Config();

    public:
//        close the file when deconstructing
        virtual ~Config();

//        set a new config file
        static void setParameterFile(const string &filename);

//        access the parameter values
        template<typename T>
        static T get(const string &key) {
            return T(Config::config_->file_[key]);
        }
    };
}

#endif //SLAMBOOK_CONFIG_H
