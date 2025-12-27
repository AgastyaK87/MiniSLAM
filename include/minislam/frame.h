#pragma once
#include "minislam/camera.h"
#include <opencv2/opencv.hpp>

namespace minislam {

struct Frame {
    typedef std::shared_ptr<Frame> Ptr;
    
    unsigned long id_;           // Frame ID (0, 1, 2...)
    double time_stamp_;          // Time (if available)
    cv::Mat image_;              // Grayscale image
    Eigen::Matrix4d pose_;       // Ground Truth Pose (T_wc: Camera to World)
    
    // We will store features here in future lessons
    
    Frame(long id, const cv::Mat& img, const Eigen::Matrix4d& pose = Eigen::Matrix4d::Identity())
        : id_(id), image_(img), pose_(pose) {}
    
    Eigen::Vector3d getPosition(){
        // Extracts the translation vector (top-right 3x1 block)
        return pose_.block<3, 1>(0, 3);
    }
};

}