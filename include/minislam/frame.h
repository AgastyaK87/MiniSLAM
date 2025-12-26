#pragma once
#include <memory>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace minislam{

    struct Frame{
        typedef std::shared_ptr<Frame> Ptr;
        unsigned long id_; // 0,1,2
        double time_stamp_; //timestamp
        cv::Mat image; //grayscale image
        Eigen::Matrix4d pose_; //T_wc (camera to world)

        Frame(long id, const cv::Mat& img, const Eigen::Matrix4d& pose = Eigen::Matrix4d::Identity())
        : id_(id), image(img), pose_(pose){}
        
        // Helper: Get the exact position (x,y,z) in World
        Eigen::Vector3d getPosition() const {
            return pose_.block<3, 1>(0, 3); // Top-right 3x1 block
        }
        
    };

}