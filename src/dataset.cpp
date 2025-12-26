#include "minislam/dataset.h"
#include <iostream>
#include <iomanip>

namespace minislam {

Dataset::Dataset(const std::string& dataset_path) 
    : path_(dataset_path), current_image_index_(0) {
    
    // 1. Load Poses
    std::string pose_file = path_ + "/poses/00.txt";
    std::ifstream fin(pose_file);
    if (!fin) { 
        std::cerr << "ERROR: Cannot find poses at " << pose_file << std::endl; 
        std::cerr << "Did you symlink the datasets folder correctly?" << std::endl;
        return; 
    }

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (!line.empty()) {
            poses_.push_back(readPose(line));
        }
    }
    std::cout << "Loaded " << poses_.size() << " poses." << std::endl;
}

Frame::Ptr Dataset::nextFrame() {
    // Generate filename: 000000.png, 000001.png...
    // Using stringstream for formatting (safer than sprintf)
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << current_image_index_;
    std::string image_file = path_ + "/sequences/00/image_0/" + ss.str() + ".png";

    cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    if (image.data == nullptr) return nullptr; // End of sequence

    // Get Pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (current_image_index_ < poses_.size()) {
        pose = poses_[current_image_index_];
    }

    Frame::Ptr new_frame(new Frame(current_image_index_, image, pose));
    current_image_index_++;
    return new_frame;
}

}