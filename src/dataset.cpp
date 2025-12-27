#include "minislam/dataset.h"

namespace minislam {

Dataset::Dataset(const std::string& dataset_path) 
    : path_(dataset_path), current_image_index_(0) {
    
    // 1. Load Ground Truth Poses
    std::ifstream fin(path_ + "/poses/00.txt");
    if (!fin) { 
        std::cerr << "Cannot find poses file at " << path_ << "/poses/00.txt" << std::endl; 
        return; 
    }

    while (!fin.eof()) {
        std::string line;
        std::getline(fin, line);
        if (!line.empty()) {
            poses_.push_back(readPose(line));
        }
    }
}

Frame::Ptr Dataset::nextFrame() {
    // Generate file name: 000000.png, 000001.png...
    char buf[20];
    sprintf(buf, "%06d.png", current_image_index_);
    std::string image_file = path_ + "/sequences/00/image_0/" + std::string(buf);

    cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
    if (image.data == nullptr) return nullptr; // End of sequence

    // Get the ground truth pose for this frame
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (current_image_index_ < poses_.size()) {
        pose = poses_[current_image_index_];
    }

    Frame::Ptr new_frame(new Frame(current_image_index_, image, pose));
    current_image_index_++;
    return new_frame;
}

}