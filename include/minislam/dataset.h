#pragma once
#include "minislam/frame.h"
#include <vector>
#include <string>
#include <fstream>

namespace minislam {

class Dataset {
public:
    Dataset(const std::string& dataset_path);
    
    // Get the next frame from the sequence
    Frame::Ptr nextFrame();

    // Helper to read the KITTI pose format (12 numbers)
    // 12 numbers = flattened 3x4 matrix (Row 1, Row 2, Row 3)
    static Eigen::Matrix4d readPose(std::string line) {
        std::stringstream ss(line);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                ss >> T(i, j);
            }
        }
        return T;
    }

private:
    std::string path_;
    int current_image_index_;
    std::vector<Eigen::Matrix4d> poses_;
};

}