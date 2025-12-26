#pragma once
#include "minislam/frame.h"
#include <vector>
#include <string>
#include <fstream>

namespace minislam{

class Dataset{
public:
    Dataset(const std::string& dataset_path);

    //Returns nullptr if sequence ends
    Frame::Ptr nextFrame();

    //Helper to parse the 12 number line into a 4x4 matrix

    static Eigen::Matrix4d readPose(const std::string& line){
        std::stringstream ss(line);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        //Fill in top 3x4 block;
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 4; j++){
                ss >> T(i, j);
            }
        }
        return T;
    }

private:
    std::string path_;
    int current_image_index_;
    std::vector<Eigen::Matrix4d> poses_;   
}
    
}