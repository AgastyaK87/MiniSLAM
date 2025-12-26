#include <iostream>
#include <opencv2/opencv.hpp>
#include "minislam/dataset.h"

int main() {
    // Ensure this path points to where you created the symlink
    std::string dataset_path = "../datasets"; 

    minislam::Dataset dataset(dataset_path);
    
    // Variables to track relative motion
    Eigen::Vector3d last_pos = Eigen::Vector3d::Zero();

    while (true) {
        minislam::Frame::Ptr frame = dataset.nextFrame();
        if (frame == nullptr) break;

        cv::imshow("KITTI Sequence 00", frame->image_);
        
        Eigen::Vector3d pos = frame->getPosition();
        
        // Print coordinates every 10 frames to avoid spam
        if (frame->id_ % 10 == 0) {
             std::cout << "Frame " << frame->id_ 
                       << " | x: " << pos.x() 
                       << " | y: " << pos.y() 
                       << " | z: " << pos.z() << std::endl;
        }

        // Exit on 'q'
        if (cv::waitKey(20) == 'q') break;
    }

    return 0;
}