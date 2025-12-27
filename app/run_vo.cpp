#include <iostream>
#include <opencv2/opencv.hpp>
#include "minislam/dataset.h"

int main() {
    // POINT THIS TO YOUR DATASET FOLDER
    std::string dataset_path = "../datasets"; 

    minislam::Dataset dataset(dataset_path);

    //camera intrinsics(Kitti sequence 00)

    double fx = 718.856, fy = 718.856, cx = 607.192, cy = 185.215;
    cv::Point2d pp(cx, cy);
    double focal = fx; 

    cv::Mat last_image, curr_image; 
    std::vector<cv::Point2f> last_keypoints, curr_keypoints;

    //Start at frame 0
    minislam::Frame::Ptr last_frame = dataset.nextFrame();
    last_image = last_frame->image_;
    // Detect features in the first image
    // GoodFeaturesToTrack is simpler/more robust than ORB for basic VO tracking
    cv::goodFeaturesToTrack(last_image, last_keypoints, 2000, 0.01, 10);
    //goodfeaturestotrack tracks features in an image with high spatial gradient variation for optical flow

    while(true){
        minislam::Frame::Ptr curr_frame = dataset.nextFrame();
        if(curr_frame == nullptr) break;
        curr_image = curr_frame -> image_;

        // 1. Track Features using Optical Flow (KLT Tracker)
        //    Instead of "Detect & Match", we "Detect & Track". It's faster.
        std::vector<uchar> status;
        std::vector<float> err;
        if(last_keypoints.empty()){
            cv::goodFeaturesToTrack(curr_image, curr_keypoints, 2000, 0.01, 10);
        }
        cv::calcOpticalFlowPyrLK(last_image, curr_image, last_keypoints, curr_keypoints, status, err);
        //opticalflowpyrlk tracks features between 2 points by estimating movement between frames
        //status is a vector of bytes that are 1 if the feature is in the next frame, 0 otherwise
        //err is a vector of errors for each feature

        std::vector<cv::Point2f> p1,p2;
        for(int i = 0; i<status.size(); i++){
            if(status[i]){
                p1.push_back(last_keypoints[i]);
                p2.push_back(curr_keypoints[i]);
            }
        }
        // Need at least 5 points to solve Essential Matrix
        if (p1.size() < 10) {
            std::cout << "Lost too many points!" << std::endl;
            break; 
        }

        //3. find essential matrix
        cv::Mat E, mask;
        E = cv::findEssentialMat(p2,p1,focal,pp,cv::RANSAC, 0.999, 1.0, mask);

        //4. recover pose
        cv::Mat R,t;
        cv::recoverPose(E,p2,p1,R,t,focal,pp,mask);

        //visualization
        cv::Mat img_show = curr_image.clone();
        cv::cvtColor(img_show, img_show, cv::COLOR_GRAY2BGR);

        for (size_t i = 0; i < p1.size(); i++) {
            if (mask.at<uchar>(i)) { // Only draw inliers
                 cv::line(img_show, p1[i], p2[i], cv::Scalar(0, 255, 0), 2);
                 cv::circle(img_show, p2[i], 3, cv::Scalar(0, 0, 255), -1);
            }
        }

        cv::imshow("VO Tracking", img_show);

        // --- THE SANITY CHECK ---
        // Print our calculated translation 't' vs the real Ground Truth movement

        //1. get real ground truth position difference
        Eigen::Vector3d real_pos_prev = last_frame -> getPosition();
        Eigen::Vector3d real_pos_curr = curr_frame -> getPosition();
        double real_distance = (real_pos_curr - real_pos_prev).norm();

        //2. our calculated t (unit len)
        double calculated_scale = cv::norm(t);

        std::cout << "Frame " << curr_frame->id_ << " | "
                  << "Matches: " << p1.size() << " | "
                  << "Calc Scale: " << calculated_scale << " | "
                  << "Real Distance: " << real_distance << " m" << std::endl;

        //next it
        last_image = curr_image.clone();
        last_keypoints = p2; 
        last_frame = curr_frame;

        //If features r low replenish them

        if(last_keypoints.size() < 1000){
            std::vector<cv::Point2f> new_pts;
            cv::goodFeaturesToTrack(last_image, new_pts, 2000, 0.01, 10);
            last_keypoints.insert(last_keypoints.end(), new_pts.begin(), new_pts.end());
        }
        
        if (cv::waitKey(33) == 'q') break;
        
        
    }
    return 0;
}