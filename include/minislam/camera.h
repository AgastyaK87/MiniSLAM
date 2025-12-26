#pragma once
#include <Eigen/Dense>
#include <memory>
#include <stdlib.h>

namespace minislam{


class Camera{
public:
// shared pointer used in SLAM to share camera config
    typedef std::shared_ptr<Camera> Ptr;
    //Intrinsics
    double fx_, fy_, cx_, cy_;
    // fx and fy are the focal length in pixels
    // cx and cy are the principal point in pixels

    //baseline (used for stereo)
    double baseline_;

    Camera(double fx, double fy, double cx, double cy, double baseline = 0.0)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline){}

    //Coordinate Transform from world point (x,y,z) to pixel point (u,v)
    Eigen::Vector2d world2pixel(const Eigen::Vector3d& p_c); //p_c is point in camera coordinates

    //Coordinate Transform from pixel point (u,v) to normalized plane (x,y,1), useful for ray casting. 
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p_p, double depth = 1.0);

    
};

}