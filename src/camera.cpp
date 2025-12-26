#include "minislam/camera.h"

namespace minislam{
Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d& p_c){

    //Math: u = fx * x / z + cx
    //      v = fy * y / z + cy
    return Eigen::Vector2d(
        fx_ * p_c(0,0) / p_c(2,0) + cx_,
        fy_ * p_c(1,0) / p_c(2,0) + cy_
    );
}

Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d& p_p, double depth){
    //math: x = (u-cx)/fx
    // y = (u - cy)/fy

    return Eigen::Vector3d(
        (p_p(0,0) - cx_)*depth/fx_,
        (p_p(1,0) - cy_)*depth/fy_,
        depth
    );
}

}