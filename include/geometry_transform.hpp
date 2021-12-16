#ifndef GEOMETRY_TRANSFORM_HPP
#define GEOMETRY_TRANSFORM_HPP
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
// v都按照x y z roll pitch yaw排列，且此处的rpy到矩阵的转换遵循XYZ内旋，即RzRyRx
void affine2Vectorf(Eigen::Affine3f aff,Eigen::Matrix<float,6,1>& v){
    pcl::getTranslationAndEulerAngles(aff,v[0],v[1],v[2],v[3],v[4],v[5]);
}

Eigen::Affine3f vector2Affinef(Eigen::Matrix<float,6,1>& v){
    return pcl::getTransformation(v[0],v[1],v[2],v[3],v[4],v[5]);
}

#endif