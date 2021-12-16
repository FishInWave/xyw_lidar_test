#ifndef GEOMETRY_TRANSFORM_HPP
#define GEOMETRY_TRANSFORM_HPP
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
void Affine2vectorf(Eigen::Affine3f aff,Eigen::Matrix<float,6,1>& v){
    pcl::getTranslationAndEulerAngles(aff,v[0],v[1],v[2],v[3],v[4],v[5]);
}


#endif