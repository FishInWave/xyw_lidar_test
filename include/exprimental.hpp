#ifndef EXPERIMENTAL_HPP
#define EXPERIMENTAL_HPP
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
/**
 * @brief 保存配准前后的点云为pcd文件，target为红色，source为绿色，转换后的source为蓝色
 * 
 * @tparam PointT 
 * @param cloud_target 
 * @param cloud_source 
 * @param transform 
 * @param filename 保存的文件名
 */
template <typename PointT,typename Scalar>
void save3PCD(pcl::PointCloud<PointT> &cloud_target, pcl::PointCloud<PointT> &cloud_source, Eigen::Matrix<Scalar,4,4> transform, std::string filename)
{
    pcl::PointCloud<PointT> cloud_trans;
    pcl::transformPointCloud(cloud_source,cloud_trans, transform.template cast<float>());
    pcl::PointCloud<pcl::PointXYZRGB> cloud_comb;
    pcl::PointXYZRGB red(255, 0, 0);
    for (int i = 0; i < cloud_target.points.size(); ++i)
    {
        red.x = cloud_target.points[i].x;
        red.y = cloud_target.points[i].y;
        red.z = cloud_target.points[i].z;
        cloud_comb.points.push_back(red);
    }
    pcl::PointXYZRGB green(0, 200, 0);
    for (int i = 0; i < cloud_source.points.size(); ++i)
    {
        green.x = cloud_source.points[i].x;
        green.y = cloud_source.points[i].y;
        green.z = cloud_source.points[i].z;
        cloud_comb.points.push_back(green);
    }
    pcl::PointXYZRGB blue(10, 20, 200);
    for (int i = 0; i < cloud_trans.points.size(); ++i)
    {
        blue.x = cloud_trans.points[i].x;
        blue.y = cloud_trans.points[i].y;
        blue.z = cloud_trans.points[i].z;
        cloud_comb.points.push_back(blue);
    }
    cloud_comb.width = 1;
    cloud_comb.height = cloud_comb.points.size();
    cloud_comb.is_dense = false;
    pcl::io::savePCDFileBinary(filename, cloud_comb);
}

/**
 * @brief 保存配准前后的点云为pcd文件，target为红色，转换后的source为蓝色
 * 
 * @tparam PointT 
 * @param cloud_target 
 * @param cloud_source 
 * @param transform 
 * @param filename 保存的文件名
 */
template <typename PointT,typename Scalar>
void save2PCD(pcl::PointCloud<PointT> &cloud_target, pcl::PointCloud<PointT> &cloud_source, Eigen::Matrix<Scalar,4,4> transform, std::string filename)
{
    pcl::PointCloud<PointT> cloud_trans;
    pcl::transformPointCloud(cloud_source,cloud_trans, transform.template cast<float>());
    pcl::PointCloud<pcl::PointXYZRGB> cloud_comb;
    pcl::PointXYZRGB red(255, 0, 0);
    for (int i = 0; i < cloud_target.points.size(); ++i)
    {
        red.x = cloud_target.points[i].x;
        red.y = cloud_target.points[i].y;
        red.z = cloud_target.points[i].z;
        cloud_comb.points.push_back(red);
    }
    pcl::PointXYZRGB blue(10, 20, 200);
    for (int i = 0; i < cloud_trans.points.size(); ++i)
    {
        blue.x = cloud_trans.points[i].x;
        blue.y = cloud_trans.points[i].y;
        blue.z = cloud_trans.points[i].z;
        cloud_comb.points.push_back(blue);
    }
    cloud_comb.width = 1;
    cloud_comb.height = cloud_comb.points.size();
    cloud_comb.is_dense = false;
    pcl::io::savePCDFileBinary(filename, cloud_comb);
}

#endif