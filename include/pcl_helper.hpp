#ifndef PCL_HELPER_HPP
#define PCL_HELPER_HPP
#include "omp.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
template <typename PointT>
void calculate_covariances(typename boost::shared_ptr<const pcl::PointCloud<PointT>> &cloud,
                           std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &covariances,int k_correspondences_, int num_threads_)
{
    pcl::search::KdTree<PointT> kdtree;
    kdtree.setInputCloud(cloud);
    covariances.resize(cloud->size());
// TODO 计算协方差时，没有进行距离判断，而是直接取了最近的10个点，这会导致scan-to-scan时劣化。
#pragma omp parallel for num_threads(num_threads_) schedule(guided, 8)
    for (int i = 0; i < cloud->size(); i++)
    {
        std::vector<int> k_indices;
        std::vector<float> k_sq_distances;
        kdtree.nearestKSearch(cloud->at(i), k_correspondences_, k_indices, k_sq_distances);

        Eigen::Matrix<double, 4, -1> neighbors(4, k_correspondences_);
        for (int j = 0; j < k_indices.size(); j++)
        {
            neighbors.col(j) = cloud->at(k_indices[j]).getVector4fMap().template cast<double>();
        }

        neighbors.colwise() -= neighbors.rowwise().mean().eval();
        Eigen::Matrix4d cov = neighbors * neighbors.transpose() / (k_correspondences_-1);

        covariances[i] = cov;
    }
}
#endif