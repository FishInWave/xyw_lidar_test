// 自带
#include <chrono>
#include <random>
#include "omp.h"
#include <fstream>

#include <glog/logging.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/common/shapes.h>
#include <pcl/point_types.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <Eigen/Dense>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
using namespace std;
using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
class lidarParse
{
private:
    union TEST
    {
        int a;
        char b;
    };
    int b;
protected:
    int a;
public:
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in;
    ros::Publisher vis_pub;
    ros::Publisher point_pub;
    int c;
    lidarParse(ros::NodeHandle nh, int argc, char **argv)
    {
        cloud_in.reset(new pcl::PointCloud<pcl::PointXYZI>);
        // 初始化GLOG
        FLAGS_log_dir = "/home/xyw/catkin_ws/src/xyw_lidar_test/log";
        google::InitGoogleLogging(argv[0]); // 以程序命名
        google::SetLogDestination(google::GLOG_INFO, "/home/xyw/catkin_ws/src/xyw_lidar_test/log/INFO_");
        google::SetStderrLogging(google::GLOG_INFO);
        google::SetLogFilenameExtension("log_");
        FLAGS_colorlogtostderr = true;          // Set log color
        FLAGS_logbufsecs = 0;                   // Set log output speed(s)
        FLAGS_max_log_size = 1024;              // Set max log file size
        FLAGS_stop_logging_if_full_disk = true; // If disk is full
        // load a pcd file
        pcl::io::loadPCDFile("/home/xyw/catkin_ws/src/xyw_lidar_test/res/000027.pcd", *cloud_in);

        LOG(INFO) << "load " << cloud_in->size() << " points";
        vis_pub = nh.advertise<visualization_msgs::MarkerArray>("vis_marker", 10);
        point_pub = nh.advertise<sensor_msgs::PointCloud2>("map", 10);
        // testKDtree();
        // testPointCloudTransform();
        // testTransformIndex();
        // testEigenVectorDir();
        // testEigenArrayAbs();
        // testGetVector4fMap();
        // testGauss();
        // testEigenTemplate<double>();
        // testMatrixNorm();
        // testAngleAxlesAndEulerAngle();
        // testCeresTransform();
        // testLLT();
        // testCovAndPositiveDefinite();
        // testIsConverged();
        // toSeeGaussDistributionShape();
        // toTestPCLRGBA();
        // toSeeWhichEnd();
        // testOfstream();
        testVectorTransform();
    }
    void testVectorTransform(){
        cout << "protected a= " << a << endl;
        cout << "private b = " << b << endl;
        cout << "public c = " << c << endl;
        Eigen::Matrix4f T1,T2,T3;
        T1.setIdentity();
        T1(0,3) = 10;
        T2.setIdentity();
        T2(0,3) = 20;
        T3.setIdentity();
        T3(0,3) = 30;
        vector<PointCloud> vec;
        vec.push_back(*cloud_in);
        vec.push_back(*cloud_in);
        vec.push_back(*cloud_in);
        vector<Eigen::Matrix4f> vec_T;
        vec_T.push_back(T1);
        vec_T.push_back(T2);
        vec_T.push_back(T3);
        PointCloud temp;
        temp.reserve(3*cloud_in->size());
        for(int i = 0;i < 3;++i){
            pcl::transformPointCloud(vec[i],vec[i],vec_T[i]);
            temp = temp + vec[i];
        }
        sensor_msgs::PointCloud2 pc;
        temp.header.frame_id = "map";
        point_pub.publish(temp);
        cout << "一个点云：" << cloud_in->size() << endl;
        cout << "三个点云：　" << temp.size() << endl;
        pcl::io::savePCDFile("/tmp/pc.pcd",temp);
        
    }
    void testOfstream(){
        std::ofstream ofs;
        ofs.open("/home/xyw/Documents/00.txt",std::ios::out);
        ofs << "hello, ofs" << std::endl;
        ofs.close();
    }
    // 电脑是小端模式
    void toSeeWhichEnd()
    {
        lidarParse::TEST test;
        test.a = 1;
        if (test.b == 1)
        {
            cout << "small endian." << endl;
        }
        else
        {
            cout << "big endian." << endl;
        }
    }
    // pcl内部实际上是按照bgr在存储的。
    // float rgb的低字节依次是BGR
    // float rgb和 int rgba的首地址其实一样，TODO 为什么r b的地址无法得到？
    // float 直接用memcpy复制到一个数组，这会使得低字节的b等于数组的[0]元素
    // int 类型的话，由于直接一次性取了32位，因此高字节是A，接下来分别是RGB。
    void toTestPCLRGBA()
    {
        pcl::PointXYZRGBA p;
        p.r = 255;
        p.g = 0;
        p.b = 0;
        LOG(INFO) << "rgb address: " << &p.rgb << " rgba address: " << &p.rgba << " r address: " << &p.r
                  << " b address: " << &p.b;
        int rgba = p.rgba;
        uint8_t r = ((int)rgba >> 16) & 0x0000ff;
        uint8_t g = ((int)rgba >> 8) & 0x0000ff;
        uint8_t b = ((int)rgba) & 0x0000ff;
        uint8_t a = (rgba) >> 24 & 0x0000ff;
        LOG(INFO) << "a? " << int(a);  
        unsigned char color[sizeof(float)];
        memcpy(color, &p.rgb, sizeof(float));
        LOG(INFO) << "p.rgb: " << p.rgb << " p.rgba: " << p.rgba;
        LOG(INFO) << "p.bgra: " << p.getBGRAVector4cMap().cast<int>().transpose();
        LOG(INFO) << "p.rgba vector: " << p.getRGBAVector4i().cast<int>().transpose();
        LOG(INFO) << "color1 B: " << (int)color[0] << " color 2 G: " << (int)color[1] << " color 3 R: " << (int)color[2] << " A: " << (int)color[3];
        LOG(INFO) << "r: " << int(r) << " g: " << int(g) << " b: " << int(b);
    }
    // 看看高斯分布
    void toSeeGaussDistributionShape()
    {
        // pcl1.8暂时没有椭圆
        // pcl::visualization::PCLVisualizer::Ptr Vis(new pcl::visualization::PCLVisualizer("Gauss viewer"));

        // 测试自己制造的点云，说明可视化时还是直接按照相应顺序比较好。
        // int number_near = 50;
        // pcl::KdTreeFLANN<PointT> kd;
        // kd.setInputCloud(cloud_in);
        // std::vector<int> index(number_near);
        // std::vector<float> dis(number_near);
        // PointCloud::Ptr near_cloud(new PointCloud());
        // double s = 0.5;
        // for (size_t i = 0; i < number_near; i++)
        // {
        //     PointT p;
        //     switch (i % 3)
        //     {
        //     case 0:
        //         p.x = 1 + 2 * s;
        //         break;
        //     case 1:
        //         p.x = 1 + 4 * s;
        //         break;
        //     case 2:
        //         p.x = 1 + 6 * s;
        //         break;
        //     }

        //     p.z = 1 + 0.1 * s;
        //     s *= -1;
        //     p.y = i;
        //     near_cloud->push_back(p);
        // }
        // Eigen::AngleAxisf ad(M_PI_4,Eigen::Vector3f(0,0,1));
        // Eigen::Matrix4f trans(Eigen::Matrix4f::Identity());
        // trans.topLeftCorner(3,3) = ad.matrix();
        // pcl::transformPointCloud(*near_cloud,*near_cloud,trans);
        // pcl::VoxelGridCovariance<pcl::PointXYZI> vf;
        // vf.setInputCloud(near_cloud);
        // vf.setLeafSize(20.0, 20.0, 20.0);
        // PointCloud::Ptr vf_point(new PointCloud());
        // vf.filter(*vf_point);
        // auto leaf = vf.getLeaf(near_cloud->at(12));
        // visualization_msgs::MarkerArray marker_array;
        // Eigen::Vector3d p_mean = leaf->getMean();
        // Eigen::Matrix3d p_cov(Eigen::Matrix3d::Identity());
        // p_cov = leaf->getCov();
        // Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(p_cov);
        // auto D = eigensolver.eigenvalues();
        // auto V = eigensolver.eigenvectors();
        // std::cout << "mean: " << p_mean.transpose() << std::endl;
        // std::cout << "cov: "
        //           << "\n"
        //           << p_cov << std::endl;
        // std::cout << "D : " << eigensolver.eigenvalues().transpose() << "\n V: \n"
        //           << eigensolver.eigenvectors() << std::endl;
        // visualization_msgs::Marker marker;
        // marker.header.frame_id = "map";
        // marker.header.stamp = ros::Time();
        // marker.id = 0;
        // marker.type = visualization_msgs::Marker::SPHERE;
        // marker.action = visualization_msgs::Marker::ADD;
        // marker.pose.position.x = p_mean[0];
        // marker.pose.position.y = p_mean[1];
        // marker.pose.position.z = p_mean[2];
        // Eigen::Matrix3d m(V);
        // // m.block<3, 1>(0, 0) = V.block<3, 1>(0, 2);
        // // m.block<3, 1>(0, 2) = V.block<3, 1>(0, 0);
        // std::cout <<"m: \n" << m << std::endl;
        // Eigen::Quaterniond quat(m);
        // quat.normalize();
        // marker.pose.orientation.x = quat.x();
        // marker.pose.orientation.y = quat.y();
        // marker.pose.orientation.z = quat.z();
        // marker.pose.orientation.w = quat.w();
        // marker.scale.x = sqrt(D(0)); // scale是指沿着x轴的放缩量。
        // marker.scale.y = sqrt(D(1));
        // marker.scale.z = sqrt(D(2));
        // marker.color.a = 0.5; // Don't forget to set the alpha!
        // marker.color.r = 0.0;
        // marker.color.g = 1.0;
        // marker.color.b = 0.0;
        // marker.lifetime = ros::Duration();
        // marker_array.markers.push_back(marker);

        // 测试KITTI点云
        pcl::VoxelGridCovariance<pcl::PointXYZI> vf;
        vf.setInputCloud(cloud_in);
        vf.setLeafSize(8.0, 8.0, 8.0);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
        vf.filter(*cloud_ptr, true);
        int size = cloud_ptr->size();
        visualization_msgs::MarkerArray marker_array;

        for (int i = 0; i < size; i++)
        {
            auto leaf = vf.getLeaf(cloud_ptr->at(i));
            Eigen::Vector3d p_mean = leaf->getMean();
            Eigen::Matrix3d p_cov = leaf->getCov();
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigensolver(p_cov);
            auto D = eigensolver.eigenvalues();
            auto V = eigensolver.eigenvectors();
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = p_mean[0];
            marker.pose.position.y = p_mean[1];
            marker.pose.position.z = p_mean[2];
            Eigen::Matrix3d m(V);
            Eigen::Quaterniond quat(m);
            marker.pose.orientation.x = quat.x();
            marker.pose.orientation.y = quat.y();
            marker.pose.orientation.z = quat.z();
            marker.pose.orientation.w = quat.w();
            marker.scale.x = sqrt(D(0));
            marker.scale.y = sqrt(D(1));
            marker.scale.z = sqrt(D(2));
            marker.color.a = 0.5; // Don't forget to set the alpha!
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration();
            marker_array.markers.push_back(marker);
        }
        cloud_in->header.frame_id = "map";
        // point_pub.publish(cloud_in);
        // vis_pub.publish(marker_array);
    }

    void testIsConverged()
    {
        Eigen::Matrix4d m;
        m << 1, -2.12182e-19, 0, 5.55112e-17,
            6.54968e-19, 1, 0, -1.38778e-17,
            0, -10842e-19, 1, 0,
            0, 0, 0, 1;
        Eigen::Isometry3d delta(m);

        Eigen::Matrix3d R = delta.linear() - Eigen::Matrix3d::Identity();
        Eigen::Vector3d t = delta.translation();

        Eigen::Matrix3d r_delta = 1.0 / 2e-3 * R.array().abs();
        Eigen::Vector3d t_delta = 1.0 / 5e-4 * t.array().abs();
        std::cout << r_delta << "\n"
                  << t_delta.transpose() << std::endl;
        std::cout << std::max(r_delta.maxCoeff(), t_delta.maxCoeff()) << std::endl;
    }
    // 合同变换的确不影响矩阵的正定性，且求协方差可以是get3fmap
    void testCovAndPositiveDefinite()
    {
        pcl::search::KdTree<PointT> kdtree;
        kdtree.setInputCloud(cloud_in);
        std::vector<int> k_indices;
        std::vector<float> k_squared_distances; //是平方距离
        int k = 20;
        kdtree.nearestKSearch(cloud_in->at(1000), k, k_indices, k_squared_distances);
        Eigen::Matrix<double, 3, -1> neighbors(3, k);
        for (int j = 0; j < k_indices.size(); j++)
        {
            neighbors.col(j) = cloud_in->at(k_indices[j]).getVector3fMap().template cast<double>();
        }

        neighbors.colwise() -= neighbors.rowwise().mean().eval();
        Eigen::Matrix3d cov = neighbors * neighbors.transpose() / k;
        LOG(INFO) << "\n"
                  << "cov: \n"
                  << cov;
        Eigen::Matrix3d L_cov = cov.llt().matrixL();
        LOG(INFO) << "\n cov_LLT: \n"
                  << L_cov * L_cov.transpose();

        Eigen::Quaterniond q(0.5, 0.6, 0.5, 0.1);
        q.normalize();
        Eigen::Matrix3d isome(Eigen::Matrix3d::Identity());
        isome.block<3, 3>(0, 0) = q.toRotationMatrix();

        Eigen::Matrix3d maha = isome * cov * isome.transpose();
        LOG(INFO) << "\n"
                  << "maha: \n"
                  << maha;
        Eigen::Matrix3d L_maha = maha.llt().matrixL();
        LOG(INFO) << "\n maha_LLT: \n"
                  << L_maha * L_maha.transpose();
    }
    // LLT=UTU
    void testLLT()
    {
        Eigen::Matrix3d m;
        m << 4, -1, 2, -1, 6, 0, 2, 0, 5;
        Eigen::Matrix3d U = m.llt().matrixU();
        Eigen::Matrix3d L = m.llt().matrixL();
        LOG(INFO) << "UTU\n"
                  << U.transpose() * U << " LLT\n"
                  << L * L.transpose()
                  << " m \n"
                  << m;
    }
    // rotation 按行排列，使用Eigen::map时，
    void testCeresTransform()
    {
        Eigen::Vector4d v(3, 4, 5, 6); // 假设 w在前
        v.normalize();
        double q[2] = {v[0], v[1]}; // 假设w在后
        Eigen::Map<Eigen::Vector4d> vec(q);
        std::cout << "vec: " << vec.transpose() << std::endl;
        std::cout << *q << " " << *(q + 2) << std::endl;
        // Eigen::Quaterniond quat(v[0], v[1], v[2], v[3]);
        // Eigen::Map<Eigen::Quaterniond> quat_map(q);
        // Eigen::Matrix3d map_matrix = quat.matrix();

        // std::cout << "v: " << v.transpose() << " q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
        // std::cout << "quat.w " << quat.w() << " v[0] " << v[0] << std::endl;
        // std::cout << "map matrix: " << map_matrix << std::endl;
        // double Rotation[9];
        // ceres::QuaternionToRotation(q, Rotation);
        // Eigen::Matrix3d m = quat.matrix();
        // LOG(INFO) << "ceres: "
        //           << "\n"
        //           << Rotation[0] << " " << Rotation[1] << " " << Rotation[2] << "\n"
        //           << Rotation[3] << " " << Rotation[4] << " " << Rotation[5] << "\n"
        //           << Rotation[6] << " " << Rotation[7] << " " << Rotation[8] << "\n"
        //           << "Eigen : "
        //           << "\n"
        //           << m << "\n";
    }
    //实验证明：小角近似是针对欧拉角的，而不是轴角
    void testAngleAxlesAndEulerAngle()
    {
        Eigen::Vector3d axles(3, 4, 5);
        Eigen::Vector3d normed = axles.normalized();
        double angle = M_PI / 180 / 40; // 0.5°
        Eigen::AngleAxisd ag(angle, normed);
        Eigen::Matrix3d om(ag.matrix());
        Eigen::Vector3d abc_normed = sqrt(angle) * normed;
        Eigen::Vector3d abc = sqrt(angle) * axles;
        LOG(INFO) << "agnle: " << angle << " axle： " << axles.transpose();
        // 没有归一化的矩阵
        Eigen::AngleAxisd Rx(abc[0], Eigen::Vector3d(1, 0, 0));
        Eigen::AngleAxisd Ry(abc[1], Eigen::Vector3d(0, 1, 0));
        Eigen::AngleAxisd Rz(abc[2], Eigen::Vector3d(0, 0, 1));

        // 归一化以后的欧拉角
        Rx = Eigen::AngleAxisd(abc_normed[0], Eigen::Vector3d(1, 0, 0));
        Ry = Eigen::AngleAxisd(abc_normed[1], Eigen::Vector3d(0, 1, 0));
        Rz = Eigen::AngleAxisd(abc_normed[2], Eigen::Vector3d(0, 0, 1));
        Eigen::Matrix3d mm = (Rz.matrix().transpose()) * (Ry.matrix().transpose()) * (Rx.matrix().transpose());
        Eigen::Matrix3d m = Rx.matrix() * Ry.matrix() * Rz.matrix();

        LOG(INFO) << "AngleAxisd matrix: "
                  << "\n"
                  << om << "\n"
                  << "AngleAxisd vector: " << abc.transpose() << "\n"
                  << "Angle Axis normed: " << abc_normed.transpose() << "\n"
                  << "Euler Angle matrix(with angleaxis vector): "
                  << "\n"
                  << m << "\n"
                  << " Euler Angle matrix: "
                  << "\n"
                  << m * mm;
    }
    void testMatrixNorm()
    {
        Eigen::Matrix3d m;
        m.setIdentity();
        m = m * 2;
        LOG(INFO) << "\n"
                  << m
                  << "m.norm()： "
                  << "\n"
                  << m.norm()
                  << " m/m.norm(): "
                  << "\n"
                  << m / m.norm()
                  << " m.normlized(): "
                  << "\n"
                  << m.normalized();
        m.normalize();
        LOG(INFO) << " m after m.normlize(): " << m;
    }
    template <typename T>
    void testEigenTemplate()
    {
        Eigen::Matrix3d m(Eigen::Matrix3d::Identity());
        Eigen::Matrix<T, 3, 3> n(m);
        LOG(INFO) << "m: "
                  << "\n"
                  << m;
        LOG(INFO) << "n: "
                  << "\n"
                  << n;
        Eigen::Vector3d v1(3, 4, 5);
        Eigen::Matrix<T, 3, 1> v2(v1);
        Eigen::Matrix<T, 3, 1> v3{T(v1.x()), T(v1.y()), T(v1.z())};
        LOG(INFO) << "v1: " << v1.transpose() << " v2: " << v2.transpose() << " v3: " << v3.transpose();
    }
    // 测试高斯分布乘法里的协方差是不是RCRT
    void testGauss()
    {
        // 生成正态分布点云，均值为（3,4,5），方差分别为0.5,2,4;
        std::normal_distribution<float> gaussX(3, 10);
        std::normal_distribution<float> gaussY(60, 20);
        std::normal_distribution<float> gaussZ(16, 40);
        std::default_random_engine root;
        cloud_in->clear();
        cloud_in->reserve(1000);
        Eigen::Matrix<float, 4, -1> m(4, 1000);
        for (int i = 0; i < 1000; i++)
        {
            pcl::PointXYZI p;
            p.x = gaussX(root);
            p.y = gaussY(root);
            p.z = gaussZ(root);
            cloud_in->push_back(p);
            m.col(i) = p.getVector4fMap();
        }
        m.colwise() -= m.rowwise().mean().eval();
        Eigen::Matrix4f cov = m * m.transpose() / 1000;
        Eigen::Isometry3f iso(Eigen::Isometry3f::Identity());
        Eigen::AngleAxisf v(M_PI / 4, Eigen::Vector3f(0, 0, 1));
        iso = v.toRotationMatrix();
        iso.translation() = Eigen::Vector3f(0.5, 0.6, 0.7);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::transformPointCloud(*cloud_in, *cloud_out, iso.matrix());
        Eigen::Matrix<float, 4, -1> n(4, 1000);
        for (int j = 0; j < 1000; j++)
        {
            n.col(j) = cloud_out->at(j).getVector4fMap();
        }
        n.colwise() -= n.rowwise().mean().eval();
        Eigen::Matrix4f cov_n = n * n.transpose() / 1000;
        Eigen::Matrix4f Trans = iso.matrix();
        LOG(INFO) << "origin cov: ";
        LOG(INFO) << cov.block<3, 3>(0, 0);
        LOG(INFO) << "transformed cov:";
        LOG(INFO) << cov_n.block<3, 3>(0, 0);
        LOG(INFO) << "AnAT:";
        LOG(INFO) << (Trans * cov * (Trans.transpose())).block<3, 3>(0, 0);
        LOG(INFO) << "m+AnAT:";
        Eigen::Matrix3f M_ANAT = (cov_n + iso.matrix() * cov * iso.matrix().transpose()).block<3, 3>(0, 0);
        Eigen::Matrix3f Mahalanobis = M_ANAT.inverse();
        LOG(INFO) << Mahalanobis;
        LOG(INFO) << Mahalanobis.transpose() * Mahalanobis;
    }

    void testGetVector4fMap()
    {
        LOG(INFO) << "Vector 4f " << cloud_in->at(20).getVector4fMap();
        LOG(INFO) << "Vector 3f " << cloud_in->at(20).getVector3fMap();
        LOG(INFO) << "Arrary 4f " << cloud_in->at(20).getArray4fMap();
        LOG(INFO) << "Arrary 3f " << cloud_in->at(20).getArray3fMap();
        LOG(INFO) << "point info " << cloud_in->at(20);
    }

    void testEigenArrayAbs()
    {
        Eigen::Matrix3f m(Eigen::Matrix3f::Identity());
        m(0, 1) = 5;
        m(1, 0) = -4;
        LOG(INFO) << "abs: " << m.array().abs();
        LOG(INFO) << "matrix: " << m;
    }
    void testEigenVectorDir()
    {
        Eigen::Vector3d p(1, 2, 3);
        Eigen::Vector3d q(4, 5, 6);
        Eigen::Matrix4f m(Eigen::Matrix4f::Identity());
        m.col(3) << 5, 5, 5;
        // Eigen::Isometry3d iso = static_cast<Eigen::Isometry3d>(m);
        Eigen::Isometry3d ii = Eigen::Isometry3d(m.cast<double>());
        Eigen::Vector3d pq = p - ii * q;
        LOG(INFO) << pq;
    }

    void testTransformIndex()
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Matrix4f m;
        m << 1, 0, 0, 0.2, 0, 1, 0, 0, 0, 0, 1, 0;
        pcl::transformPointCloud(*cloud_in, *cloud_out, m);
        int count = 0;
        for (auto i = 100; i < 120; i++)
        {
            pcl::PointXYZI p = cloud_in->at(i);
            pcl::PointXYZI q = cloud_out->at(i);
            if ((p.x + 0.2 - q.x) <= 0.00001)
            {
                count++;
            }
            LOG(INFO) << "origin: [" << p.x << "," << p.y << "," << p.z << "] "
                      << "transformed: [" << q.x << "," << q.y << "," << q.z << "] "
                      << count;
        }
    }

    void testKDtree()
    {
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        pcl::search::KdTree<pcl::PointXYZI> tree;
        tree.setInputCloud(cloud_in);
        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "1 Kd-tree spend time: " << time_used.count() << " s";
    }
    // 测试OMP For
    void testPointCloudTransform()
    {
        Eigen::Matrix3f rotate(Eigen::AngleAxisf(M_PI / 4, Eigen::Vector3f(0, 0, 1)));
        Eigen::Matrix4f m;
        m.block<3, 3>(0, 0) = rotate;
        m.block<3, 1>(0, 3) << 1, 1, 1;
        LOG(INFO) << m;
        pcl::PointCloud<pcl::PointXYZI> cloud_out;
        // pcl trans
        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
        for (int i = 0; i < 1; i++)
        {
            pcl::transformPointCloud(*cloud_in, cloud_out, m);
        }

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "PCL implement spend time: " << time_used.count() * 1000 << " ms";
        // omp trans
        t1 = chrono::steady_clock::now();
        omp_set_num_threads(std::min(6, omp_get_max_threads()));
        auto mm = static_cast<Eigen::Affine3f>(m);
        auto size = cloud_in->size();
        for (int i = 0; i < 1; i++)
        {
#pragma omp parallel for //Multi-thread
            for (int i = 0; i < size; i++)
            {
                pcl::PointXYZI point;
                point = pcl::transformPoint(cloud_in->at(i), mm);
            }
        }

        t2 = chrono::steady_clock::now();
        time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "OMP implement for small loop spend time: " << time_used.count() * 1000 << " ms";
        // omp trans
        chrono::steady_clock::time_point t3 = chrono::steady_clock::now();

#pragma omp parallel for //Multi-thread
        for (int i = 0; i < 100; i++)
        {

            for (int i = 0; i < size; i++)
            {
                pcl::PointXYZI point;
                point = pcl::transformPoint(cloud_in->at(i), mm);
            }
        }

        chrono::steady_clock::time_point t4 = chrono::steady_clock::now();
        time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t3 - t4);
        LOG(INFO) << "OMP implement for large loop spend time: " << time_used.count() * 1000 << " ms";
    }
    lidarParse() = default;
    ~lidarParse() { google::ShutdownGoogleLogging(); };
};