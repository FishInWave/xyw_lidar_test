// 自带
#include <chrono>
#include <random>
#include "omp.h"
#include <fstream>
#include <mutex>
#include <thread>
#include <deque>

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
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl_ros/point_cloud.h>
#include <dynamic_reconfigure/server.h>
#include "xyw_lidar_test/XYWLidarTestConfig.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>
// #include <nlink_parser/LinktrackAoaNodeframe0.h>
// #include <nlink_example/UwbFilter.h>
#include "time_utils.hpp"
#include <unistd.h>      // for path
#include <ros/package.h> // for path
#include "rslidar_utils.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include "KITTI_helper.hpp"
#include "so3/so3.hpp"
using namespace std;

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;
namespace xyw_lidar_test
{
    enum class LSQ{G,L,N};
    class lidarParse
    {
    private:
        dynamic_reconfigure::Server<XYWLidarTestConfig> *dsrv_;
        union TEST
        {
            int a;
            char b;
        };
        int b;
        int mark_threshold;
        bool setup_;

    protected:
        int a;
        mutex data_mutex;
        int shared_num;

    public:
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in;
        ros::Publisher vis_pub;
        ros::Publisher point1_pub, point2_pub;
        // ros::Publisher nlink_pub;
        ros::Subscriber points_sub, gpsfix, gpsvel;
        LSQ type_;
        int c;
        // 该函数证明动态调参程序会在初始化时被调用一次，move_base里这一步会完成默认参数的配置
        // 若rosparam里已经有这个参数，则config调用该参数的值，否则取cfg里的默认值
        void reconfigureCB(XYWLidarTestConfig &config, uint32_t level)
        {
            if (!setup_)
            {
                cout << "hi , it's the first time to run the reconfigureCB"
                     << ". mark_threshold: " << mark_threshold << endl;
                mark_threshold = config.mark_threshold;
                cout << "now: " << mark_threshold << endl;
                setup_ = true;
            }
            else
            {
                cout << "Its has been setuped. The mark_threshold is: " << mark_threshold << endl;
                mark_threshold = config.mark_threshold;
                cout << "now: " << mark_threshold << endl;
            }
        }
        // 1. rs pointxyzirt里的i是uint8，所以不可直接用fromROSMsg转换到pcl::PointXYZI,，应当修改rs的驱动
        // 2. 转换遵循ros->pointcloud2->pointT。前两者是二进制存储，第三个是结构明确的类，失败原因应该是按照pcl的intensity定义在RSPointXYZIRT
        //里是找不到intensity的，因为两个定义的类型不一样。
        //3. msg的时间戳是消息到达时间，点时间戳是硬件采集时间，基准不一样。
        void cloud_callback(sensor_msgs::PointCloud2ConstPtr msg)
        {
            static std::set<uint16_t>  ring_set;
            pcl::PointCloud<RsPointXYZIRT>::Ptr cloud(new pcl::PointCloud<RsPointXYZIRT>());
            pcl::PointCloud<pcl::PointXYZI>::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            // 直接根据类型去构造，要求msg和cloud对field的定义一致。
            pcl::fromROSMsg(*msg, *cloud);
            double begin_time = msg->header.stamp.toSec();

            for(auto point : *cloud){
                if(ring_set.find(point.ring) == ring_set.end()){
                    cout << point.ring << " angle: " << atan2(point.z,point.x)/M_1_PI*180 << endl;
                    ring_set.insert(point.ring);
                }
            }
            // LOG(INFO) << cloud->back().timestamp - begin_time << endl;
            // LOG(INFO) << cloud->back().timestamp - cloud->front().timestamp << endl;
            int index = 3000;
            auto point = cloud->at(index);
            // cout << "x: " << point.x << " y: " << point.y << " z: " << point.z << " intensity: " << (int)point.intensity
            //  << " ring: " << point.ring << " timestamp: " << point.timestamp
            // << endl;

            tic::TicToc t;
            t.tic();
            // for(auto point : *cloud){
            //     PointT p;
            //     p.x = point.x;
            //     p.y = point.y;
            //     p.z = point.z;
            //     p.intensity = (float) point.intensity;
            //     new_cloud.push_back(p);
            // }
            cout << t.toc() << endl;
            sensor_msgs::PointCloud2 newcloudmsg;
            pcl::toROSMsg(*new_cloud, newcloudmsg);
            point1_pub.publish(newcloudmsg);
        }
        void fix_cb(const sensor_msgs::NavSatFixPtr &msg)
        {
            // lock_guard<mutex> lock(data_mutex);
            ROS_INFO_STREAM("fix");
        }
        void vel_cb(const geometry_msgs::TwistStampedConstPtr &msg)
        {
            // lock_guard<mutex> lock(data_mutex);
            this_thread::sleep_for(chrono::milliseconds(1000));
            ROS_INFO_STREAM("vel");
        }
        lidarParse() = default;
        ~lidarParse() { 
            cout <<" hello" << endl;
            google::ShutdownGoogleLogging(); };
        lidarParse(ros::NodeHandle nh, int argc, char **argv):type_(LSQ::G)
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
            point1_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud1", 10);
            point2_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud2", 10);
            ros::NodeHandle pri_nh("~/move_base");
            // Note： 初始化dynamic_reconfigure::Server时的句柄不可使用全局句柄"/"，否则服务器里是没办法看到参数的
            dsrv_ = new dynamic_reconfigure::Server<XYWLidarTestConfig>(ros::NodeHandle("~/cfg"));
            dynamic_reconfigure::Server<XYWLidarTestConfig>::CallbackType cb = boost::bind(&lidarParse::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
            points_sub = nh.subscribe("/rslidar_points", 10, &lidarParse::cloud_callback, this);
            // nlink_pub = nh.advertise<nlink_parser::LinktrackAoaNodeframe0>("/nlink_linktrack_aoa_nodeframe0", 10);
            // nlink_example::UwbFilter msg;
            gpsfix = nh.subscribe("/gps/fix", 10, &lidarParse::fix_cb, this);
            gpsvel = nh.subscribe("/gps/vel", 10, &lidarParse::vel_cb, this);

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
            // testVectorTransform();
            // testNodeHandle();
            // testMultiThread();
            // testVectorMemoryManage();
            // testEigenConvert();
            // testIncludeOrder();
            // testTimeWasted();
            // testGetPath();
            // testTimeSDK();
            // testSlerp();
            // testEulerAndAngles();
            // testMultiSpin();
            // testKITTIsync();
            // testEigenAffineAndTransform();
            calculate();
            // testTictoc();
            // cout << testSwitch() << endl;
            // testEnumClass();
            // testSO3();
        }
        void testSO3(){
            Eigen::Vector3d v(20,4,15);
            Eigen::Quaterniond q_so3 = fast_gicp::so3_exp(v);
            Eigen::AngleAxisd angle_axisd(v.norm(),v.normalized());
            Eigen::Quaterniond q(angle_axisd);
            cout << "so3 vector: " << v.transpose()  
            << "\nq_so3: " << q_so3.coeffs().transpose() 
            << "\nq_eigen: " << q.coeffs().transpose() << endl;
        }
        void setType(LSQ type){
            type_ = type;
        }
        void testEnumClass(){
            switch(type_){
                case LSQ::G:
                {
                    cout << "G" << endl;
                    break;
                }
                case LSQ::L:
                {
                    cout << "L" << endl;
                    break;
                }
                case LSQ::N:
                {
                    cout << "N" << endl;
                    break;
                }
            }
        }
        // return则不需要break
        int testSwitch(){
            int icon = 1;
            switch(icon){
                case 1:
                    return 1;
                case 2:
                    return 2;
            }
            return 3;
        }
        //单位是ms
        void testTictoc(){
            tic::TicTocPart tictoc;
            tictoc.tic();
            this_thread::sleep_for(chrono::milliseconds(10));
            cout << tictoc.tocEnd() << endl;
        }
        void calculate()
        {
            // I2L
            // Eigen::Quaterniond q(0.999995291233, -0.000827921146993, 0.00112543266732, -0.0027323034592);
            Eigen::Matrix4d f,b;
            f << -0.995737 ,-0.0836142, -0.0389382, 375.15, 0.0827038, -0.996275, 0.024434, -142.186, -0.0408361, 0.0211095, 0.998943, 13.3424,0,0,0,1;
            b << -0.997021, -0.0658499 ,-0.0401468 ,374.553, 0.0648759, -0.997578 ,0.0251007 ,-142.154 ,-0.0417024, 0.0224214, 0.998878 ,13.3233,0,0,0,1;
            cout << f.inverse()*b << fixed << setprecision(10) << endl;
        }
        // 结论：Affine是4*3矩阵，可以通过.matix方法得到4*4矩阵，pcl的transform是根据传统的rpy计算方式得到的
        void testEigenAffineAndTransform()
        {
            float x = 1, y = 2, z = 3, r = 0.5, p = 0.6, ya = 0.7;
            Eigen::Affine3f af = pcl::getTransformation(x, y, z, r, p, ya);
            Eigen::Isometry3d is(af.matrix().cast<double>());
            cout << is.matrix() << endl;
            Eigen::Matrix4d m;
            m.block<3, 3>(0, 0) = Eigen::Matrix3d(Eigen::AngleAxisd(0.7, Eigen::Vector3d(0, 0, 1)) *
                                                  Eigen::AngleAxisd(0.6, Eigen::Vector3d(0, 1, 0)) *
                                                  Eigen::AngleAxisd(0.5, Eigen::Vector3d(1, 0, 0)));
            m.block<4, 1>(0, 3) = Eigen::Vector4d(1, 2, 3, 1);
            cout << "matrix:\n"
                 << m << endl;
        }
        // 结论：kitti的sync并没有被校准，依然需要运动补偿,sync和extract基本一样。且odom和sync的差别巨大，修正距离超过5cm的超过四分之三。
        void testKITTIsync()
        {
            std::string odompath = "/home/xyw/Downloads/0000002988.bin";
            std::vector<Eigen::Vector3f> odom_points, sync_points;
            pcl::PointCloud<pcl::PointXYZI> odom = getPointCloudFromBin(odompath, "odom", odom_points);
            pcl::PointCloud<pcl::PointXYZI> sync = getPointCloudFromBin("/home/xyw/Downloads/002988.bin", "sync", sync_points);
            odom.header.frame_id = "map";
            sync.header.frame_id = "map";

            if (odom.size() != sync.size())
            {
                cout << "the size of odom and sync is different." << endl;
            }
            int i = 0;
            int count = 0;
            int valid = 0;
            while (i++ < odom.size())
            {
                if (!isNanPoint(odom_points[i]) && !isNanPoint(sync_points[i]))
                {
                    float dis = disBetweenPoints(odom_points[i], sync_points[i]);
                    valid++;
                    if (dis > 0.05)
                    {
                        count++;
                        cout << dis << " " << flush;
                    }
                }
                else
                {
                    ROS_WARN_NAMED("warn", "nan");
                }
            }
            cout << count << "/" << valid << endl;
            //  sensor_msgs::PointCloud2 odommsg;
            //  pcl::toROSMsg(odom,odommsg);
            //  sensor_msgs::PointCloud2 syncmsg;
            //  pcl::toROSMsg(sync,syncmsg);

            point1_pub.publish(odom);
            point2_pub.publish(sync);
        }
        void testMultiSpin(){
            // fix和vel的消息同频率播放；
            // 若multispinner会使得cb被多线程回调，则可以看到fix 10Hz，vel 1Hz
            // 若spinner会使cb被单线程回调，则可以看到 fix和vel交替进行；
            // 实验结果与预期一致，说明使用multispinner会使得callback被多线程回调，互不干扰。
            // 补充说明：若在两个cb里加互斥量，则两者的频率无法保持10:1，原因是互斥锁的存在使得两个线程谁先被调用就会阻止对方；
            // 而多线程情况下，我们无法确定哪个线程会更早被唤起，即便他们都已经就绪。
        }
        // 经过测试，轴角在5°以内的话，与欧拉角的差距在0.05°左右。
        // 且与欧拉角的拆解顺序无关，轴角的第一分量与x轴欧拉角一致，其余分量依次类推。
        void testEulerAndAngles(){
            double v1,v2,v3,a;
            cout << "输入v1,v2,v3,a." << endl;
            cin >> v1 >> v2 >> v3 >> a;
            Eigen::Vector3d v(v1,v2,v3);
            Eigen::AngleAxisd angle(a,v.normalized()); // 轴角
            Eigen::Matrix3d m(angle); // 轴角的矩阵
            Eigen::Quaterniond q(angle); // 轴角的四元数
            Eigen::Matrix3d mm(q); // 轴角的四元数的矩阵
            auto euler1 = m.eulerAngles(0,1,2); // 按照 x,y,z的顺序解出欧拉角
            // Eigen::Matrix3d mm = Eigen::AngleAxisd(euler1[2],Eigen::Vector3d::UnitZ()).toRotationMatrix()* Eigen::AngleAxisd(euler1[1],Eigen::Vector3d::UnitY()).toRotationMatrix() 
            //     * Eigen::AngleAxisd(euler1[0],Eigen::Vector3d::UnitX()).toRotationMatrix();
            Eigen::Matrix3d mmm = Eigen::AngleAxisd(euler1[0],Eigen::Vector3d::UnitX()).toRotationMatrix()
                * Eigen::AngleAxisd(euler1[1],Eigen::Vector3d::UnitY()).toRotationMatrix()
                * Eigen::AngleAxisd(euler1[2],Eigen::Vector3d::UnitZ()).toRotationMatrix();
            auto euler2 = m.eulerAngles(2,1,0); // 按照 z,y,x的顺序解出欧拉角
            auto euler3 = m.eulerAngles(1,2,0); // 按照 y,z,x的顺序解出欧拉角
            auto euler4 = m.eulerAngles(0,1,0); // 按照 z,y,x的顺序解出欧拉角
            cout << "axles: " << v.normalized().transpose() * a /M_PI*180.0
            << "\n" << "euler1: " << euler1.transpose()/M_PI*180.0  
            << "\n" << "euler2: " << euler2.transpose()/M_PI*180.0 
            << "\n" << "euler3: " << euler3.transpose()/M_PI*180.0
            << "\n" << "euler4: " << euler4.transpose()/M_PI*180.0
            <<endl; 
            cout << "angles->matrix : \n" << m << "\n angle->q->matrix: \n" << mm << "\n matrix->euler->matrix: \n" << mmm << endl;   
        }
        // 与wiki一致，t表示与后者的接近程度。
        void testSlerp(){
            Eigen::Quaterniond q(1,0,0,0);
            Eigen::Quaterniond q2(0.5,0.6,0.6,0.6);
            q2.normalize();
            auto q3 = q2.slerp(1.0,q);
            cout << q3.coeffs() << endl;
        }
        //测试结果：ros 1631339391.843172550
        //     chrono  1631339391.843170881
        // 相差2ms，应该是指令执行的时间，因此可以认为ROS底层就是调的system_clock。
        void testTimeSDK(){
            //测试ROS time 和 std::chrono的计时是否一致
            ros::Time t = ros::Time::now();
            auto tp = std::chrono::system_clock::now();
            
            double chrono_time = (double) (std::chrono::duration_cast<chrono::duration<double>>(tp.time_since_epoch())).count();
            double ros_time = double(t.sec)+double(t.nsec)/1000000000.0;
            cout << "ros time: " << fixed <<  setprecision(9)<<ros_time<<" \n chrono_time: " << fixed << setprecision(9)
            << chrono_time << endl;

        }
        
        // 测试c++的路径获取,可以使用ros的package，也可使用系统自带的unistd（direct.h好像也有）
        void testGetPath(){
            string path = ros::package::getPath("xyw_lidar_test");
            cout << path << endl;
            char buffer[50];
            getcwd(buffer,50);
            cout << string(buffer) << endl;
        }
        // 测试tf2的耗时：全过程0.03ms一次，如果mutex仅进行简单赋值0.000156ms。
        // 顺便提一下，q.setRPY以及q.x()比起double之间的赋值要耗时的多，一个操作约0.005ms，约100多倍。
        // FIXME tf2::convert(q,geometry_msgs::Quaternion)没有成功
        void testTimeWasted()
        {
            geometry_msgs::PoseStamped robot_vel;
            nav_msgs::Odometry base_odom;
            base_odom.header.frame_id = "odom";
            base_odom.child_frame_id = "base_link";
            base_odom.header.stamp = ros::Time::now();
            base_odom.pose.pose.position.x = 1;
            base_odom.pose.pose.position.y = 1;
            base_odom.pose.pose.position.z = 1;
            base_odom.pose.pose.orientation.w = 1;
            base_odom.twist.twist.angular.z = 1;
            base_odom.twist.twist.linear.x = 1;
            base_odom.twist.twist.linear.y = 1;
            for (int i = 0; i < 10; ++i)
            {
                ros::Time t1 = ros::Time::now();

                robot_vel.header.frame_id = base_odom.child_frame_id;
                robot_vel.pose.position.x = base_odom.twist.twist.linear.x;
                robot_vel.pose.position.y = base_odom.twist.twist.linear.y;
                ros::Time t3 = ros::Time::now();
                tf2::Quaternion q;
                q.setRPY(0, 0, base_odom.twist.twist.angular.z);

                robot_vel.pose.orientation.w = q.w();
                robot_vel.pose.orientation.x = q.x();
                robot_vel.pose.orientation.y = q.y();
                robot_vel.pose.orientation.z = q.z();
                ros::Time t4 = ros::Time::now();
                robot_vel.header.stamp = ros::Time::now();
                ros::Time t2 = ros::Time::now();
                ros::Duration tt(t4 - t3);
                cout << "setRPY: " << tt.toSec() * 1000 << " ms" << endl;
                ros::Duration t(t2 - t1);
                cout << t.toSec() * 1000 << "ms" << endl;
            }
            geometry_msgs::Twist global_vel;
            cout << "4 line" << endl;
            for (int i = 0; i < 10; ++i)
            {
                ros::Time t1 = ros::Time::now();
                global_vel.linear.x = base_odom.twist.twist.linear.x;
                robot_vel.header.frame_id = base_odom.child_frame_id;
                global_vel.linear.y = base_odom.twist.twist.linear.y;

                global_vel.linear.z = base_odom.twist.twist.angular.z;
                ros::Time t2 = ros::Time::now();
                ros::Duration t(t2 - t1);
                cout << t.toSec() * 1000 << "ms" << endl;
            }
        }

        // 说明cmake里即便include顺序是先include，再catkin，依然优先访问了opt。
        // 无论使用""还是<>
        void testIncludeOrder()
        {
        }
        void testEigenConvert()
        {
            geometry_msgs::Pose pose;
            Eigen::Affine3d transform;
            tf::poseMsgToEigen(pose, transform);
            // cout << "pose msg: " << pose.position.x << " " << pose.position.y << " " << pose.position.z << endl;
            cout << transform.matrix() << endl;
        }

        //  只有vector需要用swap来释放内存空间
        // erase操作其实比pop慢，但调用次数多了以后，对栈的操作导致耗时增加
        void testVectorMemoryManage()
        {
            vector<int> v;
            v.reserve(1000);
            v.push_back(1);
            v.push_back(1);
            v.push_back(1);
            v.push_back(1);
            cout << v.size() << endl;     //4
            cout << v.capacity() << endl; //1000
            v.erase(v.begin());
            cout << v.size() << endl;     //3
            cout << v.capacity() << endl; //1000
            v.clear();
            cout << v.size() << endl;     //3
            cout << v.capacity() << endl; //1000
            vector<int>(v).swap(v);
            cout << v.size() << endl;     //4
            cout << v.capacity() << endl; //4
            vector<int>().swap(v);
            cout << v.size() << endl;     //0
            cout << v.capacity() << endl; //0
        }
        //测试多线程互斥{}
        // 线程1 使用unique_lock在{}锁住互斥量，每50ms修改值，持续20次，清醒后，再改变值，睡眠1s。
        // 线程2 每隔100ms读取一次这个量，读取前，需要上锁，打印，然后睡眠
        // 验证：lock_guard所谓的局部加锁，是针对{}吗？
        // 结果：{}有效
        void testMultiThread()
        {
            thread writeT(&lidarParse::writeThread, this);
            thread readT(&lidarParse::readThread, this);
            writeT.join();
            readT.join();
            cout << "This is main thread." << endl;
        }
        void readThread()
        {
            cout << "This is reading thread." << endl;
            chrono::milliseconds duration(100);
            this_thread::sleep_for(duration);
            {
                unique_lock<mutex> lock(data_mutex);

                for (size_t i = 0; i < 10; i++)
                {
                    cout << "read result is : " << shared_num << endl;
                    this_thread::sleep_for(duration);
                }
            }
            for (size_t i = 0; i < 10; i++)
            {
                cout << "Out read result is : " << shared_num << endl;
                this_thread::sleep_for(duration);
            }
            cout << "read thread shuts down." << endl;
        }
        void writeThread()
        {
            cout << "This is write Thread." << endl;
            shared_num = 0;
            chrono::milliseconds duration(50);

            unique_lock<mutex> lock(data_mutex);
            for (int i = 0; i < 20; i++)
            {
                cout << "shared_num = " << shared_num++ << endl;
                this_thread::sleep_for(duration);
            }

            for (int i = 0; i < 20; i++)
            {
                cout << "Out shared_num = " << shared_num++ << endl;
                this_thread::sleep_for(duration);
            }
            cout << "Write Thread shuts down" << endl;
        }
        // 测试ros::NodeHandle的用法以及ROS_DEBUG_STREAM
        void testNodeHandle()
        {
            ros::NodeHandle n;                              //全局句柄
            ros::NodeHandle pri_nh("~");                    //节点私有句柄
            ros::NodeHandle relative_nh("hi");              //一般相对句柄
            ros::NodeHandle global_nh("/hh");               // 一般全局句柄
            ros::NodeHandle second_nh(global_nh, "second"); // 二级句柄
            cout << "全局句柄： " << n.getNamespace()
                 << "\n 节点私有句柄： " << pri_nh.getNamespace()
                 << "\n 一般相对句柄： " << relative_nh.getNamespace()
                 << "\n 一般全局句柄： " << global_nh.getNamespace()
                 << "\n 二级句柄： " << second_nh.getNamespace() << endl;
            ROS_DEBUG_STREAM("It's a debug msg");
            ROS_DEBUG_STREAM_NAMED("hello", "Its name is hello");
        }

        // 测试点云拼接
        void testVectorTransform()
        {
            cout << "protected a= " << a << endl;
            cout << "private b = " << b << endl;
            cout << "public c = " << c << endl;
            Eigen::Matrix4f T1, T2, T3;
            T1.setIdentity();
            T1(0, 3) = 10;
            T2.setIdentity();
            T2(0, 3) = 20;
            T3.setIdentity();
            T3(0, 3) = 30;
            vector<PointCloud> vec;
            vec.push_back(*cloud_in);
            vec.push_back(*cloud_in);
            vec.push_back(*cloud_in);
            vector<Eigen::Matrix4f> vec_T;
            vec_T.push_back(T1);
            vec_T.push_back(T2);
            vec_T.push_back(T3);
            PointCloud temp;
            temp.reserve(3 * cloud_in->size());
            for (int i = 0; i < 3; ++i)
            {
                pcl::transformPointCloud(vec[i], vec[i], vec_T[i]);
                temp = temp + vec[i];
            }
            sensor_msgs::PointCloud2 pc;
            temp.header.frame_id = "map";
            // point_pub.publish(temp);
            cout << "一个点云：" << cloud_in->size() << endl;
            cout << "三个点云：　" << temp.size() << endl;
            pcl::io::savePCDFile("/tmp/pc.pcd", temp);
        }
        void testOfstream()
        {
            std::ofstream ofs;
            ofs.open("/home/xyw/Documents/00.txt", std::ios::out);
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
        // float rgb和 int rgba的首地址其实一样
        //TODO 为什么r b的地址无法得到？-》rgb address: 0x7fffb149f530 rgba address: 0x7fffb149f530 r address: ��qU b address:
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
            LOG(INFO) << "a： " << int(a);
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
        // 为了debug LiTAMIN2创造的方法
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
        // 合同变换的确不影响矩阵的正定性，且得到了一种新的求协方差的方法
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
        // rotation 按行排列
        void testCeresTransform()
        {
            Eigen::Vector4d v(3, 4, 5, 6); // 假设 w在前
            v.normalize();
            double q[4] = {v[0], v[1], v[2], v[3]}; // 假设w在后

            Eigen::Quaterniond quat(v[0], v[1], v[2], v[3]);
            Eigen::Map<Eigen::Quaterniond> quat_map(q);
            Eigen::Matrix3d map_matrix = quat.matrix();

            std::cout << "v: " << v.transpose() << " q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << std::endl;
            std::cout << "quat.w " << quat.w() << " v[0] " << v[0] << std::endl;
            std::cout << "map matrix: " << map_matrix << std::endl;
            double Rotation[9];
            ceres::QuaternionToRotation(q, Rotation);
            Eigen::Matrix3d m = quat.matrix();
            LOG(INFO) << "ceres: "
                      << "\n"
                      << Rotation[0] << " " << Rotation[1] << " " << Rotation[2] << "\n"
                      << Rotation[3] << " " << Rotation[4] << " " << Rotation[5] << "\n"
                      << Rotation[6] << " " << Rotation[7] << " " << Rotation[8] << "\n"
                      << "Eigen : "
                      << "\n"
                      << m << "\n";
        }
        // 测试Eigen的norm
        // norm返回二范数值，normlized返回归一化后的m的拷贝，normlize对m本体进行归一化
        void testMatrixNorm()
        {
            Eigen::Matrix3d m;
            m.setIdentity();
            m = m * 2;
            LOG(INFO) << "\n"
                      << m
                      << "\n m.norm()： "
                      << "\n"
                      << m.norm()
                      << "\n m/m.norm(): "
                      << "\n"
                      << m / m.norm()
                      << "\n m.normlized(): "
                      << "\n"
                      << m.normalized()
                      << "\n m after m.normalized: " << m;
            m.normalize();
            LOG(INFO) << "\n m after m.normlize(): " << m;
        }
        // 测试Eigen的泛型初始化
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
        // 答案正确
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
        // Vector和Array只是类型的区别，数值上一样
        // 获得4f还是3f，仅仅是出于齐次计算是否方便考虑，第4位总是1
        void testGetVector4fMap()
        {
            LOG(INFO) << "Vector 4f " << cloud_in->at(20).getVector4fMap();
            LOG(INFO) << "Vector 3f " << cloud_in->at(20).getVector3fMap();
            LOG(INFO) << "Arrary 4f " << cloud_in->at(20).getArray4fMap();
            LOG(INFO) << "Arrary 3f " << cloud_in->at(20).getArray3fMap();
            // Point输出是行向量
            LOG(INFO) << "point info " << cloud_in->at(20);
        }
        // 对矩阵元素进行操作可以考虑调用array()方法
        void testEigenArrayAbs()
        {
            Eigen::Matrix3f m(Eigen::Matrix3f::Identity());
            m(0, 1) = 5;
            m(1, 0) = -4;
            LOG(INFO) << "abs: " << m.array().abs();
            LOG(INFO) << "matrix: " << m;

            Eigen::Vector3d v(3,4,-10);
            cout << v.maxCoeff() << endl;
            cout << v.array().abs().maxCoeff() << endl;
        }
        // Isometry3d可以和vector3d进行运算，且float类型可以通过cast转换为double
        void testEigenVectorDir()
        {
            Eigen::Vector3d p(1, 2, 3);
            Eigen::Vector3d q(4, 5, 6);
            Eigen::Matrix4f m(Eigen::Matrix4f::Identity());
            m.col(3) << 5, 5, 5, 1;
            LOG(INFO) << m;
            // Eigen::Isometry3d iso = static_cast<Eigen::Isometry3d>(m);
            Eigen::Isometry3d ii = Eigen::Isometry3d(m.cast<double>());
            Eigen::Vector3d pq = p - ii * q;
            LOG(INFO) << pq;
        }
        // transform不会改变点云序号
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
        // 结果：PCL的transform耗时和OMP的耗时相差无几
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
            time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t4 - t3);
            LOG(INFO) << "OMP implement for large loop spend time(should be 100 times that of B):\n "
                      << time_used.count() * 1000 << " ms";
        }

    };
}