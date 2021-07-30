#include <ros/ros.h>
#include "lidar_test/lidarParse.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "xyw_lidar_test_node");
    ros::NodeHandle nh("~");
    xyw_lidar_test::lidarParse lp(nh, argc, argv);
    ros::Rate f(10);
    while (ros::ok())
    {

        ros::spinOnce();
        // lp.testNodeHandle();
        f.sleep();
    }
    // lp.toSeeGaussDistributionShape();
    ros::spin();
}