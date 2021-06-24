#include <ros/ros.h>
#include "lidar_test/lidarParse.hpp"
int main(int argc,char** argv){
    ros::init(argc,argv,"xyw_test");
    ros::NodeHandle nh;
    lidarParse lp(nh,argc,argv);
    ros::Rate f(10);
    while (ros::ok())
    {
        lp.toSeeGaussDistributionShape();
        ros::spinOnce();
        f.sleep();
    }

    
}