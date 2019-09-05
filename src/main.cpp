//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <std_srvs/Empty.h>

// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <ros/rate.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

#include <move_kuka.h>




int main(int argc, char** argv)
{
    ros::init(argc,argv,"main_node");
    ros::NodeHandle nh_main;

    std::cout << "\r\n\n\n\033[32m\033[1mSTART \033[0m" << std::endl; 

    move_kuka MK;

    ros::Rate r(300);

    ros::spinOnce();

    MK.homePosition();

//     while(ros::ok())
//     {
    MK.manager();
//         ros::spinOnce();
//         r.sleep();
//     }




    std::cout << "\r\n\n\n\033[32m\033[1mEXIT! \033[0m" << std::endl;
    return 0;
}