//general utilities
#include <cmath>
#include <fstream>
#include <string>
#include <time.h>
#include <stdlib.h>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <termios.h>
#include <iostream>
#include <stdio.h>


// ROS headers
#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_msgs/Bool.h>

// utilities
#include <trajectory_msgs/JointTrajectory.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/SVD>
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>


class trajectoryBuild
{
public:
	trajectoryBuild();

    ~trajectoryBuild();


    
private:
    //Node handle
    ros::NodeHandle n_;

    //Message Pub
    ros::Publisher pub1_, pub2_;
    lwr_controllers::PoseRPY msg_;

    // Message Sub
    ros::Subscriber sub_;
  

};