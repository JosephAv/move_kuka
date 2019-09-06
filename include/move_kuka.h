#include <eigen3/Eigen/Eigen>
#include <Eigen/Geometry>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdlib>
#include <csignal>
#include <string>
#include <boost/algorithm/string.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <eigen_conversions/eigen_kdl.h>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>


#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/package.h>
#include <ros/package.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Pose.h>

#include <rviz_visual_tools/rviz_visual_tools.h>
#include <controller_manager/controller_manager.h>
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/ControllerState.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>


#include <kdl/stiffness.hpp>
#include <kdl/trajectory.hpp>

#include <qb_interface/inertialSensor.h>
#include <qb_interface/inertialSensorArray.h>
#include "lwr_controllers/SetCartesianImpedanceCommand.h"

#include <tf/LinearMath/Matrix3x3.h>

class move_kuka
{
public:

	move_kuka();

	~move_kuka();

	void manager();
	void homePosition();

private:
  const double mm2m = 0.001;
  
  const double averta = 120/57*0.9;
  
  bool manager_done = false;
// 	ros::NodeHandle n_;
	ros::Subscriber sub_which_finger_;
	ros::Subscriber sub_imu_id_;
    std::string pkg_path_;	
    std::string finger_name_;	
    int cg_; // count_grasp_

    int trajectory_type;
    double traj_time;
    double spin_rate;
    double box_size;

    Eigen::Affine3d pose_;
    lwr_controllers::Stiffness zero_stiffness_;
    geometry_msgs::Wrench zero_wrench_;
    lwr_controllers::CartesianImpedancePoint msg_,msg_el;

    Eigen::Affine3d offset;
	// void kukaTopGrasp();
	// void kukaLateralGrasp();


    Eigen::Affine3d ee_offset;

	float degree_;
	rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
	ros::Publisher pub_command_,pub_command_t, pub_home_, pub_command_ci;
	float A_,offset_x_,offset_y_,offset_z_,delta_degree_, q_w_, q_x_, q_y_, q_z_; 
	float rest_x_, rest_y_, rest_z_, rest_q_w_, rest_q_x_, rest_q_y_, rest_q_z_;
	float stiffness_t, stiffness_r, damping, wrench; 
	bool ground_;
	double off_high_;
	bool bowl_, sliding_;
	std_msgs::Float64MultiArray joint_home;
	std_msgs::Float64MultiArray pose_home1;

	Eigen::Affine3d pose_home_affine;
	
	double num_EE[15], num_EL[15], num_ALL[31];
	char delim;

	// void tondoDatabase();
	// void openTondoDatabase(float& x,float& y,float& z,float& angle);

	int interpolation(Eigen::Affine3d x_start, Eigen::Affine3d x_finish, double traj_time_local=2.0);
	void finishPosition(float z);


	void handClosure(float v);
	ros::Publisher hand_publisher_;


	// call back
	// void callWichFinger(std_msgs::String msg);
	// void callImuId(std_msgs::Int64 imu_id);
	bool flag_which_finger_;
	bool flag_grasp_;	

	// std::fstream input_file;
	// std::vector<float> mypose;
	float mypose;
	float mypose1, mypose2, mypose3, mypose4, mypose5, mypose6, mypose7;
	// using new database
	/*
	void graspDatabase();
	void openAndLoadDB(Eigen::MatrixXf &Qrotation, Eigen::MatrixXf &translation);
	Eigen::MatrixXf Q_,T_;
	int rows_num_;

	//subscriber
	void checkError(control_msgs::JointTrajectoryControllerState msg);
	ros::Subscriber sub_error_joint_trajectory_;
	bool flag_error_joint_trajectory_;

    bool switchControllers(std::vector<std::string> target_controllers, std::vector<std::string> running_controllers);

	Eigen::Vector4f QxQ(Eigen::Vector4f Q_1, Eigen::Vector4f Q_2);
	Eigen::Vector4f ConjQ(Eigen::Vector4f Q_in);
	*/
	
      std::string csv_filename_EE, csv_filename_EL;
};