#include <move_kuka.h>

// this package has been adapted from proto_grasp to replicate simple trajectories on kuka robot. GA 
move_kuka::move_kuka()
{
	// sub_which_finger_ = n_.subscribe("/which_finger", 0, &move_kuka::callWichFinger, this);
	// sub_imu_id_ = n_.subscribe("/imu_id", 0, &move_kuka::callImuId, this);
	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor", "/rviz_visual_markers"));
	visual_tools_->deleteAllMarkers();
	//check which controller you want to use; now it is teleoperation_controller
	pub_command_ = n_.advertise<lwr_controllers::CartesianImpedancePoint>("/right_arm/cartesian_impedance_controller/command", 0);
	pub_command_t = n_.advertise<geometry_msgs::Pose>("/right_arm/teleoperation_controller/command", 0);
	pub_home_ = n_.advertise<std_msgs::Float64MultiArray>("/right_arm/teleoperation_controller/home", 0);

	hand_publisher_ = n_.advertise<std_msgs::Float64>("/right_hand/hand_position", 0);
	// flag_which_finger_ = flag_grasp_ = false;

	trajectory_type = 1;
	n_.param<int>("/trajectory_type", trajectory_type, 0);
	n_.param<double>("/traj_time", traj_time, 2.0);
	n_.param<double>("/spin_rate", spin_rate, 300.0);
	n_.param<double>("/box_size", box_size, 0.15);

	n_.param<float>("/A_",A_,0.10);

	n_.param<float>("/rest_x_",rest_x_,-0.80);
	n_.param<float>("/rest_y_",rest_y_,0.40);
	n_.param<float>("/rest_z_",rest_z_,0.10);
	n_.param<float>("/rest_q_w_",rest_q_w_,0.702); 
	n_.param<float>("/rest_q_x_",rest_q_x_,0.000); 
	n_.param<float>("/rest_q_y_",rest_q_y_,0.712); 
	n_.param<float>("/rest_q_z_",rest_q_z_,0.000); 

	n_.param<float>("/offset_x_",offset_x_,-0.80);
	n_.param<float>("/offset_y_",offset_y_,0.40);
	n_.param<float>("/offset_z_",offset_z_,0.10);
	n_.param<float>("/q_w_",q_w_,0.702); 
	n_.param<float>("/q_x_",q_x_,0.000); 
	n_.param<float>("/q_y_",q_y_,0.712); 
	n_.param<float>("/q_z_",q_z_,0.000); 
	n_.param<bool>("/ground_",ground_,false); 
	n_.param<double>("/off_high_",off_high_,0.0); 
	n_.param<bool>("/bowl_",bowl_,false); 
	n_.param<bool>("/sliding_",sliding_,false); 
  
    joint_home.data.resize(7);
    n_.param<double>("/a1_home",joint_home.data[0],0.0);
    n_.param<double>("/a2_home",joint_home.data[1],0.0);
    n_.param<double>("/e1_home",joint_home.data[2],0.0);
    n_.param<double>("/a3_home",joint_home.data[3],0.0);
    n_.param<double>("/a4_home",joint_home.data[4],0.0);
    n_.param<double>("/a5_home",joint_home.data[5],0.0);
    n_.param<double>("/a6_home",joint_home.data[6],0.0);
    
    std::cout<<"JOINT HOME: [" ;
    for(int i =0; i < 7; i++)
    {
      std::cout<<joint_home.data[i]<<"  ";
  	}
    std::cout<<"]"<<std::endl;
    
	n_.param<float>("/delta_degree_",delta_degree_,0.30);

	n_.param<float>("/stiffness_t",stiffness_t,2000);
	n_.param<float>("/stiffness_r",stiffness_r,150);
	n_.param<float>("/damping",damping,0.7);
	n_.param<float>("/wrench",wrench,0.0);


	pkg_path_ = ros::package::getPath("move_kuka");


	// rest msg for publisher
	zero_stiffness_.x = stiffness_t;
	zero_stiffness_.y = stiffness_t;
	zero_stiffness_.z = stiffness_t;
	zero_stiffness_.rx = stiffness_r;
	zero_stiffness_.ry = stiffness_r;
	zero_stiffness_.rz = stiffness_r;
	msg_.k_FRI = zero_stiffness_;

	zero_stiffness_.x = damping;
	zero_stiffness_.y = damping;
	zero_stiffness_.z = damping;
	zero_stiffness_.rx = damping;
	zero_stiffness_.ry = damping;
	zero_stiffness_.rz = damping;
	msg_.d_FRI = zero_stiffness_;

	zero_wrench_.force.x = wrench;
	zero_wrench_.force.y = wrench;
	zero_wrench_.force.z = wrench;
	zero_wrench_.torque.x = wrench;
	zero_wrench_.torque.y = wrench;
	zero_wrench_.torque.z = wrench;
	msg_.f_FRI = zero_wrench_;

	visual_tools_->deleteAllMarkers();

	degree_ = 0;


	// handClosure(0.3);
}

move_kuka::~move_kuka()
{

}


void move_kuka::homePosition()
{
	// get current transofrmation between "vito_anchor" and "right_palm_link"
	std::string link_from = "/vito_anchor";
	std::string link_to   = "/right_arm_7_link"; 

	tf::TransformListener listener;
	tf::StampedTransform t;

	ros::spinOnce();

	bool tf_ok=true;
	int tf_i=0;

	while(ros::ok())
	{
		try
		{
			tf_ok=true;
			listener.waitForTransform(link_from, link_to, ros::Time(0), ros::Duration(20.0)); //ros::Duration(2.5)
			// listener.lookupTransform(link_to, link_from,  ros::Time(0), t);
			listener.lookupTransform(link_from, link_to,  ros::Time(0), t);
		}
		catch (tf::TransformException ex)
		{
			tf_ok=false;
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		if(tf_ok)
		{
			break;
		}
		else
		{
			std::cout<<"trial tf: #"<<tf_i++<<std::endl;
		}
	}

	Eigen::Quaterniond q_init, q_rest;
	Eigen::Vector3d v_init;
	tf::quaternionTFToEigen(t.getRotation(), q_init);
	tf::vectorTFToEigen(t.getOrigin(), v_init);

	Eigen::Affine3d pose_init, pose_rest;

	pose_init = Eigen::Affine3d(q_init);
	pose_init.translation() = Eigen::Vector3d(v_init(0), v_init(1), v_init(2)); // translate x,y,z
	//put a quaternion instead of rpy in pose_home
	/////////pose_home = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
    // pose_home = Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees

	// define the rest pose and go from any initial position to the rest position
    q_rest.w() = rest_q_w_;
    q_rest.x() = rest_q_x_;
    q_rest.y() = rest_q_y_;
    q_rest.z() = rest_q_z_;

    pose_rest = Eigen::Affine3d(q_rest);
	pose_rest.translation() = Eigen::Vector3d( rest_x_ , rest_y_ , rest_z_  ); // translate x,y,z


	interpolation(pose_init, pose_rest, traj_time);

	pub_home_.publish(joint_home);

	std::cout << "\r\n\n\n\033[32m\033[1mRest Position \033[0m" << std::endl;
	std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;
	getchar();


// // This commented part moves the robot from init position to home position

//  	// go from the rest position to the home position and wait for glove calibration
//     q_home.w() = q_w_;
//     q_home.x() = q_x_;
//     q_home.y() = q_y_;
//     q_home.z() = q_z_;

//     pose_home = Eigen::Affine3d(q_home);

//     // pose_home = Eigen::AngleAxisd(0 / 2., Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees
// 	// pose_home.translation() = Eigen::Vector3d( offset_x_ + A_ , offset_y_ , offset_z_  ); // translate x,y,z


// 	if (trajectory_type == 2)
// 	{
// 		offset_y_ = offset_y_ - box_size/2;
// 	}
// 	pose_home.translation() = Eigen::Vector3d( offset_x_ , offset_y_ , offset_z_  ); // translate x,y,z

// 	std::cout << "pose offset_x_: " << offset_x_ << std::endl;
// 	std::cout << "pose offset_y_: " << offset_y_ << std::endl;
// 	std::cout << "pose offset_z_: " << offset_z_ << std::endl;

// 	std::cout << "pose_init"<< std::endl;
// 	std::cout << pose_init.translation() << std::endl;

// 	std::cout << "pose_home"<< std::endl;
// 	std::cout << pose_home.translation() << std::endl;

// 	Eigen::Vector3d rpy = pose_init.rotation().eulerAngles(2,1,0);
// 	// std::cout << rpy << std::endl;
// 	rpy = pose_home.rotation().eulerAngles(2,1,0);
// 	// std::cout << rpy << std::endl;

// 	// interpolation(pose_home, pose_home, traj_time);


// 	interpolation(pose_rest, pose_home, traj_time);

// 	handClosure(0.2);

// 	std::cout << "\r\n\n\n\033[32m\033[1mHome Position \033[0m" << std::endl;
// 	std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;

// 	getchar();

}


//------------------------------------------------------------------------------------------
//                                                                                   manager
//------------------------------------------------------------------------------------------
void move_kuka::manager()
{
	ros::Rate r(spin_rate);

	// while (1)
	// {
		std::ifstream input_file;
		input_file.open ("/home/averta/catkin_ws/src/move_kuka/src/my_input_file1.txt");


		if(input_file.is_open()){
			std::cout << "file correctly open" << std::endl;
		}


		while (input_file >> mypose1 >> mypose2 >> mypose3 >> mypose4 >> mypose5 >> mypose6 >> mypose7){
			std::cout << "mypose is: " << mypose1 << " " << mypose2 << " " << mypose3 << " " << mypose4 << " " << mypose5 << " " << mypose6 << " " << mypose7 <<  "\n" << std::endl;
			joint_home.data[0] = mypose1;
			joint_home.data[1] = mypose2;
			joint_home.data[2] = mypose3;
			joint_home.data[3] = mypose4;
			joint_home.data[4] = mypose5;
			joint_home.data[5] = mypose6;
			joint_home.data[6] = mypose7;
			pub_home_.publish(joint_home);
		}
		
		// }





	// for(int i=0; i<10; i++)
	// {	
	// 	// std::cout <<"moving"<<std::endl;
	// 	joint_home.data[0] = 0.50;
	// 	pub_home_.publish(joint_home);
	    
	//     std::cout<<"JOINT HOME: [" ;
	//     for(int j =0; j < 7; j++)
	//     {
	//       std::cout<<joint_home.data[j]<<"  ";
	//   	}
	//     std::cout<<"]"<<std::endl;

	// 	// ros::spinOnce();
	// 	r.sleep();

	// }


		ros::shutdown();

}


//------------------------------------------------------------------------------------------
//                                                                             interpolation
//------------------------------------------------------------------------------------------
int move_kuka::interpolation(Eigen::Affine3d x_start, Eigen::Affine3d x_finish, double traj_time_local)
{
	float th = 0.001;	//trade error
	float alpha = 0.015;
	float translation_error = 1;
	float c = 0; //for slerp

	Eigen::Affine3d x_next, x_now, x_prev;

	x_now  = x_start;
	x_prev = x_start;

	geometry_msgs::Pose x_finish_frame, x_now_frame;

	// read quaternion from GeometryPoseMsg and convert them to Eigen::Quaterniond
	//passing from affine3d to geometry_msg
	tf::poseEigenToMsg(x_now, x_now_frame);
	tf::poseEigenToMsg(x_finish, x_finish_frame);

	Eigen::Quaterniond q_start(x_now_frame.orientation.w, x_now_frame.orientation.x, x_now_frame.orientation.y, x_now_frame.orientation.z);
	Eigen::Quaterniond q_finish(x_finish_frame.orientation.w, x_finish_frame.orientation.x, x_finish_frame.orientation.y, x_finish_frame.orientation.z);
	Eigen::Quaterniond q_err;

	ros::Rate r(spin_rate);
	while (c <= 1 && ros::ok())
	{
		// update orientation
		double ctanh(std::tanh(4*c));
		if (c <= 1)
			q_err = q_start.slerp(ctanh, q_finish);

		x_next = Eigen::AngleAxisd(q_err);

		// updata translation
		if (translation_error > th)
			x_next.translation() = x_now.translation() + ctanh * (x_finish.translation() - x_now.translation() ) ;

		// visual tool
		visual_tools_->publishAxis(x_next, 0.1, 0.01, "axis");
		visual_tools_->trigger();

		// set control
		tf::poseEigenToMsg(x_next, msg_.x_FRI);
		geometry_msgs::Pose pose;
		pose = msg_.x_FRI;
		pub_command_.publish(msg_);
		pub_command_t.publish(pose);
		ros::spinOnce();

		c += (1.0 / spin_rate) / traj_time_local;

		if(0)//(flag_which_finger_ && !flag_grasp_)
		{
			// std::cout<<"TOUCHED"<<std::endl;
			pose_ = x_next;
			return 0;
		}
		r.sleep();
	}

	// std::cout<<"INTERPOLATED"<<std::endl;
	// sleep(1.0);
	// update global pose_ for next steps
	pose_ = x_next;
	return 1;
}



//------------------------------------------------------------------------------------------
//                                                                            finishPosition
//------------------------------------------------------------------------------------------
void move_kuka::finishPosition(float z)
{
	Eigen::Affine3d pose_finish;
	//handClosure(0.0);
	// pose_finish = Eigen::AngleAxisd(- M_PI / 2, Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees
	pose_finish = pose_;
	pose_finish.translation() = pose_finish.translation() + Eigen::Vector3d(0, 0, z); // translate x,y,z

	interpolation(pose_, pose_finish, traj_time);

	std::cout << "\r\n\n\n\033[32m\033[1mGrasped Object\033[0m" << std::endl;
	
}

//------------------------------------------------------------------------------------------
//                                                                               handClosure
//------------------------------------------------------------------------------------------
void move_kuka::handClosure(float v)
{
	std_msgs::Float64 msg;
	msg.data = v;
	hand_publisher_.publish(msg);
	ros::spinOnce();
}






