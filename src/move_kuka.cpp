#include <move_kuka.h>

// this package has been adapted from proto_grasp to replicate simple trajectories on kuka robot. GA 
move_kuka::move_kuka()
{
	sub_which_finger_ = n_.subscribe("/which_finger", 0, &move_kuka::callWichFinger, this);
	sub_imu_id_ = n_.subscribe("/imu_id", 0, &move_kuka::callImuId, this);
	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor", "/rviz_visual_markers"));
	visual_tools_->deleteAllMarkers();
	//check which controller you want to use; now it is teleoperation_controller
	pub_command_ = n_.advertise<lwr_controllers::CartesianImpedancePoint>("/right_arm/cartesian_impedance_controller/command", 0);
	pub_command_t = n_.advertise<geometry_msgs::Pose>("/right_arm/teleoperation_controller/command", 0);
	pub_home_ = n_.advertise<std_msgs::Float64MultiArray>("/right_arm/teleoperation_controller/home", 0);

	hand_publisher_ = n_.advertise<std_msgs::Float64>("/right_hand/hand_position", 0);
	flag_which_finger_ = flag_grasp_ = false;

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

	Eigen::Quaterniond q_init, q_rest, q_home;
	Eigen::Vector3d v_init;
	tf::quaternionTFToEigen(t.getRotation(), q_init);
	tf::vectorTFToEigen(t.getOrigin(), v_init);

	Eigen::Affine3d pose_init, pose_home, pose_rest;

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

	// pub_home_.publish(joint_home);
	std::cout << "\r\n\n\n\033[32m\033[1mRest Position \033[0m" << std::endl;
	std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;
	getchar();


 	// go from the rest position to the home position and wait for glove calibration
    q_home.w() = q_w_;
    q_home.x() = q_x_;
    q_home.y() = q_y_;
    q_home.z() = q_z_;

    pose_home = Eigen::Affine3d(q_home);

    // pose_home = Eigen::AngleAxisd(0 / 2., Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees
	// pose_home.translation() = Eigen::Vector3d( offset_x_ + A_ , offset_y_ , offset_z_  ); // translate x,y,z


	if (trajectory_type == 2)
	{
		offset_y_ = offset_y_ - box_size/2;
	}
	pose_home.translation() = Eigen::Vector3d( offset_x_ , offset_y_ , offset_z_  ); // translate x,y,z

	std::cout << "pose offset_x_: " << offset_x_ << std::endl;
	std::cout << "pose offset_y_: " << offset_y_ << std::endl;
	std::cout << "pose offset_z_: " << offset_z_ << std::endl;

	std::cout << "pose_init"<< std::endl;
	std::cout << pose_init.translation() << std::endl;

	std::cout << "pose_home"<< std::endl;
	std::cout << pose_home.translation() << std::endl;

	Eigen::Vector3d rpy = pose_init.rotation().eulerAngles(2,1,0);
	// std::cout << rpy << std::endl;
	rpy = pose_home.rotation().eulerAngles(2,1,0);
	// std::cout << rpy << std::endl;

	// interpolation(pose_home, pose_home, traj_time);


	interpolation(pose_rest, pose_home, traj_time);

	handClosure(0.2);

	std::cout << "\r\n\n\n\033[32m\033[1mHome Position \033[0m" << std::endl;
	std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;

	getchar();

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

		if(flag_which_finger_ && !flag_grasp_)
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
//                                                                            	  kukaCircle
//------------------------------------------------------------------------------------------
void move_kuka::kukaTopGrasp()
{
//vertical line 
	Eigen::Affine3d x_new;
	int flag_local = 1;

	while( flag_local == 1)
	{
		
		sleep(1.0);
		x_new = pose_;
		x_new.translation() = x_new.translation() + Eigen::Vector3d(0.0, 0.0, box_size); // translate x,y,z =
		if(interpolation(pose_, x_new, traj_time)==0){
			flag_local = 0.0;
			break;
		}
		sleep(1.0);
		// x_new = pose_;
		// x_new.translation() = x_new.translation() + Eigen::Vector3d(0.0, 0.0, -box_size); // translate x,y,z =
		// if(interpolation(pose_, x_new, traj_time)==0){
		// 	flag_local = 0.0;
		// 	break;
		// }
		// sleep(1.0);
		flag_which_finger_ = 1;

	}
}

void move_kuka::kukaLateralGrasp()
{
//horizontal line that starts going left
	Eigen::Affine3d x_new;
	int flag_local = 1;

	
	x_new = Eigen::AngleAxisd(90 * M_PI / 180, Eigen::Vector3d::UnitZ()); // rotate along "AXIS" axis by 90 degrees

	interpolation(pose_, pose_*x_new, 2*traj_time);

	getchar();

	while( flag_local == 1)
	{
		
		sleep(1.0);
		x_new = pose_;
		x_new.translation() = x_new.translation() + Eigen::Vector3d(0.0, box_size, 0.0); // translate x,y,z =
		if(interpolation(pose_, x_new, traj_time)==0){
			flag_local = 0.0;
			break;
		}
		sleep(1.0);
		// x_new = pose_;
		// x_new.translation() = x_new.translation() + Eigen::Vector3d(0.0, -box_size, 0.0); // translate x,y,z =
		// if(interpolation(pose_, x_new, traj_time)==0){
		// 	flag_local = 0.0;
		// 	break;
		// }
		// sleep(1.0);

		flag_which_finger_ = 1;
	}
}



//------------------------------------------------------------------------------------------
// Giuse End
//------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------
//                                                                             tondoDatabase
//------------------------------------------------------------------------------------------
void move_kuka::tondoDatabase()
{
	Eigen::Affine3d pose_grasp;

	float x, y, z, angle;

	openTondoDatabase(x, y, z, angle);

	if (bowl_ == true)
	{
		pose_grasp = Eigen::AngleAxisd(angle * M_PI / 180, Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees

		pose_grasp.translation() = Eigen::Vector3d( z, -y, x); // translate x,y,z // all poses are local
		// pose_grasp.translation() = Eigen::Vector3d( z, y, -x); // translate x,y,z // all poses are local
		// pose_grasp.translation() = Eigen::Vector3d( x, y, z); // translate x,y,z // all poses are local
		ROS_INFO_STREAM("Executing Bowl Primitive");
		interpolation(pose_, pose_*pose_grasp, traj_time/2);
	}
	else {
		if (sliding_ == true)
		{
			ROS_INFO_STREAM("Executing Sliding Primitive");
			pose_grasp = Eigen::AngleAxisd(0 * M_PI / 180, Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees

			double press = 0.04;
			pose_grasp.translation() = Eigen::Vector3d( -press, 0, 0); // translate x,y,z // all poses are local
			interpolation(pose_, pose_*pose_grasp, traj_time/2);

			double transl = 0.25;
			pose_grasp.translation() = Eigen::Vector3d( 0, 0, -transl); // translate x,y,z // all poses are local
			interpolation(pose_, pose_*pose_grasp, traj_time/2);

			pose_grasp.translation() = Eigen::Vector3d( press, 0, 0); // translate x,y,z // all poses are local
			interpolation(pose_, pose_*pose_grasp, traj_time/2);

			pose_grasp = Eigen::AngleAxisd(angle * M_PI / 180, Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees
			pose_grasp.translation() = Eigen::Vector3d( z, -y, x); // translate x,y,z // all poses are local
			interpolation(pose_, pose_*pose_grasp, traj_time/2);
		}
		else {
			pose_grasp = Eigen::AngleAxisd(angle * M_PI / 180, Eigen::Vector3d::UnitZ()); // rotate along "AXIS" axis by 90 degrees

			pose_grasp.translation() = Eigen::Vector3d( z, -y, x); // translate x,y,z // all poses are local
			// pose_grasp.translation() = Eigen::Vector3d( z, y, -x); // translate x,y,z // all poses are local
			// pose_grasp.translation() = Eigen::Vector3d( x, y, z); // translate x,y,z // all poses are local
			ROS_INFO_STREAM("Executing Tondo Primitive");
			interpolation(pose_, pose_*pose_grasp, traj_time/2);
		}
	}
	
}


//------------------------------------------------------------------------------------------
//                                                                         openTondoDatabase
//------------------------------------------------------------------------------------------
void move_kuka::openTondoDatabase(float& x, float& y, float& z, float& angle)
{
	if (ground_ == true)
		{
			x = 0;
			y = 0;
			z = 0+off_high_;
			angle = 0;

		}
		else if (bowl_ == true)
		{
			x = 0.1;
			y = 0;
			z = -0.17;
			angle = 60;

		}
		else if (sliding_ == true)
		{
			x = 0.05;
			y = 0;
			z = -0.1;
			angle = 25;

		}
		else{
			if (finger_name_ == "frontal_thumb")
			{
				x = 0.005;
				y = 0.045;
				z = 0.00; //-0.020
				angle = 35;
			}
			else if (finger_name_ == "frontal_index")
			{
				x = 0.08; // 0.13
				y = 0.015;
				z = 0.02;
				angle = 15; //-15
 			}
			else if (finger_name_ == "frontal_middle")
			{
				x = 0.085; // 0.135
				y = -0.015;
				z = 0.025;
				angle = 12;
			}
			else if (finger_name_ == "frontal_ring")
			{
				x = 0.07; // 0.12
				y = -0.045;
				z = 0.015;
				angle = 5;
			}
			else if (finger_name_ == "frontal_little")
			{
				x = 0.06; // 0.11
				y = -0.07;
				z = 0.03;
				angle = -11;
			}
			else if (finger_name_ == "side_index")
			{
				x = 0.045; //0.070 0.060
 				y = 0.038; //0.04
				z = 0.015; //0.020
				angle = 45;
			}
			else if (finger_name_ == "side_little")
			{
				x = 0.045; // 0.075
				y = -0.03;
				z = 0.05;
				angle = -35;
			}
			else if (finger_name_ == "side_thumb")
			{
				x = -0.005;
				y = 0.04;
				z = -0.03;
				angle = 40;
			}
			else if (finger_name_ == "vertical_thumb")
			{
				x = -0.005;
				y = 0.045;
				z = -0.04;
				angle = 45;
			}
			else if (finger_name_ == "vertical_index")
			{
				x = 0.060; // 0.060
				y = 0.015;
				z = 0.02;  //-0.02
				angle = 20;
			}
			else if (finger_name_ == "vertical_ring")
			{
				x = 0.045; // 0.075
				y = -0.025;
				z = 0.015;   // -0.005
				angle = -6; // 6
			}
			else if (finger_name_ == "vertical_little")
			{
				x = 0.075; //  0.040 0.075
				y = -0.04;
				z = 0.0; // 0.01 -0.01
				angle = -16;
			}
			else if (finger_name_ == "vertical_middle")
			{
				x = 0.065; // 0.045 0.085
				y = -0.005;
				z = 0.015; //0.015 -0.04
				angle = 13;
			}
			else
			{
				x = 0;
				y = 0;
				z = 0;
				angle = 0;
				ROS_ERROR("Error in Alessandro Tondo Database");
			}
			}
			z = z +off_high_;
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


void move_kuka::callImuId(std_msgs::Int64 imu_id){

		
		if(imu_id.data >= 0){
			if (imu_id.data == 0) {
				std::cout << "\r\n\n\n\033[32m\033[1mOK!\033[0m" << std::endl;	
			} 
			//trajectory_type = 0; //SONO QUI
			
		}
}	

//------------------------------------------------------------------------------------------
//                                                                                  callBack
//------------------------------------------------------------------------------------------
void move_kuka::callWichFinger(std_msgs::String msg)
{

	if (msg.data == "frontal_thumb")				//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_middle")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_ring")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_index")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "frontal_little")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_index")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_thumb")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "side_little")			//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_thumb")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_middle")     //////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_ring")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_index")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else if (msg.data == "vertical_little")		//////////////////////// TONDO DATABASE
	{
		flag_which_finger_   = true;
		finger_name_ = msg.data;
	}
	else
	{
		flag_which_finger_ = false;
		/* NOTHING TO DO */
	}
}


//------------------------------------------------------------------------------------------
//                                                                                   manager
//------------------------------------------------------------------------------------------
void move_kuka::manager()
{
ROS_INFO_STREAM("Using trajectory: " << trajectory_type);
	if (!flag_which_finger_  && !flag_grasp_ )
	{
		// handClosure(0.3);
		switch (trajectory_type) {
		case 1:
			kukaTopGrasp();
			ROS_INFO_STREAM("Performing a top grasp" << trajectory_type);
			break;
		case 2:
			kukaLateralGrasp();
			ROS_INFO_STREAM("Performing a lateral grasp" << trajectory_type);
			break;
		default:
			ROS_INFO("No trajectoye chossen");
		}
	}
	else
	{ //will not enter here
		flag_grasp_ = true;
		// tondoDatabase();
		// sleep(2);

		handClosure(1.0);
		// handClosure(1.0);
		visual_tools_->deleteAllMarkers();
		sleep(3);
		finishPosition(0.25);
		sleep(3);
		flag_grasp_ =  false;
		std::cout << "\r\n\n\n\033[32m\033[1mPress to open the hand... \033[0m" << std::endl;
		getchar();
		handClosure(0.0);

		ros::shutdown();
	}


	ros::spinOnce();
}



