#include <move_kuka.h>

// NOTE
// TRANSFORM FROM VITO_ILIAD.URDF.XACRO FROM WORLD TO RIGHT SHOULDER
// <origin xyz="0.77 0.801 1.607" rpy="3.1415 -0.7854 0"/>

// this package has been adapted from proto_grasp to replicate simple trajectories on kuka robot. GA 
move_kuka::move_kuka()
{
  	ros::NodeHandle n_("~");

	// sub_which_finger_ = n_.subscribe("/which_finger", 0, &move_kuka::callWichFinger, this);
	// sub_imu_id_ = n_.subscribe("/imu_id", 0, &move_kuka::callImuId, this);
	visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("vito_anchor", "/rviz_visual_markers"));
	visual_tools_->deleteAllMarkers();
	//check which controller you want to use; now it is teleoperation_controller
// 	pub_command_t = n_.advertise<geometry_msgs::Pose>("/right_arm/teleoperation_controller/command", 0);
	pub_command_ci = n_.advertise<std_msgs::Float64MultiArray>("/right_arm/vito_bridge_controller/commandCart_right", 0);
	pub_home_ = n_.advertise<std_msgs::Float64MultiArray>("/right_arm/teleoperation_controller/home", 0);

	hand_publisher_ = n_.advertise<std_msgs::Float64>("/right_hand/hand_position", 0);
	// flag_which_finger_ = flag_grasp_ = false;

	trajectory_type = 1;
	n_.param<int>("/trajectory_type", trajectory_type, 0);
	n_.param<double>("/traj_time", traj_time, 2.0);
	n_.param<double>("/spin_rate", spin_rate, 100.0);
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
    
	pose_home1.data.resize(6);
	pose_home1.data[0] = 0; // 0.5;
	pose_home1.data[1] = 0; // 0.0;
	pose_home1.data[2] = 0; // 1.0;
	pose_home1.data[3] = 0.0;
	pose_home1.data[4] = 0.0;
	pose_home1.data[5] = 0.0;

    std::cout<<"Pose HOME: " << pose_home1;

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
	msg_el.k_FRI = zero_stiffness_;

	zero_stiffness_.x = damping;
	zero_stiffness_.y = damping;
	zero_stiffness_.z = damping;
	zero_stiffness_.rx = damping;
	zero_stiffness_.ry = damping;
	zero_stiffness_.rz = damping;
	msg_.d_FRI = zero_stiffness_;
	msg_el.d_FRI = zero_stiffness_;

	zero_wrench_.force.x = wrench;
	zero_wrench_.force.y = wrench;
	zero_wrench_.force.z = wrench;
	zero_wrench_.torque.x = wrench;
	zero_wrench_.torque.y = wrench;
	zero_wrench_.torque.z = wrench;
	msg_.f_FRI = zero_wrench_;
	msg_el.f_FRI = zero_wrench_;

	visual_tools_->deleteAllMarkers();

	degree_ = 0;


	// handClosure(0.3);
	
  
  n_.getParam("csv_file_EE", csv_filename_EE);
  n_.getParam("csv_file_EL", csv_filename_EL);
  
  ROS_INFO_STREAM("EE trajectory file: " << csv_filename_EE);  
  ROS_INFO_STREAM("ELBOW trajectory file: " << csv_filename_EL);  

  if(!csv_filename_EE.empty() && !csv_filename_EL.empty()){
    //load data for EE and EL reference
    std::ifstream csv_file_EE, csv_file_EL;
    csv_file_EE.open (csv_filename_EE);
    csv_file_EL.open (csv_filename_EL);
    if(csv_file_EE.is_open())
	    std::cout << "file end-effector correctly open" << std::endl;
    else
      ROS_ERROR("Could not open input csv file");
    if(csv_file_EL.is_open())
	    std::cout << "file elbow correctly open" << std::endl;
    else
      ROS_ERROR("Could not open input csv file");
    
    unsigned int line = 0;
    
    Eigen::Matrix4d num_matrix_ee, num_matrix_el ;

	
    while(!csv_file_EE.eof() && !csv_file_EL.eof() && line < 1)
    {
      std::cout << line++ << ": ";
      for (int i=0; i< 16; i++)
      {
	csv_file_EE >> num_EE[i];
	csv_file_EE >> delim;
	std::cout << num_EE[i] << " ";
	
	csv_file_EL >> num_EL[i];
	csv_file_EL >> delim;
	std::cout << num_EL[i] << " ";
      }
      std::cout << std::endl;  
      
      // pub msg
    num_matrix_ee << num_EE[0], num_EE[4], num_EE[8], num_EE[12]*mm2m*averta,
		     num_EE[1], num_EE[5], num_EE[9], num_EE[13]*mm2m*averta, 
		     num_EE[2], num_EE[6], num_EE[10], num_EE[14]*mm2m*averta, 
		     num_EE[3], num_EE[7], num_EE[11], num_EE[15];
	      
    num_matrix_el << num_EL[0], num_EL[4], num_EL[8], num_EL[12]*mm2m*averta_elbow,
		     num_EL[1], num_EL[5], num_EL[9], num_EL[13]*mm2m*averta_elbow, 
		     num_EL[2], num_EL[6], num_EL[10], num_EL[14]*mm2m*averta_elbow, 
		     num_EL[3], num_EL[7], num_EL[11], num_EL[15];
		     
    }     
// 		     ///////////////// MERDA
//     Eigen::Matrix4d num_matrix_ee, num_matrix_el ;
// 
//     //Load EE data
//     std::ifstream csv_file_EE;
//     csv_file_EE.open (csv_filename_EE);
//  
//     if(csv_file_EE.is_open())
// 	    std::cout << "file EE correctly open" << std::endl;
//     else
//       ROS_ERROR("Could not open input csv file");
//     
//     for (int i=0; i< 16; i++)
//     {
//       csv_file_EE >> num_EE[i];
//       csv_file_EE >> delim;
//       std::cout << num_EE[i] << " ";
//     }
//     std::cout << std::endl;
//    
//     csv_file_EE.close();
//     
//     num_matrix_ee << num_EE[0], num_EE[4], num_EE[8], num_EE[12]*mm2m*averta,
// 		num_EE[1], num_EE[5], num_EE[9], num_EE[13]*mm2m*averta, 
// 		num_EE[2], num_EE[6], num_EE[10], num_EE[14]*mm2m*averta, 
// 		num_EE[3], num_EE[7], num_EE[11], num_EE[15];
//     
//     
//     
//     // fine qui ok
//     
//     //Load Elobw data
//     std::ifstream csv_file_EL;
//     csv_file_EL.open (csv_filename_EL);
//     
//     
//     if(csv_file_EL.is_open())
// 	    std::cout << "file EL correctly open" << std::endl;
//     else
//       ROS_ERROR("Could not open input csv file");
//     
//     for (int i=0; i< 16; i++)
//     {
//       csv_file_EL >> num_EL[i];
//       csv_file_EL >> delim2;
//       std::cout << num_EL[i] << " ";
//     }
//     std::cout << std::endl;
//    
//     csv_file_EL.close();
//     
//     num_matrix_el << num_EL[0], num_EL[4], num_EL[8], num_EL[12]*mm2m*averta,
// 		num_EL[1], num_EL[5], num_EL[9], num_EL[13]*mm2m*averta, 
// 		num_EL[2], num_EL[6], num_EL[10], num_EL[14]*mm2m*averta, 
// 		num_EL[3], num_EL[7], num_EL[11], num_EL[15];
// 		        
//     //put all in the same vector
//     for (int i=0; i< 16; i++)
//     {
//       num_ALL[i] = num_EE[i];
//       num_ALL[i+16] = num_EL[i];
//     }
//     
//     
//       //set homing pose    
// 
// 		
// 
// 
//     std::cout << ">>>>>> DEBUG INFO FOR TARGET POSES <<<<<<<" << std::endl;
//     std::cout << " First pose for end effector \n" << num_matrix_ee << std::endl;
//     std::cout << __LINE__ << "num_EE: ";
//     for(int i=0;i<16;i++)
//       std::cout << num_EE[i] << " ";
//     std::cout << std::endl;
//     std::cout << "Last element (should be 1): " << num_EE[15];
//     //   assert(num_EE[15] == 1.0);
//     std::cout << " First pose for elbow \n" << num_matrix_el << std::endl;
//     std::cout << "num_EL: ";
//     for(int i=0;i<16;i++)
//       std::cout << num_EL[i] << " ";
//     std::cout << std::endl;
//     std::cout << "Last element (should be 1): " << num_EL[15];
//     //   assert(num_EL[15] == 1.0);
//     std::cout << "<<<<<< DEBUG INFO FOR TARGET POSES >>>>>>>" << std::endl;

    pose_home_affine.matrix() = num_matrix_ee;
  
    pose_home_affine_ELB.matrix() = num_matrix_el;
  
    elbow_task = true;
  }
  else if(!csv_filename_EE.empty()){
    std::ifstream csv_file_EE;
    csv_file_EE.open (csv_filename_EE);
 
    if(csv_file_EE.is_open())
	    std::cout << "file correctly open" << std::endl;
    else
  
    ROS_ERROR("Could not open input csv file");
    
    for (int i=0; i< 16; i++)
    {
      csv_file_EE >> num_EE[i];
      csv_file_EE >> delim;
      std::cout << num_EE[i] << " ";
    }
    std::cout << std::endl;
   
    csv_file_EE.close();
    
    //put all in the same vector
    for (int i=0; i< 16; i++)
    {
      num_ALL[i] = num_EE[i];
      num_ALL[i+16] = 0.0;
    }    
    
    //set homing pose    
    Eigen::Matrix4d num_matrix_ee, num_matrix_el ;
    num_matrix_ee << num_EE[0], num_EE[4], num_EE[8], num_EE[12]*mm2m*averta,
		    num_EE[1], num_EE[5], num_EE[9], num_EE[13]*mm2m*averta, 
		    num_EE[2], num_EE[6], num_EE[10], num_EE[14]*mm2m*averta, 
		    num_EE[3], num_EE[7], num_EE[11], num_EE[15];
		
    std::cout << ">>>>>> DEBUG INFO FOR TARGET POSES <<<<<<<" << std::endl;
    std::cout << " First pose for end effector \n" << num_matrix_ee << std::endl;
    std::cout << "Last element (should be 1): " << num_EE[15];
    std::cout << "No elbow target pose" << std::endl;
    std::cout << "<<<<<< DEBUG INFO FOR TARGET POSES >>>>>>>" << std::endl;
    
    
    pose_home_affine.matrix() = num_matrix_ee;
  }
  
  
  // set offset translation and rotation

//   KDL::Frame offset_frame(
//     KDL::Rotation::RPY(
//       0.0,
//       0.7854,
//       0.0
//     ),
//     KDL::Vector(-125*mm2m, 
// 				      -148*mm2m, 
// 				      115*mm2m)
// 			 );
//  
  
  KDL::Frame offset_frame(
    KDL::Rotation::RPY(
      0.0,
      0.7854,
      0.0
    ),
    KDL::Vector(-125*mm2m, 
		-148*mm2m, 
		 115*mm2m)
			 );
  offset = Eigen::Affine3d::Identity();
  tf::transformKDLToEigen(offset_frame,offset);
  
  ROS_INFO_STREAM("Setting offset");
  std::cout << "\tTranslation: " << std::endl <<  offset.translation() << std::endl;
  std::cout << "\tRotation: " << std::endl <<  offset.rotation() << std::endl;
  
  // end effector offset matrix
    KDL::Frame rotation_ee_frame1(
    KDL::Rotation::RPY(
      0.0,
      1.57,
      0
    ),
    KDL::Vector(0, 
		0, 
		0)
			 );
  Eigen::Affine3d ee_offset1 = Eigen::Affine3d::Identity();
  tf::transformKDLToEigen(rotation_ee_frame1,ee_offset1);
  
  KDL::Frame rotation_ee_frame(
    KDL::Rotation::RPY(
      0.0,
      0,
      1.57
    ),
    KDL::Vector(0, 
		0, 
		0)
			 );
  ee_offset = Eigen::Affine3d::Identity();
  tf::transformKDLToEigen(rotation_ee_frame,ee_offset);
  
  ee_offset = ee_offset1*ee_offset;
  
  
  ROS_INFO_STREAM("Homing pose in ");
  std::cout << "\tTranslation: " << std::endl <<  pose_home_affine.translation() << std::endl;
  std::cout << "\tRotation: " << std::endl <<  pose_home_affine.rotation() << std::endl;

}

move_kuka::~move_kuka()
{

}


void move_kuka::homePosition()
{
  // get current transofrmation between "vito_anchor" and "right_palm_link"
  std::string link_from = "/right_arm_base_link";
  std::string link_to   = "/right_arm_7_link"; 
  std::string link_from_ELB = "/right_arm_base_link";
  std::string link_to_ELB   = "/right_arm_3_link";  // TODO CHECK
  
  tf::TransformListener listener;
  tf::TransformListener listener_ELB;
  tf::StampedTransform t;
  tf::StampedTransform t_ELB;

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
    
    if(elbow_task)
    {
      try
      {
	      tf_ok=true;
	      listener_ELB.waitForTransform(link_from_ELB, link_to_ELB, ros::Time(0), ros::Duration(20.0)); //ros::Duration(2.5)
	      listener_ELB.lookupTransform(link_from_ELB, link_to_ELB,  ros::Time(0), t_ELB);
      }
      catch (tf::TransformException ex)
      {
	      tf_ok=false;
	      ROS_ERROR("%s", ex.what());
	      ros::Duration(1.0).sleep();
      }
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

  //Eigen::Quaterniond q_init, q_rest;
  Eigen::Quaterniond q_init;
  Eigen::Vector3d v_init;
  tf::quaternionTFToEigen(t.getRotation(), q_init);
  tf::vectorTFToEigen(t.getOrigin(), v_init);

 
  //Eigen::Affine3d pose_init, pose_rest;
  Eigen::Affine3d pose_init;

  pose_init = Eigen::Affine3d(q_init);
  pose_init.translation() = Eigen::Vector3d(v_init(0), v_init(1), v_init(2)); // translate x,y,z
  
  //put a quaternion instead of rpy in pose_home
  /////////pose_home = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()); // rotate along "AXIS" axis by 90 degrees
  // pose_home = Eigen::AngleAxisd(-M_PI / 2., Eigen::Vector3d::UnitY()); // rotate along "AXIS" axis by 90 degrees

  // define the rest pose and go from any initial position to the rest position

  //tf::Quaternion q_ref;
  //q_ref.setRPY(pose_home1.data[3], pose_home1.data[4], pose_home1.data[5]);
  //q_rest.x() = q_ref.x();
  //q_rest.y() = q_ref.y();
  //q_rest.z() = q_ref.z();
  //q_rest.w() = q_ref.w();
  
  //  pose_rest = Eigen::Affine3d(q_rest);
  //pose_rest.translation() = Eigen::Vector3d( pose_home1.data[0] , pose_home1.data[1] , pose_home1.data[2]); // translate x,y,z

  ROS_INFO("Begin homing...");
  
  ROS_INFO("Open hand.");
  handClosure(0.0);

  
  if(elbow_task) // ELBOW HOMING
  {
    Eigen::Quaterniond q_init_ELB;
    Eigen::Vector3d v_init_ELB;
    tf::quaternionTFToEigen(t_ELB.getRotation(), q_init_ELB);
    tf::vectorTFToEigen(t_ELB.getOrigin(), v_init_ELB);
    
    Eigen::Affine3d pose_init_ELB;

    pose_init_ELB = Eigen::Affine3d(q_init_ELB);
    pose_init_ELB.translation() = Eigen::Vector3d(v_init_ELB(0), v_init_ELB(1), v_init_ELB(2)); // translate x,y,z
  
    interpolation(pose_init, pose_home_affine, pose_init_ELB, pose_home_affine_ELB, 10.0);
  }
  else // NO ELBOW INTERPOLATION
    interpolation(pose_init, pose_home_affine, 10.0);// traj_time);
  
  ROS_INFO("Home is where the heart is");
// 	pub_home_.publish(joint_home);

/*    for(int i=0; i<100; i++){
		pub_command_ci.publish(pose_home1);
    }*/

    //while(true){
// 	pub_command_ci.publish(pose_home1);
    //}

/*    for(int i=0; i<6; i++){
    	pose_home1.data[i] =1.0;
    }*/

	//pub_command_ci.publish(pose_home1);

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

  std::cout << "\r\n\n\n\033[32m\033[1mPress to close the hand... \033[0m" << std::endl;
  getchar();
  sleep(3);
  handClosure(0.8);
  sleep(3);
  
  std::cout << "\r\n\n\n\033[32m\033[1mRest Position \033[0m" << std::endl;
  std::cout << "\r\n\n\n\033[32m\033[1mPress to continue.. \033[0m" << std::endl;
  getchar();
}


//------------------------------------------------------------------------------------------
//                                                                                   manager
//------------------------------------------------------------------------------------------
void move_kuka::manager()
{
  if(manager_done)
  {
    ROS_INFO("Nothing to do here.");
    return;
  }
      
//   ros::Rate r(25); // normale
  ros::Rate r(10); // drinking task

  
  // Elbow and End Effector
  if(!csv_filename_EE.empty() && !csv_filename_EL.empty())
  {
    //load data for EE reference
    std::ifstream csv_file_EE, csv_file_EL;
    csv_file_EE.open (csv_filename_EE);
    csv_file_EL.open (csv_filename_EL);
    if(csv_file_EE.is_open())
	    std::cout << "file end-effector correctly open" << std::endl;
    else
      ROS_ERROR("Could not open input csv file");
    if(csv_file_EL.is_open())
	    std::cout << "file elbow correctly open" << std::endl;
    else
      ROS_ERROR("Could not open input csv file");
    
    unsigned int line = 0;
    while(!csv_file_EE.eof() && !csv_file_EL.eof())
    {
//       if (touched_object)
//       {
// 	ROS_INFO("Detected contact. Stopping trajectory and starting reactive grasping.");
// 	break;
//       }
      
      std::cout << line++ << ": ";
      for (int i=0; i< 16; i++)
      {
	csv_file_EE >> num_EE[i];
	csv_file_EE >> delim;
	std::cout << num_EE[i] << " ";
	
	csv_file_EL >> num_EL[i];
	csv_file_EL >> delim;
	std::cout << num_EL[i] << " ";
      }
      std::cout << std::endl;  
      
      // pub msg
    Eigen::Matrix4d num_matrix_ee, num_matrix_el ;
    num_matrix_ee << num_EE[0], num_EE[4], num_EE[8], num_EE[12]*mm2m*averta,
		     num_EE[1], num_EE[5], num_EE[9], num_EE[13]*mm2m*averta, 
		     num_EE[2], num_EE[6], num_EE[10], num_EE[14]*mm2m*averta, 
		     num_EE[3], num_EE[7], num_EE[11], num_EE[15];
	      
    num_matrix_el << num_EL[0], num_EL[4], num_EL[8], num_EL[12]*mm2m*averta_elbow,
		     num_EL[1], num_EL[5], num_EL[9], num_EL[13]*mm2m*averta_elbow, 
		     num_EL[2], num_EL[6], num_EL[10], num_EL[14]*mm2m*averta_elbow, 
		     num_EL[3], num_EL[7], num_EL[11], num_EL[15];
		    
      Eigen::Affine3d pose_todo_ee,pose_todo_el;
      pose_todo_ee.matrix() = num_matrix_ee;
      pose_todo_ee = offset*pose_todo_ee*ee_offset;

      pose_ = pose_todo_ee;
      
      pose_todo_el.matrix() = num_matrix_el;
      pose_todo_el = offset*pose_todo_el*ee_offset;
      
      tf::poseEigenToMsg(pose_todo_ee, msg_.x_FRI);
      
      tf::poseEigenToMsg(pose_todo_el, msg_el.x_FRI);
      
      geometry_msgs::Pose pose, pose_el;
      pose = msg_.x_FRI;
      pose_el = msg_el.x_FRI;
      

      // publish to vito_bridge_controller
      std_msgs::Float64MultiArray pose_cartesian_interp_;
      pose_cartesian_interp_.data.resize(12);
      pose_cartesian_interp_.data[0] = pose.position.x;
      pose_cartesian_interp_.data[1] = pose.position.y;
      pose_cartesian_interp_.data[2] = pose.position.z;


      double roll,pitch,yaw;	
      tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
      tf::Matrix3x3 mtx(q);
      mtx.getRPY(roll,pitch,yaw);
      pose_cartesian_interp_.data[3] = roll;
      pose_cartesian_interp_.data[4] = pitch;
      pose_cartesian_interp_.data[5] = yaw;
      
      pose_cartesian_interp_.data[6] = pose_el.position.x;
      pose_cartesian_interp_.data[7] = pose_el.position.y;
      pose_cartesian_interp_.data[8] = pose_el.position.z;

      tf::Quaternion q_el(pose_el.orientation.x,pose_el.orientation.y,pose_el.orientation.z,pose_el.orientation.w);
      tf::Matrix3x3 mtx_el(q_el);
      mtx_el.getRPY(roll,pitch,yaw);
      pose_cartesian_interp_.data[9] = roll;
      pose_cartesian_interp_.data[10] = pitch;
      pose_cartesian_interp_.data[11] = yaw;
      
      pub_command_ci.publish(pose_cartesian_interp_);
      // 
      r.sleep();
    }
    csv_file_EE.close();
    csv_file_EL.close();
  }
  // End Effector Only
  else if(!csv_filename_EE.empty())
  {
    //load data for EE reference
    std::ifstream csv_file_EE;
    csv_file_EE.open (csv_filename_EE);
    if(csv_file_EE.is_open())
	    std::cout << "file correctly open" << std::endl;
    else
    ROS_ERROR("Could not open input csv file");
    unsigned int line = 0;
    while(!csv_file_EE.eof())
    {
//       if (touched_object)
//       {
// 	ROS_INFO("Detected contact. Stopping trajectory and starting reactive grasping.");
// 	break;
//       }
      std::cout << line++ << ": ";
      for (int i=0; i< 16; i++)
      {
	csv_file_EE >> num_EE[i];
	csv_file_EE >> delim;
	std::cout << num_EE[i] << " ";
      }
      std::cout << std::endl;  
      
      // pub msg
    Eigen::Matrix4d num_matrix_ee;
    num_matrix_ee << num_EE[0], num_EE[4], num_EE[8], num_EE[12]*mm2m*averta,
		     num_EE[1], num_EE[5], num_EE[9], num_EE[13]*mm2m*averta, 
		     num_EE[2], num_EE[6], num_EE[10], num_EE[14]*mm2m*averta, 
		     num_EE[3], num_EE[7], num_EE[11], num_EE[15];
		
      Eigen::Affine3d pose_todo;
      pose_todo.matrix() = num_matrix_ee;
    //   pose_todo = offset*pose_todo;
      pose_todo = offset*pose_todo*ee_offset;
      
      pose_ = pose_todo;
      
      tf::poseEigenToMsg(pose_todo, msg_.x_FRI);
      geometry_msgs::Pose pose;
      pose = msg_.x_FRI;
      
      // publish to vito_bridge_controller
      std_msgs::Float64MultiArray pose_cartesian_interp_;
      pose_cartesian_interp_.data.resize(6);
      pose_cartesian_interp_.data[0] = pose.position.x;
      pose_cartesian_interp_.data[1] = pose.position.y;
      pose_cartesian_interp_.data[2] = pose.position.z;

	      
      double roll,pitch,yaw;	
      tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
      tf::Matrix3x3 mtx(q);
      mtx.getRPY(roll,pitch,yaw);
      pose_cartesian_interp_.data[3] = roll;
      pose_cartesian_interp_.data[4] = pitch;
      pose_cartesian_interp_.data[5] = yaw;
      
      pub_command_ci.publish(pose_cartesian_interp_);
      // 
      r.sleep();
    }
    csv_file_EE.close();
  }

  touched_object = true; // HACK truffone
  
  if (touched_object)
  {
    ROS_INFO("Managing contact.");
    flag_grasp_ = true;
//     mkTondoDatabase();
    sleep(2);

    handClosure(1.0);
    // handClosure(1.0);
    visual_tools_->deleteAllMarkers();
    sleep(3);
//     finishPosition(0.05);
    sleep(3);
    flag_grasp_ =  false;
    std::cout << "\r\n\n\n\033[32m\033[1mPress to open the hand... \033[0m" << std::endl;
    getchar();
    sleep(3);
    handClosure(0.0);
    sleep(3);
  }

  
  
  
  
  manager_done = true;
  ROS_INFO("Manager done.");


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
	
	x_finish = offset*x_finish*ee_offset;
	
	tf::poseEigenToMsg(x_now, x_now_frame);
	tf::poseEigenToMsg(x_finish, x_finish_frame);

	Eigen::Quaterniond q_start(x_now_frame.orientation.w, x_now_frame.orientation.x, x_now_frame.orientation.y, x_now_frame.orientation.z);
	Eigen::Quaterniond q_finish(x_finish_frame.orientation.w, x_finish_frame.orientation.x, x_finish_frame.orientation.y, x_finish_frame.orientation.z);
	Eigen::Quaterniond q_err;

	ros::Rate r(spin_rate);
	
	while (c <= 1 && ros::ok())
	{
	  std::cout << c << std::endl;
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
// 		pub_command_.publish(msg_);
// 		pub_command_t.publish(pose);
		
		// publish to vito_bridge_controller
		std_msgs::Float64MultiArray pose_cartesian_interp_;
		pose_cartesian_interp_.data.resize(6);
		pose_cartesian_interp_.data[0] = pose.position.x;
		pose_cartesian_interp_.data[1] = pose.position.y;
		pose_cartesian_interp_.data[2] = pose.position.z;
	
		
	double roll,pitch,yaw;	
	tf::Quaternion q(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
	tf::Matrix3x3 mtx(q);
	mtx.getRPY(roll,pitch,yaw);
	pose_cartesian_interp_.data[3] = roll;
	pose_cartesian_interp_.data[4] = pitch;
	pose_cartesian_interp_.data[5] = yaw;
	
		pub_command_ci.publish(pose_cartesian_interp_);
		
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

int move_kuka::interpolation(Eigen::Affine3d x_start_EE, Eigen::Affine3d x_finish_EE, Eigen::Affine3d x_start_ELB, Eigen::Affine3d x_finish_ELB, double traj_time_local)
{
  
  std::cout << "Interpolation will move EE from\n" << x_start_EE.translation() << " to \n " << x_finish_EE.translation() << std::endl;
  std::cout << "Interpolation will rotate EE from\n" << x_start_EE.rotation() << " to \n " << x_finish_EE.rotation() << std::endl;
  
  std::cout << "Interpolation will move ELB from\n" << x_start_ELB.translation() << " to \n " << x_finish_ELB.translation() << std::endl;
  std::cout << "Interpolation will rotate ELB from\n" << x_start_ELB.rotation() << " to \n " << x_finish_ELB.rotation() << std::endl;
  
  float th = 0.001;	//trade error
  float alpha = 0.015;
  float c = 0; //for slerp

  Eigen::Affine3d x_next_EE, x_now_EE, x_prev_EE;
  Eigen::Affine3d x_next_ELB, x_now_ELB, x_prev_ELB;

  x_now_EE  = x_start_EE;
  x_prev_EE = x_start_EE;
  x_now_ELB  = x_start_ELB;
  x_prev_ELB = x_start_ELB;
  
  geometry_msgs::Pose x_finish_frame_EE, x_now_frame_EE;
  geometry_msgs::Pose x_finish_frame_ELB, x_now_frame_ELB;

  // read quaternion from GeometryPoseMsg and convert them to Eigen::Quaterniond
  //passing from affine3d to geometry_msg
  
  x_finish_EE = offset*x_finish_EE*ee_offset;
  x_finish_ELB = offset*x_finish_ELB*ee_offset;
  
  std::cout << "Target EE pose\n" << x_finish_EE.translation() << std::endl;
  std::cout << "Target ELB pose\n" << x_finish_ELB.translation() << std::endl;
  
  tf::poseEigenToMsg(x_now_EE, x_now_frame_EE);
  tf::poseEigenToMsg(x_finish_EE, x_finish_frame_EE);
  tf::poseEigenToMsg(x_now_ELB, x_now_frame_ELB);
  tf::poseEigenToMsg(x_finish_ELB, x_finish_frame_ELB);

  Eigen::Quaterniond q_start_EE(x_now_frame_EE.orientation.w, x_now_frame_EE.orientation.x, x_now_frame_EE.orientation.y, x_now_frame_EE.orientation.z);
  Eigen::Quaterniond q_finish_EE(x_finish_frame_EE.orientation.w, x_finish_frame_EE.orientation.x, x_finish_frame_EE.orientation.y, x_finish_frame_EE.orientation.z);
  Eigen::Quaterniond q_err_EE;

  Eigen::Quaterniond q_start_ELB(x_now_frame_ELB.orientation.w, x_now_frame_ELB.orientation.x, x_now_frame_ELB.orientation.y, x_now_frame_ELB.orientation.z);
  Eigen::Quaterniond q_finish_ELB(x_finish_frame_ELB.orientation.w, x_finish_frame_ELB.orientation.x, x_finish_frame_ELB.orientation.y, x_finish_frame_ELB.orientation.z);
  Eigen::Quaterniond q_err_ELB;
  
  ros::Rate r(spin_rate);
  
  while (c <= 1 && ros::ok())
  {
    std::cout << "\r" << c << std::flush;
    // update orientation
    double ctanh(std::tanh(4*c));
    if (c <= 1)
    {
	    q_err_EE = q_start_EE.slerp(ctanh, q_finish_EE);
	    q_err_ELB = q_start_ELB.slerp(ctanh, q_finish_ELB);
    }

    std::cout << " q_err_EE: " << q_err_EE.w() << q_err_EE.x() << q_err_EE.y() << q_err_EE.z() << std::endl;

    x_next_EE = Eigen::AngleAxisd(q_err_EE);
    x_next_ELB = Eigen::AngleAxisd(q_err_ELB);

    std::cout << " x_next_EE: " << x_next_EE.translation() << std::endl;

//     // visual tool
//     visual_tools_->publishAxis(x_next_EE, 0.1, 0.01, "axis");
//     visual_tools_->trigger();
    
    x_next_EE.translation() = x_now_EE.translation() + ctanh * (x_finish_EE.translation() - x_now_EE.translation() ) ;
    x_next_ELB.translation() = x_now_ELB.translation() + ctanh * (x_finish_ELB.translation() - x_now_ELB.translation() ) ;
    
    // set control
    tf::poseEigenToMsg(x_next_EE, msg_.x_FRI);
    tf::poseEigenToMsg(x_next_ELB, msg_el.x_FRI);
    
//     std::cout << " msg_.x_FRI " << msg_.x_FRI.position.x << ", " << msg_.x_FRI.position.y << ", " << msg_.x_FRI.position.z << std::flush;
//     std::cout << " msg_el.x_FRI " << msg_el.x_FRI << std::flush;
    

    
    geometry_msgs::Pose pose_EE, pose_ELB;
    pose_EE = msg_.x_FRI;
    pose_ELB = msg_el.x_FRI;
    
    // publish to vito_bridge_controller
    std_msgs::Float64MultiArray pose_cartesian_interp_;
    pose_cartesian_interp_.data.resize(12);
    pose_cartesian_interp_.data[0] = pose_EE.position.x;
    pose_cartesian_interp_.data[1] = pose_EE.position.y;
    pose_cartesian_interp_.data[2] = pose_EE.position.z;

    double roll,pitch,yaw;	
    tf::Quaternion q_EE(pose_EE.orientation.x,pose_EE.orientation.y,pose_EE.orientation.z,pose_EE.orientation.w);
    tf::Matrix3x3 mtx_EE(q_EE);
    mtx_EE.getRPY(roll,pitch,yaw);
    pose_cartesian_interp_.data[3] = roll;
    pose_cartesian_interp_.data[4] = pitch;
    pose_cartesian_interp_.data[5] = yaw;
    
    pose_cartesian_interp_.data[6] = pose_ELB.position.x;
    pose_cartesian_interp_.data[7] = pose_ELB.position.y;
    pose_cartesian_interp_.data[8] = pose_ELB.position.z;
    
    
    tf::Quaternion q_ELB(pose_ELB.orientation.x,pose_ELB.orientation.y,pose_ELB.orientation.z,pose_ELB.orientation.w);
    tf::Matrix3x3 mtx_ELB(q_ELB);
    mtx_ELB.getRPY(roll,pitch,yaw);
    pose_cartesian_interp_.data[9] = roll;
    pose_cartesian_interp_.data[10] = pitch;
    pose_cartesian_interp_.data[11] = yaw;

    pub_command_ci.publish(pose_cartesian_interp_);
    
    ros::spinOnce();

    c += (1.0 / spin_rate) / traj_time_local;

    r.sleep();
  }
  std::cout << std::endl;
  // update global pose_ for next steps
  pose_ = x_next_EE;
  
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

	interpolation(pose_, pose_finish, 10.0);

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




//------------------------------------------------------------------------------------------
//                                                                             tondoDatabase
//------------------------------------------------------------------------------------------
void move_kuka::mkTondoDatabase()
{
	Eigen::Affine3d pose_grasp;

	float x, y, z, angle;

	mkOpenTondoDatabase(x, y, z, angle);

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
void move_kuka::mkOpenTondoDatabase(float& x, float& y, float& z, float& angle)
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