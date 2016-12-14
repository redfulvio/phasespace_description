#include "glove_broadcaster.h"


glove_coordinates::glove_coordinates()
{
	joint_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1000);
	finger_sub_ = n_.subscribe("/which_finger", 100, &glove_coordinates::checkFinger, this);

	step_ = 1;

	pkg_path_ = ros::package::getPath("phasespace_description");

	check_finger_flag_ = false;
	load_data_flag_ = false;

	joint_state_.name.resize(7);
	joint_state_.position.resize(7);
	joint_state_.name[0] = "right_hand_synergy_joint";
	joint_state_.name[1] = "glove_erste_joint_roll";
	joint_state_.name[2] = "glove_erste_joint_pitch";
	joint_state_.name[3] = "glove_erste_joint_yaw";
	joint_state_.name[4] = "glove_erste_joint_x";
	joint_state_.name[5] = "glove_erste_joint_y";
	joint_state_.name[6] = "glove_erste_joint_z";
}

/***************************************************************************************/
/***************************************************************************************/

glove_coordinates::~glove_coordinates()
{
}

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::checkFinger(std_msgs::String finger)
{
	s_ = finger.data;
	check_finger_flag_ = true;
	load_data_flag_ = true;
}

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::openAndLoad()
{
	int i=0;
	int j=0;

	std::string s, s1, s2, s3, s4;
	std::string line1, line2, line3, line4;

	std::vector<std::string> q_vector_string;
	std::vector<std::string> t_vector_string;

	std::vector<std::string> acc_vector_string;

	double roll, pitch, yaw;

	// s = "little";
	
	s1 = pkg_path_ + "/measurements/" + s_ + "_rotation.txt";
	s2 = pkg_path_ + "/measurements/" + s_+ + "_traslation.txt";
	s3 = pkg_path_ + "/measurements/" + s_ + "_acc.txt";
	s4 = pkg_path_ + "/measurements/" + s_ + "_gyro.txt";

	//open files
	rotation_file_.open(s1.c_str());
	translation_file_.open(s2.c_str());
	acc_file_.open(s3.c_str());
	gyro_file_.open(s4.c_str());

	//check
	if(rotation_file_.is_open()){
		std::cout << "rotation_file_ is open" << std::endl;
	}
	else{
		std::cout <<" error occured while opening rotation_file_" << std::endl;
	}

	if(translation_file_.is_open()){
		std::cout << "translation_file_ is open" << std::endl;
	}
	else{
		std::cout <<" error occured while opening translation_file_" << std::endl;
	}


	//take rows number
	for (int p=0; std::getline(gyro_file_, line4); p++)
		matrix_size_ = p;

	matrix_size_++;


	Qrotation_.resize(matrix_size_,4); 
	RPY_.resize(matrix_size_,3); 
	translation_.resize(matrix_size_,3);
	offset_R_.resize(1,3);
	offset_T_.resize(1,3);

	acc_.resize(matrix_size_,21);
	norm_acc_.resize(matrix_size_,7);

	//load translation matrix
	for (i=0; std::getline(translation_file_, line2); i++)
	{
		boost::split(t_vector_string, line2, boost::is_any_of("\t\n"));

		for (j=0; j<3; j++)
		{
			float n = std::atof(t_vector_string[j].c_str());
			translation_(i,j) = n;
		}
	}


		//load rotation matrix
	for (i=0; std::getline(rotation_file_, line1); i++)
	{
		boost::split(q_vector_string, line1, boost::is_any_of("\t\n"));
		for (j=0; j<4; j++)
		{
			float n = std::atof(q_vector_string[j].c_str());
			Qrotation_(i,j) = n;
		}
	}


	//build matrix with roll, pitch and yaw angles
	for (i= 0; i<matrix_size_; i++)
	{
		tf::Quaternion q(Qrotation_(i,1), Qrotation_(i,2), Qrotation_(i,3), Qrotation_(i,0));
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);
		RPY_(i,0) = roll;
		RPY_(i,1) = pitch;
		RPY_(i,2) = yaw;
	}

	// build offset values
	for (i=0; i<3; i++)
	{
		offset_T_(0,i) = translation_(0,i);
		offset_R_(0,i) = RPY_(0,i);
	}

	// load acc
	for (i=0; std::getline(acc_file_, line3); i++)
	{
		boost::split(acc_vector_string, line3, boost::is_any_of("\t\n"));
		for (j=0; j<21; j++)
		{
			float n = std::atof(acc_vector_string[j].c_str());
			acc_(i,j) = n;
		}
	}

	//normalizing acc
	for (i=0; i<matrix_size_; i++)
	{
		for (j=0; j<7; j++)
		{
			norm_acc_(i,j) = sqrt( pow(acc_(i,(3*j)),2) + pow(acc_(i,(3*j+1)),2) + pow(acc_(i,(3*j+2)),2) ); 
		}
	}
}

/***************************************************************************************/
/***************************************************************************************/


void glove_coordinates::updateJointStates()
{
	joint_state_.position[1] = RPY_(step_,0) - offset_R_(0,0);
	joint_state_.position[2] = RPY_(step_,1) - offset_R_(0,1);
	joint_state_.position[3] = RPY_(step_,2) - offset_R_(0,2);
	joint_state_.position[4] = - ( translation_(step_,0) - offset_T_(0,0) );
	joint_state_.position[5] = - ( translation_(step_,1) - offset_T_(0,1) );
	joint_state_.position[6] = - ( translation_(step_,2) - offset_T_(0,2) );
}

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::sentJointState()
{
	joint_state_.header.stamp = ros::Time::now();
    joint_pub_.publish(joint_state_);
    ros::spinOnce();
}

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::resetJointState()
 {
 	joint_state_.position[0] = 0 ;
	joint_state_.position[1] = 0 ;
 	joint_state_.position[2] = 0 ;
 	joint_state_.position[3] = 0 ;
 	joint_state_.position[4] = 0 ;
 	joint_state_.position[5] = 0 ;
 	joint_state_.position[6] = 0 ;

 	sentJointState();
 }

/***************************************************************************************/
/***************************************************************************************/

 void glove_coordinates::checkHit()
 {
 	int j;
 	double trade = 1.2;
 	// 1.2 for little and index

 	for (j=0; j<5; j++)
 	{
 		if (norm_acc_(step_,j) > trade)
 		{
 			std::cout<< "finger number " << j << "hit" << std::endl;
 		}
 	}
 	
 }

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::manager()
{
	if (load_data_flag_)
	{
		resetJointState();

		openAndLoad();

		load_data_flag_ = false;
	}

	if(step_ < matrix_size_ && check_finger_flag_)
	{
		
		updateJointStates();

		sentJointState();

		checkHit();

		step_++;
	}
	else
	{
		step_ = 1;
		check_finger_flag_ = false;

		rotation_file_.close();
		translation_file_.close();
		acc_file_.close();
		gyro_file_.close();

	}
}