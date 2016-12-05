#include "glove_broadcaster.h"


glove_coordinates::glove_coordinates()
{

	joint_pub_ = n_.advertise<sensor_msgs::JointState>("joint_states", 1);

	vr_.resize(1,4);
	vt_.resize(1,3);

	count_ = 0;


	odom_trans.header.frame_id = "world";
    odom_trans.child_frame_id = "glied_link";

}

/***************************************************************************************/
/***************************************************************************************/

glove_coordinates::~glove_coordinates()
{
	rotation_file_.close();
	translation_file_.close();
}

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::openAndLoad()
{
	int i=0;
	int j=0;

	std::string s, s1, s2;

	s = "index";

	s2 = "src/phasespace_imu_integration/measurements/" + s + "_traslation.txt";
	s1 = "src/phasespace_imu_integration/measurements/" + s + "_rotation.txt";

	rotation_file_.open(s1.c_str() );
	translation_file_.open(s1.c_str());

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


	std::string line;

	for (i=0; std::getline(rotation_file_, line); i++)
	{
		rotation_file_ >> m_rotation_(i,1);
		rotation_file_ >> m_rotation_(i,2);
		rotation_file_ >> m_rotation_(i,3);
		rotation_file_ >> m_rotation_(i,4);
	}

	for (j=0; std::getline(translation_file_, line); j++)
	{
		translation_file_ >> m_translation_(j,1);
		translation_file_ >> m_translation_(j,2);
		translation_file_ >> m_translation_(j,3);
	}

	std::cout <<"size m_rotation_\t" << m_rotation_.size() << std::endl;
	std::cout <<"size m_translation_\t" << m_translation_.size() << std::endl;


}

/***************************************************************************************/
/***************************************************************************************/

// void glove_coordinates::newCoordinates()
// {
// 	// transform_.setOrigin( tf::Vector3( m_translation_(count_,0), m_translation_(count_,1), m_translation_(count_,2) ) );
//  //  	transform_.setRotation( tf::Quaternion( m_rotation_(count_,0), m_rotation_(count_,1), m_rotation_(count_,2), m_rotation_(count_,3) ) );

// 	transform_.setOrigin( tf::Vector3(count_/10, count_/20, 0 ) );
//   	transform_.setRotation( tf::Quaternion( 0, 2, 0, 1 ) );

//   	count_++;
// }

/***************************************************************************************/
/***************************************************************************************/

// void glove_coordinates::sendCoordinates()
// {
// 	br_.sendTransform(tf::StampedTransform(transform_, ros::Time::now(), "world", "glied"));
// }

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::updateJointStates()
{
	joint_state.header.stamp = ros::Time::now();

	joint_state.name.resize(7);
	joint_state.name[0] = "glove_erste_joint_roll";
	joint_state.position[0] = 0;

	joint_state.name[1] = "glove_erste_joint_pitch";
	joint_state.position[1] = 0;

	joint_state.name[2] = "glove_erste_joint_yaw";
	joint_state.position[2] = 0;

	joint_state.name[3] = "glove_erste_joint_x";
	joint_state.position[3] = 0;

	joint_state.name[4] = "glove_erste_joint_y";
	joint_state.position[4] = 0;

	joint_state.name[5] = "glove_erste_joint_z";
	joint_state.position[5] = 0;

	joint_state.name[6] = "glied";
	joint_state.position[6] = 0;
}


/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::updateTransform()
{
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.transform.translation.x = count_/10;
    odom_trans.transform.translation.y = count_/20;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(1/2);
}

/***************************************************************************************/
/***************************************************************************************/

void glove_coordinates::sentJointStateAndTrandform()
{
    joint_pub_.publish(joint_state);
    br_.sendTransform(odom_trans);

    count_++;
}

/***************************************************************************************/
/***************************************************************************************/


void glove_coordinates::master()
{
	updateJointStates();

	updateTransform();

	sentJointStateAndTrandform();
}