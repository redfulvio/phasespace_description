#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <csignal>
#include <string>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>


class glove_coordinates
{
	std::ifstream rotation_file_;
	std::ifstream translation_file_;

	ros::NodeHandle n_;
	ros::Publisher joint_pub_;

	std::vector<float> vr_;
	std::vector<float> vt_;
	Eigen::MatrixXf m_rotation_;
	Eigen::MatrixXf m_translation_;

	tf::Transform transform_;
    tf::Quaternion q_;

    int count_;

	tf::TransformBroadcaster br_;


    sensor_msgs::JointState joint_state;
    geometry_msgs::TransformStamped odom_trans;

public:

	
	glove_coordinates();
	
	~glove_coordinates();

	// void openAndLoad(std::string s);
	void openAndLoad();


	// void newCoordinates();

	// void sendCoordinates();

	// void setJointStates();



	void updateJointStates();

	void updateTransform();

	void sentJointStateAndTrandform();

	void master();

};