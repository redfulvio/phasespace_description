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
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <tf/transform_datatypes.h>


class glove_coordinates
{

public:

	glove_coordinates();
	
	~glove_coordinates();

	void manager();

private:

	std::ifstream rotation_file_;
	std::ifstream translation_file_;
	std::ifstream acc_file_;
	std::ifstream gyro_file_;

	ros::NodeHandle n_;
	ros::Publisher joint_pub_;
	ros::Subscriber finger_sub_;

	Eigen::MatrixXd RPY_;
	Eigen::MatrixXf Qrotation_;
	Eigen::MatrixXf translation_;

	Eigen::MatrixXf offset_R_;
	Eigen::MatrixXd offset_T_;

	Eigen::MatrixXf acc_;
	Eigen::MatrixXf norm_acc_;

    int step_;
    int matrix_size_;

    std::string s_;
    std::string pkg_path_;	

    sensor_msgs::JointState joint_state_;

	void openAndLoad();

	void checkFinger(std_msgs::String finger);

	bool check_finger_flag_;
	bool load_data_flag_;

	void updateJointStates();

	void sentJointState();

	void resetJointState();

	void checkHit();

};