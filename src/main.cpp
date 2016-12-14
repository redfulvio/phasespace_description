#include <ros/ros.h>
#include <ros/rate.h>
#include <boost/asio.hpp>
#include "std_msgs/String.h"
#include "glove_broadcaster.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "phasespace_description_node");

  ros::NodeHandle n;
  
  glove_coordinates GC;

  ros::Rate loop_rate(30);


	while(ros::ok())
	{
		GC.manager();

    ros::spinOnce();

    loop_rate.sleep();
	}

	return 0;

}


