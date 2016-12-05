#include <ros/ros.h>
#include <ros/rate.h>
#include <boost/asio.hpp>
#include "std_msgs/String.h"
#include "glove_broadcaster.h"

using namespace std;



int kbhit(void) 
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "phasespace_description_node");

	ros::NodeHandle n;

	glove_coordinates GC;

	std::string s;

  // ros::Rate loop_rate(30);


  // std::cout<<"\n\n## Choose file to load ##\n\n"<<std::endl;
  // cin >> s;
  // GC.openAndLoad();  //decido quale file caricare


	while(ros::ok())
	{

		GC.master();

    ros::spinOnce();

	}

	return 0;

}


