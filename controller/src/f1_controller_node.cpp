#include "initial/f1_controller.h"
#include <ros/ros.h>


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "f1_controller_node");
	ros::NodeHandle nh;
	
	F1Controller controller(nh);
	
	ros::spin();
	
	return 0;
}
