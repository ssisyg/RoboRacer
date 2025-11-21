#include <ros/ros.h>
#include "initial/racing_line_planner.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "racing_line_node");

	ros::NodeHandle nh;

	RacingLinePlanner racingLine(nh);
	ros::spin();


	return 0;
}
