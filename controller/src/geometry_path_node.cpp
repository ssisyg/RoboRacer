#include <ros/ros.h>
#include "initial/geometry_path.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "geometry_path_node");
    ros::NodeHandle nh;

    initial::GeometryPath geometryPath(nh);

    ros::spin();
    return 0;
}

