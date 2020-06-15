#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "nezha_base/serial/serial_interface.h"



/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "test_nezha_base_node");
	base::SerialInterface serial_interface;
	serial_interface.loop();
	return 0;

}
