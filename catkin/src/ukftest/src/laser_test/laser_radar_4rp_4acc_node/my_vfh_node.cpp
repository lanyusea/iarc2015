#include <ros/ros.h>
#include "myRadar.h"

int main (int argc, char **argv)
{
	ros::init(argc, argv, "laser_subscriber");
	OpenRadar openRadar;
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	int buffer_size;
	pnh.param<int>("buffer_size", buffer_size, 5);
	ros::Subscriber sub=n.subscribe("/scan", 1, &OpenRadar::laser_callback, &openRadar);
	ros::spin();
	return 0;	
}
