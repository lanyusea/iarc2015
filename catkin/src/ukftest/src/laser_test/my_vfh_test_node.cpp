#include <ros/ros.h>
#include "OpenRadar.h"
//#include "imuReader.h"



int main (int argc, char **argv)
{
	ros::init(argc, argv, "laser_subscriber");
	OpenRadar openRadar;
 	//imuReader* my_imu;
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");

	bool has_imu;
	pnh.param<bool>("has_imu", has_imu, false);
	int buffer_size;
	pnh.param<int>("buffer_size", buffer_size, 5);
	//subscriber get ros topic and call class openRadar to process the data
	//ros::Subscriber sub=n1.subscribe("/iarc_uav/laser/scan", 10, &OpenRadar::laser_callback, &openRadar);
	ros::Subscriber sub=n.subscribe("/scan", 10, &OpenRadar::laser_callback, &openRadar);
	//ros::Subscriber imu_sub;
	if (has_imu)
	{
		//my_imu = new imuReader(buffer_size);
		//imu_sub = n.subscribe("/uav_info",5,&imuReader::read, my_imu);
	}
	ros::spin();
	return 0;	
}
