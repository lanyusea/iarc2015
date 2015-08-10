/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	reg_rest_node.cpp
* Brief: 
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/25 22:00
* History:
************************************************************************/
// ros
#include "ros/ros.h"

#include <ukftest/UAV.h>

//YU YUN
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

using namespace std;
using namespace ros;
// Ros
ros::Subscriber imusub;
void imuCallback(const ukftest::UAV& imu);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "reg_test_node");
	ros::NodeHandle nh;

	imusub = nh.subscribe("uav_imu", 10, imuCallback);
	
	cout<< "reg test node start!"<<endl;
	ros::spin();
	cout<< "reg test node shutdown!"<<endl;
	return 0;
}    

void imuCallback(const ukftest::UAV& imu)
{
	cout<< "b_laser_IMUcall!"<<endl;

	vector4f q_be;

	q_be[0] = imu.orientation.w; 
	q_be[1] = -imu.orientation.x;
	q_be[2] = -imu.orientation.y;
	q_be[3] = -imu.orientation.z;

	float yaw,pitch,roll;
    	quat_to_eular(&yaw, &pitch, &roll, q_be);
	cout<< "yaw  : "<<yaw / M_PI * 180.0f<<endl
    	    << "pitch: "<<pitch / M_PI * 180.0f<<endl
            << "roll : "<<roll /  M_PI * 180.0f<<endl<<endl;

}
