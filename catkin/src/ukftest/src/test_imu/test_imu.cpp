/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	test_imu.cpp
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
	ros::init(argc, argv, "test_imu_node");
	ros::NodeHandle nh;
	 
	imusub = nh.subscribe("uav_imu", 10, imuCallback);
	
	cout<< "test_imu start!"<<endl;
	ros::spin();
	cout<< "test_imu shutdown!"<<endl;
	return 1;
}    


void imuCallback(const ukftest::UAV& imu)
{
	Time timenow = Time::now();
	static Time timestart = timenow;
	cout<< "Time passed: "<<timenow - timestart <<endl;
	vector4f q_eb;
	matrix3f R_eb;

	q_eb[0] = imu.orientation.w; 
	q_eb[1] = imu.orientation.x;
	q_eb[2] = imu.orientation.y;
	q_eb[3] = imu.orientation.z;
	
	float yaw2,pitch2,roll2;
    	quat_to_eular(&yaw2, &pitch2, &roll2, q_eb);
	cout<< "yaw   : "<<yaw2 / M_PI * 180.0f<<endl
    	    << "pitch : "<<pitch2 / M_PI * 180.0f<<endl
            << "roll  : "<<roll2 /  M_PI * 180.0f<<endl;
	static float yaw0 = yaw2, pitch0 = pitch2, roll0 = roll2;
	cout<< "yaw0  : "<<yaw0 / M_PI * 180.0f<<endl
    	    << "pitch0: "<<pitch0 / M_PI * 180.0f<<endl
            << "roll0 : "<<roll0 /  M_PI * 180.0f<<endl<<endl;

	cout<< "yaw error: "<< (yaw2 - yaw0)/ M_PI * 180.0f<< endl;

}


