/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	simulate_imuvo_node.cpp
* Brief: a simulation node for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/6/18 21:00
* History: 
************************************************************************/

#include <ros/ros.h>
// msg
#include <ukftest/UAV.h>
#include <ukftest/trackInfo.h>

using namespace ros;
using namespace std;

double imuFreq;
double voFreq;
Publisher imu_pub;
Publisher vo_pub;
float a = 0.01;
float s = 0.0;
float imu_cont = 0 ,vo_cont = 0;;

float  Tfunc(float a, float dt)
{
	return a * dt * dt / 2.0;
}

float Vfunc(float a , float dt)
{
	return a * dt;
}

float Sfunc(float v, float dt)
{
	s = s + v * dt;
	return s;
}
void imuCallback(const ros ::TimerEvent& e)
{
	
	//ROS_INFO("imu send!!!");
	//std::cout<<"imu Freq: "<<imuFreq<<std::endl;
	ukftest::UAV imu;
	imu_cont ++;

	imu.orientation.w = 1;
	imu.orientation.x = 0;
	imu.orientation.y = 0;
	imu.orientation.z = 0;
	
	imu.linear_a.x    = a;
	imu.linear_a.y    = 0;
	imu.linear_a.z    = 0;

	imu.angular_v.x   = 0;
	imu.angular_v.y   = 0;
	imu.angular_v.z   = 0;
	
	float v =-Vfunc(a, 1.0/imuFreq*imu_cont);
	cout << "imu_freq: "<< imuFreq<<" Hz\n";
	cout << "vo_freq : "<< voFreq <<" Hz\n";
	cout << "v_tc_y  : "<< v  << endl;
	cout << "s_tc_y  : "<< Sfunc(v, 1.0/imuFreq)<<endl;
	imu_pub.publish(imu);
	
}

void voCallback(const ros ::TimerEvent& e)
{
	//ROS_INFO("vo send!!!");
	//std::cout<<"vo Freq: "<<voFreq<<std::endl;
	
	ukftest::trackInfo vo;
	vo_cont ++;

	vo.pose.orientation.w = 1;
	vo.pose.orientation.x = 0;
	vo.pose.orientation.y = 0;
	vo.pose.orientation.z = 0;

	vo.pose.position.x    = 0;
	vo.pose.position.y    = - Tfunc(a, 1.0/voFreq*vo_cont);
	vo.pose.position.z    = 0;
	
	vo.isTracking         = 1;

	vo_pub.publish(vo);

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "simulate_imuvo_node");
	ros::NodeHandle nh;
	ros::NodeHandle nh2("~");

	imu_pub = nh.advertise<ukftest::UAV>("imu_measurement", 10);
	vo_pub = nh.advertise<ukftest::trackInfo>("vo_measurement", 10);
	//add a timer
	ros::Timer imuTimer;
	ros::Timer voTimer;
	nh2.param("imu_freq", imuFreq, 200.0);
	nh2.param("vo_freq", voFreq, 25.0);

	imuTimer = nh.createTimer(ros::Duration(1.0/imuFreq), imuCallback);
	voTimer  = nh.createTimer(ros::Duration(1.0/voFreq), voCallback);

	ros::spin();
  
	return 0;
}
