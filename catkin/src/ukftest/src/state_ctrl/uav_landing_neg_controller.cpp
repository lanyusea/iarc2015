/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	uav_landing_neg_controller.cpp
* Brief: 
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/25 22:00
* History:
************************************************************************/
// ros
#include "ros/ros.h"

// msg
#include <std_msgs/Float32.h>

using namespace std;
using namespace ros;
// Ros
ros::Subscriber statussub;
ros::Publisher ctrlpub, timepub;
ros::Time initTime;
//////////////

bool is_debug_on = true;
bool isInitTime  = false;

double landingTime;

void landing();
void statusCallback(const std_msgs::Float32& statusMsg);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_landing_neg_controller_node");
	ros::NodeHandle nh;

	nh.param("landing_time", landingTime, 15.0);
	// 1s
	ros::Rate loop_rate(1);
	
	statussub = nh.subscribe("uav_flight_status", 10, statusCallback);
	ctrlpub = nh.advertise<std_msgs::Float32>("landing_ctrl",10);
	timepub = nh.advertise<std_msgs::Float32>("landing_time",10);

	cout<< "uav landing controller start!"<<endl;

	while(ros::ok())
	{

		landing();
		ros::spinOnce();
    		loop_rate.sleep();		
	}

	cout<< "uav landing controller shutdown!"<<endl;
	return 1;
}    

void statusCallback(const std_msgs::Float32& statusMsg)
{
	if(!isInitTime && statusMsg.data == 3) 
	{
		initTime = Time::now();
		isInitTime = true;	
	}
	if(isInitTime && statusMsg.data == 1) 
	{
		isInitTime = false;	
	}
}

void landing()
{
	std_msgs::Float32 outMsg;
	if(isInitTime)
	{
		float dt = (Time::now() - initTime).toSec();
		if( dt > landingTime) // time to landing
		{
			outMsg.data = 1;
			ctrlpub.publish(outMsg);
		}
		else
		{
			outMsg.data = 0;
			ctrlpub.publish(outMsg);
		}
		std_msgs::Float32 timeMsg;
		float timeRemain;
		timeRemain = landingTime - dt;
		if(timeRemain < 0) timeRemain = 0.0f;
		timeMsg.data = timeRemain;
		timepub.publish(timeMsg);

	}
	else
	{
		outMsg.data = 0;
		ctrlpub.publish(outMsg);
	}
	if(is_debug_on)
	{

		cout<< "Out put massage: \n";
		cout<< "isLanding:" << outMsg.data << endl;

	}
}

