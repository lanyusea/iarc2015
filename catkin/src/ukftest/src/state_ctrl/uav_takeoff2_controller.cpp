/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	uav_takeoff_controller.cpp
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
#include <ukftest/Ctrl.h>
#include <ukftest/UAV.h>


using namespace std;
using namespace ros;
// Ros
ros::Subscriber statussub,imusub;
ros::Publisher ctrlpub, timepub;
ros::Time initTime;
//////////////
int takeoffCnt = 0 ;
int roll = -200;

bool is_debug_on = false;
bool isInitTime  = false;
bool getHeight = false;
double setHeight;
double takeoffTime;

void takeoff();
void statusCallback(const std_msgs::Float32& statusMsg);
void imuCallback(const ukftest::UAV& imu);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_takeoff_controller_node");
	ros::NodeHandle nh;

	nh.param("takeoff_time", takeoffTime, 3.0);
	nh.param("set_height", setHeight, 1.7);
	ros::Rate loop_rate(20);
	
	statussub = nh.subscribe("uav_flight_status", 10, statusCallback);
	imusub    = nh.subscribe("/uav_imu", 5, imuCallback);
	ctrlpub = nh.advertise<ukftest::Ctrl>("takeoff_ctrl",10);
	timepub = nh.advertise<std_msgs::Float32>("takeoff_time",10);

	cout<< "uav takeoff controller start!"<<endl;

	while(ros::ok())
	{
		takeoff();
		ros::spinOnce();
    		loop_rate.sleep();		
	}

	cout<< "uav takeoff controller shutdown!"<<endl;
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

void takeoff()
{
	ukftest::Ctrl outMsg;
	if(isInitTime)
	{
		float dt = (Time::now() - initTime).toSec();
		if( dt > takeoffTime && getHeight) // time to landing
		{
			outMsg.isCtrl = 0;
			outMsg.ctrl.z = (int)(0);         //yaw_rate  
			outMsg.ctrl.x = (int)(0);         //pitch
			outMsg.ctrl.y = (int)(0);         //roll
			outMsg.ctrl.w = (int)(0);         //vel
			ctrlpub.publish(outMsg);
			roll = 0;
		}
		else if( getHeight)
		{
			outMsg.isCtrl = 1;
			outMsg.ctrl.z = (int)(0);         //yaw_rate  
			outMsg.ctrl.x = (int)(0);         //pitch
			outMsg.ctrl.y = (int)(0);         //roll
			outMsg.ctrl.w = (int)(0);         //vel
			ctrlpub.publish(outMsg);
		}
		else 
		{
			outMsg.isCtrl = 1;
			outMsg.ctrl.z = (int)(0);         //yaw_rate  
			outMsg.ctrl.x = (int)(0);         //pitch
			outMsg.ctrl.y = (int)(roll);         //roll
			outMsg.ctrl.w = (int)(-8);         //vel 10cm/s
			ctrlpub.publish(outMsg);
		}
		std_msgs::Float32 timeMsg;
		float timeRemain;
		timeRemain = takeoffTime - dt;
		if(timeRemain < 0) timeRemain = 0.0f;
		timeMsg.data = timeRemain;
		timepub.publish(timeMsg);

	}
	else
	{
		outMsg.isCtrl = 0;
		outMsg.ctrl.z = (int)(0);         //yaw_rate  
		outMsg.ctrl.x = (int)(0);         //pitch
		outMsg.ctrl.y = (int)(0);         //roll
		outMsg.ctrl.w = (int)(0);         //vel
		ctrlpub.publish(outMsg);
	}
	if(is_debug_on)
	{
		cout<< "Out put massage: \n";
		cout<< "yaw rate  :" << outMsg.ctrl.z << endl;
		cout<< "pitch     :" << outMsg.ctrl.x << endl;
		cout<< "roll      :" << outMsg.ctrl.y << endl;
		cout<< "vel       :" << outMsg.ctrl.w << endl;
		cout<< "isCtrl:" << outMsg.isCtrl << endl;
	}
}


void imuCallback(const ukftest::UAV& imu)
{
	//cout<< "IMUcall!"<<endl;
	float imuHeight = imu.height;
	//cout<< "IMU.height"<<imu.height<<endl;
	if (imu.height > setHeight)
		getHeight = true;
	
}
