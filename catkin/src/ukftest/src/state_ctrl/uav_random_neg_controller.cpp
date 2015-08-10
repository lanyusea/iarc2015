/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	uav_random_neg_controller.cpp
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
#include <ukftest/randomCtrl.h>

using namespace std;
using namespace ros;
// Ros
ros::Publisher ctrlpub;
ros::Time newTime, oldTime;
//////////////

int timeCnt = 0;

bool is_debug_on;
double loopTime;
double randFlytime;
double pitchVal;

void randomFly();

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_random_neg_controller_node");
	ros::NodeHandle nh;

	nh.param("loop_time", loopTime, 10.0); 
	nh.param("random_flying_time", randFlytime, 3.0); 
	nh.param("pitch_value", pitchVal, -200.0); 

	// 1s
	ros::Rate loop_rate(1);

	is_debug_on = true;

	ctrlpub = nh.advertise<ukftest::randomCtrl>("random_ctrl",10);

	cout<< "uav random controller start!"<<endl;
	
	//oldTime = Time::now();

	while(ros::ok())
	{

		randomFly();
		ros::spinOnce();
    		loop_rate.sleep();		
	}

	cout<< "uav random controller shutdown!"<<endl;
	return 1;
}    

void randomFly()
{
	timeCnt ++;
	//newTime = Time::now();
	//float dt = (newTime - oldTime).toSec();
	ukftest::randomCtrl outMsg;
	outMsg.header.stamp = ros::Time::now();
	outMsg.header.frame_id = "laser_random_ctrl";
	
	if(timeCnt > 0 && timeCnt <= (int)randFlytime)
	{
		outMsg.isRandflying = 1;
		outMsg.ctrl.z = (int)(0);         //yaw_rate  
		outMsg.ctrl.x = (int)(pitchVal);      //pitch
		outMsg.ctrl.y = (int)(0);         //roll
		outMsg.ctrl.w = (int)(0);         //vel
		ctrlpub.publish(outMsg);
	}
	else
	{
		outMsg.isRandflying = 0;
		outMsg.ctrl.z = (int)(0);         //yaw_rate  
		outMsg.ctrl.x = (int)(0);         //pitch
		outMsg.ctrl.y = (int)(0);         //roll
		outMsg.ctrl.w = (int)(0);         //vel
		ctrlpub.publish(outMsg);
	}
	if(timeCnt == (int)loopTime) timeCnt = 0;
	if(is_debug_on)
	{

		cout<< "Out put massage: \n";
		cout<< "yaw rate  :" << outMsg.ctrl.z << endl;
		cout<< "pitch     :" << outMsg.ctrl.x << endl;
		cout<< "roll      :" << outMsg.ctrl.y << endl;
		cout<< "vel       :" << outMsg.ctrl.w << endl;
		cout<< "isRandflying:" << outMsg.isRandflying << endl;

	}
}

