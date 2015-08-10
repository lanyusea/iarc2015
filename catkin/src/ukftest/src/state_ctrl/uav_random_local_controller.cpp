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

// math
#include <math.h>
#include <stdlib.h>

// msg
#include <ukftest/randomCtrl.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace ros;
// Ros
ros::Publisher ctrlpub;
ros::Subscriber viconsub;
//////////////

int timeCnt = 0;

bool is_debug_on;
double loopTime;
double randFlytime;
double pitchVal;
double rollVal;
double pitchRang;
double rollRang;

double x_0, y_0 , x_1, y_1;

void randomFly();
void viconCallback(const geometry_msgs::TransformStamped& msg);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_random_neg_controller_node");
	ros::NodeHandle nh;

	nh.param("loop_time", loopTime, 10.0); 
	nh.param("random_flying_time", randFlytime, 3.0); 
	nh.param("init_pitch_value", pitchVal, -200.0); 
	nh.param("init_roll_value", rollVal, 0.0);
	nh.param("random_pitch_range", pitchRang, 150.0); 
	nh.param("rabdom_roll_range", rollRang, 150.0); 
 
	nh.param("random_x_0", x_0, 2.0); 
	nh.param("random_y_0", y_0, 2.0); 
	nh.param("random_x_1", x_1, 3.0); 
	nh.param("random_y_1", y_1, 3.0); 
	// 1s
	ros::Rate loop_rate(1);

	is_debug_on = true;

	ctrlpub = nh.advertise<ukftest::randomCtrl>("random_ctrl",10);
	viconsub = nh.subscribe("/improc_and_localization/position_estimation", 10,viconCallback);

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

	ukftest::randomCtrl outMsg;
	outMsg.header.stamp = ros::Time::now();
	outMsg.header.frame_id = "laser_random_ctrl";
	
	if(timeCnt > 0 && timeCnt <= (int)randFlytime)
	{
		outMsg.isRandflying = 1;
		outMsg.ctrl.z = (int)(0);         //yaw_rate  
		outMsg.ctrl.x = (int)(pitchVal);  //pitch
		outMsg.ctrl.y = (int)(rollVal);   //roll
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

void viconCallback(const geometry_msgs::TransformStamped& msg)
{

	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;

	float x0 = abs(x - x_0);
	float x1 = abs(x - x_1);
	float y0 = abs(y - y_0);
	float y1 = abs(y - y_1);
	
	if (x0 < 0.5)  pitchVal = -pitchRang;
	else if (x1 < 0.5)  pitchVal =  pitchRang;
	else if (y0 < 0.5)  rollVal  = -rollRang;
	else if (y1 < 0.5)  rollVal  =  rollRang;
	else
	{
		int a = rand() % 4;
		if(a == 0)
		{
			// up-right
			pitchVal = -pitchRang;
			rollVal  =  rollRang;
		}
		else if (a == 1)
		{
			// up-left
			pitchVal = -pitchRang;
			rollVal  = -rollRang;
		}
		else if (a == 2)
		{
			// down-left
			pitchVal = pitchRang;
			rollVal  = -rollRang;
		}
		else
		{
			// down-right
			pitchVal = pitchRang;
			rollVal  = rollRang;
		}
	}
}


