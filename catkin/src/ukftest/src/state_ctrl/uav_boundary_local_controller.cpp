/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	uav_boundary_local_controller.cpp
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
#include <ukftest/Ctrl.h>
#include <geometry_msgs/TransformStamped.h>

using namespace std;
using namespace ros;
// Ros
ros::Publisher ctrlpub;
ros::Subscriber viconsub;
//////////////

bool is_debug_on = true;

bool isOut = false;

double pitchVal;
double rollVal;
double pitchRang;
double rollRang;

double x_0, y_0 , x_1, y_1 , x_2, y_2 , x_3, y_3;

int timeCnt;

float centerX, centerY;

void boundaryCtrl();
void viconCallback(const geometry_msgs::TransformStamped& msg);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_boundary_local_controller_node");
	ros::NodeHandle nh;

	nh.param("pitch_range", pitchRang, 150.0); 
	nh.param("roll_range", rollRang, 150.0); 
 
	nh.param("x_0", x_0, 0.0); 
	nh.param("y_0", y_0, 0.0); 
	nh.param("x_1", x_1, 5.0); 
	nh.param("y_1", y_1, 5.0); 

	nh.param("x_2", x_2, 2.0); 
	nh.param("y_2", y_2, 2.0); 
	nh.param("x_3", x_3, 3.0); 
	nh.param("y_3", y_3, 3.0); 	
	// 1s
	ros::Rate loop_rate(10);

	ctrlpub = nh.advertise<ukftest::Ctrl>("boundary_ctrl",10);
	viconsub = nh.subscribe("/improc_and_localization/position_estimation", 10,viconCallback);

	cout<< "uav random controller start!"<<endl;
	
	centerX = (x_2 + x_3) / 2.0;
	centerY = (y_2 + y_3) / 2.0;
	while(ros::ok())
	{

		boundaryCtrl();
		ros::spinOnce();
    		loop_rate.sleep();		
	}

	cout<< "uav random controller shutdown!"<<endl;
	return 1;
}    

void boundaryCtrl()
{
	ukftest::Ctrl outMsg;
	outMsg.header.stamp = ros::Time::now();
	outMsg.header.frame_id = "boundary_ctrl";
	
	if(isOut)
	{
		outMsg.isCtrl = 1;
		outMsg.ctrl.z = (int)(0);         //yaw_rate  
		outMsg.ctrl.x = (int)(pitchVal);  //pitch
		outMsg.ctrl.y = (int)(rollVal);   //roll
		outMsg.ctrl.w = (int)(0);         //vel
		ctrlpub.publish(outMsg);
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
		cout<< "isOut:" << outMsg.isCtrl << endl;

	}
}

void viconCallback(const geometry_msgs::TransformStamped& msg)
{
	float x = msg.transform.translation.x;
	float y = msg.transform.translation.y;


	if(x > x_1 || x < x_0 || y > y_1 || y < y_0 && timeCnt <(10*4) &&  timeCnt >(10*2))
	{
		isOut = true;
		if(x > x_1) pitchVal =  pitchRang;
		else if(x > x_0 && x <= x_1) pitchVal = 0.0;
			else pitchVal = -pitchRang;
		
		if(y > y_1) rollVal = rollRang;
		else if(y > y_0 && y <= y_1) rollVal = 0.0;
			else rollVal  = -rollRang;
		timeCnt ++ ;
		if(timeCnt > 38) timeCnt =0;
	}
	else if(x > x_2 && x < x_3 && y > y_2 && y < y_3)
	{
		isOut = true;
		if(x < centerX) pitchVal =  pitchRang;
		else pitchVal =  -pitchRang;
		if(y < centerY) rollVal =  rollRang;
		else rollVal =  -rollRang;
	}
	else if(timeCnt >= 0)
	{
		isOut = false;
		pitchVal = 0.0;
		rollVal  = 0.0;
		timeCnt ++;
		if(timeCnt> 40)  timeCnt=0;
	}
}


