/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	uav_opticalflow_neg_controller.cpp
* Brief: 
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/25 22:00
* History:
************************************************************************/
// ros
#include "ros/ros.h"
// pid
#include "pid.h"
// msg
#include <ukftest/laserInfo.h>
#include <ukftest/UAV.h>
#include <ukftest/Ctrl.h>
// debug msg
#include <irobot_tracker/trackerDebug.h>


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
ros::Publisher ctrlpub;
ros::Publisher debug_pub;
 
// PID related variables
PID *ctrl_opticalflow_x, *ctrl_opticalflow_y;
float imuVX , imuVY , imuVZ;
double Kp_opticalflow_vel, Ki_opticalflow_vel,Kd_opticalflow_vel;
float opticalflow_accX,opticalflow_accY,opticalflow_accZ;
vector3f acc_b,v_b;
///////////////////////////////////
double pid_gain;
double controlLimit;

vector4f controlInput;

bool is_debug_on;
// variables related to generate debug messages
float p_speed_x, i_speed_x, d_speed_x, error_speed_x;
float p_speed_y, i_speed_y, d_speed_y, error_speed_y;

//
double freq;

void imuCallback(const ukftest::UAV& imu);
void spin_callback(const ros::TimerEvent& e);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_opticalflow_neg_controller_node");
	ros::NodeHandle nh;
	ros::Timer timer;

	is_debug_on = true;
	//nh.param("is_debug_on", is_debug_on, true);

	nh.param("pid_gain_opticalflow", pid_gain, 1.0);
	nh.param("controlLimit_opticalflow", controlLimit, 5.0);
	// opticalflow ctrl
	nh.param("Kp_opticalflow_vel", Kp_opticalflow_vel, 2.00);
	nh.param("Ki_opticalflow_vel", Ki_opticalflow_vel, 0.00);
	nh.param("Kd_opticalflow_vel", Kd_opticalflow_vel, 0.00);

	nh.param("freq", freq, 10.0); 

	timer = nh.createTimer(ros::Duration(1.0/max(freq,1.0)), spin_callback);
	
	// avoid ctrl
	ctrl_opticalflow_x = new PID(Kp_opticalflow_vel, Ki_opticalflow_vel, Kd_opticalflow_vel, -30, 30, -800, 800); 
	ctrl_opticalflow_y = new PID(Kp_opticalflow_vel, Ki_opticalflow_vel, Kd_opticalflow_vel, -30, 30, -800, 800); 

	ctrlpub = nh.advertise<ukftest::Ctrl>("opticalflow_ctrl",10);
	imusub = nh.subscribe("uav_imu", 10, imuCallback);
	debug_pub = nh.advertise<irobot_tracker::trackerDebug>("opticalflow_debug",10);
	
	cout<< "uav opticalflow controller start!"<<endl;
	ros::spin();
	cout<< "uav opticalflow controller shutdown!"<<endl;
	return 1;
}    

void spin_callback(const ros::TimerEvent& e)
{
	cout<< "call opticalflow ctrl!"<<endl;
	ukftest::Ctrl outMsg;
	outMsg.header.stamp = ros::Time::now();
	outMsg.header.frame_id = "laser_opticalflow_ctrl";

	float delta_t = 1.0/freq;

	v_b[0] = 10; //cm/s
	v_b[1] = 0;
	v_b[2] = 0;

	cout <<"target _V: "<<"x " <<v_b[0] << " |y " << v_b[1]<< endl;
	cout <<"current_V: "<<"x " <<imuVX << " |y " << imuVY<< endl;
		
	ctrl_opticalflow_x -> set_point(v_b[0]);
	ctrl_opticalflow_y -> set_point(v_b[1]);

	acc_b[0] = ctrl_opticalflow_x -> update(imuVX, delta_t, &error_speed_x, &p_speed_x, &i_speed_x, &d_speed_x); //control
	acc_b[1] = ctrl_opticalflow_y -> update(imuVY, delta_t, &error_speed_y, &p_speed_y, &i_speed_y, &d_speed_y); //control
	acc_b[2] = 0;
		
	// from acceleration to angle
	controlInput[0] = pid_gain*atan2(-acc_b[0], 980.0)/M_PI*180; //body x , pitch
	controlInput[1] = pid_gain*atan2(acc_b[1], 980.0)/M_PI*180; //body y , roll
	controlInput[2] = pid_gain*acc_b[2];                        //body z , vel
	controlInput[3] = 0;   //assume no yaw control

	// constrain control input
	if (controlInput[0] > controlLimit) 
	controlInput[0] = controlLimit;
	if (controlInput[0] < -controlLimit)
	controlInput[0] = -controlLimit;
	if (controlInput[1] > controlLimit) 
	controlInput[1] = controlLimit;
	if (controlInput[1] < -controlLimit)
	controlInput[1] = -controlLimit;

	outMsg.isCtrl = 1;
	outMsg.ctrl.z = (int)(controlInput[3]*100);         //yaw_rate  
	outMsg.ctrl.x = (int)(controlInput[0]*100);         //pitch
	outMsg.ctrl.y = (int)(controlInput[1]*100);         //roll
	outMsg.ctrl.w = (int)(controlInput[2]*100);         //vel
	ctrlpub.publish(outMsg);
	if(is_debug_on)
	{

		cout<< "Out put massage: \n";
		cout<< "yaw rate  :" << outMsg.ctrl.z << endl;
		cout<< "pitch     :" << outMsg.ctrl.x << endl;
		cout<< "roll      :" << outMsg.ctrl.y << endl;
		cout<< "vel       :" << outMsg.ctrl.w << endl;
		cout<< "isOpticalflow:" << outMsg.isCtrl << endl;

		irobot_tracker::trackerDebug debug_msg;
		debug_pub.publish(debug_msg);
	}
}

void imuCallback(const ukftest::UAV& imu)
{
	//cout<< "b_laser_IMUcall!"<<endl;
	imuVX = imu.linear_v.x;
	imuVY = imu.linear_v.y;
	imuVZ = imu.linear_v.z;
	
	vector4f q_be;
	vector3f v_eb, v_bb;
	matrix3f R_be;
	v_eb[0] = imuVX;
	v_eb[1] = imuVY;
	v_eb[2] = imuVZ;
	q_be[0] = imu.orientation.w; 
	q_be[1] = -imu.orientation.x;
	q_be[2] = -imu.orientation.y;
	q_be[3] = -imu.orientation.z;
	quat_to_DCM(R_be, q_be);
	matrix3f_multi_vector3f(v_bb, R_be, v_eb); //v_bb = R_be * v_eb;

	imuVX = v_bb[0];
	imuVY = v_bb[1];
	imuVZ = v_bb[2];
}


