/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	odom_estimation_node.h
* Brief: odom estimation node for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/6/19 22:00
* History: modified from YUYUN and YANGSHUO pidctrl
************************************************************************/
// ros
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
// eigen
#include <eigen3/Eigen/Dense>
// pid
#include "pid.h"
// msg
#include <ukftest/ukfData.h>

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
ros::Subscriber ukfsub;
ros::Publisher ctrlpub;
ros::Publisher debug_pub;
 
// PID related variables
PID *ctrl_x, *ctrl_y, *ctrl_z, *ctrl_yaw;
PID *ctrl_dx, *ctrl_dy, *ctrl_dz, *ctrl_dyaw;
//PID *ctrl_ddx, *ctrl_ddy, *ctrl_ddz;
double Kp_pos, Kp_vel, Kp_acc,
       Kd_pos, Kd_vel, Kd_acc,
       Ki_pos, Ki_vel, Ki_acc;
double pid_gain;
double controlLimit;
float target_velX, target_velY, target_velZ;
float target_accX, target_accY, target_accZ;
//float target_angleX, target_angleY, target_angleZ;
const float targetPosition[3] = {-85.0,-13.0,0.0};
const float targetOrientation[4] = {0.0, 0.0, -1.0, 0.0}; // w x y z 
const float targetYaw = 0.0;
vector3f acc_e, acc_b; // final control variables
vector4f controlInput, q_be_filter;
matrix3f R_be_filter;
bool is_debug_on;
// variables related to generate debug messages
float d_pval, d_ival, d_dval, d_error;

float p_pos_x, i_pos_x, d_pos_x, error_pos_x;
float p_speed_x, i_speed_x, d_speed_x, error_speed_x;
//float p_acc_x, i_acc_x, d_acc_x, error_acc_x;

float p_pos_y, i_pos_y, d_pos_y, error_pos_y;
float p_speed_y, i_speed_y, d_speed_y, error_speed_y;
//float p_acc_y, i_acc_y, d_acc_y, error_acc_y;

void ukfCallback(const ukftest::ukfData& ukf);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	is_debug_on = true;
	//nh.param("is_debug_on", is_debug_on, true);
	nh.param("Kp_pos", Kp_pos, 0.90);
	nh.param("Ki_pos", Ki_pos, 0.15);
	nh.param("Kd_pos", Kd_pos, 0.45);

	nh.param("Kp_vel", Kp_vel, 1.80);
	nh.param("Ki_vel", Ki_vel, 0.00);
	nh.param("Kd_vel", Kd_vel, 0.34);

	nh.param("Kp_acc", Kp_acc, 0.27);
	nh.param("Ki_acc", Ki_acc, 0.20);
	nh.param("Kd_acc", Kd_acc, 0.06);
	nh.param("pid_gain", pid_gain, 1.0);

	nh.param("controlLimit", controlLimit, 5.0);
	
	//init pid controllers 
	ctrl_x = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_x -> set_point(targetPosition[0]);
	ctrl_y = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_y -> set_point(targetPosition[1]);
	ctrl_z = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_z -> set_point(targetPosition[2]);

	ctrl_dx = new PID(Kp_vel, Ki_vel, Kd_vel, -30, 30, -800, 800); 
	ctrl_dy = new PID(Kp_vel, Ki_vel, Kd_vel, -30, 30, -800, 800); 
	ctrl_dz = new PID(Kp_vel, Ki_vel, Kd_vel, -200, 200, -800, 800); 

	/*
	ctrl_ddx = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 
	ctrl_ddy = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 
	ctrl_ddz = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 
	*/

	ctrl_yaw = new PID(0.8, 0.0, 1.0, -200, 200, -15, 15); ctrl_yaw -> set_point(targetYaw);
	
	ctrlpub = nh.advertise<geometry_msgs::Quaternion>("/board_ctrl",10);
	ukfsub = nh.subscribe("/laser_neg_estimation_node/odom_combined", 10, ukfCallback);
	debug_pub = nh.advertise<irobot_tracker::trackerDebug>("/tracker_debug",10);
	
	cout<< "uav controller start!"<<endl;
	ros::spin();
	cout<< "uav controller shutdown!"<<endl;
	return 1;
}    

void ukfCallback(const ukftest::ukfData& ukf)
{
	
	cout<< "call ukf ctrl!"<<endl;
	geometry_msgs::Quaternion outMsg;
	if(ukf.isctrl ==1)
	{
		float delta_t = ukf.dt;

		// until here, ukf-> state_post should contain a good estimation of T_tc
		// ready to generate control info
		target_velX = ctrl_x -> update(ukf.xyz.x, delta_t, &error_pos_x, &p_pos_x, &i_pos_x, &d_pos_x);   //control roll
		target_velY = ctrl_y -> update(ukf.xyz.y, delta_t, &error_pos_y, &p_pos_y, &i_pos_y, &d_pos_y);   //control pitch
		target_velZ = ctrl_z -> update(ukf.xyz.z, delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control vel

		/*
		cout << "=====================================" << endl;
		cout <<"dt: "<< delta_t << endl;
		cout <<"ukf.xyz.x: "<< ukf.xyz.x<< endl;
		  
		cout <<"error_pos_x: "<< error_pos_x << endl;
		  
		cout <<"p_pos_x: "<< p_pos_x << endl;
		cout <<"i_pos_x: "<< i_pos_x << endl;
		cout <<"d_pos_x: "<< d_pos_x << endl;
		  
		cout <<"target_velX: "<< target_velX << endl;
		*/

		ctrl_dx -> set_point(target_velX);
		ctrl_dy -> set_point(target_velY);
		ctrl_dz -> set_point(target_velZ);

		target_accX = ctrl_dx -> update(ukf.v.x, delta_t, &error_speed_x, &p_speed_x, &i_speed_x, &d_speed_x); //control roll
		target_accY = ctrl_dy -> update(ukf.v.y, delta_t, &error_speed_y, &p_speed_y, &i_speed_y, &d_speed_y);   //control pitch
		target_accZ = ctrl_dz -> update(ukf.v.z, delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control vel
		/*
		ctrl_ddx -> set_point(target_accX);
		ctrl_ddy -> set_point(target_accY);
		ctrl_ddz -> set_point(target_accZ);

		target_angleX = ctrl_ddx -> update(ukf.a.x, delta_t, &error_acc_x, &p_acc_x, &i_acc_x, &d_acc_x);   //control vel
		target_angleY = ctrl_ddy -> update(ukf.a.y, delta_t, &error_acc_y, &p_acc_y, &i_acc_y, &d_acc_y);   //control vel
		target_angleZ = ctrl_ddz -> update(ukf.a.z, delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control vel
		*/
		// target is a_et we can acquire the a_eb by adding the - 
		acc_e[0] = -target_accX;
		acc_e[1] = -target_accY;
		acc_e[2] = -target_accZ;


		// transform acc_e to acc_b
		q_be_filter[0] = ukf.q.w;
		q_be_filter[1] = - ukf.q.x;
		q_be_filter[2] = - ukf.q.y;
		q_be_filter[3] = - ukf.q.z;

		quat_to_DCM(R_be_filter, q_be_filter);

		matrix3f_multi_vector3f(acc_b, R_be_filter, acc_e); //acc_b = R_be_filter'*acc_e;

		// from acceleration to angle
		controlInput[0] = pid_gain*atan2(-acc_b[0], 980.0)/M_PI*180; //body x , pitch
		controlInput[1] = pid_gain*atan2(acc_b[1], 980.0)/M_PI*180; //body y , roll
		controlInput[2] = pid_gain*acc_b[2];                        //body z , vel
		controlInput[2] = 0;
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
	
		outMsg.z = (int)(controlInput[3]*100);         //yaw_rate  
		outMsg.x = (int)(controlInput[0]*100);         //pitch
		outMsg.y = (int)(controlInput[1]*100);         //roll
		outMsg.w = (int)(controlInput[2]*100);         //vel
		ctrlpub.publish(outMsg);
	}
	else
	{
		outMsg.z = 0;         //yaw_rate  
		outMsg.x = 0;         //pitch
		outMsg.y = 0;         //roll
		outMsg.w = 0;         //vel
		ctrlpub.publish(outMsg);
	
	}

	if(is_debug_on)
	{

		cout<< "Out put massage: \n";
		cout<< "yaw rate:" << outMsg.z << endl;
		cout<< "pitch   :" << outMsg.x << endl;
		cout<< "roll    :" << outMsg.y << endl;
		cout<< "vel     :" << outMsg.w << endl;

		irobot_tracker::trackerDebug debug_msg;
		
		// x
		debug_msg.target_pos.x = targetPosition[0] - 100;
		debug_msg.filter_pos.x = ukf.xyz.x - 100;
		debug_msg.error_pos.x = error_pos_x - 100;
		debug_msg.p_pos.x = p_pos_x - 100; 
		debug_msg.i_pos.x = i_pos_x - 100; 
		debug_msg.d_pos.x = d_pos_x - 100; 

		debug_msg.target_speed.x = target_velX; 
		debug_msg.filter_speed.x = ukf.v.x;
		debug_msg.error_speed.x = error_speed_x;
		debug_msg.p_speed.x = p_speed_x; 
		debug_msg.i_speed.x = i_speed_x; 
		debug_msg.d_speed.x = d_speed_x; 

		debug_msg.target_acc.x = target_accX + 100;
		debug_msg.imu_linear_a.x = ukf.a.x + 100;

		// y
		debug_msg.target_pos.y = targetPosition[1] - 100;
		debug_msg.filter_pos.y = ukf.xyz.y - 100;
		debug_msg.error_pos.y = error_pos_y - 100;
		debug_msg.p_pos.y = p_pos_y - 100; 
		debug_msg.i_pos.y = i_pos_y - 100; 
		debug_msg.d_pos.y = d_pos_y - 100; 

		debug_msg.target_speed.y = target_velY; 
		debug_msg.filter_speed.y = ukf.v.y ;
		debug_msg.error_speed.y = error_speed_y ;
		debug_msg.p_speed.y = p_speed_y ; 
		debug_msg.i_speed.y = i_speed_y ; 
		debug_msg.d_speed.y = d_speed_y ; 

		debug_msg.target_acc.y = target_accY + 100;
		debug_msg.imu_linear_a.y = ukf.a.y + 100;

		debug_pub.publish(debug_msg);
	}
}




