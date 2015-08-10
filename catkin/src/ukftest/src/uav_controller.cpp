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
PID *ctrl_ddx, *ctrl_ddy, *ctrl_ddz;
double Kp_pos, Kp_vel, Kp_acc,
       Kd_pos, Kd_vel, Kd_acc,
       Ki_pos, Ki_vel, Ki_acc;
double Kp_pos_y, Kp_vel_y, Kp_acc_y,
       Kd_pos_y, Kd_vel_y, Kd_acc_y,
       Ki_pos_y, Ki_vel_y, Ki_acc_y;
double Kp_pos_z, Kp_vel_z, Kp_acc_z,
       Kd_pos_z, Kd_vel_z, Kd_acc_z,
       Ki_pos_z, Ki_vel_z, Ki_acc_z;
double Kp_yaw, Ki_yaw, Kd_yaw;
int countX = 0,countY = 0,countZ = 0, countYaw = 0;
bool tracking = true, hovering = false, landing =false;
double pid_gain;
double controlLimit, controlLimitVel, controlLimitYawrate;
float target_velX, target_velY, target_velZ;
float target_accX, target_accY, target_accZ;
float target_angleX, target_angleY, target_angleZ;
float yaw_tc , target_yaw_rate;
const float targetPosition[3] = {0.0,0.0,40.0};
const float targetOrientation[4] = {0.0, 0.0, -1.0, 0.0}; // w x y z 
const float targetYaw = 0.0;
vector3f acc_t, acc_c, acc_b, v_tb , v_cb, v_bb ; // final control variables
vector4f controlInput, q_tc_filter;
matrix3f R_tc_filter, R_tmp ,R_bc;
bool is_debug_on;
// variables related to generate debug messages
float d_pval, d_ival, d_dval, d_error;

float p_pos_x, i_pos_x, d_pos_x, error_pos_x;
float p_speed_x, i_speed_x, d_speed_x, error_speed_x;
float p_acc_x, i_acc_x, d_acc_x, error_acc_x;

float p_pos_y, i_pos_y, d_pos_y, error_pos_y;
float p_speed_y, i_speed_y, d_speed_y, error_speed_y;
float p_acc_y, i_acc_y, d_acc_y, error_acc_y;

float p_pos_z, i_pos_z, d_pos_z, error_pos_z;
float p_speed_z, i_speed_z, d_speed_z, error_speed_z;
float p_acc_z, i_acc_z, d_acc_z, error_acc_z;

float p_yaw, i_yaw, d_yaw, error_yaw;

void ukfCallback(const ukftest::ukfData& ukf);
void autoLanding(float& ctrlZ, float& ctrlYaw, int& cX, int& cY, int& cZ, int & cYaw,float errorX, float errorY, float errorZ ,float errorYaw);
void getYaw(vector4f q_tc, float &yaw);

int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "uav_controller_node");
	ros::NodeHandle nh;
	ros::NodeHandle pnh("~");
	is_debug_on = true;
	//nh.param("is_debug_on", is_debug_on, true);
	// ctrl x
	nh.param("Kp_pos", Kp_pos, 0.90);
	nh.param("Ki_pos", Ki_pos, 0.15);
	nh.param("Kd_pos", Kd_pos, 0.45);

	nh.param("Kp_vel", Kp_vel, 1.80);
	nh.param("Ki_vel", Ki_vel, 0.00);
	nh.param("Kd_vel", Kd_vel, 0.34);

	nh.param("Kp_acc", Kp_acc, 0.27);
	nh.param("Ki_acc", Ki_acc, 0.20);
	nh.param("Kd_acc", Kd_acc, 0.06);

	// ctrl y
	nh.param("Kp_pos_y", Kp_pos_y, 0.90);
	nh.param("Ki_pos_y", Ki_pos_y, 0.15);
	nh.param("Kd_pos_y", Kd_pos_y, 0.45);

	nh.param("Kp_vel_y", Kp_vel_y, 1.80);
	nh.param("Ki_vel_y", Ki_vel_y, 0.00);
	nh.param("Kd_vel_y", Kd_vel_y, 0.34);

	nh.param("Kp_acc_y", Kp_acc_y, 0.27);
	nh.param("Ki_acc_y", Ki_acc_y, 0.20);
	nh.param("Kd_acc_y", Kd_acc_y, 0.06);

	// ctrl z
	nh.param("Kp_pos_z", Kp_pos_z, 0.40);
	nh.param("Ki_pos_z", Ki_pos_z, 0.00);
	nh.param("Kd_pos_z", Kd_pos_z, 0.00);

	nh.param("Kp_vel_z", Kp_vel_z, 0.18);
	nh.param("Ki_vel_z", Ki_vel_z, 0.00);
	nh.param("Kd_vel_z", Kd_vel_z, 0.00);

	nh.param("Kp_acc_z", Kp_acc_z, 0.027);
	nh.param("Ki_acc_z", Ki_acc_z, 0.020);
	nh.param("Kd_acc_z", Kd_acc_z, 0.006);



	// ctrl yaw
	nh.param("Kp_yaw", Kp_yaw, 0.800);
	nh.param("Ki_yaw", Ki_yaw, 0.010);
	nh.param("Kd_yaw", Kd_yaw, 1.000);

	//////////
	nh.param("pid_gain", pid_gain, 1.0);
	
	nh.param("controlLimit", controlLimit, 5.0);
	nh.param("controlLimitVel", controlLimitVel, 0.5);
	nh.param("controlLimitYawrate", controlLimitYawrate, 10.0);
	////////////////////////////////////////////////////////
	matrix3f_set_value(R_bc, 0, -1, 0,
				 1,  0, 0,
				 0,  0, 1);
	//init pid controllers 
	ctrl_x = new PID(Kp_pos, Ki_pos, Kd_pos, -200, 200, -950, 950); ctrl_x -> set_point(targetPosition[0]);
	ctrl_y = new PID(Kp_pos_y, Ki_pos_y, Kd_pos_y, -200, 200, -950, 950); ctrl_y -> set_point(targetPosition[1]);
	ctrl_z = new PID(Kp_pos_z, Ki_pos_z, Kd_pos_z, -200, 200, -950, 950); ctrl_z -> set_point(targetPosition[2]);

	ctrl_dx = new PID(Kp_vel, Ki_vel, Kd_vel, -30, 30, -800, 800); 
	ctrl_dy = new PID(Kp_vel_y, Ki_vel_y, Kd_vel_y, -30, 30, -800, 800); 
	ctrl_dz = new PID(Kp_vel_z, Ki_vel_z, Kd_vel_z, -200, 200, -800, 800); 

	ctrl_ddx = new PID(Kp_acc, Ki_acc, Kd_acc, -200, 200, -800, 800); 
	ctrl_ddy = new PID(Kp_acc_y, Ki_acc_y, Kd_acc_y, -200, 200, -800, 800); 
	ctrl_ddz = new PID(Kp_acc_z, Ki_acc_z, Kd_acc_z, -200, 200, -800, 800); 


	ctrl_yaw = new PID(Kp_yaw, Ki_yaw, Kd_yaw, -200, 200, -15, 15); ctrl_yaw -> set_point(targetYaw);
	
	ctrlpub = nh.advertise<geometry_msgs::Quaternion>("/board_ctrl",10);
	ukfsub = nh.subscribe("/odom_estimation_node/odom_combined", 10, ukfCallback);
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
		target_velZ = ctrl_z -> update(ukf.xyz.z, delta_t, &error_pos_z, &d_pval, &d_ival, &d_dval);   //control vel
		v_tb[0] = target_velX;
		v_tb[1] = target_velY;
		v_tb[2] = target_velZ;
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

		ctrl_ddx -> set_point(target_accX);
		ctrl_ddy -> set_point(target_accY);
		ctrl_ddz -> set_point(target_accZ);

		target_angleX = ctrl_ddx -> update(ukf.a.x, delta_t, &error_acc_x, &p_acc_x, &i_acc_x, &d_acc_x);   //control vel
		target_angleY = ctrl_ddy -> update(ukf.a.y, delta_t, &error_acc_y, &p_acc_y, &i_acc_y, &d_acc_y);   //control vel
		target_angleZ = ctrl_ddz -> update(ukf.a.z, delta_t, &d_error, &d_pval, &d_ival, &d_dval);   //control vel

		acc_t[0] = target_angleX; //+ target_accX;
		acc_t[1] = target_angleY; //+ target_accY;
		acc_t[2] = target_angleZ; //+ target_accZ;
		
		acc_t[0] = target_accX;
		acc_t[1] = target_accY;
		acc_t[2] = target_accZ;

		// transform acc_t to acc_b
		q_tc_filter[0] = ukf.q.w;
		q_tc_filter[1] = ukf.q.x;
		q_tc_filter[2] = ukf.q.y;
		q_tc_filter[3] = ukf.q.z;

		getYaw(q_tc_filter, yaw_tc);
		target_yaw_rate = - ctrl_yaw-> update(yaw_tc, delta_t, &error_yaw, &p_yaw, &i_yaw, &d_yaw);
		cout<< 	"targetYaw: "<< targetYaw << "| Yaw: "
			<< yaw_tc <<" | ctrl:"<< target_yaw_rate<< " |error: "<<  error_yaw<<endl;	
		
		quat_to_DCM(R_tc_filter, q_tc_filter);

		get_matrix3f_transpose(R_tmp, R_tc_filter);
		matrix3f_multi_vector3f(acc_c, R_tmp, acc_t); //acc_c = R_tc_filter'*acc_t;
		matrix3f_multi_vector3f(acc_b, R_bc, acc_c); //acc_b = R_bc*acc_c;
		
		matrix3f_multi_vector3f(v_cb, R_tmp, v_tb); //acc_c = R_tc_filter'*acc_t;
		matrix3f_multi_vector3f(v_bb, R_bc, v_cb); //acc_b = R_bc*acc_c;
		// from acceleration to angle
		controlInput[0] = pid_gain*atan2(-acc_b[0], 980.0)/M_PI*180; //body x , pitch
		controlInput[1] = pid_gain*atan2(acc_b[1], 980.0)/M_PI*180; //body y , roll
		controlInput[2] = pid_gain* v_bb[2];                        //body z , vel
		controlInput[3] = pid_gain* target_yaw_rate;   			// yaw control

		// constrain control input
		if (controlInput[0] > controlLimit) 
		controlInput[0] = controlLimit;
		if (controlInput[0] < -controlLimit)
		controlInput[0] = -controlLimit;
		if (controlInput[1] > controlLimit) 
		controlInput[1] = controlLimit;
		if (controlInput[1] < -controlLimit)
		controlInput[1] = -controlLimit;

		if (controlInput[2] > controlLimitVel) 
		controlInput[2] = controlLimitVel;
		if (controlInput[2] < -controlLimitVel)
		controlInput[2] = -controlLimitVel;

		if (controlInput[3] > controlLimitYawrate) 
		controlInput[3] = controlLimitYawrate;
		if (controlInput[3] < -controlLimitYawrate)
		controlInput[3] = -controlLimitYawrate;

		
		autoLanding(controlInput[2],controlInput[3], countX, countY, countZ, countYaw,
				 error_pos_x, error_pos_y, error_pos_z, error_yaw);
		outMsg.z = (int)(controlInput[3]*100);         //yaw_rate  
		outMsg.x = (int)(controlInput[0]*100);         //pitch
		outMsg.y = (int)(controlInput[1]*100);         //roll
		outMsg.w = (int)(controlInput[2]*100);         //vel
		//outMsg.w = 0; 
		ctrlpub.publish(outMsg);
	}
	else
	{
		
		//!!!
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
		debug_msg.error_acc.x = error_acc_x + 100;
		debug_msg.p_acc.x = p_acc_x + 100; 
		debug_msg.i_acc.x = i_acc_x + 100; 
		debug_msg.d_acc.x = d_acc_x + 100; 

		// y
		debug_msg.target_pos.y = targetPosition[1] - 100;
		debug_msg.filter_pos.y = ukf.xyz.y - 100;
		debug_msg.error_pos.y = error_pos_y - 100;
		debug_msg.p_pos.y = p_pos_y - 100; 
		debug_msg.i_pos.y = i_pos_y - 100; 
		debug_msg.d_pos.y = d_pos_y - 100; 

		debug_msg.target_speed.y = target_velY; 
		debug_msg.filter_speed.y = ukf.v.y;
		debug_msg.error_speed.y = error_speed_y;
		debug_msg.p_speed.y = p_speed_y; 
		debug_msg.i_speed.y = i_speed_y; 
		debug_msg.d_speed.y = d_speed_y; 

		debug_msg.target_acc.y = target_accY + 100;
		debug_msg.imu_linear_a.y = ukf.a.y + 100;
		debug_msg.error_acc.y = error_acc_y + 100;
		debug_msg.p_acc.y = p_acc_y + 100; 
		debug_msg.i_acc.y = i_acc_y + 100; 
		debug_msg.d_acc.y = d_acc_y + 100; 

		// z
		debug_msg.target_pos.z = targetPosition[2] - 100;
		debug_msg.filter_pos.z = ukf.xyz.z - 100;
		debug_msg.error_pos.z = error_pos_z - 100;
		debug_msg.p_pos.z = p_pos_z - 100; 
		debug_msg.i_pos.z = i_pos_z - 100; 
		debug_msg.d_pos.z = d_pos_z - 100; 

		debug_msg.target_speed.z = target_velZ * 100.0f; 
		debug_msg.filter_speed.z = ukf.v.z;
		debug_msg.error_speed.z = error_speed_z;
		debug_msg.p_speed.z = p_speed_z; 
		debug_msg.i_speed.z = i_speed_z; 
		debug_msg.d_speed.z = d_speed_z; 

		debug_msg.target_acc.z = target_accZ + 100;
		debug_msg.imu_linear_a.z = ukf.a.z + 100;
		debug_msg.error_acc.z = error_acc_z + 100;
		debug_msg.p_acc.z = p_acc_z + 100; 
		debug_msg.i_acc.z = i_acc_z + 100; 
		debug_msg.d_acc.z = d_acc_z + 100; 

		debug_pub.publish(debug_msg);
	}
}


void autoLanding(float& ctrlZ, float& ctrlYaw, int& cX, int& cY, int& cZ, int& cYaw, float errorX, float errorY, float errorZ, float errorYaw)
{
	// auto landing
	if (tracking)
        {
		if ( abs(errorZ) < 7) cZ++;
       		if ( cZ > 15 && abs(errorZ) < 5)
		{
			hovering = true;
			tracking = false;
		}
                cout<< "Tracking!!!!!!!!!!"<<endl;
		cout<< "errorZ: "<< errorZ<< endl;
		cout<< "40! :" <<cZ <<endl;
        }

	if ( hovering)
	{
		ctrlZ = 0;
		if ( abs(errorX)<7) cX++;
		if ( abs(errorY)<7) cY++;
		if ( abs(errorYaw)<1) cYaw++;
		if ( cYaw > 200 && abs(errorYaw) <1) ctrlYaw = 0;
		if ( cX > 300 && cY > 300 && abs(errorX) < 1 && abs(errorY) < 1 && ctrlYaw == 0) 
		{
			hovering = false; 
			landing = true;
		}
		cout<< "Hovering!!!!!!!!!!!"<<endl;
		cout<< "errorX: "<< errorX<<" | "<<"errorY: "<<errorY <<"| errorYaw: "<<errorYaw<<endl;
		cout<< "300! :"<< cX << " | "<<cY <<" | "<< cYaw <<endl;
		
	}
	if ( landing) 
	{
		ctrlZ = 0.70;
		cout<< "Landing!!!!!!!!!!!!"<<endl;
	}
}

void getYaw(vector4f q_tc, float &yaw)
{
	float yaw2,pitch2,roll2;
    	quat_to_eular(&yaw2, &pitch2, &roll2, q_tc);
	yaw = yaw2 / M_PI * 180.0f;
	
	cout<< "yaw  : "<< yaw <<endl
		<< "pitch: "<<pitch2 / M_PI * 180.0f<<endl
		<< "roll : "<<roll2 /  M_PI * 180.0f<<endl<<endl;
		
}










