/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	NEGtracker.cpp
* Brief: NEGtracker
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/7/03 14:00
* History:
************************************************************************/


//ros
#include <ros/ros.h>

//opencv lib
#include <opencv2/opencv.hpp>

//YU YUN
#include "math_basic.h"
#include "math_vector.h"
#include "math_matrix.h"
#include "math_quaternion.h"
#include "math_rotation.h"

//eigen lib
#include <eigen3/Eigen/Dense>

//msg
#include "ukftest/UAV.h"
#include "ukftest/markerInfo.h"

using namespace std;
using namespace cv;
using namespace Eigen;


//K matrix
/*
Mat K = (Mat_< float >(3,3)<<
            622.6379/2,0,320/2,
            0,622.6379/2,240/2,
            0,0,1);
*/
Mat K=(Mat_<float>(3,3)<<
	180.2346315,0,159.5,
	0,180.2346315,119.5,
	0,0,1);

Mat distCoeff = (Mat_<float>(5,1) << -0.276089956,0.1087447314,0.0,0.0,-0.024036442);
// Z_et is height
Vector3f T_et;

VectorXf imu_data;

//unit mm
float heighttest = 380.0f;

int imu_callback_counter_ = 0;
int uv_callback_counter_ = 0;


void getXYZ(Vector3f& T_et, const Vector3f& uv1, const Vector4f& meas_imu, const float heigt);

void uvCallback(const ukftest::markerInfo& uv_data);
void imuCallback(const ukftest::UAV& imu);


int main (int argc, char** argv)
{
	// ros init and parameters retrieve
	ros::init(argc, argv, "NEGtracker_node");

	std::cout<< "NEGtracker node start!" <<endl;

	ros::NodeHandle nh;

	ros::Subscriber uvSub, imuSub;

	uvSub  = nh.subscribe("marker_pose", 10, uvCallback);
	imuSub = nh.subscribe("uav_imu", 10, imuCallback);

	ros::spin();
}



void uvCallback(const ukftest::markerInfo& uv_data)
{
	if(uv_callback_counter_ == 0) cout<< "uvCallback \n";
	// uv in image


	// debug
	cout<< "T_ct: "<< uv_data.pose.position.x<<" | "<< uv_data.pose.position.y<<" | "<< uv_data.pose.position.z<<"\n";

	// uv1
	Vector3f uv1;
	uv1(0) = uv_data.uv1.x;
	uv1(1) = uv_data.uv1.y;
	uv1(2) = 1.0f;

	Mat src(1,1,CV_32FC2);
	Mat dst(1,1,CV_32FC2);
	src.ptr<float>(0)[0] = uv1(0);
	src.ptr<float>(0)[1] = uv1(1);

	undistortPoints(src,dst,K,distCoeff);

	uv1(0) = dst.ptr<float>(0)[0] * K.at<float>(0,0) + K.at<float>(0,2);
	uv1(1) = dst.ptr<float>(0)[1] * K.at<float>(1,1) + K.at<float>(1,2);

	cout<< "dst: "<<dst.ptr<float>(0)[0]<<" | "<<dst.ptr<float>(0)[1]<<endl;
	// debug
	cout<< "uv1: "<< uv1(0)<<" | "<< uv1(1)<<" | "<< uv1(2)<<"\n";
	if(imu_callback_counter_ > 0)
	{
		//debug
		vector4f q_debug;
		q_debug[0] = imu_data(0);
		q_debug[1] = imu_data(1);
		q_debug[2] = imu_data(2);
		q_debug[3] = imu_data(3);
		float yaw, pitch, roll;
		quat_to_eular(&yaw, &pitch, &roll, q_debug);
		cout<< "yaw: "<< yaw/ M_PI * 180.0f<< "| pitch: "<< pitch/ M_PI * 180.0f<< "| roll: "<< roll/ M_PI * 180.0f<< endl;
		// imu
		Vector4f q_eb;
		q_eb(0) = imu_data(0);
		q_eb(1) = imu_data(1);
		q_eb(2) = imu_data(2);
		q_eb(3) = imu_data(3);

		getXYZ(T_et, uv1, q_eb, heighttest);

		cout<< "T_et: \n"<< T_et << endl;
	}

	uv_callback_counter_++;
}



void imuCallback(const ukftest::UAV& imu)
{

	if(imu_callback_counter_ == 0)cout<< "imuCallback \n";
	// receive data

	imu_data.resize(10);
	// q_eb
	imu_data(0) = imu. orientation.w;
	imu_data(1) = imu. orientation.x;
	imu_data(2) = imu. orientation.y;
	imu_data(3) = imu. orientation.z;

	// a_eb or a_ec  where we set the unit of a to cm/s^2
	imu_data(4) = imu. linear_a.x ;
	imu_data(5) = imu. linear_a.y ;
	imu_data(6) = imu. linear_a.z ;

	// w_bb or w_bc
	imu_data(7) = imu. angular_v.x/180.0f*M_PI;
	imu_data(8) = imu. angular_v.y/180.0f*M_PI;
	imu_data(9) = imu. angular_v.z/180.0f*M_PI;

	imu_callback_counter_++;
}

void getXYZ(Vector3f& T_et, const Vector3f& uv1, const Vector4f& meas_imu, const float heigt)
{
	vector4f q_eb;

	matrix3f R_cb, R_eb, R_be, R_cetmp;
	matrix3f_set_value(R_cb,
			 0,  1, 0,
			-1,  0, 0,
			 0,  0, 1);

	q_eb[0] = meas_imu(0);
	q_eb[1] = meas_imu(1);
	q_eb[2] = meas_imu(2);
	q_eb[3] = meas_imu(3);

	quat_to_DCM(R_eb, q_eb);
	get_matrix3f_transpose(R_be, R_eb);

	matrix3f_multi_matrix3f(R_cetmp, R_cb, R_be);
	// opencv lib
	//R_cNEG
	Mat R_ce = (Mat_< float >(3,3)<<
				R_cetmp[0][0],R_cetmp[0][1],R_cetmp[0][2],
				R_cetmp[1][0],R_cetmp[1][1],R_cetmp[1][2],
				R_cetmp[2][0],R_cetmp[2][1],R_cetmp[2][2]);

	Mat XYZ = (Mat_< float >(3,1)<<	0.0f,
					0.0f,
					1.0f);

	Mat uv = (Mat_< float >(3,1)<<	uv1(0),
					uv1(1),
					uv1(2));
	Mat offset = (Mat_< float >(3,1)<<	0.0f,
						88.0f,
						-10.0f);
	XYZ = (K * R_ce).inv() * uv;
	Mat RoffSet = R_ce.inv() * offset;
	XYZ = ((heigt + RoffSet.at<float>(2,0))/ XYZ.at<float>(2,0)) * XYZ - RoffSet;
	// end opencv lib
	T_et(0) = XYZ.at<float>(0,0);
	T_et(1) = XYZ.at<float>(1,0);
	T_et(2) = XYZ.at<float>(2,0);
}







