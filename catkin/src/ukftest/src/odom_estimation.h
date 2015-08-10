/************************************************************************
* Copyright(c) 2014  YING Jiahang
* All rights reserved.
*
* File:	odom_estimation.h
* Brief: odom estimation for ROS UKF
* Version: 1.0
* Author: YING Jiahang
* Email: yingjiahang@gmail.com
* Date:	2014/6/16 14:00
* History: modified from ROS ekf http://wiki.ros.org/robot_pose_ekf
************************************************************************/

#pragma once

//filter
#include "SRUKF.h"

// ros
#include <ros/ros.h>
#include <ros/console.h>
// eigen lib
#include <eigen3/Eigen/Dense>

using namespace ros;

namespace estimation
{

	void process_func(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt);
	void measure_func_imu(VectorXf& z_t, const VectorXf& x_t);
	void measure_func_voimu(VectorXf& z_t, const VectorXf& x_t);
	void measure_func_vo(VectorXf& z_t, const VectorXf& x_t);

	class OdomEstimation
	{
	public:
		OdomEstimation();
		~OdomEstimation();

		SRUKF* ukf;

		// create VO MODEL
		MatrixXf voR;
		MatrixXf voQ;
		// create VO&IMU MODEL
		MatrixXf voImuR;
		MatrixXf voImuQ;
		// create IMU MODEL
		MatrixXf imuR;
		MatrixXf imuQ;

		bool update(bool imu_active, bool vo_active, const Time& filter_time);

		void initialize(const Time& time);

		bool isInitialized() {return filter_initialized_;};

		bool isValidFdata();
		
		bool getEstimate(VectorXf& estimate);

		void addMeasurement_vo(const VectorXf& meas_vo);

		void addMeasurement_imu(const VectorXf& meas_imu);


	private:

		double q_; //Std of process 
		double r_; //Std of measurement
		double n_; //n
		double m_; //m
		float dt;
		bool filter_initialized_, imu_initialized_, vo_initialized_;
		float a_tc_old[3], a_tc_now[3], a_ec_old[3] , a_ec_now[3];
		ros::Time filter_time_old_;
		// SRUKF related variables
		void (*f_func)(VectorXf& x_t, const VectorXf& x_t_1, const vector3f a, const float dt);   // function pointer to process function
		void (*h_func)(VectorXf& z_t, const VectorXf& x_t);    

		//tilt and torsion to get a new T_tc
		void getCoVoImu(VectorXf& filter_compose, const VectorXf& meas_vo, const VectorXf& meas_imu);
		
		bool preInitialize(const ros::Time& time);

		VectorXf s_init, s_meas;
		VectorXf vo_meas, vo_init_meas;
		VectorXf imu_meas, imu_init_meas;
		VectorXf init_compose, now_compose;
		VectorXf voImu_meas;
		VectorXf filter_data;
	};  // class
}; // namespace
